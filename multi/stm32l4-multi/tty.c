/*
 * Phoenix-RTOS
 *
 * STM32L4 TTY driver
 *
 * Copyright 2017, 2018, 2020 Phoenix Systems
 * Author: Jan Sikorski, Aleksander Kaminski, Andrzej Glowinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <paths.h>
#include <posix/utils.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <sys/time.h>
#include <libtty.h>
#include <libtty-lf-fifo.h>

#include "libmulti/libdma.h"
#include "stm32l4-multi.h"
#include "common.h"
#include "config.h"
#include "gpio.h"
#include "tty.h"
#include "rcc.h"


#define TTY1_POS 0
#define TTY2_POS (TTY1_POS + TTY1)
#define TTY3_POS (TTY2_POS + TTY2)
#define TTY4_POS (TTY3_POS + TTY3)
#define TTY5_POS (TTY4_POS + TTY4)

#define TTY_CNT (TTY1 + TTY2 + TTY3 + TTY4 + TTY5)

#define THREAD_POOL    3
#define THREAD_STACKSZ 768
#define THREAD_PRIO    1

#if TTY_CNT != 0

#define IS_POWER_OF_TWO(n) ((n > 0) && (n & (n - 1)) == 0)

#if (!IS_POWER_OF_TWO(TTY1_DMA_RXSZ)) || (!IS_POWER_OF_TWO(TTY2_DMA_RXSZ)) || \
	(!IS_POWER_OF_TWO(TTY3_DMA_RXSZ)) || (!IS_POWER_OF_TWO(TTY4_DMA_RXSZ)) || \
	(!IS_POWER_OF_TWO(TTY5_DMA_RXSZ))
#error "Size of RX DMA buffer has to be a power of two!"
#endif

#if (!IS_POWER_OF_TWO(TTY1_DMA_RXFIFOSZ)) || (!IS_POWER_OF_TWO(TTY2_DMA_RXFIFOSZ)) || \
	(!IS_POWER_OF_TWO(TTY3_DMA_RXFIFOSZ)) || (!IS_POWER_OF_TWO(TTY4_DMA_RXFIFOSZ)) || \
	(!IS_POWER_OF_TWO(TTY5_DMA_RXFIFOSZ))
#error "Size of RX DMA FIFO buffer has to be a power of two!"
#endif

#if (TTY1_DMA && !TTY1) || (TTY2_DMA && !TTY2) || (TTY3_DMA && !TTY3) || \
	(TTY4_DMA && !TTY4) || (TTY5_DMA && !TTY5)
#error "DMA mode cannot be enabled on a disabled TTY!"
#endif

typedef struct {
	char stack[512] __attribute__((aligned(8)));

	volatile unsigned int *base;
	volatile int enabled;

	int bits;
	int parity;
	int baud;

	handle_t cond;
	handle_t inth;
	handle_t irqlock;

	libtty_common_t ttyCommon;

	enum {
		tty_irq,
		tty_dma,
	} type;
	union {
		struct {
			uint8_t rxFifoBuffer[16];
			lf_fifo_t rxFifo;
			volatile int rxready;
		} irq;
		struct {
			const struct libdma_per *per;
			volatile int txDoneFlag;
			size_t txbufsz;
			unsigned char *txbuf;

			unsigned char *rxbuf;
			size_t rxbufsz;
			unsigned int rxbufszMask;

			volatile unsigned int readPos;
			lf_fifo_t rxFifo;

			struct {
				size_t droppedBytes;
			} debug;
		} dma;
	} data;
} tty_ctx_t;


static struct {
	unsigned char poolstack[THREAD_POOL][THREAD_STACKSZ] __attribute__((aligned(8)));
	tty_ctx_t ctx[TTY_CNT];

	unsigned int port;
} uart_common;


static const struct {
	int enabled;
	int dma;
	size_t dmaBufSizeRx;
	size_t dmaRxFifoSize;
	size_t dmaBufSizeTx;
	int pos;
	size_t libttyBufSize;
} ttySetup[] = {
	{ TTY1, TTY1_DMA, TTY1_DMA_RXSZ, TTY1_DMA_RXFIFOSZ, TTY1_DMA_TXSZ, TTY1_POS, TTY1_LIBTTY_BUFSZ },
	{ TTY2, TTY2_DMA, TTY2_DMA_RXSZ, TTY2_DMA_RXFIFOSZ, TTY2_DMA_TXSZ, TTY2_POS, TTY2_LIBTTY_BUFSZ },
	{ TTY3, TTY3_DMA, TTY3_DMA_RXSZ, TTY3_DMA_RXFIFOSZ, TTY3_DMA_TXSZ, TTY3_POS, TTY3_LIBTTY_BUFSZ },
	{ TTY4, TTY4_DMA, TTY4_DMA_RXSZ, TTY4_DMA_RXFIFOSZ, TTY4_DMA_TXSZ, TTY4_POS, TTY4_LIBTTY_BUFSZ },
	{ TTY5, TTY5_DMA, TTY5_DMA_RXSZ, TTY5_DMA_RXFIFOSZ, TTY5_DMA_TXSZ, TTY5_POS, TTY5_LIBTTY_BUFSZ },
};


/* clang-format off */
enum { cr1 = 0, cr2, cr3, brr, gtpr, rtor, rqr, isr, icr, rdr, tdr };


enum { tty_parnone = 0, tty_pareven, tty_parodd };
/* clang-format on */


static inline int tty_txready(tty_ctx_t *ctx)
{
	return (*(ctx->base + isr) & (1 << 7)) ? 1 : 0;
}


static int tty_irqHandler(unsigned int n, void *arg)
{
	tty_ctx_t *ctx = (tty_ctx_t *)arg;

	if ((*(ctx->base + isr) & ((1 << 5) | (1 << 3))) != 0) {
		/* Clear overrun error bit */
		*(ctx->base + icr) |= (1 << 3);

		lf_fifo_push(&ctx->data.irq.rxFifo, *(ctx->base + rdr));
		ctx->data.irq.rxready = 1;
	}

	if (tty_txready(ctx) != 0) {
		*(ctx->base + cr1) &= ~(1 << 7);
	}

	return 1;
}


/* Nonnegative type must be one of dma_ht or dm_tc,
   negative type means idle line detected in which case it's negated number of bytes read. */
static void tty_dmaCallback(void *arg, int type)
{
	tty_ctx_t *ctx = (tty_ctx_t *)arg;
	unsigned long currPos = ctx->data.dma.rxbufsz - libdma_leftToRx(ctx->data.dma.per);
	unsigned long readPos = ctx->data.dma.readPos;

	(void)type;

	while (currPos != readPos) {
		if (lf_fifo_push(&ctx->data.dma.rxFifo, ctx->data.dma.rxbuf[readPos]) == 0) {
			ctx->data.dma.debug.droppedBytes++;
		}
		readPos = (readPos + 1) & ctx->data.dma.rxbufszMask;
	}

	ctx->data.dma.readPos = currPos;
}


static int tty_irqHandlerDMA(unsigned int n, void *arg)
{
	tty_ctx_t *ctx = (tty_ctx_t *)arg;

	/* Check for the idle line. */
	if ((*(ctx->base + isr) & ((1 << 4))) == 0) {
		return -1;
	}
	/* Clear idle line bit */
	*(ctx->base + icr) |= (1 << 4);

	if (libdma_leftToRx(ctx->data.dma.per) != 0) {
		tty_dmaCallback(ctx, -1);
	}

	return 1;
}


static void tty_dmaHandleRx(tty_ctx_t *ctx)
{
	uint8_t c;
	int wake = 0, wakeHelper;

	if (lf_fifo_pop(&ctx->data.dma.rxFifo, &c) == 0) {
		return;
	}

	libtty_putchar_lock(&ctx->ttyCommon);
	do {
		libtty_putchar_unlocked(&ctx->ttyCommon, c, &wakeHelper);
		wake |= wakeHelper;
	} while (lf_fifo_pop(&ctx->data.dma.rxFifo, &c) != 0);
	libtty_putchar_unlock(&ctx->ttyCommon);

	if (wake != 0) {
		libtty_wake_reader(&ctx->ttyCommon);
	}
}


static void tty_dmaHandleTx(tty_ctx_t *ctx)
{
	unsigned int i;

	if (libtty_txready(&ctx->ttyCommon) != 0) {
		for (i = 0; (i < ctx->data.dma.txbufsz) && (libtty_txready(&ctx->ttyCommon) != 0); i++) {
			ctx->data.dma.txbuf[i] = libtty_popchar(&ctx->ttyCommon);
		}
		libtty_wake_writer(&ctx->ttyCommon);

		*(ctx->base + icr) |= (1 << 6);

		(void)libdma_txAsync(ctx->data.dma.per, ctx->data.dma.txbuf, i, &ctx->data.dma.txDoneFlag);
	}
}


static int tty_dmatxready(tty_ctx_t *ctx)
{
	return ((ctx->data.dma.txDoneFlag != 0) && (libtty_txready(&ctx->ttyCommon) != 0)) ? 1 : 0;
}


static int tty_dmarxready(tty_ctx_t *ctx)
{
	return (lf_fifo_empty(&ctx->data.dma.rxFifo) == 0) ? 1 : 0;
}


static int tty_uartenabled(tty_ctx_t *ctx)
{
	return (*(ctx->base + cr1) & 1) ? 1 : 0;
}


static void tty_dmathread(void *arg)
{
	tty_ctx_t *ctx = (tty_ctx_t *)arg;

	/* Start rx routine. */
	libdma_infiniteRxAsync(ctx->data.dma.per, ctx->data.dma.rxbuf, ctx->data.dma.rxbufsz, tty_dmaCallback, ctx);

	for (;;) {
		if (tty_dmarxready(ctx) == 0) {
			mutexLock(ctx->irqlock);
			while (((tty_dmarxready(ctx) == 0) && (tty_dmatxready(ctx) == 0)) || (tty_uartenabled(ctx) == 0)) {
				condWait(ctx->cond, ctx->irqlock, 0);
			}
			mutexUnlock(ctx->irqlock);
		}

		tty_dmaHandleRx(ctx);
		tty_dmaHandleTx(ctx);
	}
}


static void tty_irqthread(void *arg)
{
	tty_ctx_t *ctx = (tty_ctx_t *)arg;
	int keptidle = 0;
	uint8_t mask;

	for (;;) {
		uint8_t rxbyte;
		unsigned rxcount;

		mutexLock(ctx->irqlock);
		while (((ctx->data.irq.rxready == 0) && !((tty_txready(ctx) != 0) && ((libtty_txready(&ctx->ttyCommon) != 0) || (keptidle != 0)))) || (tty_uartenabled(ctx) == 0)) {
			condWait(ctx->cond, ctx->irqlock, 0);
		}
		mutexUnlock(ctx->irqlock);

		if ((ctx->ttyCommon.term.c_cflag & CSIZE) == CS7) {
			mask = 0x7f;
		}
		else if ((ctx->ttyCommon.term.c_cflag & CSIZE) == CS6) {
			mask = 0x3f;
		}
		else {
			mask = 0xff;
		}

		ctx->data.irq.rxready = 0;
		dataBarier();

		/* limiting byte count to 8 to avoid starving tx */
		for (rxcount = 8; (rxcount != 0) && (lf_fifo_pop(&ctx->data.irq.rxFifo, &rxbyte) != 0); rxcount--) {
			libtty_putchar(&ctx->ttyCommon, rxbyte & mask, NULL);
		}
		if (rxcount == 0) {
			/* aborted due to rxcount limit - setting rxready to skip next condWait */
			ctx->data.irq.rxready = 1;
		}


		if (libtty_txready(&ctx->ttyCommon) != 0) {
			if (tty_txready(ctx) != 0) {
				if (keptidle == 0) {
					keptidle = 1;
					keepidle(1);
				}

				/* TODO add small TX fifo that can be read directly from IRQ */
				*(ctx->base + tdr) = libtty_getchar(&ctx->ttyCommon, NULL);
				*(ctx->base + cr1) |= (1 << 7);
			}
		}
		else if (keptidle != 0) {
			keptidle = 0;
			keepidle(0);
		}
	}
}


static void tty_signalTxReady(void *ctx)
{
	condSignal(((tty_ctx_t *)ctx)->cond);
}


static int _tty_configure(tty_ctx_t *ctx, char bits, char parity, char enable)
{
	int err = EOK;
	unsigned int tcr1 = 0;
	unsigned int flags;
	char tbits = bits;

	ctx->enabled = 0;

	if (parity != tty_parnone) {
		tcr1 |= 1 << 10;
		tbits += 1; /* We need one extra for parity */
	}

	switch (tbits) {
		case 9:
			tcr1 &= ~(1 << 28);
			tcr1 |= (1 << 12);
			break;

		case 8:
			tcr1 &= ~((1 << 28) | (1 << 12));
			break;

		case 7:
			tcr1 &= ~(1 << 12);
			tcr1 |= (1 << 28);
			break;

		default:
			err = -1;
			break;
	}

	if (err == EOK) {
		*(ctx->base + cr1) &= ~1;
		dataBarier();
		*(ctx->base + cr1) = tcr1;

		if (parity == tty_parodd) {
			*(ctx->base + cr1) |= 1 << 9;
		}
		else {
			*(ctx->base + cr1) &= ~(1 << 9);
		}

		*(ctx->base + icr) = -1;
		(void)*(ctx->base + rdr);

		if (enable != 0) {
			flags = (1 << 5) | (1 << 3) | (1 << 2);
			if (ctx->type == tty_dma) {
				/* Idle line interrupt enable. */
				flags |= (1 << 4);
			}
			*(ctx->base + cr1) |= flags;

			dataBarier();
			*(ctx->base + cr1) |= 1;
			ctx->enabled = 1;
		}

		dataBarier();
	}

	return err;
}


static void tty_setCflag(void *uart, tcflag_t *cflag)
{
	tty_ctx_t *ctx = (tty_ctx_t *)uart;
	char bits, parity = tty_parnone;

	if ((*cflag & CSIZE) == CS6) {
		bits = 6;
	}
	else if ((*cflag & CSIZE) == CS7) {
		bits = 7;
	}
	else {
		bits = 8;
	}

	if ((*cflag & PARENB) != 0) {
		if ((*cflag & PARODD) != 0) {
			parity = tty_parodd;
		}
		else {
			parity = tty_pareven;
		}
	}

	if ((bits != ctx->bits) || (parity != ctx->parity)) {
		_tty_configure(ctx, bits, parity, 1);
		condSignal(ctx->cond);
	}

	ctx->bits = bits;
	ctx->parity = parity;
}


static void tty_setBaudrate(void *uart, speed_t baud)
{
	tty_ctx_t *ctx = (tty_ctx_t *)uart;
	int baudr = libtty_baudrate_to_int(baud);
	int flags;

	if (ctx->baud != baudr) {
		*(ctx->base + cr1) &= ~1;
		dataBarier();

		*(ctx->base + brr) = getCpufreq() / baudr;

		*(ctx->base + icr) = -1;
		(void)*(ctx->base + rdr);

		flags = (1 << 5) | (1 << 3) | (1 << 2);
		if (ctx->type == tty_dma) {
			/* Idle line interrupt enable. */
			flags |= (1 << 4);
		}
		*(ctx->base + cr1) |= flags;
		dataBarier();
		*(ctx->base + cr1) |= 1;
		ctx->enabled = 1;
		condSignal(ctx->cond);
	}

	ctx->baud = baudr;
}


static tty_ctx_t *tty_getCtx(id_t id)
{
	tty_ctx_t *ctx = NULL;

	if (id == 0) {
		id = usart1 + UART_CONSOLE;
	}

	id -= 1;

	if ((id >= usart1) && (id <= uart5)) {
		ctx = &uart_common.ctx[ttySetup[id - usart1].pos];
	}

	return ctx;
}


static void tty_thread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;
	tty_ctx_t *ctx;
	unsigned long request;
	const void *in_data, *out_data = NULL;
	pid_t pid;
	int err;
	id_t id;

	for (;;) {
		while (msgRecv(uart_common.port, &msg, &rid) < 0) {
			;
		}

		priority(msg.priority);

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				ctx = tty_getCtx(msg.i.io.oid.id);
				if (ctx == NULL) {
					msg.o.io.err = -EINVAL;
					break;
				}

				msg.o.io.err = EOK;
				break;

			case mtWrite:
				ctx = tty_getCtx(msg.i.io.oid.id);
				if (ctx == NULL) {
					msg.o.io.err = -EINVAL;
					break;
				}
				msg.o.io.err = libtty_write(&ctx->ttyCommon, msg.i.data, msg.i.size, msg.i.io.mode);
				break;

			case mtRead:
				ctx = tty_getCtx(msg.i.io.oid.id);
				if (ctx == NULL) {
					msg.o.io.err = -EINVAL;
					break;
				}
				msg.o.io.err = libtty_read(&ctx->ttyCommon, msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtGetAttr:
				ctx = tty_getCtx(msg.i.attr.oid.id);
				if ((msg.i.attr.type != atPollStatus) || (ctx == NULL)) {
					msg.o.attr.err = -EINVAL;
					break;
				}
				msg.o.attr.val = libtty_poll_status(&ctx->ttyCommon);
				msg.o.attr.err = EOK;
				break;

			case mtDevCtl:
				in_data = ioctl_unpack(&msg, &request, &id);
				ctx = tty_getCtx(id);
				if (ctx == NULL) {
					err = -EINVAL;
				}
				else {
					pid = ioctl_getSenderPid(&msg);
					err = libtty_ioctl(&ctx->ttyCommon, pid, request, in_data, &out_data);
				}
				ioctl_setResponse(&msg, request, err, out_data);
				break;
		}

		msgRespond(uart_common.port, &msg, rid);

		priority(THREAD_PRIO);
	}
}


ssize_t tty_log(const char *str, size_t len)
{
	return libtty_write(&tty_getCtx(0)->ttyCommon, str, len, 0);
}


void tty_createDev(void)
{
	oid_t oid;

	oid.port = uart_common.port;
	oid.id = 0;
	create_dev(&oid, _PATH_TTY);
	create_dev(&oid, _PATH_CONSOLE);
}
#endif


int tty_init(void)
{
#if TTY_CNT != 0
	unsigned int tty, i;
	char fname[] = "uartx";
	speed_t baudrate = B115200;
	oid_t oid;
	libtty_callbacks_t callbacks;
	tty_ctx_t *ctx;
	int err;
	static const struct {
		volatile uint32_t *base;
		int dev;
		unsigned irq;
	} info[] = {
		{ (void *)0x40013800, pctl_usart1, usart1_irq },
		{ (void *)0x40004400, pctl_usart2, usart2_irq },
		{ (void *)0x40004800, pctl_usart3, usart3_irq },
		{ (void *)0x40004c00, pctl_uart4, uart4_irq },
		{ (void *)0x40005000, pctl_uart5, uart5_irq },
	};

	portCreate(&uart_common.port);
	oid.port = uart_common.port;

	for (tty = 0; tty <= uart5 - usart1; ++tty) {
		if (ttySetup[tty].enabled == 0) {
			continue;
		}

		ctx = &uart_common.ctx[ttySetup[tty].pos];

		devClk(info[tty].dev, 1);

		callbacks.arg = ctx;
		callbacks.set_baudrate = tty_setBaudrate;
		callbacks.set_cflag = tty_setCflag;
		callbacks.signal_txready = tty_signalTxReady;

		if (libtty_init(&ctx->ttyCommon, &callbacks, ttySetup[tty].libttyBufSize, baudrate) < 0) {
			return -1;
		}

		mutexCreate(&ctx->irqlock);
		condCreate(&ctx->cond);

		ctx->base = info[tty].base;
		ctx->bits = -1;
		ctx->parity = -1;
		ctx->baud = -1;

		if (ttySetup[tty].dma == 0) {
			lf_fifo_init(&ctx->data.irq.rxFifo, ctx->data.irq.rxFifoBuffer, sizeof(ctx->data.irq.rxFifoBuffer));
			ctx->data.irq.rxready = 0;

			ctx->type = tty_irq;
		}
		else {
			uint8_t *rxFifoBuf;
			libdma_init();
			err = libdma_acquirePeripheral(dma_uart, tty, &ctx->data.dma.per);
			if (err < 0) {
				return err;
			}
			/* Configure dma for tx and rx, medium priority, transfer size 8bits, increment memory address by 1 after each transfer. */
			libdma_configurePeripheral(ctx->data.dma.per, dma_mem2per, 0x1, (void *)(ctx->base + tdr), 0x0, 0x0, 0x1, 0x0, &ctx->cond);
			libdma_configurePeripheral(ctx->data.dma.per, dma_per2mem, 0x1, (void *)(ctx->base + rdr), 0x0, 0x0, 0x1, 0x0, &ctx->cond);
			*(ctx->base + cr3) |= (1 << 7) | (1 << 6); /* Enable DMA for transmission and reception. */

			ctx->data.dma.txDoneFlag = 1;
			ctx->data.dma.readPos = 0;

			ctx->data.dma.debug.droppedBytes = 0;

			ctx->data.dma.rxbufsz = ttySetup[tty].dmaBufSizeRx;
			ctx->data.dma.rxbufszMask = ctx->data.dma.rxbufsz - 1;
			ctx->data.dma.rxbuf = malloc(ctx->data.dma.rxbufsz);
			if (ctx->data.dma.rxbuf == NULL) {
				return -ENOMEM;
			}
			ctx->data.dma.txbufsz = ttySetup[tty].dmaBufSizeTx;
			ctx->data.dma.txbuf = malloc(ctx->data.dma.txbufsz);
			if (ctx->data.dma.txbuf == NULL) {
				free(ctx->data.dma.rxbuf);
				return -ENOMEM;
			}
			rxFifoBuf = malloc(ttySetup[tty].dmaRxFifoSize);
			if (rxFifoBuf == NULL) {
				free(ctx->data.dma.rxbuf);
				free(ctx->data.dma.txbuf);
				return -ENOMEM;
			}
			lf_fifo_init(&ctx->data.dma.rxFifo, rxFifoBuf, ttySetup[tty].dmaRxFifoSize);

			ctx->type = tty_dma;
		}

		/* Set up UART to 9600,8,n,1 16-bit oversampling */
		_tty_configure(ctx, 8, tty_parnone, 1);
		tty_setBaudrate(ctx, baudrate);

		if (ttySetup[tty].dma == 0) {
			interrupt(info[tty].irq, tty_irqHandler, (void *)ctx, ctx->cond, NULL);
			beginthread(tty_irqthread, 1, ctx->stack, sizeof(ctx->stack), (void *)ctx);
		}
		else {
			interrupt(info[tty].irq, tty_irqHandlerDMA, (void *)ctx, ctx->cond, NULL);
			beginthread(tty_dmathread, 1, ctx->stack, sizeof(ctx->stack), (void *)ctx);
		}

		ctx->enabled = 1;

		fname[sizeof(fname) - 2] = '0' + tty;
		oid.id = tty + 1;
		create_dev(&oid, fname);
	}

	for (i = 0; i < THREAD_POOL; ++i) {
		beginthread(tty_thread, THREAD_PRIO, uart_common.poolstack[i], sizeof(uart_common.poolstack[i]), (void *)i);
	}

	return EOK;
#else
	return 0;
#endif
}
