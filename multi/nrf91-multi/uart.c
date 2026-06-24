/*
 * Phoenix-RTOS
 *
 * nRF91 UART driver
 *
 * Copyright 2023, 2024 Phoenix Systems
 * Author: Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <paths.h>
#include <posix/utils.h>
#include <stdatomic.h>
#include <sys/msg.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <sys/types.h>
#include <libtty.h>
#include <libtty-lf-fifo.h>

#include <board_config.h>

#include "common.h"
#include "uart.h"

#define UART_CNT (UART0 + UART1 + UART2 + UART3)

#define THREADS_CNT     3
#define THREAD_STACKSZ  768
#define THREAD_PRIORITY 1

typedef struct {
	char stack[512] __attribute__((aligned(8)));

	volatile unsigned int *base;
	volatile char *txDma;
	volatile char *rxDma;

	unsigned char stopBits;
	int parity;
	int baud;
	int cnt;
	bool init;
	bool resetCnt;

	handle_t cond;
	handle_t irqlock;

	libtty_common_t ttyCommon;

	volatile bool rxdrdy;
	atomic_int rxdCount;
} uart_ctx_t;


static struct {
	unsigned char stack[THREADS_CNT][THREAD_STACKSZ] __attribute__((aligned(8)));
	uart_ctx_t ctx[UART_CNT];

	unsigned int port;
} uart_common;


enum { uarte_startrx = 0,
	uarte_stoprx,
	uarte_starttx,
	uarte_stoptx,
	uarte_flushrx = 11,
	uarte_events_cts = 64,
	uarte_events_ncts,
	uarte_events_rxdrdy,
	uarte_events_endrx = 68,
	uarte_events_txdrdy = 71,
	uarte_events_endtx,
	uarte_events_error,
	uarte_events_rxto = 81,
	uarte_events_rxstarted = 83,
	uarte_events_txstarted,
	uarte_events_txstopped = 86,
	uarte_inten = 192,
	uarte_intenset,
	uarte_intenclr,
	uarte_errorsrc = 288,
	uarte_enable = 320,
	uarte_psel_rts = 322,
	uarte_psel_txd,
	uarte_psel_cts,
	uarte_psel_rxd,
	uarte_baudrate = 329,
	uarte_rxd_ptr = 333,
	uarte_rxd_maxcnt,
	uarte_rxd_amount,
	uarte_txd_ptr = 337,
	uarte_txd_maxcnt,
	uarte_txd_amount,
	uarte_config = 347 };


enum { no_parity,
	parity_even
};


enum { baud_9600 = 0x00275000,
	baud_115200 = 0x01d60000,
	baud_230400 = 0x03b00000,
	baud_250000 = 0x04000000,
	baud_460800 = 0x07400000,
	baud_921600 = 0x0f000000,
	baud_1000000 = 0x10000000 };


enum { uart0 = 0,
	uart1,
	uart2,
	uart3 };


/* indicates whether the tx line is ready to transmit new data */
static bool uart_txready(uart_ctx_t *ctx)
{
	bool ret;
	ret = ctx->init;

	mutexLock(ctx->irqlock);
	/* before the first transaction tx is also ready even if the event hasn't occurred */
	if (ctx->init == false) {
		/* true means that the data has been sent from txd */
		ret = *(ctx->base + uarte_events_txdrdy);
		*(ctx->base + uarte_events_txdrdy) = 0u;
		ctx->init = false;
	}
	mutexUnlock(ctx->irqlock);

	return ret;
}


static int uart_irqHandler(unsigned int n, void *arg)
{
	uart_ctx_t *ctx = (uart_ctx_t *)arg;

	if (*(ctx->base + uarte_events_rxdrdy) != 0u) {
		/* clear rxdrdy event flag */
		*(ctx->base + uarte_events_rxdrdy) = 0u;
		atomic_fetch_add(&ctx->rxdCount, 1);
		ctx->rxdrdy = true;
	}

	if (*(ctx->base + uarte_events_endtx) != 0u) {
		/* disable endtx interrupt and clear flag */
		*(ctx->base + uarte_events_endtx) = 0u;
		*(ctx->base + uarte_intenclr) = 0x100u;
	}

	if (*(ctx->base + uarte_events_endrx) != 0u) {
		/* clear endrx event flag */
		*(ctx->base + uarte_events_endrx) = 0u;
		ctx->resetCnt = true;
		*(ctx->base + uarte_startrx) = 1u;
	}

	return 0;
}


static void uart_irqthread(void *arg)
{
	uart_ctx_t *ctx = (uart_ctx_t *)arg;
	bool keptIdle = false;
	int i = 0;

	while (true) {
		mutexLock(ctx->irqlock);
		/* wait until new data is received and libtty fifo is not empty */
		while (!ctx->rxdrdy && (libtty_txready(&ctx->ttyCommon) == 0)) {
			condWait(ctx->cond, ctx->irqlock, 0);
		}
		mutexUnlock(ctx->irqlock);

		ctx->rxdrdy = false;
		dataBarier();

		/* send the data directly from dma */
		for (; (ctx->cnt < atomic_load(&ctx->rxdCount)) && (ctx->cnt < UART_RX_DMA_SIZE); ctx->cnt++) {
			libtty_putchar(&ctx->ttyCommon, ctx->rxDma[ctx->cnt], NULL);
		}

		/* check whether all bytes from the previous transaction have been read */
		if (ctx->resetCnt && (ctx->cnt >= UART_RX_DMA_SIZE)) {
			ctx->cnt = 0;
			atomic_fetch_sub(&ctx->rxdCount, UART_RX_DMA_SIZE);
		}

		if (libtty_txready(&ctx->ttyCommon) != 0) {
			if (uart_txready(ctx)) {
				ctx->txDma[0] = libtty_getchar(&ctx->ttyCommon, NULL);
				/* enable endtx interrupt and start transmission */
				*(ctx->base + uarte_intenset) = 0x100;
				*(ctx->base + uarte_starttx) = 1u;
			}
		}
	}
}


static void uart_signalTxReady(void *ctx)
{
	condSignal(((uart_ctx_t *)ctx)->cond);
}


static void _uart_clearUartEvents(uart_ctx_t *ctx)
{
	*(ctx->base + uarte_events_cts) = 0u;
	*(ctx->base + uarte_events_ncts) = 0u;
	*(ctx->base + uarte_events_rxdrdy) = 0u;
	*(ctx->base + uarte_events_endrx) = 0u;

	*(ctx->base + uarte_events_txdrdy) = 0u;
	*(ctx->base + uarte_events_endtx) = 0u;
	*(ctx->base + uarte_events_error) = 0u;

	*(ctx->base + uarte_events_rxto) = 0u;
	*(ctx->base + uarte_events_rxstarted) = 0u;
	*(ctx->base + uarte_events_txstarted) = 0u;
	*(ctx->base + uarte_events_txstopped) = 0u;
}


static void _uart_configure(uart_ctx_t *ctx)
{
	/* Disable uart instance before initialization */
	*(ctx->base + uarte_enable) = 0u;
	dataBarier();

	/* Reset config:
	 * Default settings - hardware flow control disabled, exclude parity bit, one stop bit */
	*(ctx->base + uarte_config) = 0u;
	dataBarier();

	if (ctx->parity == parity_even) {
		/* Include even parity bit */
		*(ctx->base + uarte_config) = (0x7 << 1);
	}

	if (ctx->stopBits == 2) {
		*(ctx->base + uarte_config) |= (1u << 4);
	}
	/* TODO: add pins configuration and selecting them, now it's done in plo

	/* Set default max number of bytes in specific buffers */
	*(ctx->base + uarte_txd_maxcnt) = 1u;
	*(ctx->base + uarte_rxd_maxcnt) = UART_RX_DMA_SIZE;

	/* Set default memory regions for uart dma */
	*(ctx->base + uarte_txd_ptr) = (unsigned int)ctx->txDma;
	*(ctx->base + uarte_rxd_ptr) = (unsigned int)ctx->rxDma;

	/* clear all event flags */
	_uart_clearUartEvents(ctx);

	/* Disable all uart interrupts */
	*(ctx->base + uarte_intenclr) = 0xffffffffu;
	/* Enable rxdrdy and endrx interruts */
	*(ctx->base + uarte_intenset) = 0x14;
	dataBarier();

	/* Enable uarte instance */
	*(ctx->base + uarte_enable) = 0x8;
	dataBarier();
	ctx->cnt = 0;
	*(ctx->base + uarte_startrx) = 1u;
	dataBarier();
}


static void _uart_setCflag(uart_ctx_t *ctx, const tcflag_t *cflag)
{
	int parity = 0;
	int stopBits = 1;

	/* CS bits ignored - character size is 8 and can't be changed on this target */

	/* PARODD ignored - not possible to set odd parity in nrf uart module
	 * if parity is enabled it's always even
	 */
	if ((*cflag & PARENB) != 0u) {
		parity = parity_even;
	}

	if ((*cflag & CSTOPB) != 0u) {
		stopBits = 2;
	}

	if ((parity != ctx->parity) || (stopBits != ctx->stopBits)) {
		ctx->parity = parity;
		ctx->stopBits = stopBits;
		_uart_configure(ctx);
		condSignal(ctx->cond);
	}
}


static void _uart_setBaudrate(uart_ctx_t *ctx, speed_t baud)
{
	int baudr = libtty_baudrate_to_int(baud);

	if (ctx->baud != baudr) {
		switch (baudr) {
			case 9600:
				*(ctx->base + uarte_baudrate) = baud_9600;
				break;
			case 115200:
				*(ctx->base + uarte_baudrate) = baud_115200;
				break;
			case 23040:
				*(ctx->base + uarte_baudrate) = baud_230400;
				break;
			case 460800:
				*(ctx->base + uarte_baudrate) = baud_460800;
				break;
			case 921600:
				*(ctx->base + uarte_baudrate) = baud_921600;
				break;
			case 1000000:
				*(ctx->base + uarte_baudrate) = baud_1000000;
				break;
			default:
				*(ctx->base + uarte_baudrate) = baud_115200;
				break;
		}
		dataBarier();
	}

	ctx->baud = baudr;
}


static uart_ctx_t *uart_getCtx(id_t id)
{
	uart_ctx_t *ctx = NULL;

	if ((id >= uart0) && (id <= uart3)) {
		ctx = &uart_common.ctx[id];
	}

	return ctx;
}


static void uart_thread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;
	uart_ctx_t *ctx;
	unsigned long request;
	const void *in_data, *out_data = NULL;
	pid_t pid;
	int err;
	id_t id;

	while (1) {
		while (msgRecv(uart_common.port, &msg, &rid) < 0) {
		}

		priority(msg.priority);

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				ctx = uart_getCtx(msg.i.io.oid.id);
				if (ctx == NULL) {
					msg.o.io.err = -EINVAL;
					break;
				}
				msg.o.io.err = EOK;
				break;

			case mtWrite:
				ctx = uart_getCtx(msg.i.io.oid.id);
				if (ctx == NULL) {
					msg.o.io.err = -EINVAL;
					break;
				}
				msg.o.io.err = libtty_write(&ctx->ttyCommon, msg.i.data, msg.i.size, msg.i.io.mode);
				break;

			case mtRead:
				if ((ctx = uart_getCtx(msg.i.io.oid.id)) == NULL) {
					msg.o.io.err = -EINVAL;
					break;
				}
				msg.o.io.err = libtty_read(&ctx->ttyCommon, msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtGetAttr:
				if ((msg.i.attr.type != atPollStatus) || ((ctx = uart_getCtx(msg.i.attr.oid.id)) == NULL)) {
					msg.o.attr.err = -EINVAL;
					break;
				}
				msg.o.attr.val = libtty_poll_status(&ctx->ttyCommon);
				msg.o.attr.err = EOK;
				break;

			case mtDevCtl:
				in_data = ioctl_unpack(&msg, &request, &id);
				if ((ctx = uart_getCtx(id)) == NULL) {
					err = -EINVAL;
				}
				else {
					pid = ioctl_getSenderPid(&msg);
					err = libtty_ioctl(&ctx->ttyCommon, pid, request, in_data, &out_data);
				}
				ioctl_setResponse(&msg, request, err, out_data);
				break;
			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(uart_common.port, &msg, rid);

		priority(THREAD_PRIORITY);
	}
}


ssize_t uart_read(const char *str, size_t len, unsigned int mode)
{
	return libtty_read(&uart_getCtx(UART_CONSOLE)->ttyCommon, str, len, mode);
}


ssize_t uart_log(const char *str, size_t len, unsigned int mode)
{
	return libtty_write(&uart_getCtx(UART_CONSOLE)->ttyCommon, str, len, mode);
}


void uart_createConsoleDev(void)
{
	oid_t oid;

	oid.port = uart_common.port;
	/* id 0 is reserved for a console */
	oid.id = 0;
	create_dev(&oid, _PATH_CONSOLE);
}


int uart_init(void)
{
	const int uartConfig[] = { UART0, UART1, UART2, UART3 };
	unsigned int uart;
	char fname[] = "uartx";
	speed_t baudrate = UART_BAUDRATE_TERMIOS;
	oid_t oid;
	libtty_callbacks_t callbacks;
	uart_ctx_t *ctx;

	static const struct {
		volatile uint32_t *base;
		unsigned int irq;
		volatile char *txDma;
		volatile char *rxDma;
	} info[] = {
		{ (void *)0x50008000, uarte0_irq + 16, (volatile char *)0x2003C000, (volatile char *)0x20038000 },
		{ (void *)0x50009000, uarte1_irq + 16, (volatile char *)0x20038000, (volatile char *)0x2003A000 },
		{ (void *)0x5000A000, uarte2_irq + 16, (volatile char *)0x2003C000, (volatile char *)0x20038000 },
		{ (void *)0x5000B000, uarte3_irq + 16, (volatile char *)0x20038000, (volatile char *)0x2003A000 }
	};

	portCreate(&uart_common.port);
	oid.port = uart_common.port;

	for (uart = uart0; uart <= uart3; uart++) {
		if (uartConfig[uart] == 0) {
			continue;
		}

		ctx = &uart_common.ctx[uart];

		callbacks.arg = ctx;
		callbacks.set_baudrate = _uart_setBaudrate;
		callbacks.set_cflag = _uart_setCflag;
		callbacks.signal_txready = uart_signalTxReady;

		if (libtty_init(&ctx->ttyCommon, &callbacks, 512, baudrate) < 0) {
			return -1;
		}

		mutexCreate(&ctx->irqlock);
		condCreate(&ctx->cond);

		ctx->base = info[uart - uart0].base;
		ctx->txDma = info[uart - uart0].txDma;
		ctx->rxDma = info[uart - uart0].rxDma;
		ctx->rxdrdy = false;
		ctx->parity = no_parity;
		ctx->stopBits = 1;
		ctx->baud = -1;
		ctx->init = true;
		ctx->cnt = 0u;
		ctx->resetCnt = false;

		atomic_init(&ctx->rxdCount, 0);

		_uart_configure(ctx);
		_uart_setBaudrate(ctx, baudrate);

		interrupt(info[uart - uart0].irq, uart_irqHandler, (void *)ctx, ctx->cond, NULL);

		beginthread(uart_irqthread, 1, ctx->stack, sizeof(ctx->stack), (void *)ctx);

		fname[sizeof(fname) - 2] = '0' + uart - uart0;
		/* id 0 - console, 1 - uart0, 2 - uart1... */
		oid.id = uart - uart0 + 1;
		create_dev(&oid, fname);
	}

	for (int i = 0; i < THREADS_CNT; ++i) {
		beginthread(uart_thread, THREAD_PRIORITY, uart_common.stack[i], sizeof(uart_common.stack[i]), (void *)i);
	}

	return 0;
}
