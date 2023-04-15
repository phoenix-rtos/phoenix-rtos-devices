/*
 * Phoenix-RTOS
 *
 * nRF91 TTY driver
 *
 * Copyright 2017, 2018, 2020, 2022 Phoenix Systems
 * Author: Jan Sikorski, Aleksander Kaminski, Andrzej Glowinski, Damian Loewnau
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
#include <libtty.h>

#include "nrf91-multi.h"
#include <board_config.h>
// #include "config.h"
#include "common.h"
#include "tty.h"

#define TTY0_POS 0
#define TTY1_POS (TTY0_POS + TTY0)
#define TTY2_POS (TTY1_POS + TTY1)
#define TTY3_POS (TTY2_POS + TTY2)

#define TTY_CNT (TTY0 + TTY1 + TTY2 + TTY3)

#define THREAD_POOL    3
#define THREAD_STACKSZ 768
#define THREAD_PRIO    1

typedef struct {
	char stack[512] __attribute__((aligned(8)));

	volatile unsigned int *base;
	volatile int enabled;

	volatile char *tx_dma;
	volatile char *rx_dma;

	int bits;
	int parity;
	int baud;
	unsigned int cnt;
	unsigned char init;

	handle_t cond;
	handle_t inth;
	handle_t irqlock;

	libtty_common_t tty_common;

	volatile unsigned char rxbuff;
	volatile int rxready;
} tty_ctx_t;


static struct {
	unsigned char poolstack[THREAD_POOL][THREAD_STACKSZ] __attribute__((aligned(8)));
	tty_ctx_t ctx[TTY_CNT];

	unsigned int port;
} uart_common;

static unsigned int common_port;


static const int uartConfig[] = { TTY0, TTY1, TTY2, TTY3 };


static const int uartPos[] = { TTY0_POS, TTY1_POS, TTY2_POS, TTY3_POS };


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


enum { baud_9600 = 0x00275000,
	baud_115200 = 0x01D60000 };


enum { uart0 = 0,
	uart1,
	uart2,
	uart3 };


static int tty_txready(tty_ctx_t *ctx)
{
	int ret;
	ret = ctx->init;

	if (!ret) {
		ret = *(ctx->base + uarte_events_txdrdy);
		*(ctx->base + uarte_events_txdrdy) = 0u;
		ctx->init = 0;
	}

	return ret;
}


static int tty_irqHandler(unsigned int n, void *arg)
{
	tty_ctx_t *ctx = (tty_ctx_t *)arg;

	if (*(ctx->base + uarte_events_rxdrdy)) {
		/* clear rxdrdy event flag */
		*(ctx->base + uarte_events_rxdrdy) = 0u;

		ctx->rxbuff = ctx->rx_dma[ctx->cnt];
		ctx->cnt++;
		ctx->rxready = 1;
	}

	if (*(ctx->base + uarte_events_endtx)) {
		/* disable endtx interrupt and clear flag */
		*(ctx->base + uarte_events_endtx) = 0u;
		*(ctx->base + uarte_intenclr) = 0x100;
	}

	if (*(ctx->base + uarte_events_endrx)) {
		/* clear endrx event flag */
		*(ctx->base + uarte_events_endrx) = 0u;
		ctx->cnt = 0;
		*(ctx->base + uarte_startrx) = 1u;
	}

	return 1;
}


static void tty_irqthread(void *arg)
{
	tty_ctx_t *ctx = (tty_ctx_t *)arg;
	int keptidle = 0;

	/* TODO add small TX and RX buffers that can be directly read / written in irq handlers */

	while (1) {
		mutexLock(ctx->irqlock);
		// there was no tty_txready!!!
		while ((!ctx->rxready && !(libtty_txready(&ctx->tty_common) || keptidle)) || !(*(ctx->base + uarte_enable) & 0x08))
			condWait(ctx->cond, ctx->irqlock, 0);
		mutexUnlock(ctx->irqlock);

		if (ctx->rxready) {
			libtty_putchar(&ctx->tty_common, ctx->rxbuff, NULL);
			ctx->rxready = 0;
		}

		if (libtty_txready(&ctx->tty_common)) {
			if (tty_txready(ctx)) {
				if (!keptidle) {
					keptidle = 1;
					keepidle(1);
				}
				ctx->tx_dma[0] = libtty_getchar(&ctx->tty_common, NULL);
				/* enable endtx interrupt and start transmission */
				*(ctx->base + uarte_intenset) = 0x100;
				*(ctx->base + uarte_starttx) = 1u;
			}
		}
		else if (keptidle) {
			keptidle = 0;
			keepidle(0);
		}
	}
}


static void tty_signalTxReady(void *ctx)
{
	condSignal(((tty_ctx_t *)ctx)->cond);
}


static void _tty_clearUartEvents(tty_ctx_t *ctx)
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


static void _tty_configure(tty_ctx_t *ctx, unsigned char parity, char enable)
{
	/* Disable uart instance before initialization */
	ctx->enabled = 0;
	*(ctx->base + uarte_enable) = 0u;
	dataBarier();

	/* Reset config:
	   Default settings - hardware flow control disabled, exclude parity bit, one stop bit */
	*(ctx->base + uarte_config) = 0u;
	dataBarier();

	if (parity) {
		/* Include even parity bit */
		*(ctx->base + uarte_config) = (0x7 << 1);
	}
	/* TODO: add pins configuartion and selecting them, now it's done in plo
	I have to ask about it coz can't find gpio configuration in tty/uart/spi drivers for stm */
	// uart_configPins(minor);
	/* Select pins */
	// *(ctx->base + uarte_psel_txd) = uartInfo[minor].txpin;
	// *(ctx->base + uarte_psel_rxd) = uartInfo[minor].rxpin;
	// *(ctx->base + uarte_psel_rts) = uartInfo[minor].rtspin;
	// *(ctx->base + uarte_psel_cts) = uartInfo[minor].ctspin;

	/* Set default max number of bytes in specific buffers */
	*(ctx->base + uarte_txd_maxcnt) = 1;
	*(ctx->base + uarte_rxd_maxcnt) = UART_RX_DMA_SIZE;

	/* Set default memory regions for uart dma */
	*(ctx->base + uarte_txd_ptr) = (unsigned int)ctx->tx_dma;
	*(ctx->base + uarte_rxd_ptr) = (unsigned int)ctx->rx_dma;

	/* clear all event flags */
	_tty_clearUartEvents(ctx);

	/* Disable all uart interrupts */
	*(ctx->base + uarte_intenclr) = 0xFFFFFFFF;
	/* Enable rxdrdy and endrx interruts */
	*(ctx->base + uarte_intenset) = 0x14;
	dataBarier();

	/* Enable uarte instance */
	*(ctx->base + uarte_enable) = 0x8;
	dataBarier();
	ctx->cnt = 0;
	*(ctx->base + uarte_startrx) = 1u;
	ctx->enabled = 1;
	dataBarier();
}


static void tty_setCflag(void *uart, tcflag_t *cflag)
{
	tty_ctx_t *ctx = (tty_ctx_t *)uart;
	unsigned char bits, parity = 0;

	if ((*cflag & CSIZE) == CS6)
		bits = 6;
	else if ((*cflag & CSIZE) == CS7)
		bits = 7;
	else
		bits = 8;

	if (*cflag & PARENB) {
		/* There is no possibility to set even/odd parity in nrf uart module */
		parity = 1;
	}

	if (parity != ctx->parity) {
		_tty_configure(ctx, parity, 1);
		condSignal(ctx->cond);
	}

	ctx->bits = bits;
	ctx->parity = parity;
}


static void tty_setBaudrate(void *uart, speed_t baud)
{
	tty_ctx_t *ctx = (tty_ctx_t *)uart;
	int baudr = libtty_baudrate_to_int(baud);

	if (ctx->baud != baudr) {
		switch (baudr) {
			case 9600:
				*(ctx->base + uarte_baudrate) = baud_9600;
				break;
			case 115200:
			default:
				*(ctx->base + uarte_baudrate) = baud_115200;
		}
		dataBarier();

		ctx->enabled = 1;
		condSignal(ctx->cond);
	}

	ctx->baud = baudr;
}


static tty_ctx_t *tty_getCtx(id_t id)
{
	tty_ctx_t *ctx = NULL;

	//why console even if 0 is set?? probably to not need to remember what is consoel
	if (!id) {
		id = uart0 + UART_CONSOLE;
	}

	if (id >= uart0 && id <= uart3) {
		ctx = &uart_common.ctx[uartPos[id]];
	}

	return ctx;
}


static void tty_thread(void *arg)
{
	msg_t msg;
	unsigned long rid;
	tty_ctx_t *ctx;
	unsigned long request;
	const void *in_data, *out_data = NULL;
	pid_t pid;
	int err;
	id_t id;

	while (1) {
		while (msgRecv(common_port, &msg, &rid) < 0)
			;

		priority(msg.priority);

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				if ((ctx = tty_getCtx(msg.i.io.oid.id)) == NULL) {
					msg.o.io.err = -EINVAL;
					break;
				}

				msg.o.io.err = EOK;
				break;

			case mtWrite:
				if ((ctx = tty_getCtx(msg.i.io.oid.id)) == NULL) {
					msg.o.io.err = -EINVAL;
					break;
				}
				msg.o.io.err = libtty_write(&ctx->tty_common, msg.i.data, msg.i.size, msg.i.io.mode);
				break;

			case mtRead:
				if ((ctx = tty_getCtx(msg.i.io.oid.id)) == NULL) {
					msg.o.io.err = -EINVAL;
					break;
				}
				msg.o.io.err = libtty_read(&ctx->tty_common, msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtGetAttr:
				if ((msg.i.attr.type != atPollStatus) || ((ctx = tty_getCtx(msg.i.attr.oid.id)) == NULL)) {
					msg.o.attr.err = -EINVAL;
					break;
				}
				msg.o.attr.val = libtty_poll_status(&ctx->tty_common);
				msg.o.attr.err = EOK;
				break;

			case mtDevCtl:
				in_data = ioctl_unpack(&msg, &request, &id);
				if ((ctx = tty_getCtx(id)) == NULL) {
					err = -EINVAL;
				}
				else {
					pid = ioctl_getSenderPid(&msg);
					err = libtty_ioctl(&ctx->tty_common, pid, request, in_data, &out_data);
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
	// for 0 changes it to console, 
	return libtty_write(&tty_getCtx(0)->tty_common, str, len, 0);
}


// looks good
void tty_createDev(void)
{
#if CONSOLE_IS_TTY
	oid_t oid;

	oid.port = uart_common.port;
	oid.id = 0;
	// create_dev(&oid, "tty");
	create_dev(&oid, _PATH_TTY);
	create_dev(&oid, _PATH_CONSOLE);
#endif
}


// looks good
int tty_init(void)
{
	unsigned int uart, i;
	char fname[] = "uartx";
	speed_t baudrate = B115200;
	oid_t oid;
	libtty_callbacks_t callbacks;
	tty_ctx_t *ctx;

	/* TODO: add pins! */
	/* Supported configuartions - uart0/uart2 + uart1/uart3
	   sizes of uart dma memory regions are set to max value of txd_maxcnt/rxd_maxcnt register (8191)
	   uart0 pins - default uart instance for nrf9160 dk, connected to VCOM0
	   uart1 pins -second uart interface on nrf9160 dk called nRF91_UART_2 on the board's schematic
	   uart0 dma - ram7: section 2 and 3
	   uart1 dma - ram7: section 0 and 1 */

	static const struct {
		volatile uint32_t *base;
		unsigned int irq;
		volatile char *tx_dma;
		volatile char *rx_dma;
	} info[] = {			/* same tx on uart1 and rx on uart0!!! */
		{ (void *)0x50008000, uarte0_irq + 16, (volatile char *)0x2003C000, (volatile char *)0x20038000 },
		{ (void *)0x50009000, uarte1_irq + 16, (volatile char *)0x20038000, (volatile char *)0x2003A000 },
		{ (void *)0x5000A000, uarte2_irq + 16, (volatile char *)0x2003C000, (volatile char *)0x20038000 },
		{ (void *)0x5000B000, uarte3_irq + 16, (volatile char *)0x20038000, (volatile char *)0x2003A000 }
	};

	portCreate(&uart_common.port);
	oid.port = uart_common.port;

	//     oid.id = 0;
	//    create_dev(&oid, "tty");

	for (uart = uart0; uart <= uart3; uart++) {
		if (!uartConfig[uart])
			continue;


		ctx = &uart_common.ctx[uart];

		callbacks.arg = ctx;
		callbacks.set_baudrate = tty_setBaudrate;
		callbacks.set_cflag = tty_setCflag;
		callbacks.signal_txready = tty_signalTxReady;

		if (libtty_init(&ctx->tty_common, &callbacks, 512, baudrate) < 0) {
			return -1;
		}

		mutexCreate(&ctx->irqlock);
		condCreate(&ctx->cond);

		ctx->base = info[uart - uart0].base;
		ctx->tx_dma = info[uart - uart0].tx_dma;
		ctx->rx_dma = info[uart - uart0].rx_dma;
		ctx->rxready = 0;
		ctx->bits = -1;
		ctx->parity = -1;
		ctx->baud = -1;
		ctx->init = 1;

		/* Set up UART to 115200, with parity checking disabled */
		_tty_configure(ctx, 0, 1);
		/* TODO: add support for other br values */
		tty_setBaudrate(ctx, baudrate);

		/* #flashcards conditional arg in interrupt?? */
		interrupt(info[uart - uart0].irq, tty_irqHandler, (void *)ctx, ctx->cond, NULL);

		beginthread(tty_irqthread, 1, ctx->stack, sizeof(ctx->stack), (void *)ctx);

		ctx->enabled = 1;

		fname[sizeof(fname) - 2] = '0' + uart - uart0;
		//this +1 makes sense, because 0 is reserved by klog/uart console, so I thinl I will set it back
		oid.id = uart - uart0 + 1; //port.1, 
		create_dev(&oid, fname);
	}

	/* TODO: ask why we are starting 3 threads here */
	for (i = 0; i < THREAD_POOL; ++i)
		beginthread(tty_thread, THREAD_PRIO, uart_common.poolstack[i], sizeof(uart_common.poolstack[i]), (void *)i);

	return 0;
}
