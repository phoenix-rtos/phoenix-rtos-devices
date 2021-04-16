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
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <libtty.h>

#include "stm32-multi.h"
#include "config.h"
#include "common.h"
#include "gpio.h"
#include "tty.h"
#include "rcc.h"

#define TTY1_POS 0
#define TTY2_POS (TTY1_POS + TTY1)
#define TTY3_POS (TTY2_POS + TTY2)
#define TTY4_POS (TTY3_POS + TTY3)
#define TTY5_POS (TTY4_POS + TTY4)

#define TTY_CNT (TTY1 + TTY2 + TTY3 + TTY4 + TTY5)

#define THREAD_POOL 3
#define THREAD_STACKSZ 512
#define THREAD_PRIO 1

#if TTY_CNT != 0
typedef struct {
	char stack[256] __attribute__ ((aligned(8)));

	volatile unsigned int *base;
	volatile int enabled;

	int bits;
	int parity;
	int baud;

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


static const int uartConfig[] = { TTY1, TTY2, TTY3, TTY4, TTY5 };


static const int uartPos[] = { TTY1_POS, TTY2_POS, TTY3_POS, TTY4_POS, TTY5_POS };


enum { cr1 = 0, cr2, cr3, brr, gtpr, rtor, rqr, isr, icr, rdr, tdr };


enum { tty_parnone = 0, tty_pareven, tty_parodd };


static inline int tty_txready(tty_ctx_t *ctx)
{
	return *(ctx->base + isr) & (1 << 7);
}


static int tty_irqHandler(unsigned int n, void *arg)
{
	tty_ctx_t *ctx = (tty_ctx_t *)arg;

	if (*(ctx->base + isr) & ((1 << 5) | (1 << 3))) {
		/* Clear overrun error bit */
		*(ctx->base + icr) |= (1 << 3);

		ctx->rxbuff = *(ctx->base + rdr);
		ctx->rxready = 1;
	}

	if (tty_txready(ctx))
		*(ctx->base + cr1) &= ~(1 << 7);

	return 1;
}


static void tty_irqthread(void *arg)
{
	tty_ctx_t *ctx = (tty_ctx_t *)arg;
	int keptidle = 0;

	/* TODO add small TX and RX buffers that can be directly read / written in irq handlers */

	while (1) {
		mutexLock(ctx->irqlock);
		while ((!ctx->rxready && !(tty_txready(ctx) && (libtty_txready(&ctx->tty_common) || keptidle))) || !(*(ctx->base + cr1) & 1))
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
				*(ctx->base + tdr) = libtty_getchar(&ctx->tty_common, NULL);
				*(ctx->base + cr1) |= (1 << 7);
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


static int _tty_configure(tty_ctx_t *ctx, char bits, char parity, char enable)
{
	int err = EOK;
	unsigned int tcr1 = 0;
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

		if (parity == tty_parodd)
			*(ctx->base + cr1) |= 1 << 9;
		else
			*(ctx->base + cr1) &= ~(1 << 9);

		*(ctx->base + icr) = -1;
		(void)*(ctx->base + rdr);

		if (enable) {
			*(ctx->base + cr1) |= (1 << 5) | (1 << 3) | (1 << 2);
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

	if ((*cflag & CSIZE) == CS6)
		bits = 6;
	else if ((*cflag & CSIZE) == CS7)
		bits = 7;
	else
		bits = 8;

	if (*cflag & PARENB) {
		if (*cflag & PARODD)
			parity = tty_parodd;
		else
			parity = tty_pareven;
	}

	if (bits != ctx->bits || parity != ctx->parity) {
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

	if (ctx->baud != baudr) {
		*(ctx->base + cr1) &= ~1;
		dataBarier();

		*(ctx->base + brr) = getCpufreq() / baudr;

		*(ctx->base + icr) = -1;
		(void)*(ctx->base + rdr);

		*(ctx->base + cr1) |= (1 << 5) | (1 << 3) | (1 << 2);
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

	if (!id)
		id = usart1 + UART_CONSOLE;

	id -= 1;

	if (id >= usart1 && id <= uart5)
		ctx = &uart_common.ctx[uartPos[id - usart1]];

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
		while (msgRecv(uart_common.port, &msg, &rid) < 0)
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
			if ((ctx = tty_getCtx(msg.i.attr.oid.id)) == NULL) {
				msg.o.attr.val = -EINVAL;
				break;
			}

			if (msg.i.attr.type != atPollStatus)
				msg.o.attr.val = -EINVAL;
			else
				msg.o.attr.val = libtty_poll_status(&ctx->tty_common);

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
#endif


void tty_log(const char *str)
{
#if CONSOLE_IS_TTY
	libtty_write(&tty_getCtx(0)->tty_common, str, strlen(str), 0);
#endif
}


int tty_init(void)
{
#if TTY_CNT != 0
	unsigned int uart, i;
	char fname[] = "/dev/uartx";
	oid_t oid;
	libtty_callbacks_t callbacks;
	tty_ctx_t *ctx;
	const struct {
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

#if CONSOLE_IS_TTY
	oid.id = 0;
	portRegister(uart_common.port, "/dev/tty", &oid);
#endif

	for (uart = usart1; uart <= uart5; ++uart) {
		if (!uartConfig[uart])
			continue;

		ctx = &uart_common.ctx[uartPos[uart - usart1]];

		devClk(info[uart - usart1].dev, 1);

		callbacks.arg = ctx;
		callbacks.set_baudrate = tty_setBaudrate;
		callbacks.set_cflag = tty_setCflag;
		callbacks.signal_txready = tty_signalTxReady;

		if (libtty_init(&ctx->tty_common, &callbacks, 64) < 0)
			return -1;

		mutexCreate(&ctx->irqlock);
		condCreate(&ctx->cond);

		ctx->base = info[uart - usart1].base;
		ctx->rxready = 0;
		ctx->bits = -1;
		ctx->parity = -1;
		ctx->baud = -1;

		/* Set up UART to 9600,8,n,1 16-bit oversampling */
		_tty_configure(ctx, 8, tty_parnone, 1);
		tty_setBaudrate(ctx, B115200);

		interrupt(info[uart - usart1].irq, tty_irqHandler, (void *)ctx, ctx->cond, NULL);

		beginthread(tty_irqthread, 1, ctx->stack, sizeof(ctx->stack), (void *)ctx);

		ctx->enabled = 1;

		fname[sizeof(fname) - 2] = '0' + uart - usart1;
		oid.id = uart - usart1 + 1;
		portRegister(uart_common.port, fname, &oid);
	}

	for (i = 0; i < THREAD_POOL; ++i)
		beginthread(tty_thread, THREAD_PRIO, uart_common.poolstack[i], sizeof(uart_common.poolstack[i]), (void *)i);

	return EOK;
#else
	return 0;
#endif
}
