/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4 UART driver
 *
 * Copyright 2017, 2018, 2020 Phoenix Systems
 * Author: Jan Sikorski, Aleksander Kaminski, Andrzej Glowinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <sys/file.h>

#include "libuart.h"
#include "../common.h"


enum { cr1 = 0, cr2, cr3, brr, gtpr, rtor, rqr, isr, icr, rdr, tdr };


enum { uart_parnone = 0, uart_pareven, uart_parodd };


static inline int libuart_txready(libuart_ctx *ctx)
{
	return *(ctx->base + isr) & (1 << 7);
}


static int libuart_irqHandler(unsigned int n, void *arg)
{
	libuart_ctx *ctx = (libuart_ctx *)arg;

	if (*(ctx->base + isr) & ((1 << 5) | (1 << 3))) {
		/* Clear overrun error bit */
		*(ctx->base + icr) |= (1 << 3);

		ctx->rxbuff = *(ctx->base + rdr);
		ctx->rxready = 1;
	}

	if (libuart_txready(ctx))
		*(ctx->base + cr1) &= ~(1 << 7);

	return 1;
}


static void libuart_irqthread(void *arg)
{
	libuart_ctx *ctx = (libuart_ctx *)arg;

	while (1) {
		mutexLock(ctx->irqlock);
		while ((!ctx->rxready && !(libuart_txready(ctx) && libtty_txready(&ctx->tty_common))) || !(*(ctx->base + cr1) & 1))
			condWait(ctx->cond, ctx->irqlock, 0);
		mutexUnlock(ctx->irqlock);

		if (ctx->rxready) {
			libtty_putchar(&ctx->tty_common, ctx->rxbuff, NULL);
			ctx->rxready = 0;
		}

		if (libuart_txready(ctx) && libtty_txready(&ctx->tty_common)) {
			*(ctx->base + tdr) = libtty_getchar(&ctx->tty_common, NULL);
			*(ctx->base + cr1) |= (1 << 7);
		}
	}
}


static void libuart_signalTxReady(void *ctx)
{
	condSignal(((libuart_ctx *)ctx)->cond);
}


static int _libuart_configure(libuart_ctx *ctx, char bits, char parity, char enable)
{
	int err = EOK;
	unsigned int tcr1 = 0;
	char tbits = bits;

	ctx->enabled = 0;

	if (parity != uart_parnone) {
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

		if (parity == uart_parodd)
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


static void libuart_setCflag(void *uart, tcflag_t *cflag)
{
	libuart_ctx *ctx = (libuart_ctx *)uart;
	char bits, parity = uart_parnone;

	if ((*cflag & CSIZE) == CS6)
		bits = 6;
	else if ((*cflag & CSIZE) == CS7)
		bits = 7;
	else
		bits = 8;

	if (*cflag & PARENB) {
		if (*cflag & PARODD)
			parity = uart_parodd;
		else
			parity = uart_pareven;
	}

	if (bits != ctx->bits || parity != ctx->parity) {
		_libuart_configure(ctx, bits, parity, 1);
		condSignal(ctx->cond);
	}

	ctx->bits = bits;
	ctx->parity = parity;
}


static void libuart_setBaudrate(void *uart, speed_t baud)
{
	libuart_ctx *ctx = (libuart_ctx *)uart;
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


ssize_t libuart_write(libuart_ctx *ctx, const void *buff, size_t bufflen, unsigned int mode)
{
	return libtty_write(&ctx->tty_common, buff, bufflen, mode);
}


ssize_t libuart_read(libuart_ctx *ctx, void *buff, size_t bufflen, unsigned int mode)
{
	return libtty_read(&ctx->tty_common, buff, bufflen, mode);
}


int libuart_getAttr(libuart_ctx *ctx, int type)
{
	int ret;

	if (type != atPollStatus) {
		ret = -EINVAL;
	}
	else {
		ret = libtty_poll_status(&ctx->tty_common);
	}

	return ret;
}


int libuart_devCtl(libuart_ctx *ctx, pid_t pid, unsigned int request, const void* inData, const void** outData)
{
	int ret;

	ret = libtty_ioctl(&ctx->tty_common, pid, (unsigned int)request, inData, outData);

	return ret;
}


int libuart_init(libuart_ctx *ctx, unsigned int uart)
{
	libtty_callbacks_t callbacks;
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

	uart -= usart1;

	if (uart >= (sizeof(info) / sizeof(info[0])))
		return -1;

	devClk(info[uart].dev, 1);

	callbacks.arg = ctx;
	callbacks.set_baudrate = libuart_setBaudrate;
	callbacks.set_cflag = libuart_setCflag;
	callbacks.signal_txready = libuart_signalTxReady;

	if (libtty_init(&ctx->tty_common, &callbacks, 64) < 0)
		return -1;

	mutexCreate(&ctx->irqlock);
	condCreate(&ctx->cond);

	ctx->base = info[uart].base;
	ctx->rxready = 0;
	ctx->bits = -1;
	ctx->parity = -1;
	ctx->baud = -1;

	/* Set up UART to 9600,8,n,1 16-bit oversampling */
	_libuart_configure(ctx, 8, uart_parnone, 1);
	libuart_setBaudrate(ctx, B9600);

	interrupt(info[uart].irq, libuart_irqHandler, (void *)ctx, ctx->cond, NULL);

	beginthread(libuart_irqthread, 1, ctx->stack, sizeof(ctx->stack), (void *)ctx);

	ctx->enabled = 1;

	return 0;
}
