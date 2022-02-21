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

#include "libmulti/libuart.h"
#include "../common.h"


enum { cr1 = 0, cr2, cr3, brr, gtpr, rtor, rqr, isr, icr, rdr, tdr };


static int libuart_txirq(unsigned int n, void *arg)
{
	libuart_ctx *ctx = (libuart_ctx *)arg;
	int release = -1;

	if ((*(ctx->base + cr1) & (1 << 7)) && (*(ctx->base + isr) & (1 << 7))) {
		/* Txd buffer empty */
		if (ctx->txbeg != ctx->txend) {
			*(ctx->base + tdr) = *(ctx->txbeg++);
		}
		else {
			*(ctx->base + cr1) &= ~(1 << 7);
			ctx->txbeg = NULL;
			ctx->txend = NULL;
			release = 1;
		}
	}

	return release;
}


static int libuart_rxirq(unsigned int n, void *arg)
{
	libuart_ctx *ctx = (libuart_ctx *)arg;
	int release = -1;

	/* Clear wakeup from stop mode flag */
	if (n == lpuart1_irq)
		*(ctx->base + icr) |= 1 << 20;

	if (*(ctx->base + isr) & ((1 << 5) | (1 << 3))) {
		/* Clear overrun error bit */
		*(ctx->base + icr) |= (1 << 3);

		/* Rxd buffer not empty */
		ctx->rxdfifo[ctx->rxdw++] = *(ctx->base + rdr);
		ctx->rxdw %= sizeof(ctx->rxdfifo);

		if (ctx->rxdr == ctx->rxdw)
			ctx->rxdr = (ctx->rxdr + 1) % sizeof(ctx->rxdfifo);
	}

	if (ctx->rxbeg != NULL) {
		while (ctx->rxdr != ctx->rxdw && ctx->rxbeg != ctx->rxend) {
			*(ctx->rxbeg++) = ctx->rxdfifo[ctx->rxdr++];
			ctx->rxdr %= sizeof(ctx->rxdfifo);
			(*ctx->read)++;
		}

		if (ctx->rxbeg == ctx->rxend) {
			ctx->rxbeg = NULL;
			ctx->rxend = NULL;
			ctx->read = NULL;
		}
		release = 1;
	}

	return release;
}


int libuart_configure(libuart_ctx *ctx, char bits, char parity, unsigned int baud, char enable)
{
	int err = EOK, baseClk = getCpufreq();
	unsigned int tcr1 = 0;
	char tbits = bits;

	ctx->enabled = 0;
	condBroadcast(ctx->rxcond);

	mutexLock(ctx->txlock);
	mutexLock(ctx->rxlock);
	mutexLock(ctx->lock);

	dataBarier();

	ctx->txbeg = NULL;
	ctx->txend = NULL;

	ctx->rxbeg = NULL;
	ctx->rxend = NULL;
	ctx->read = NULL;
	ctx->rxdr = 0;
	ctx->rxdw = 0;

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
		ctx->bits = bits;

		*(ctx->base + cr1) &= ~1;
		dataBarier();
		*(ctx->base + cr1) = tcr1;

		ctx->baud = baud;
		*(ctx->base + brr) = baseClk / baud;

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

	mutexUnlock(ctx->lock);
	mutexUnlock(ctx->rxlock);
	mutexUnlock(ctx->txlock);

	return err;
}


int libuart_write(libuart_ctx *ctx, const void* buff, unsigned int bufflen)
{
	if (!bufflen)
		return 0;

	if (!ctx->enabled)
		return -EIO;

	mutexLock(ctx->txlock);
	mutexLock(ctx->lock);

	keepidle(1);

	*(ctx->base + tdr) = *((unsigned char *)buff);
	ctx->txbeg = (void *)((unsigned char *)buff + 1);
	ctx->txend = (void *)((unsigned char *)buff + bufflen);
	*(ctx->base + cr1) |= 1 << 7;

	while (ctx->txbeg != ctx->txend)
		condWait(ctx->txcond, ctx->lock, 0);
	mutexUnlock(ctx->lock);

	keepidle(0);
	mutexUnlock(ctx->txlock);

	return bufflen;
}


int libuart_read(libuart_ctx *ctx, void* buff, unsigned int count, char mode, unsigned int timeout)
{
	int i, err;
	volatile unsigned int read = 0;
	char mask = 0x7f;

	if (!ctx->enabled)
		return -EIO;

	if (!count)
		return 0;

	mutexLock(ctx->rxlock);
	mutexLock(ctx->lock);

	ctx->read = &read;
	ctx->rxend = (char *)buff + count;

	/* This field works as trigger for rx interrupt to store data in buffer
	 * instead of FIFO */
	ctx->rxbeg = buff;

	/* Provoke UART exception to fire so that existing data from
	 * rxdfifo is copied into buff. The handler will clear this
	 * bit. */

	*(ctx->base + cr1) |= 1 << 7;

	while (ctx->rxbeg != ctx->rxend) {
		err = condWait(ctx->rxcond, ctx->lock, timeout);

		if (mode == uart_mnblock || (timeout && err == -ETIME) || !ctx->enabled) {
			ctx->rxbeg = NULL;
			ctx->rxend = NULL;
			ctx->read = NULL;
			break;
		}
	}

	if (ctx->bits < 8) {
		if (ctx->bits == 6)
			mask = 0x3f;

		for (i = 0; i < read; ++i)
			((char *)buff)[i] &= mask;
	}
	mutexUnlock(ctx->lock);

	mutexUnlock(ctx->rxlock);

	return read;
}


int libuart_init(libuart_ctx *ctx, unsigned int uart)
{

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

	if (uart >= (sizeof(info) / sizeof(info[0])))
		return -1;

	devClk(info[uart].dev, 1);

	mutexCreate(&ctx->rxlock);
	condCreate(&ctx->rxcond);
	mutexCreate(&ctx->txlock);
	condCreate(&ctx->txcond);

	mutexCreate(&ctx->lock);

	ctx->base = info[uart].base;

	/* Set up UART to 9600,8,n,1 16-bit oversampling */
	libuart_configure(ctx, 8, uart_parnone, 115200, 1);

	interrupt(info[uart].irq, libuart_rxirq, (void *)ctx, ctx->rxcond, NULL);
	interrupt(info[uart].irq, libuart_txirq, (void *)ctx, ctx->txcond, NULL);

	ctx->enabled = 1;

	return 0;
}
