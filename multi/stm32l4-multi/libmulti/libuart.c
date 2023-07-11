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
#include <sys/minmax.h>
#include <sys/time.h>

#include "libmulti/libuart.h"
#include "libmulti/libdma.h"

#include "../config.h"
#include "../common.h"


/* clang-format off */
enum { cr1 = 0, cr2, cr3, brr, gtpr, rtor, rqr, isr, icr, rdr, tdr };
/* clang-format on */


static const struct {
	volatile uint32_t *base;
	int dev;
	unsigned irq;
} libuart_info[] = {
	{ (void *)0x40013800, pctl_usart1, usart1_irq },
	{ (void *)0x40004400, pctl_usart2, usart2_irq },
	{ (void *)0x40004800, pctl_usart3, usart3_irq },
	{ (void *)0x40004c00, pctl_uart4, uart4_irq },
	{ (void *)0x40005000, pctl_uart5, uart5_irq },
};


static int libuart_txirq(unsigned int n, void *arg)
{
	libuart_ctx *ctx = (libuart_ctx *)arg;
	int release = -1;

	if (((*(ctx->base + cr1) & (1 << 7)) != 0) && ((*(ctx->base + isr) & (1 << 7)) != 0)) {
		/* Txd buffer empty */
		if (ctx->data.irq.txbeg != ctx->data.irq.txend) {
			*(ctx->base + tdr) = *(ctx->data.irq.txbeg++);
		}
		else {
			*(ctx->base + cr1) &= ~(1 << 7);
			ctx->data.irq.txbeg = NULL;
			ctx->data.irq.txend = NULL;
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
	if (n == lpuart1_irq) {
		*(ctx->base + icr) |= 1 << 20;
	}

	if (*(ctx->base + isr) & ((1 << 5) | (1 << 3))) {
		/* Clear overrun error bit */
		*(ctx->base + icr) |= (1 << 3);

		/* Rxd buffer not empty */
		ctx->data.irq.rxdfifo[ctx->data.irq.rxdw++] = *(ctx->base + rdr);
		ctx->data.irq.rxdw %= sizeof(ctx->data.irq.rxdfifo);

		if (ctx->data.irq.rxdr == ctx->data.irq.rxdw) {
			ctx->data.irq.rxdr = (ctx->data.irq.rxdr + 1) % sizeof(ctx->data.irq.rxdfifo);
		}
	}

	if (ctx->data.irq.rxbeg != NULL) {
		while (ctx->data.irq.rxdr != ctx->data.irq.rxdw && ctx->data.irq.rxbeg != ctx->data.irq.rxend) {
			*(ctx->data.irq.rxbeg++) = ctx->data.irq.rxdfifo[ctx->data.irq.rxdr++];
			ctx->data.irq.rxdr %= sizeof(ctx->data.irq.rxdfifo);
			(*ctx->data.irq.read)++;
		}

		if (ctx->data.irq.rxbeg == ctx->data.irq.rxend) {
			ctx->data.irq.rxbeg = NULL;
			ctx->data.irq.rxend = NULL;
			ctx->data.irq.read = NULL;
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

	if (ctx->type == uart_irq) {

		condBroadcast(ctx->data.irq.rxcond);

		mutexLock(ctx->data.irq.txlock);
		mutexLock(ctx->data.irq.rxlock);
		mutexLock(ctx->data.irq.lock);

		dataBarier();

		ctx->data.irq.txbeg = NULL;
		ctx->data.irq.txend = NULL;

		ctx->data.irq.rxbeg = NULL;
		ctx->data.irq.rxend = NULL;
		ctx->data.irq.read = NULL;
		ctx->data.irq.rxdr = 0;
		ctx->data.irq.rxdw = 0;
	}
	else {
		mutexLock(ctx->data.dma.txlock);
		mutexLock(ctx->data.dma.rxlock);
	}

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

		if (parity == uart_parodd) {
			*(ctx->base + cr1) |= 1 << 9;
		}
		else {
			*(ctx->base + cr1) &= ~(1 << 9);
		}

		*(ctx->base + icr) = -1;
		(void)*(ctx->base + rdr);

		if (enable != 0) {
			*(ctx->base + cr1) |= (1 << 5) | (1 << 3) | (1 << 2);
			dataBarier();
			*(ctx->base + cr1) |= 1;
			ctx->enabled = 1;
		}

		dataBarier();
	}

	if (ctx->type == uart_irq) {
		mutexUnlock(ctx->data.irq.lock);
		mutexUnlock(ctx->data.irq.rxlock);
		mutexUnlock(ctx->data.irq.txlock);
	}
	else {
		mutexUnlock(ctx->data.dma.rxlock);
		mutexUnlock(ctx->data.dma.txlock);
	}
	return err;
}


static int libuart_irqWrite(libuart_ctx *ctx, const void *buff, unsigned int bufflen)
{
	mutexLock(ctx->data.irq.txlock);
	mutexLock(ctx->data.irq.lock);

	keepidle(1);

	*(ctx->base + tdr) = *((unsigned char *)buff);
	ctx->data.irq.txbeg = (void *)((unsigned char *)buff + 1);
	ctx->data.irq.txend = (void *)((unsigned char *)buff + bufflen);
	*(ctx->base + cr1) |= 1 << 7;

	while (ctx->data.irq.txbeg != ctx->data.irq.txend) {
		condWait(ctx->data.irq.txcond, ctx->data.irq.lock, 0);
	}
	mutexUnlock(ctx->data.irq.lock);

	keepidle(0);
	mutexUnlock(ctx->data.irq.txlock);

	return bufflen;
}


static int libuart_dmaWrite(libuart_ctx *ctx, const void *buff, unsigned int bufflen)
{
	int written, writesz;

	mutexLock(ctx->data.dma.txlock);

	for (written = 0; written < bufflen; written += writesz) {
		*(ctx->base + icr) |= (1 << 6);
		writesz = libdma_tx(ctx->data.dma.per, ((char *)buff) + written, min(DMA_MAX_LEN, bufflen - written), dma_modeNormal, 0);
		if (writesz == 0) {
			break;
		}
	}

	mutexUnlock(ctx->data.dma.txlock);

	return written;
}


int libuart_write(libuart_ctx *ctx, const void *buff, unsigned int bufflen)
{
	if (bufflen == 0) {
		return 0;
	}

	if (ctx->enabled == 0) {
		return -EIO;
	}

	if (ctx->type == uart_irq) {
		return libuart_irqWrite(ctx, buff, bufflen);
	}
	return libuart_dmaWrite(ctx, buff, bufflen);
}


static void libuart_readMaskBits(libuart_ctx *ctx, void *buff, unsigned int read)
{
	char mask = 0x7f;
	int i;

	if (ctx->bits < 8) {
		if (ctx->bits == 6) {
			mask = 0x3f;
		}

		for (i = 0; i < read; ++i) {
			((char *)buff)[i] &= mask;
		}
	}
}


static int libuart_dmaRead(libuart_ctx *ctx, void *buff, unsigned int count, char mode, unsigned int timeout)
{
	int read, readsz;
	int timedout;
	time_t now, end, rxtimeout;
	int dmamode = (mode == uart_mnblock) ? dma_modeNoBlock : dma_modeNormal;

	mutexLock(ctx->data.dma.rxlock);

	rxtimeout = timeout;
	if (timeout > 0) {
		gettime(&now, NULL);
		end = now + timeout;
	}

	for (read = 0, timedout = 0; (timedout == 0) && (read < count); read += readsz) {
		/* Clear overrun error. */
		if ((*(ctx->base + isr) & ((1 << 5) | (1 << 3))) != 0) {
			*(ctx->base + icr) |= (1 << 3);
			((char *)buff)[read] = *(ctx->base + rdr);
			readsz = 1;
			continue;
		}

		*(ctx->base + cr3) |= (1 << 6); /* Enable DMA for reception. */
		readsz = libdma_rx(ctx->data.dma.per, ((char *)buff) + read, min(DMA_MAX_LEN, count - read), dmamode, rxtimeout);
		*(ctx->base + cr3) &= ~(1 << 6); /* Disable DMA for reception. */

		if (readsz == 0) {
			break;
		}
		if (timeout != 0) {
			gettime(&now, NULL);
			if (now >= end) {
				timedout = 1;
			}
			else {
				rxtimeout = end - now;
			}
		}
	}

	mutexUnlock(ctx->data.dma.rxlock);

	libuart_readMaskBits(ctx, buff, read);

	return read;
}


static int libuart_irqRead(libuart_ctx *ctx, void *buff, unsigned int count, char mode, unsigned int timeout)
{
	int err;
	volatile unsigned int read = 0;

	mutexLock(ctx->data.irq.rxlock);
	mutexLock(ctx->data.irq.lock);

	ctx->data.irq.read = &read;
	ctx->data.irq.rxend = (char *)buff + count;

	/* This field works as trigger for rx interrupt to store data in buffer
	 * instead of FIFO */
	ctx->data.irq.rxbeg = buff;

	/* Provoke UART exception to fire so that existing data from
	 * rxdfifo is copied into buff. The handler will clear this
	 * bit. */

	*(ctx->base + cr1) |= 1 << 7;

	while (ctx->data.irq.rxbeg != ctx->data.irq.rxend) {
		err = condWait(ctx->data.irq.rxcond, ctx->data.irq.lock, timeout);

		if ((mode == uart_mnblock) || (timeout && err == -ETIME) || (ctx->enabled == 0)) {
			ctx->data.irq.rxbeg = NULL;
			ctx->data.irq.rxend = NULL;
			ctx->data.irq.read = NULL;
			break;
		}
	}

	libuart_readMaskBits(ctx, buff, read);
	mutexUnlock(ctx->data.irq.lock);

	mutexUnlock(ctx->data.irq.rxlock);

	return read;
}


int libuart_read(libuart_ctx *ctx, void *buff, unsigned int count, char mode, unsigned int timeout)
{
	if (ctx->enabled == 0) {
		return -EIO;
	}

	if (count == 0) {
		return 0;
	}

	if (ctx->type == uart_irq) {
		return libuart_irqRead(ctx, buff, count, mode, timeout);
	}
	return libuart_dmaRead(ctx, buff, count, mode, timeout);
}


static int libuart_dmaInit(libuart_ctx *ctx, unsigned int uart)
{
	int err;

	ctx->type = uart_dma;

	libdma_init();
	err = libdma_acquirePeripheral(dma_uart, uart, &ctx->data.dma.per);
	if (err < 0) {
		return err;
	}
	libdma_configurePeripheral(ctx->data.dma.per, dma_mem2per, dma_priorityHigh, (void *)(ctx->base + tdr), 0x0, 0x0, 0x1, 0x0, NULL);
	libdma_configurePeripheral(ctx->data.dma.per, dma_per2mem, dma_priorityVeryHigh, (void *)(ctx->base + rdr), 0x0, 0x0, 0x1, 0x0, NULL);
	*(ctx->base + cr3) |= (1 << 7); /* Enable DMA for transmission. */

	mutexCreate(&ctx->data.dma.rxlock);
	mutexCreate(&ctx->data.dma.txlock);

	return 0;
}


static void libuart_irqInit(libuart_ctx *ctx, unsigned int uart)
{
	ctx->type = uart_irq;

	mutexCreate(&ctx->data.irq.rxlock);
	condCreate(&ctx->data.irq.rxcond);
	mutexCreate(&ctx->data.irq.txlock);
	condCreate(&ctx->data.irq.txcond);

	mutexCreate(&ctx->data.irq.lock);

	interrupt(libuart_info[uart].irq, libuart_rxirq, (void *)ctx, ctx->data.irq.rxcond, NULL);
	interrupt(libuart_info[uart].irq, libuart_txirq, (void *)ctx, ctx->data.irq.txcond, NULL);
}


int libuart_init(libuart_ctx *ctx, unsigned int uart, int dma)
{
	int err;

	if (uart >= (sizeof(libuart_info) / sizeof(libuart_info[0]))) {
		return -1;
	}

	devClk(libuart_info[uart].dev, 1);
	ctx->base = libuart_info[uart].base;
	/* Set up UART to 9600,8,n,1 16-bit oversampling */
	libuart_configure(ctx, 8, uart_parnone, 115200, 1);

	if (dma == 0) {
		libuart_irqInit(ctx, uart);
	}
	else {
		err = libuart_dmaInit(ctx, uart);
		if (err < 0) {
			return err;
		}
	}

	ctx->enabled = 1;

	return 0;
}
