/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4 UART driver
 *
 * Copyright 2017, 2018, 2020, 2023 Phoenix Systems
 * Author: Jan Sikorski, Aleksander Kaminski, Andrzej Glowinski, Jan Wisniewski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
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


static struct {
	size_t rxfifosz;
} libuart_config[] = {
	{ UART1_RXFIFOSZ },
	{ UART2_RXFIFOSZ },
	{ UART3_RXFIFOSZ },
	{ UART4_RXFIFOSZ },
	{ UART5_RXFIFOSZ },
};


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


static inline size_t libuart_incrementWrap(size_t value, size_t size)
{
	value += 1;
	return (value < size) ? value : 0;
}


static int libuart_txirq(unsigned int n, void *arg)
{
	libuart_ctx *ctx = (libuart_ctx *)arg;
	int release = -1;

	if (((*(ctx->base + cr1) & (1 << 7)) != 0) && ((*(ctx->base + isr) & (1 << 7)) != 0)) {
		/* Txd buffer empty */
		if (ctx->data.irq.txbeg != NULL) {
			if (ctx->data.irq.txbeg != ctx->data.irq.txend) {
				*(ctx->base + tdr) = *(ctx->data.irq.txbeg);
				ctx->data.irq.txbeg += 1;
			}

			if (ctx->data.irq.txbeg == ctx->data.irq.txend) {
				ctx->data.irq.txbeg = NULL;
				ctx->data.irq.txend = NULL;
				release = 1;
				*(ctx->base + cr1) &= ~(1 << 7);
			}
		}
		else {
			/* wake-up from libuart_triggerInterrupt */
			*(ctx->base + cr1) &= ~(1 << 7);
		}
	}

	return release;
}


static int libuart_rxirq(unsigned int n, void *arg)
{
	libuart_ctx *ctx = (libuart_ctx *)arg;
	int release = -1;

	/* Clear wake-up from stop mode flag */
	if (n == lpuart1_irq) {
		*(ctx->base + icr) |= 1 << 20;
	}

	/*
	 * Reading twice as we can have bytes in both data and shift registers
	 * It is not obvious from manual that second byte is available in this scenario
	 */
	for (int i = 0; i < 2; ++i) {
		if ((*(ctx->base + isr) & (1 << 5)) == 0) {
			/* Rxd buffer empty */
			break;
		}

		ctx->data.irq.rxdfifo[ctx->data.irq.rxdw] = *(ctx->base + rdr);
		ctx->data.irq.rxdw = libuart_incrementWrap(ctx->data.irq.rxdw, ctx->data.irq.rxdfifosz);

		/* discard oldest byte if queue full */
		if (ctx->data.irq.rxdr == ctx->data.irq.rxdw) {
			ctx->data.irq.rxdr = libuart_incrementWrap(ctx->data.irq.rxdr, ctx->data.irq.rxdfifosz);
		}
	}

	/* TODO: consider disabling overrun exception completely */
	if (*(ctx->base + isr) & (1 << 3)) {
		/* Clear overrun error bit */
		*(ctx->base + icr) |= (1 << 3);
	}

	if (ctx->data.irq.rxbeg != NULL) {
		while ((ctx->data.irq.rxdr != ctx->data.irq.rxdw) && (ctx->data.irq.rxbeg != ctx->data.irq.rxend)) {
			*(ctx->data.irq.rxbeg) = ctx->data.irq.rxdfifo[ctx->data.irq.rxdr];
			ctx->data.irq.rxdr = libuart_incrementWrap(ctx->data.irq.rxdr, ctx->data.irq.rxdfifosz);
			ctx->data.irq.rxbeg += 1;
			*ctx->data.irq.read += 1;
		}

		if ((ctx->data.irq.rxbeg == ctx->data.irq.rxend) || (ctx->data.irq.rxnblock != 0)) {
			ctx->data.irq.rxbeg = NULL;
			release = 1;
		}
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
		ctx->data.irq.rxnblock = 0;
		ctx->data.irq.rxdr = 0;
		ctx->data.irq.rxdw = 0;
	}
	else {
		condBroadcast(ctx->data.dma.rxcond);

		mutexLock(ctx->data.dma.txlock);
		mutexLock(ctx->data.dma.rxlock);
		mutexLock(ctx->data.dma.rxcondlock);
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
		mutexUnlock(ctx->data.dma.rxcondlock);
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

	ctx->data.irq.txbeg = (void *)((unsigned char *)buff);
	ctx->data.irq.txend = (void *)((unsigned char *)buff + bufflen);
	*(ctx->base + cr1) |= 1 << 7;

	while (ctx->data.irq.txbeg != NULL) {
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


static void libuart_triggerInterrupt(libuart_ctx *ctx)
{
	/* Enable TX Empty interrupt that will eventually be triggered. */
	*(ctx->base + cr1) |= 1 << 7;
}


static int libuart_dmaRead(libuart_ctx *ctx, void *buff, unsigned int count, char mode, unsigned int timeout)
{
	time_t now, end = 0, rxtimeout;
	size_t read;
	int err;

	mutexLock(ctx->data.dma.rxlock);
	mutexLock(ctx->data.dma.rxcondlock);

	rxtimeout = timeout;
	if (timeout > 0) {
		gettime(&now, NULL);
		end = now + timeout;
	}

	ctx->data.dma.read = 0;
	ctx->data.dma.rxbufsz = count;
	ctx->data.dma.rxbuf = buff;

	libuart_triggerInterrupt(ctx);

	/* Enable idle line interrupt. */
	*(ctx->base + cr1) |= (1 << 4);

	while (ctx->data.dma.read != count) {
		err = condWait(ctx->data.dma.rxcond, ctx->data.dma.rxcondlock, rxtimeout);

		if ((err == -ETIME) || (mode == uart_mnblock) || (ctx->enabled == 0)) {
			break;
		}
		if (timeout > 0) {
			gettime(&now, NULL);
			if (now >= end) {
				break;
			}
			else {
				rxtimeout = end - now;
			}
		}
	}

	/* Disable idle line interrupt. */
	*(ctx->base + cr1) &= ~(1 << 4);

	ctx->data.dma.rxbuf = NULL;
	read = ctx->data.dma.read;

	mutexUnlock(ctx->data.dma.rxcondlock);
	mutexUnlock(ctx->data.dma.rxlock);

	libuart_readMaskBits(ctx, buff, read);

	return read;
}


static int libuart_irqRead(libuart_ctx *ctx, void *buff, unsigned int count, char mode, unsigned int timeout)
{
	volatile unsigned int read = 0;

	mutexLock(ctx->data.irq.rxlock);
	mutexLock(ctx->data.irq.lock);

	assert(ctx->data.irq.rxbeg == NULL);

	ctx->data.irq.read = &read;
	ctx->data.irq.rxend = (char *)buff + count;

	if (mode == uart_mnblock) {
		/* end transaction as soon as irq handler is triggered */
		ctx->data.irq.rxnblock = 1;
		timeout = 0;
	}
	else {
		ctx->data.irq.rxnblock = 0;
	}

	/* This field works as trigger for rx interrupt to store data in buffer
	 * instead of FIFO */
	ctx->data.irq.rxbeg = buff;

	/* Provoke UART exception to fire so that existing data from
	 * rxdfifo is copied into buff. The handler will clear this
	 * bit. */
	libuart_triggerInterrupt(ctx);

	while (ctx->data.irq.rxbeg != NULL) {
		const int err = condWait(ctx->data.irq.rxcond, ctx->data.irq.lock, timeout);

		if ((err == -ETIME) || ctx->enabled == 0) {
			/* rxbeg will be set to NULL as soon as irq is handled */
			ctx->data.irq.rxnblock = 1;
			libuart_triggerInterrupt(ctx);
			break;
		}
	}

	/* if ended early wait until last interrupt is handled */
	while (ctx->data.irq.rxbeg != NULL) {
		condWait(ctx->data.irq.rxcond, ctx->data.irq.lock, 0);
	}

	ctx->data.irq.rxend = NULL;
	ctx->data.irq.read = NULL;

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


static void libuart_infiniteRxHandler(void *arg, int type)
{
	libuart_ctx *ctx = arg;
	size_t endPos;
	size_t read;
	size_t rxfifopos;
	size_t rxfifoprevend;
	size_t rxbufsz;
	size_t torx;
	size_t infifo;
	size_t cpysz;
	char *rxbuf = (char *)ctx->data.dma.rxbuf;

	endPos = (ctx->data.dma.rxfifosz - libdma_leftToRx(ctx->data.dma.per)) % ctx->data.dma.rxfifosz;

	/* Check overrun */
	rxfifopos = ctx->data.dma.rxfifopos;
	rxfifoprevend = ctx->data.dma.rxfifoprevend;

	if (((rxfifoprevend < rxfifopos) && ((rxfifopos < endPos) || (endPos < rxfifoprevend))) ||
		((rxfifopos <= rxfifoprevend) && ((rxfifopos < endPos) && (endPos < rxfifoprevend)))) {
		rxfifopos = endPos;
	}

	infifo = (ctx->data.dma.rxfifosz + endPos - rxfifopos) % ctx->data.dma.rxfifosz;
	if (((endPos != rxfifoprevend) && (infifo == 0)) || (ctx->data.dma.rxfifofull != 0)) {
		ctx->data.dma.rxfifofull = 1;
		infifo = ctx->data.dma.rxfifosz;
	}

	if (rxbuf != NULL) {
		rxbufsz = ctx->data.dma.rxbufsz;
		read = ctx->data.dma.read;

		torx = min(rxbufsz - read, infifo);
		if (torx != 0) {
			ctx->data.dma.rxfifofull = 0;
			cpysz = min(ctx->data.dma.rxfifosz - rxfifopos, torx);
			memcpy(rxbuf + read, ctx->data.dma.rxfifo + rxfifopos, cpysz);
			if (torx != cpysz) {
				memcpy(rxbuf + read + cpysz, ctx->data.dma.rxfifo, torx - cpysz);
			}

			rxfifopos = (rxfifopos + torx) % ctx->data.dma.rxfifosz;
			ctx->data.dma.read = read + torx;
		}
	}

	ctx->data.dma.rxfifopos = rxfifopos % ctx->data.dma.rxfifosz;
	ctx->data.dma.rxfifoprevend = endPos;
}


static int uart_irqDMA(unsigned int n, void *arg)
{
	libuart_ctx *ctx = arg;
	int read;
	uint32_t status = *(ctx->base + isr);

	/* Check for the idle line. */
	if ((status & (1 << 4)) != 0) {
		/* Clear idle line bit */
		*(ctx->base + icr) |= (1 << 4);
	} /* RX user request interrupt. */
	else if ((status & (1 << 7)) != 0) {
		*(ctx->base + cr1) &= ~(1 << 7);
	}
	else {
		return -1;
	}

	read = libdma_leftToRx(ctx->data.dma.per);

	if (read != 0) {
		libuart_infiniteRxHandler(ctx, -read);
	}

	return 1;
}


static int libuart_dmaInit(libuart_ctx *ctx, unsigned int uart)
{
	int err;

	ctx->type = uart_dma;

	libdma_init();

	if (ctx->data.dma.rxfifosz >= DMA_MAX_LEN) {
		return -EINVAL;
	}

	ctx->data.dma.rxfifopos = 0;
	ctx->data.dma.rxfifoprevend = 0;
	ctx->data.dma.rxfifosz = libuart_config[uart].rxfifosz;
	ctx->data.dma.rxfifo = malloc(ctx->data.dma.rxfifosz);
	ctx->data.dma.rxfifofull = 0;

	if (ctx->data.dma.rxfifo == NULL) {
		return -ENOMEM;
	}

	ctx->data.dma.read = 0;
	ctx->data.dma.rxbufsz = 0;
	ctx->data.dma.rxbuf = NULL;

	err = libdma_acquirePeripheral(dma_uart, uart, &ctx->data.dma.per);
	if (err < 0) {
		free((void *)ctx->data.dma.rxfifo);
		return err;
	}

	mutexCreate(&ctx->data.dma.rxlock);
	mutexCreate(&ctx->data.dma.rxcondlock); /* No synchronization purpose only existing to conform to condWait API. */
	mutexCreate(&ctx->data.dma.txlock);
	condCreate(&ctx->data.dma.rxcond);

	libdma_configurePeripheral(ctx->data.dma.per, dma_mem2per, dma_priorityHigh, (void *)(ctx->base + tdr), 0x0, 0x0, 0x1, 0x0, NULL);
	libdma_configurePeripheral(ctx->data.dma.per, dma_per2mem, dma_priorityVeryHigh, (void *)(ctx->base + rdr), 0x0, 0x0, 0x1, 0x0, &ctx->data.dma.rxcond);

	*(ctx->base + cr3) |= (1 << 7) | (1 << 6); /* Enable DMA for transmission and reception. */

	/* Clear overflow bit. */
	*(ctx->base + icr) |= (1 << 3);

	(void)libdma_infiniteRxAsync(ctx->data.dma.per, (void *)ctx->data.dma.rxfifo, ctx->data.dma.rxfifosz, libuart_infiniteRxHandler, ctx);

	interrupt(libuart_info[uart].irq, uart_irqDMA, (void *)ctx, ctx->data.dma.rxcond, NULL);

	return 0;
}


static int libuart_irqInit(libuart_ctx *ctx, unsigned int uart)
{
	ctx->type = uart_irq;

	ctx->data.irq.rxdfifosz = libuart_config[uart].rxfifosz;
	ctx->data.irq.rxdfifo = malloc(ctx->data.irq.rxdfifosz);

	if (ctx->data.irq.rxdfifo == NULL) {
		return -ENOMEM;
	}

	mutexCreate(&ctx->data.irq.rxlock);
	condCreate(&ctx->data.irq.rxcond);
	mutexCreate(&ctx->data.irq.txlock);
	condCreate(&ctx->data.irq.txcond);

	mutexCreate(&ctx->data.irq.lock);

	interrupt(libuart_info[uart].irq, libuart_rxirq, (void *)ctx, ctx->data.irq.rxcond, NULL);
	interrupt(libuart_info[uart].irq, libuart_txirq, (void *)ctx, ctx->data.irq.txcond, NULL);

	return 0;
}


int libuart_init(libuart_ctx *ctx, unsigned int uart, int dma)
{
	int err;

	if (uart >= (sizeof(libuart_info) / sizeof(libuart_info[0]))) {
		return -1;
	}

	devClk(libuart_info[uart].dev, 1);
	ctx->base = libuart_info[uart].base;
	/* Set up UART to 115200,8,n,1 16-bit oversampling */
	libuart_configure(ctx, 8, uart_parnone, 115200, 1);

	if (dma == 0) {
		err = libuart_irqInit(ctx, uart);
	}
	else {
		err = libuart_dmaInit(ctx, uart);
	}

	if (err < 0) {
		return err;
	}

	ctx->enabled = 1;

	return 0;
}
