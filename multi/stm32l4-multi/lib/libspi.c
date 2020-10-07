/*
 * Phoenix-RTOS
 *
 * STM32L4 SPI driver
 *
 * Copyright 2018, 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <sys/threads.h>
#include <sys/msg.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

#include "../common.h"
#include "libdma.h"
#include "libspi.h"


static const int spi2pctl[] = { pctl_spi1, pctl_spi2, pctl_spi3 };


enum { cr1 = 0, cr2 = 2, sr = 4, dr = 6, crcpr = 8, rxcrcr = 10, txcrcr = 12 };


static const struct {
	unsigned int base;
	int irq;
} spiinfo[3] = { { 0x40013000, spi1_irq }, { 0x40003800, spi2_irq }, { 0x40003c00, spi3_irq } };


static int libspi_spino(libspi_ctx_t *ctx)
{
	if ((uintptr_t)ctx->base == (uintptr_t)spiinfo[0].base)
		return spi1;
	else if ((uintptr_t)ctx->base == (uintptr_t)spiinfo[1].base)
		return spi2;

	return spi3;
}


static int libspi_irqHandler(unsigned int n, void *arg)
{
	libspi_ctx_t *ctx = (libspi_ctx_t *)arg;

	*(ctx->ibuff++) = *((volatile uint8_t *)(ctx->base + dr));
	if (--ctx->cnt)
		*((volatile uint8_t *)(ctx->base + dr)) = *(ctx->obuff++);
	else {
		*(ctx->base + cr2) &= ~(1 << 6);
		return 1;
	}

	return -1;
}


static unsigned char libspi_readwrite(libspi_ctx_t *ctx, unsigned char txd)
{
	unsigned char rxd;

	/* Initiate transmission */
	*((volatile uint8_t *)(ctx->base + dr)) = txd;

	/* Wait until RXNE==1 */
	while (!(*(ctx->base + sr) & 0x1))
		;

	rxd = *((volatile uint8_t *)(ctx->base + dr));

	return rxd;
}


static void libspi_readwriteIrq(libspi_ctx_t *ctx, unsigned char *ibuff, unsigned char *obuff, size_t bufflen)
{
	ctx->ibuff = ibuff;
	ctx->obuff = obuff + 1;
	ctx->cnt = bufflen;

	/* Initiate transmission */
	*(ctx->base + cr2) |= 1 << 6;
	*((volatile uint8_t *)(ctx->base + dr)) = obuff[0];

	mutexLock(ctx->irqLock);
	while (ctx->cnt)
		condWait(ctx->cond, ctx->irqLock, 1);
	mutexUnlock(ctx->irqLock);
}


static void libspi_readwriteDma(libspi_ctx_t *ctx, unsigned char *ibuff, unsigned char *obuff, size_t bufflen)
{
	unsigned int dmach = (1 << 1) | (ibuff != NULL);

	*(ctx->base + cr2) |= dmach;
	libdma_transferSpi(libspi_spino(ctx), ibuff, obuff, bufflen);
	*(ctx->base + cr2) &= ~dmach;

	if (ibuff != NULL)
		return;

	/* Wait until RXNE=1 and TXE=1, i.e. the whole transfer is complete */
	while ((*(ctx->base + sr) & 0x3) != 0x3)
		;

	/* Empty out the RX FIFO completely - leaving anything in the FIFO will corrupt subsequent transfers */
	while (*(ctx->base + sr) & 0x1)
		(void) *((volatile uint8_t *)(ctx->base + dr));
}


int libspi_transaction(libspi_ctx_t *ctx, int dir, unsigned char cmd, unsigned int addr, unsigned char flags, unsigned char *ibuff, unsigned char *obuff, size_t bufflen)
{
	int i;
	unsigned int addrsz;

	addrsz = (flags >> SPI_ADDRSHIFT) & SPI_ADDRMASK;

	mutexLock(ctx->mutex);
	keepidle(1);

	if (flags & spi_cmd)
		libspi_readwrite(ctx, cmd);

	if (addrsz > 0) {
		if (flags & spi_addrlsb) {
			for (i = 0; i < addrsz; ++i) {
				libspi_readwrite(ctx, addr & 0xFF);
				addr >>= 8;
			}
		}
		else {
			for (i = 0; i < addrsz; ++i) {
				libspi_readwrite(ctx, (addr >> (addrsz - 1) * 8) & 0xFF);
				addr <<= 8;
			}
		}
	}

	if (flags & spi_dummy)
		libspi_readwrite(ctx, 0);

	if (bufflen >= 6) {
		if (ctx->usedma)
			libspi_readwriteDma(ctx, ibuff, obuff, bufflen);
		else
			libspi_readwriteIrq(ctx, ibuff, obuff, bufflen);
	}
	else {
		if (dir == spi_dir_read) {
			for (i = 0; i < bufflen; ++i)
				ibuff[i] = libspi_readwrite(ctx, 0);
		}
		else if (dir == spi_dir_write) {
			for (i = 0; i < bufflen; ++i)
				libspi_readwrite(ctx, obuff[i]);
		}
		else {
			for (i = 0; i < bufflen; ++i)
				ibuff[i] = libspi_readwrite(ctx, obuff[i]);
		}
	}

	/* Wait until BSY=0 */
	while ((*(ctx->base + sr) & (1 << 7)) != 0x0)
		;

	keepidle(0);
	mutexUnlock(ctx->mutex);

	return bufflen;
}


int libspi_configure(libspi_ctx_t *ctx, char mode, char bdiv, int enable)
{
	unsigned int t;

	mutexLock(ctx->mutex);

	devClk(spi2pctl[libspi_spino(ctx) - spi1], 1);
	*(ctx->base + cr1) &= ~(1 << 6);

	/* Set mode and baud div */
	t = *(ctx->base + cr1) & ~((0x7 << 3) | 0x3);
	*(ctx->base + cr1) = t | (((unsigned int)bdiv & 0x7) << 3) | (1 << 2) | (mode & 0x3);

	/* 8-bit RXNE threshold, 8 bits, motorola frame format, SS output enable */
	*(ctx->base + cr2) |= (1 << 12) | (0x7 << 8) | (1 << 2);

	if (enable)
		*(ctx->base + cr1) |= 1 << 6;
	else
		devClk(spi2pctl[libspi_spino(ctx) - spi1], 0);

	mutexUnlock(ctx->mutex);

	return EOK;
}


int libspi_init(libspi_ctx_t *ctx, unsigned int spi, int useDma)
{
	libdma_init();

	if (spi > spi3 || ctx == NULL)
		return -1;

	ctx->base = (void *)spiinfo[spi - spi1].base;
	ctx->usedma = useDma;

	mutexCreate(&ctx->mutex);
	mutexCreate(&ctx->irqLock);
	condCreate(&ctx->cond);

	libspi_configure(ctx, 0, 0, 1);

	if (useDma) {
		libdma_configureSpi(spi, dma_mem2per, 0x1, (void *)(ctx->base + dr), 0x0, 0x0, 0x1, 0x0);
		libdma_configureSpi(spi, dma_per2mem, 0x1, (void *)(ctx->base + dr), 0x0, 0x0, 0x1, 0x0);
	}
	else {
		interrupt(spiinfo[spi - spi1].irq, libspi_irqHandler, (void *)ctx, ctx->cond, &ctx->inth);
	}

	return 0;
}
