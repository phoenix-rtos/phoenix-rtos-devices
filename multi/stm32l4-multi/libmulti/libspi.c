/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4 SPI driver
 *
 * Copyright 2018, 2020 Phoenix Systems
 * Author: Aleksander Kaminski, Daniel Sawka
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <sys/pwman.h>
#include <errno.h>
#include <stdint.h>

#include "../common.h"
#include "libmulti/libdma.h"
#include "libmulti/libspi.h"


enum { cr1 = 0, cr2 = 2, sr = 4, dr = 6, crcpr = 8, rxcrcr = 10, txcrcr = 12 };


static const struct {
	unsigned int base;
	unsigned int pctl;
} spiinfo[] = {
	{ 0x40013000, pctl_spi1 },
	{ 0x40003800, pctl_spi2 },
	{ 0x40003c00, pctl_spi3 }
};


static int libspi_spino(libspi_ctx_t *ctx)
{
	if ((uintptr_t)ctx->base == (uintptr_t)spiinfo[0].base)
		return spi1;
	else if ((uintptr_t)ctx->base == (uintptr_t)spiinfo[1].base)
		return spi2;

	return spi3;
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


static void libspi_readwriteDma(libspi_ctx_t *ctx, unsigned char *ibuff, const unsigned char *obuff, size_t bufflen)
{
	unsigned int dmach = (1 << 1) | (ibuff != NULL);

	*(ctx->base + cr2) |= dmach;
	libdma_transferSpi(libspi_spino(ctx), ibuff, obuff, bufflen);
	*(ctx->base + cr2) &= ~dmach;
}


int libspi_transaction(libspi_ctx_t *ctx, int dir, unsigned char cmd, unsigned int addr, unsigned char flags, unsigned char *ibuff, const unsigned char *obuff, size_t bufflen)
{
	unsigned int i, addrsz;

	addrsz = (flags >> SPI_ADDRSHIFT) & SPI_ADDRMASK;

	keepidle(1);

	/* Enable SPI */
	*(ctx->base + cr1) |= 1 << 6;
	dataBarier();

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

	if (bufflen >= 6 && ctx->usedma) {
		libspi_readwriteDma(ctx, ibuff, obuff, bufflen);
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

	/* Wait until FTLVL=0 (TXFIFO empty) */
	while (((*(ctx->base + sr) >> 11) & 0x3) != 0) {
		;
	}

	/* Wait until BSY=0 */
	while ((*(ctx->base + sr) & (1 << 7)) != 0x0) {
		;
	}

	/* Disable SPI */
	dataBarier();
	*(ctx->base + cr1) &= ~(1 << 6);
	dataBarier();

	/* Wait until FRLVL=0 (empty out RXFIFO completely).
	Leaving anything in the FIFO will corrupt subsequent transfers */
	while (((*(ctx->base + sr) >> 9) & 0x3) != 0) {
		(void)*((volatile uint8_t *)(ctx->base + dr));
	}

	keepidle(0);

	return bufflen;
}


int libspi_configure(libspi_ctx_t *ctx, char mode, char bdiv, int enable)
{
	unsigned int t;

	devClk(spiinfo[libspi_spino(ctx) - spi1].pctl, 1);
	*(ctx->base + cr1) &= ~(1 << 6);
	dataBarier();

	/* Set mode and baud div */
	t = *(ctx->base + cr1) & ~((0x7 << 3) | 0x3);
	*(ctx->base + cr1) = t | (((unsigned int)bdiv & 0x7) << 3) | (1 << 2) | (mode & 0x3);

	/* 8-bit RXNE threshold, 8 bits, motorola frame format, SS output enable */
	*(ctx->base + cr2) |= (1 << 12) | (0x7 << 8) | (1 << 2);
	dataBarier();

	if (!enable) {
		devClk(spiinfo[libspi_spino(ctx) - spi1].pctl, 0);
	}

	return EOK;
}


int libspi_init(libspi_ctx_t *ctx, unsigned int spi, int useDma)
{
	if (useDma)
		libdma_init();

	if (spi > spi3 || ctx == NULL)
		return -1;

	ctx->base = (void *)spiinfo[spi - spi1].base;
	ctx->usedma = useDma;

	libspi_configure(ctx, 0, 0, 1);

	if (useDma) {
		libdma_configureSpi(spi, dma_mem2per, 0x1, (void *)(ctx->base + dr), 0x0, 0x0, 0x1, 0x0);
		libdma_configureSpi(spi, dma_per2mem, 0x1, (void *)(ctx->base + dr), 0x0, 0x0, 0x1, 0x0);
	}

	return 0;
}
