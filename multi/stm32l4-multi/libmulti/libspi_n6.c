/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32N6 SPI driver
 *
 * Copyright 2018, 2020, 2025 Phoenix Systems
 * Author: Aleksander Kaminski, Daniel Sawka, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <sys/interrupt.h>
#include <sys/threads.h>
#include <sys/pwman.h>
#include <errno.h>
#include <stdint.h>
#include <byteswap.h>

#include "../common.h"
#include "libmulti/libdma.h"
#include "libmulti/libspi.h"
#include "../rcc.h"
#include "../stm32n6_regs.h"

#define MAX_SPI spi6

#define SR_TXC    (1 << 12) /* TX FIFO empty */
#define SR_TXP    (1 << 1)  /* TX FIFO below threshold */
#define SR_RXP    (1 << 0)  /* RX FIFO above threshold */
#define SR_RXWNE  (1 << 15) /* RX FIFO contains at least 4 bytes */
#define SR_RXPLVL (3 << 13) /* Number of frames in RX FIFO (if < 4 bytes) */

#define SR_RXNE (SR_RXWNE | SR_RXPLVL) /* If any bit set, RX FIFO not empty */


static const struct libspi_peripheralInfo {
	uintptr_t base;
	unsigned int dev;
	enum ipclks clksel;    /* Clock selector */
	enum clock_ids clksrc; /* ID of source clock */
	uint32_t fifoThr;      /* Desired FIFO threshold (FIFO size differs on some instances) */
} spiInfo[MAX_SPI - spi1 + 1] = {
	{ (uintptr_t)SPI1_BASE, pctl_spi1, pctl_ipclk_spi1sel, clkid_per, 8 },
	{ (uintptr_t)SPI2_BASE, pctl_spi2, pctl_ipclk_spi2sel, clkid_per, 8 },
	{ (uintptr_t)SPI3_BASE, pctl_spi3, pctl_ipclk_spi3sel, clkid_per, 8 },
	{ (uintptr_t)SPI4_BASE, pctl_spi4, pctl_ipclk_spi4sel, clkid_per, 4 },
	{ (uintptr_t)SPI5_BASE, pctl_spi5, pctl_ipclk_spi5sel, clkid_per, 4 },
	{ (uintptr_t)SPI6_BASE, pctl_spi6, pctl_ipclk_spi6sel, clkid_per, 8 },
};


static const unsigned char *libspi_fifoFill(volatile uint32_t *base, const unsigned char *obuff, size_t *size_ptr)
{
	volatile uint8_t *txData = (void *)(base + spi_txdr);
	while (*size_ptr > 0) {
		if ((*(base + spi_sr) & SR_TXP) == 0) {
			return obuff;
		}

		if (obuff == NULL) {
			*txData = 0;
		}
		else {
			*txData = *obuff;
			obuff++;
		}

		(*size_ptr)--;
	}

	return obuff;
}


static unsigned char *libspi_fifoDrain(volatile uint32_t *base, unsigned char *ibuff, size_t *size_ptr)
{
	volatile uint8_t *rxData = (void *)(base + spi_rxdr);
	while (*size_ptr > 0) {
		if ((*(base + spi_sr) & SR_RXNE) == 0) {
			return ibuff;
		}

		if (ibuff == NULL) {
			(void)*rxData;
		}
		else {
			*ibuff = *rxData;
			ibuff++;
		}

		(*size_ptr)--;
	}

	return ibuff;
}


int libspi_transaction(libspi_ctx_t *ctx, int dir, unsigned char cmd, unsigned int addr, unsigned char flags, unsigned char *ibuff, const unsigned char *obuff, size_t bufflen)
{
	unsigned int addrsz = (flags >> SPI_ADDRSHIFT) & SPI_ADDRMASK;
	unsigned int ignoreBytes = 0;
	/* Ensure buffers are set to NULL if they are not used */
	if (dir == spi_dir_read) {
		obuff = NULL;
	}
	else if (dir == spi_dir_write) {
		ibuff = NULL;
	}

	/* Enable SPI */
	*(ctx->base + spi_cr1) |= 1;
	dataBarier();

	/* TX/RX data register need to be accessed as 8-bit */
	volatile uint8_t *txData = (volatile uint8_t *)(ctx->base + spi_txdr);

	/* Send command, address and dummy bytes if requested.
	 * They can be sent all at once because FIFO is large enough (at least 8 bytes) */
	if ((flags & spi_cmd) != 0) {
		*txData = cmd;
		ignoreBytes += 1;
	}

	if (addrsz > 0) {
		ignoreBytes += addrsz;
		if ((flags & spi_addrlsb) == 0) {
			uint32_t addr_swapped = bswap_32((uint32_t)addr);
			addr = addr_swapped >> ((4 - addrsz) * 8);
		}

		for (unsigned int i = 0; i < addrsz; i++) {
			*txData = addr & 0xff;
			addr >>= 8;
		}
	}

	if ((flags & spi_dummy) != 0) {
		ignoreBytes += 1;
		*txData = 0;
	}

	size_t txLen = bufflen;
	size_t rxLen = bufflen;
	/* Pre-fill the FIFO before we start transaction */
	obuff = libspi_fifoFill(ctx->base, obuff, &txLen);
	/* Start transaction */
	*(ctx->base + spi_cr1) |= 1 << 9;
	while ((txLen > 0) || (rxLen > 0)) {
		obuff = libspi_fifoFill(ctx->base, obuff, &txLen);
		if (ignoreBytes > 0) {
			libspi_fifoDrain(ctx->base, NULL, &ignoreBytes);
		}
		else {
			ibuff = libspi_fifoDrain(ctx->base, ibuff, &rxLen);
		}
	}

	while ((*(ctx->base + spi_sr) & SR_TXC) == 0) {
		/* Wait for transmission to complete */
	}

	/* Disable SPI */
	*(ctx->base + spi_cr1) &= ~(1 << 9);
	dataBarier();
	*(ctx->base + spi_cr1) &= ~1;
	dataBarier();

	return bufflen;
}


int libspi_configure(libspi_ctx_t *ctx, char mode, char bdiv, int enable)
{
	uint32_t v;

	devClk(spiInfo[ctx->spiNum - spi1].dev, 1);
	*(ctx->base + spi_cr1) &= ~1;
	dataBarier();

	uint32_t bdiv_bits = ((((uint32_t)bdiv) & 0x7) << 28);
	uint32_t dma_bits = (ctx->per != NULL) ? (0x3 << 14) : 0; /* Enable TX and RX DMA */
	uint32_t fifoThr_bits = ((spiInfo[ctx->spiNum - spi1].fifoThr - 1) & 0xf) << 5;
	v = *(ctx->base + spi_cfg1);
	v &= ~0xf05fc3ff;
	v |=
			bdiv_bits |    /* Baud rate divider */
			(0 << 22) |    /* CRC calculation disabled */
			(0 << 16) |    /* CRC not used */
			dma_bits |     /* DMA mode */
			fifoThr_bits | /* FIFO threshold */
			(7 << 0);      /* Data size (8 bits) */
	*(ctx->base + spi_cfg1) = v;

	uint32_t mode_bits = ((((uint32_t)mode) & 0x3) << 24);
	v = *(ctx->base + spi_cfg2);
	v &= ~0xf7fee0ff;
	v |=
			(1 << 31) | /* SPI peripheral controls GPIOs when disabled */
			(1 << 29) | /* SS output enable */
			(0 << 28) | /* SS active low */
			mode_bits | /* SPI polarity and phase */
			(0 << 23) | /* MSB first */
			(1 << 22) | /* Master mode */
			(0 << 19) | /* SPI Motorola */
			(0 << 17) | /* Full duplex */
			(0 << 15) | /* No I/O swap */
			(0 << 13) | /* RDY signal not used */
			(0 << 4) |  /* No inter-data delay */
			(0 << 0);   /* No SS delay */
	*(ctx->base + spi_cfg2) = v;

	*(ctx->base + spi_i2scfgr) &= ~1; /* Disable I2S mode */

	if (enable == 0) {
		devClk(spiInfo[ctx->spiNum - spi1].dev, 0);
	}

	return EOK;
}


int libspi_init(libspi_ctx_t *ctx, unsigned int spi, int useDma)
{
	if ((spi < spi1) || (spi > MAX_SPI) || (ctx == NULL)) {
		return -1;
	}

	ctx->spiNum = spi;
	ctx->base = (void *)spiInfo[spi - spi1].base;
	if (rcc_setClksel(spiInfo[spi - spi1].clksel, spiInfo[spi - spi1].clksrc) < 0) {
		return -1;
	}

	if (useDma != 0) {
		/* TODO: support DMA transfers */
		return -1;
	}
	else {
		ctx->per = NULL;
	}

	return libspi_configure(ctx, 0, 0, 1);
}
