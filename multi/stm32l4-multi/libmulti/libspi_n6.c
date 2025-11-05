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

#define SR_TXC    (1 << 12) /* Transaction complete */
#define SR_SUSP   (1 << 11) /* Master mode suspended */
#define SR_OVR    (1 << 6)  /* RX FIFO overrun */
#define SR_TXP    (1 << 1)  /* TX FIFO free space at or above threshold */
#define SR_RXP    (1 << 0)  /* RX FIFO above threshold */
#define SR_RXWNE  (1 << 15) /* RX FIFO contains at least 4 bytes */
#define SR_RXPLVL (3 << 13) /* Number of frames in RX FIFO (if < 4 bytes) */

#define SR_RXNE (SR_RXWNE | SR_RXPLVL) /* If any bit set, RX FIFO not empty */


static const struct libspi_peripheralInfo {
	uintptr_t base;
	unsigned int dev;
	enum ipclks clksel;    /* Clock selector */
	enum clock_ids clksrc; /* ID of source clock */
	uint32_t fifoSize;
} spiInfo[MAX_SPI - spi1 + 1] = {
	{ (uintptr_t)SPI1_BASE, pctl_spi1, pctl_ipclk_spi1sel, clkid_per, 16 },
	{ (uintptr_t)SPI2_BASE, pctl_spi2, pctl_ipclk_spi2sel, clkid_per, 16 },
	{ (uintptr_t)SPI3_BASE, pctl_spi3, pctl_ipclk_spi3sel, clkid_per, 16 },
	{ (uintptr_t)SPI4_BASE, pctl_spi4, pctl_ipclk_spi4sel, clkid_per, 8 },
	{ (uintptr_t)SPI5_BASE, pctl_spi5, pctl_ipclk_spi5sel, clkid_per, 8 },
	{ (uintptr_t)SPI6_BASE, pctl_spi6, pctl_ipclk_spi6sel, clkid_per, 16 },
};


static void libspi_suspendController(libspi_ctx_t *ctx)
{
	dataBarier();
	if ((*(ctx->base + spi_sr) & SR_TXC) != 0) {
		/* Controller is already stopped */
		return;
	}

	*(ctx->base + spi_cr1) |= 1 << 10; /* Set CSUSP bit */
	dataBarier();
	/* This should return quickly, if there were no errors all data has been transferred by now */
	while ((*(ctx->base + spi_sr) & SR_SUSP) == 0) {
		/* Wait for SUSP flag */
	}

	*(ctx->base + spi_ifcr) = SR_SUSP;
}


static int libspi_transactionDMA(libspi_ctx_t *ctx, size_t ignoreBytes, unsigned char *ibuff, const unsigned char *obuff, size_t bufflen)
{
	int ret;
	*(ctx->base + spi_cr1) |= 1 << 9;
	ret = libdma_transfer(ctx->per, ibuff, obuff, bufflen);

	libspi_suspendController(ctx);
	*(ctx->base + spi_cr1) &= ~1;
	dataBarier();
	return ret;
}


static inline uint8_t libspi_getByteOrZero(const uint8_t *buff, size_t i)
{
	return (buff == NULL) ? 0 : buff[i];
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
	volatile uint8_t *rxData = (volatile uint8_t *)(ctx->base + spi_rxdr);

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

	/* TODO: to support transactions with preamble, multi-buffer DMA transactions will be necessary */
	if ((ctx->per != NULL) && (ignoreBytes == 0)) {
		return libspi_transactionDMA(ctx, ignoreBytes, ibuff, obuff, bufflen);
	}

	uint32_t fifoSize = spiInfo[ctx->spiNum - spi1].fifoSize;
	size_t transactionLength = bufflen + ignoreBytes;
	size_t rx_i = 0;
	size_t tx_i = 0;
	bool overflow = false;

	/* Fill up TX FIFO to the limit */
	while (((tx_i + ignoreBytes) < fifoSize) && (tx_i < bufflen)) {
		*txData = libspi_getByteOrZero(obuff, tx_i);
		tx_i++;
	}

	/* Start transaction */
	*(ctx->base + spi_cr1) |= 1 << 9;
	while (rx_i < transactionLength) {
		/* We ignore the hardware TXP flag because when FIFO threshold is set to 1 byte it's misleading.
		 * If we followed it, we could put enough data into TX FIFO that we would overflow RX FIFO.
		 * Instead we simply put 1 byte into TX FIFO for every byte we pop from RX FIFO. */
		uint32_t status = *(ctx->base + spi_sr);
		if ((status & SR_OVR) != 0) {
			overflow = true;
			break;
		}

		if ((status & SR_RXNE) != 0) {
			if ((ibuff != NULL) && (rx_i >= ignoreBytes)) {
				ibuff[rx_i - ignoreBytes] = *rxData;
			}
			else {
				(void)*rxData;
			}

			rx_i++;
			if (tx_i < bufflen) {
				*txData = libspi_getByteOrZero(obuff, tx_i);
				tx_i++;
			}
		}
	}

	if (overflow) {
		libspi_suspendController(ctx);
		*(ctx->base + spi_cr1) &= ~1;
		dataBarier();
		return -EIO;
	}

	while ((*(ctx->base + spi_sr) & SR_TXC) == 0) {
		/* Wait for transmission to complete */
	}

	/* Disable SPI */
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

	uint32_t bdiv_bits;
	if (bdiv == spi_bdiv_1) {
		bdiv_bits = (1u << 31);
	}
	else if (bdiv <= spi_bdiv_256) {
		bdiv_bits = ((((uint32_t)bdiv) & 0x7) << 28);
	}
	else {
		return -EINVAL;
	}

	uint32_t dma_bits = (ctx->per != NULL) ? (0x3 << 14) : 0; /* Enable TX and RX DMA */
	uint32_t fifoThr_bits;
	if ((ctx->per == NULL) && (spiInfo[ctx->spiNum - spi1].fifoSize >= 2)) {
		fifoThr_bits = (((spiInfo[ctx->spiNum - spi1].fifoSize / 2) - 1) & 0xf) << 5;
	}
	else {
		/* TODO: when DMA is used, notify DMA after every byte.
		 * This results in worse performance at high baud rates, but otherwise RX DMA may never be requested
		 * for final bytes. A solution to that would be to set TSIZE in SPI_CR2, but that would limit transactions
		 * to 0xFFFE bytes for full-featured instances or 0x3FE bytes for limited instances. */
		fifoThr_bits = 0;
	}

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
		libdma_init();
		int err = libdma_acquirePeripheral(dma_spi, spi - spi1, &ctx->per);
		if (err < 0) {
			return err;
		}

		libdma_configurePeripheral(ctx->per, dma_mem2per, dma_priorityMedium, (void *)(ctx->base + spi_txdr), 0x0, 0x0, 0x1, 0x0, NULL);
		libdma_configurePeripheral(ctx->per, dma_per2mem, dma_priorityMedium, (void *)(ctx->base + spi_rxdr), 0x0, 0x0, 0x1, 0x0, NULL);
	}
	else {
		ctx->per = NULL;
	}

	return libspi_configure(ctx, 0, 0, 1);
}
