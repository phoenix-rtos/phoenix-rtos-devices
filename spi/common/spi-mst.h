/*
 * Phoenix-RTOS
 *
 * SPI master interface
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef PHOENIX_SPI_MST_H_
#define PHOENIX_SPI_MST_H_


#include <stddef.h>
#include <stdint.h>


enum {
	spi_transfer_xfer = 0,
	spi_transfer_tx,
	spi_transfer_rx
};


typedef struct {
	uint8_t type;     /* spi_transfer_xfer/tx/rx */
	uint16_t delayUs; /* Delay after this descriptor (CS stays low) */
	uint32_t len;     /* Length of the associated buffer(s) */
} spi_transferDesc_t;


/* SPI context structure - implementation-defined */
typedef struct spi_ctx spi_ctx_t;


/* SPI endianness */
enum {
	spi_lsb = 0,
	spi_msb
};


/* SPI clock mode (phase and polarity) */
#define SPI_CPHA  (1U << 0)
#define SPI_CPOL  (1U << 1)
#define SPI_MODE0 (0U | 0U)
#define SPI_MODE1 (0U | SPI_CPHA)
#define SPI_MODE2 (SPI_CPOL | 0U)
#define SPI_MODE3 (SPI_CPOL | SPI_CPHA)

/* SPI external slave select line. Passed to spi_xfer() for manual SS control */
#define SPI_SS_EXTERNAL ((unsigned int)-1)


/* Performs SPI transaction with SPI slave given by ss */
int spi_transaction(spi_ctx_t *ctx, uint32_t ss, const spi_transferDesc_t *descriptors, size_t descCnt, const void *txBuf, void *rxBuf);


/* Returns SPI clock speed (in Hz) */
int spi_getSpeed(spi_ctx_t *ctx, uint32_t *speed);


/* Sets SPI clock speed (in Hz) */
int spi_setSpeed(spi_ctx_t *ctx, uint32_t speed);


/* Returns SPI clock mode */
int spi_getMode(spi_ctx_t *ctx, uint8_t *mode);


/* Sets SPI clock mode */
int spi_setMode(spi_ctx_t *ctx, uint8_t mode);


/* Returns SPI bit order */
int spi_getBitOrder(spi_ctx_t *ctx, uint8_t *bitOrder);


/* Sets SPI bit order (spi_lsb/msb) */
int spi_setBitOrder(spi_ctx_t *ctx, uint8_t bitOrder);


/* Initializes SPI controller */
int spi_init(unsigned int dev, spi_ctx_t *ctx);


#endif
