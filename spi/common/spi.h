/*
 * Phoenix-RTOS
 *
 * SPI interface
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _PHOENIX_SPI_H_
#define _PHOENIX_SPI_H_

#include <stddef.h>


/* SPI clock mode (phase and polarity) */
#define SPI_CPHA  (1 << 0)
#define SPI_CPOL  (1 << 1)
#define SPI_MODE0 (0 | 0)
#define SPI_MODE1 (0 | SPI_CPHA)
#define SPI_MODE2 (SPI_CPOL | 0)
#define SPI_MODE3 (SPI_CPOL | SPI_CPHA)

/* SPI external slave select line. Passed to spi_xfer() for manual SS control */
#define SPI_SS_EXTERNAL ((unsigned int)-1)


/* Performs SPI transaction with SPI slave given by ss */
extern int spi_xfer(unsigned int dev, unsigned int ss, const void *out, size_t olen, void *in, size_t ilen, size_t iskip);


/* Returns SPI clock speed (in Hz) */
extern int spi_getSpeed(unsigned int dev, unsigned int *speed);


/* Sets SPI clock speed (in Hz) */
extern int spi_setSpeed(unsigned int dev, unsigned int speed);


/* Returns SPI clock mode */
extern int spi_getMode(unsigned int dev, unsigned char *mode);


/* Sets SPI clock mode */
extern int spi_setMode(unsigned int dev, unsigned char mode);


/* Initializes SPI controller */
extern int spi_init(unsigned int dev);


#endif
