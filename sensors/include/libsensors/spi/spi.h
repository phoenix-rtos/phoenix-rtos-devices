/*
 * Phoenix-RTOS
 *
 * Sensors SPI communication interface
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _SENSORS_SPI_H_
#define _SENSORS_SPI_H_

#include <spi.h>
#include <spi-msg.h>


/* Performs SPI transfer with a sensor */
extern int sensorsspi_xfer(const spimsg_ctx_t *ctx, oid_t *ss, const void *out, size_t olen, void *in, size_t ilen, size_t iskip);


/* Initializes SPI sensor device communication */
extern int sensorsspi_open(const char *devSPI, const char *devSS, oid_t *spi, oid_t *ss);


/* Initializes SPI sensors communication */
extern int sensorsspi_init(void);


#endif
