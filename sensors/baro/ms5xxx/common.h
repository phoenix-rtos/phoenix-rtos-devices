/*
 * Phoenix-RTOS
 *
 * Common part of TE Connectivity MS5xxx series barometric sensors.
 *
 * Copyright 2025 Phoenix Systems
 * Author: Grzegorz Mazurczak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _MS5XXX_COMMON_
#define _MS5XXX_COMMON_

#include <stdlib.h>
#include <sensors-spi.h>

#include <libsensors/sensor.h>

/* MS5XXX COMMANDS */

/* Reset command */
#define CMD_RESET 0x1e

/* Conversion request command and oversampling (OSR) rate flags */
#define CMD_CVRT_PRESS 0x40
#define CMD_CVRT_TEMP  0x50
#define CVRT_OSR_256   0x00
#define CVRT_OSR_512   0x02
#define CVRT_OSR_1024  0x04
#define CVRT_OSR_2048  0x06
#define CVRT_OSR_4096  0x08

/* Read ADC conversion result command */
#define CMD_READ_ADC 0x00

/* Read PROM command and PROM address specifiers */
#define CMD_READ_PROM 0xa0
#define PROM_SIZE     8

/* MS5XXX DELAYS */

/* OSR dependent minimum wait (microseconds) for conversion. Datasheet values ceil-ed to full milliseconds */
#define OSR_256_SLEEP  1000
#define OSR_512_SLEEP  2000
#define OSR_1024_SLEEP 3000
#define OSR_2048_SLEEP 5000
#define OSR_4096_SLEEP 10000

/* Device reset duration */
#define RESET_SLEEP  10000 /* Datasheet says 2800. It seems some data may be lost with such sleep */
#define HW_ERROR_REP 10


typedef struct {
	spimsg_ctx_t spiCtx;
	oid_t spiSS;
	sensor_event_t evt;
	unsigned int ms5Type;     /* integer containing ms5xxx type (the xxx digits) */
	uint16_t prom[PROM_SIZE]; /* Prom memory: [ reserved, C1, C2, C3, C4, C5, C6, CRC ] */
	char stack[1024] __attribute__((aligned(8)));
} ms5xxx_ctx_t;


/* Sends conversion request `convCmd`, waits `usDelat` for conversion and reads ADC to `res` */
int ms5xxx_measure(ms5xxx_ctx_t *ctx, uint8_t convCmd, time_t usDelay, uint32_t *res);

/* Prepare sensor before use, reset, read and validate PROM content. */
int ms5xxx_hwSetup(ms5xxx_ctx_t *ctx);

#endif
