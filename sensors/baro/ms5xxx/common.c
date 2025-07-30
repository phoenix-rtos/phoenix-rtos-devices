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


#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>
#include <spi.h>
#include <sensors-spi.h>

#include <libsensors/sensor.h>

#include "common.h"


/* Based on sample code in AN520 application note for MS56xx/MS57xx/MS58xx pressure sensor by Measurement Specialties */
static uint8_t ms5xxx_crc4(const uint16_t *prom)
{
	uint16_t cnt, bit, word;
	uint32_t rem = 0; /* crc reminder */

	/* Operation is performed on bytes */
	for (cnt = 0; cnt < (PROM_SIZE * sizeof(prom[0])); cnt++) {
		/* The CRC code is calculated and written in factory with the LSB byte in the prom (prom[7]) set to 0x00 */
		if (cnt != (PROM_SIZE * 2 - 1)) {
			word = prom[cnt >> 1];

			/* High/low byte selection */
			if ((cnt & 1) == 1) {
				rem ^= word & 0x00ff;
			}
			else {
				rem ^= word >> 8;
			}
		}

		for (bit = 0; bit < 8; ++bit) {
			rem <<= 1;
			if ((rem & 0x10000) != 0) {
				rem ^= 0x3000;
			}
		}
	}

	return ((rem >> 12) & 0xf); /* final 4-bit reminder is CRC code */
}


int ms5xxx_measure(ms5xxx_ctx_t *ctx, uint8_t convCmd, time_t usDelay, uint32_t *res)
{
	uint8_t readCmd = CMD_READ_ADC;
	uint32_t tmp;
	uint8_t val[3];

	if (sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &convCmd, sizeof(convCmd), NULL, 0, 0) < 0) {
		fprintf(stderr, "ms5%u: failed conv. request\n", ctx->ms5Type);
		return -1;
	}

	usleep(usDelay);

	if (sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &readCmd, sizeof(readCmd), val, sizeof(val), sizeof(readCmd)) < 0) {
		fprintf(stderr, "ms5%u: adc read failed\n", ctx->ms5Type);
		return -1;
	}
	tmp = ((uint32_t)val[0] << 16) | ((uint32_t)val[1] << 8) | ((uint32_t)val[2]);

	/* If ADC is read too early then it returns 0 */
	if (tmp == 0) {
		return -1;
	}

	*res = tmp;

	return 0;
}


int ms5xxx_hwSetup(ms5xxx_ctx_t *ctx)
{
	uint8_t cmd, i;

	/* Reset ms5525 sequence */
	cmd = CMD_RESET;
	if (sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &cmd, sizeof(cmd), NULL, 0, 0) < 0) {
		fprintf(stderr, "ms5%u: failed to reset device\n", ctx->ms5Type);
		return -1;
	}
	usleep(RESET_SLEEP);

	/* PROM reading */
	for (i = 0; i < PROM_SIZE; i++) {
		cmd = CMD_READ_PROM | (i << 1); /* LSB of prom address is always 0 */
		if (sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &cmd, sizeof(cmd), &ctx->prom[i], sizeof(ctx->prom[i]), sizeof(cmd)) < 0) {
			fprintf(stderr, "ms5%u: failed read PROM C%i\n", ctx->ms5Type, i + 1);
			return -1;
		}
		ctx->prom[i] = (ctx->prom[i] << 8) | (ctx->prom[i] >> 8); /* swap higher and lower bytes */
	}

	/* Check CRC of PROM (lower nibble of last prom byte) with calculated CRC */
	if (ms5xxx_crc4(ctx->prom) != (ctx->prom[PROM_SIZE - 1] & 0x000f)) {
		fprintf(stderr, "ms5%u: wrong PROM crc\n", ctx->ms5Type);
		return -1;
	}

	return 0;
}
