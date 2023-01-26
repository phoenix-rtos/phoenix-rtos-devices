/*
 * Phoenix-RTOS
 *
 * Driver for ms5611 (barometer & temperature sensor)
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
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

#include "../sensors.h"

/* MS5611 COMMANDS */

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

/* MS5611 DELAYS */

/* OSR dependent minimum wait (microseconds) for conversion. Datasheet values ceil-ed to full milliseconds */
#define OSR_256_SLEEP  1000
#define OSR_512_SLEEP  2000
#define OSR_1024_SLEEP 3000
#define OSR_2048_SLEEP 5000
#define OSR_4096_SLEEP 10000

/* Device reset duration */
#define RESET_SLEEP  3000 /* Datasheet says 2800. It seems some data may be lost with such sleep */
#define HW_ERROR_REP 10


typedef struct {
	spimsg_ctx_t spiCtx;
	oid_t spiSS;
	sensor_event_t evtBaro;
	uint16_t prom[PROM_SIZE]; /* Prom memory: [ reserved, C1, C2, C3, C4, C5, C6, CRC ] */
	char stack[512] __attribute__((aligned(8)));
} ms5611_ctx_t;


/* Based on sample code in AN520 application note for MS56xx/MS57xx/MS58xx pressure sensor by Measurement Specialties */
uint8_t ms5611_crc4(const uint16_t *prom)
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


/* Sends conversion request `convCmd`, waits `usDelat` for conversion and reads ADC to `res` */
static int ms5611_measure(ms5611_ctx_t *ctx, uint8_t convCmd, time_t usDelay, uint32_t *res)
{
	static const uint8_t readCmd = CMD_READ_ADC;
	uint32_t tmp;
	uint8_t val[3];

	if (sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &convCmd, sizeof(convCmd), NULL, 0, 0) < 0) {
		fprintf(stderr, "ms5611: failed conv. request\n");
		return -1;
	}

	usleep(usDelay);

	if (sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &readCmd, sizeof(readCmd), val, sizeof(val), sizeof(readCmd)) < 0) {
		fprintf(stderr, "ms5611: adc read failed\n");
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


static void ms5611_publishthr(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	ms5611_ctx_t *ctx = info->ctx;
	int32_t temp, press, cnt = 0;
	time_t now;

	/* breaking naming convention on purpose to match datasheet notation 1:1 */
	uint32_t D1 = 0, D2 = 0;
	int64_t dT;
	int64_t off, sens;

	/* Initialize temperature with too big delay for certain response */
	while (ms5611_measure(ctx, (CMD_CVRT_TEMP | CVRT_OSR_4096), OSR_4096_SLEEP * 2, &D2) < 0) {
		usleep(10000);
		if (++cnt >= HW_ERROR_REP) {
			cnt = 0;
			fprintf(stderr, "ms5611: temp. init error\n");
		}
	}
	cnt = 0;

	/* Translate temperature */
	dT = (int32_t)D2 - (((int32_t)ctx->prom[5]) << 8);  /* dT = D2 - C5 * 2^8 */
	temp = 2000 + ((dT * (int64_t)ctx->prom[6]) >> 23); /* TEMP = 2000 + dT * C6 / 2^23 */

	while (1) {
		/* Read pressure */
		if (ms5611_measure(ctx, (CMD_CVRT_PRESS | CVRT_OSR_4096), OSR_4096_SLEEP, &D1) < 0) {
			usleep(1000);
			if (++cnt >= HW_ERROR_REP) {
				cnt = 0;
				fprintf(stderr, "ms5611: pressure read errors\n");
			}
			continue;
		}

		gettime(&now, NULL);

		/* Read temperature. Does not repeat failed read - fall back to old temperature */
		if (ms5611_measure(ctx, (CMD_CVRT_TEMP | CVRT_OSR_1024), OSR_1024_SLEEP, &D2) == 0) {
			/* Translate temperature */
			dT = (int32_t)D2 - (((int32_t)ctx->prom[5]) << 8);  /* dT = D2 - C5 * 2^8 */
			temp = 2000 + ((dT * (int64_t)ctx->prom[6]) >> 23); /* TEMP = 2000 + dT * C6 / 2^23 */
		}

		/* Translate press */
		off = ((int64_t)ctx->prom[2] << 16) + ((dT * (int64_t)ctx->prom[4]) >> 7);  /* OFF = C2 * 2^16 + (C4 * dT) / 2^7 */
		sens = ((int64_t)ctx->prom[1] << 15) + ((dT * (int64_t)ctx->prom[3]) >> 8); /* SENS = C1 * 2^15 + (C4 * dT) / 2^8 */
		press = ((((int64_t)D1 * sens) >> 21) - off) >> 15;                         /* P = ((D1 * SENS) / 2^21 - OFF) / 2^15 */

		/* MS5611 operating ranges: temperature from -40C to +80C, and pressure: 45000Pa, 110000 Pa */
		if (press > 45000 && press < 120000 && temp > -4000 && temp < 8500) {
			ctx->evtBaro.timestamp = now;
			ctx->evtBaro.baro.pressure = press;
			ctx->evtBaro.baro.temp = (temp + 27315 + 50) / 100;

			sensors_publish(info->id, &ctx->evtBaro);
		}
	}

	return;
}


static int ms5611_hwSetup(ms5611_ctx_t *ctx)
{
	uint8_t cmd, i;

	/* Reset ms5611 sequence */
	cmd = CMD_RESET;
	if (sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &cmd, sizeof(cmd), NULL, 0, 0) < 0) {
		fprintf(stderr, "ms5611: failed to reset device\n");
		return -1;
	}
	usleep(RESET_SLEEP);

	/* PROM reading */
	for (i = 0; i < PROM_SIZE; i++) {
		cmd = CMD_READ_PROM | (i << 1); /* LSB of prom address is always 0 */
		if (sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &cmd, sizeof(cmd), &ctx->prom[i], sizeof(ctx->prom[i]), sizeof(cmd)) < 0) {
			fprintf(stderr, "ms5611: failed read PROM C%i\n", i + 1);
			return -1;
		}
		ctx->prom[i] = (ctx->prom[i] << 8) | (ctx->prom[i] >> 8); /* swap higher and lower bytes */
	}

	/* Check CRC of PROM (lower nibble of last prom byte) with calculated CRC */
	if (ms5611_crc4(ctx->prom) != (ctx->prom[PROM_SIZE - 1] & 0x000f)) {
		fprintf(stderr, "ms5611: wrong PROM crc\n");
		return -1;
	}

	return 0;
}


static int ms5611_start(sensor_info_t *info)
{
	int err;
	ms5611_ctx_t *ctx = (ms5611_ctx_t *)info->ctx;

	err = beginthread(ms5611_publishthr, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("ms5611: launched sensor\n");
	}

	return err;
}


static int ms5611_alloc(sensor_info_t *info, const char *args)
{
	ms5611_ctx_t *ctx;
	char *ss;
	int err;

	/* sensor context allocation */
	ctx = malloc(sizeof(ms5611_ctx_t));
	if (ctx == NULL) {
		return -ENOMEM;
	}

	ctx->evtBaro.type = SENSOR_TYPE_BARO;
	ctx->evtBaro.baro.devId = info->id;

	/* filling sensor info structure */
	info->ctx = ctx;
	info->types = SENSOR_TYPE_BARO;

	ctx->spiCtx.mode = SPI_MODE3;
	ctx->spiCtx.speed = 10000000;

	ss = strchr(args, ':');
	if (ss != NULL) {
		*(ss++) = '\0';
	}

	/* initialize SPI device communication */
	err = sensorsspi_open(args, ss, &ctx->spiCtx.oid, &ctx->spiSS);
	if (err < 0) {
		printf("ms5611: Can`t initialize SPI device\n");
		free(ctx);
		return err;
	}

	/* hardware setup of Barometer */
	if (ms5611_hwSetup(ctx) < 0) {
		printf("ms5611: failed to setup device\n");
		free(ctx);
		return -1;
	}

	return EOK;
}


void __attribute__((constructor)) ms5611_register(void)
{
	static sensor_drv_t sensor = {
		.name = "ms5611",
		.alloc = ms5611_alloc,
		.start = ms5611_start
	};

	sensors_register(&sensor);
}
