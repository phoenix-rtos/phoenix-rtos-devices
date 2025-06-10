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

#include <libsensors/sensor.h>

#include "common.h"


static void ms5611_publishthr(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	ms5xxx_ctx_t *ctx = info->ctx;
	int32_t temp, press, cnt = 0;
	time_t now;

	/* breaking naming convention on purpose to match datasheet notation 1:1 */
	uint32_t D1 = 0, D2 = 0;
	int64_t dT;
	int64_t off, sens;

	/* Initialize temperature with too big delay for certain response */
	while (ms5xxx_measure(ctx, (CMD_CVRT_TEMP | CVRT_OSR_4096), OSR_4096_SLEEP * 2, &D2) < 0) {
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
		if (ms5xxx_measure(ctx, (CMD_CVRT_PRESS | CVRT_OSR_4096), OSR_4096_SLEEP, &D1) < 0) {
			usleep(1000);
			if (++cnt >= HW_ERROR_REP) {
				cnt = 0;
				fprintf(stderr, "ms5611: pressure read errors\n");
			}
			continue;
		}

		gettime(&now, NULL);

		/* Read temperature. Does not repeat failed read - fall back to old temperature */
		if (ms5xxx_measure(ctx, (CMD_CVRT_TEMP | CVRT_OSR_1024), OSR_1024_SLEEP, &D2) == 0) {
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
			ctx->evt.timestamp = now;
			ctx->evt.baro.pressure = press;
			ctx->evt.baro.temp = (temp + 27315 + 50) / 100;

			sensors_publish(info->id, &ctx->evt);
		}
	}

	return;
}


static int ms5611_start(sensor_info_t *info)
{
	int err;
	ms5xxx_ctx_t *ctx = (ms5xxx_ctx_t *)info->ctx;

	err = beginthread(ms5611_publishthr, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("ms5611: launched sensor\n");
	}

	return err;
}


static int ms5611_alloc(sensor_info_t *info, const char *args)
{
	ms5xxx_ctx_t *ctx;
	char *ss;
	int err;

	/* sensor context allocation */
	ctx = malloc(sizeof(ms5xxx_ctx_t));
	if (ctx == NULL) {
		return -ENOMEM;
	}

	ctx->evt.type = SENSOR_TYPE_BARO;
	ctx->evt.baro.devId = info->id;
	ctx->ms5Type = 611;

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
	if (ms5xxx_hwSetup(ctx) < 0) {
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
