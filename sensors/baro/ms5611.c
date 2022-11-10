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

typedef struct {
	spimsg_ctx_t spiCtx;
	oid_t spiSS;
	sensor_event_t evtBaro;
	char stack[512] __attribute__((aligned(8)));
} ms5611_ctx_t;


static void ms5611_publishthr(void *data)
{
	/* TODO: data acquisition thread and calculations based on PROM coefficients */

	return;
}


static int ms5611_hwSetup(ms5611_ctx_t *ctx)
{
	/* TODO: reset the device, read PROM data into ctx */

	return -1;
}


static int ms5611_start(sensor_info_t *info)
{
	int err;
	ms5611_ctx_t *ctx = (ms5611_ctx_t *)info->ctx;

	err = beginthread(ms5611_publishthr, 4, ctx->stack, sizeof(ctx->stack), info);
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
