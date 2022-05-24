/*
 * Phoenix-RTOS
 *
 * Driver for IMU lps25xx (accelerometer, gyroscope)
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>

#include "../sensors.h"


typedef struct {
	sensor_event_t evBaro;
	char stack[512] __attribute__((aligned(8)));
} lps25xx_ctx_t;


static inline time_t lps25xx_timeMsGet(void)
{
	time_t now;
	gettime(&now, NULL);

	return now / 1000;
}


static void lps25xx_publishthr(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	lps25xx_ctx_t *ctx = info->ctx;

	while (1) {
		sleep(1);
		/* TODO: set interval and publish data to sensor manager */

		/* Sample data */
		ctx->evBaro.baro.pressure = 8574834;
		ctx->evBaro.timestamp = lps25xx_timeMsGet();
		sensors_publish(info->id, &ctx->evBaro);
	}
}


static int lps25xx_start(sensor_info_t *info)
{
	int err;
	lps25xx_ctx_t *ctx = malloc(sizeof(lps25xx_ctx_t));

	if (ctx == NULL) {
		return -ENOMEM;
	}

	ctx->evBaro.type = SENSOR_TYPE_BARO;
	ctx->evBaro.baro.devId = info->id;

	info->ctx = ctx;

	err = beginthread(lps25xx_publishthr, 4, ctx->stack, sizeof(ctx->stack), info);
	if (err < 0) {
		free(ctx);
	}
	else {
		printf("lps25xx: launched sensor\n");
	}

	return err;
}


static int lps25xx_alloc(sensor_info_t *info, const char *args)
{
	info->types = SENSOR_TYPE_BARO;

	/* TODO: parse arguments and initialize device */

	return EOK;
}


void __attribute__((constructor)) lps25xx_register(void)
{
	static sensor_drv_t sensor = {
		.name = "lps25xx",
		.alloc = lps25xx_alloc,
		.start = lps25xx_start
	};

	sensors_register(&sensor);
}
