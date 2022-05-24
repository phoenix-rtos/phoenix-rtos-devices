/*
 * Phoenix-RTOS
 *
 * Driver for IMU lsm9dsxx
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
	sensor_event_t evtAccel;
	sensor_event_t evtGyro;
	char stack[512] __attribute__((aligned(8)));
} lsm9dsxx_ctx_t;


static inline time_t lps25xx_timeMsGet(void)
{
	time_t now;
	gettime(&now, NULL);

	return now / 1000;
}


static void lsm9dsxx_threadPublish(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	lsm9dsxx_ctx_t *ctx = info->ctx;

	while (1) {
		sleep(1);
		/* TODO: set interval and publish data to sensor manager */

		/* Sample data */
		ctx->evtAccel.accels.accelX = 23;
		ctx->evtAccel.accels.accelY = 24;
		ctx->evtAccel.accels.accelZ = 25;
		ctx->evtAccel.timestamp = lps25xx_timeMsGet();
		sensors_publish(info->id, &ctx->evtAccel);

		ctx->evtGyro.gyro.gyroX = 15;
		ctx->evtGyro.gyro.gyroY = 16;
		ctx->evtGyro.gyro.gyroZ = 17;
		ctx->evtGyro.timestamp = lps25xx_timeMsGet();
		sensors_publish(info->id, &ctx->evtGyro);
	}
}


static int lsm9dsxx_start(sensor_info_t *info)
{
	int err;
	lsm9dsxx_ctx_t *ctx = malloc(sizeof(lsm9dsxx_ctx_t));

	if (ctx == NULL) {
		return -ENOMEM;
	}

	ctx->evtAccel.type = SENSOR_TYPE_ACCEL;
	ctx->evtAccel.accels.devId = info->id;

	ctx->evtGyro.type = SENSOR_TYPE_GYRO;
	ctx->evtGyro.gyro.devId = info->id;

	info->ctx = ctx;

	err = beginthread(lsm9dsxx_threadPublish, 4, ctx->stack, sizeof(ctx->stack), info);
	if (err < 0) {
		free(ctx);
	}
	else {
		printf("lsm9dsxx: launched sensor\n");
	}

	return err;
}


static int lsm9dsxx_alloc(sensor_info_t *info, const char *args)
{
	info->types = SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO;

	/* TODO: parse arguments and initialize device */

	return EOK;
}


void __attribute__((constructor)) lsm9dsxx_register(void)
{
	static sensor_drv_t sensor = {
		.name = "lsm9dsxx",
		.alloc = lsm9dsxx_alloc,
		.start = lsm9dsxx_start
	};

	sensors_register(&sensor);
}
