/*
 * Phoenix-RTOS
 *
 * Simsensor for baro sensors. Used in potential tests
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>

#include "../simsensor_common/event_queue.h"
#include "../simsensor_common/simsensor_reader.h"
#include "../sensors.h"

#define DEFAULT_BARO_SIM_QLEN     10
#define DEFAULT_BARO_TIME_HORIZON 100000

typedef struct {
	event_queue_t eventQueue;
	simsens_reader_t reader;
	char stack[512] __attribute__((aligned(8)));
} barosim_ctx_t;


static void barosim_freeCtx(barosim_ctx_t *ctx)
{
	eventQueue_free(&ctx->eventQueue);
	reader_close(&ctx->reader);
	free(ctx);
}


static void barosim_publishthr(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	barosim_ctx_t *ctx = info->ctx;
	sensor_event_t event;
	time_t now;

	while (1) {
		if (reader_read(&ctx->reader, &ctx->eventQueue) != 0) {
			fprintf(stderr, "baro_sim: error during file reading");
			barosim_freeCtx(ctx);
			return;
		}

		while (eventQueue_dequeue(&ctx->eventQueue, &event) == 0) {
			if (event.type == READER_STOP_EVENT) {
				barosim_freeCtx(ctx);
				return;
			}

			gettime(&now, NULL);

			if (event.timestamp > now) {
				usleep(event.timestamp - now);
			}

			event.baro.devId = info->id;
			sensors_publish(info->id, &event);
		}
	}

	return;
}


static int barosim_start(sensor_info_t *info)
{
	int err;
	barosim_ctx_t *ctx = (barosim_ctx_t *)info->ctx;

	err = beginthread(barosim_publishthr, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("baro_sim: launched simsensor\n");
	}

	return err;
}


static int barosim_alloc(sensor_info_t *info, const char *args)
{
	barosim_ctx_t *ctx;

	ctx = malloc(sizeof(barosim_ctx_t));
	if (ctx == NULL) {
		return -ENOMEM;
	}


	if (eventQueue_init(&ctx->eventQueue, DEFAULT_BARO_SIM_QLEN) != 0) {
		free(ctx);
		return -1;
	}

	if (reader_open(&ctx->reader, args, SENSOR_TYPE_BARO, DEFAULT_BARO_TIME_HORIZON) != 0) {
		free(ctx);
		eventQueue_free(&ctx->eventQueue);
		return -1;
	}

	info->ctx = ctx;
	info->types = SENSOR_TYPE_BARO;

	return 0;
}


void __attribute__((constructor)) barosim_register(void)
{
	static sensor_drv_t sensor = {
		.name = "baro_sim",
		.alloc = barosim_alloc,
		.start = barosim_start
	};

	sensors_register(&sensor);
}
