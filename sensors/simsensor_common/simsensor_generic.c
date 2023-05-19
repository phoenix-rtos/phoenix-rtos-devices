/*
 * Phoenix-RTOS
 *
 * Generic functions for all sensor simulators
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "simsensor_generic.h"

#include "event_queue.h"
#include "simsensor_reader.h"

#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <sys/threads.h>


typedef struct {
	const char *name;
	event_queue_t eventQueue;
	simsens_reader_t reader;
	char stack[2048] __attribute__((aligned(8)));
} simsens_ctx_t;


static void simsens_publishthr(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	simsens_ctx_t *ctx = info->ctx;
	sensor_event_t event;
	time_t now;
	bool run = true;

	while (run) {
		if (reader_read(&ctx->reader, &ctx->eventQueue) != 0) {
			fprintf(stderr, "simsensor %s: error during file reading", ctx->name);

			endthread();
		}

		/* If event queue is empty, then we know, that there is not new event for at least timHorizon time span. */
		if (eventQueue_empty(&ctx->eventQueue) == true) {
			/* Sleeping fraction of free time to give time for other operations eg. reading the scenario file */
			usleep((ctx->reader.timeHorizon * 3) / 4);
			continue;
		}

		while (eventQueue_dequeue(&ctx->eventQueue, &event) == 0) {
			switch (event.type) {
				case READER_STOP_EVENT:
					run = false;
					continue;
				case SENSOR_TYPE_ACCEL:
					event.accels.devId = info->id;
					break;
				case SENSOR_TYPE_BARO:
					event.baro.devId = info->id;
					break;
				case SENSOR_TYPE_GPS:
					event.gps.devId = info->id;
					break;
				case SENSOR_TYPE_GYRO:
					event.gyro.devId = info->id;
					break;
				case SENSOR_TYPE_MAG:
					event.mag.devId = info->id;
					break;
				case SENSOR_TYPE_TEMP:
					event.temp.devId = info->id;
					break;
				default:
					fprintf(stderr, "simsensor %s: unknown sensor type.\n", ctx->name);
					break;
			}

			gettime(&now, NULL);

			if (event.timestamp > now) {
				usleep(event.timestamp - now);
			}

			sensors_publish(info->id, &event);
		}
	}

	endthread();
}


int simsens_start(sensor_info_t *info)
{
	int err;
	simsens_ctx_t *ctx = (simsens_ctx_t *)info->ctx;

	err = beginthread(simsens_publishthr, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("Simsensor: launched %s\n", ctx->name);
	}

	return err;
}


int simsens_alloc(sensor_info_t *info, const char *args, const simsens_settings_t *settings)
{
	simsens_ctx_t *ctx;

	ctx = malloc(sizeof(simsens_ctx_t));
	if (ctx == NULL) {
		fprintf(stderr, "simsensor %s: malloc error\n", settings->name);
		return -ENOMEM;
	}

	ctx->name = settings->name;

	if (eventQueue_init(&ctx->eventQueue, settings->messageQueueLen) != 0) {
		fprintf(stderr, "simsensor %s: event queue init failure\n", settings->name);
		free(ctx);
		return -1;
	}

	if (reader_open(&ctx->reader, args, settings->sensorTypes, settings->timeHorizon) != 0) {
		fprintf(stderr, "simsensor %s: cannot open reader\n", settings->name);
		eventQueue_free(&ctx->eventQueue);
		free(ctx);
		return -1;
	}

	info->ctx = ctx;
	info->types = settings->sensorTypes;

	return 0;
}
