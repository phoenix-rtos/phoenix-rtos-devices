/*
 * Phoenix-RTOS
 *
 * Driver for GPS module PA6H
 *
 * Copyright 2022 Phoenix Systems
 * Author: Damian Loewnau, Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>

#include "../sensors.h"
#include "nmea.h"

#define PA6H_STR "pa6h:"

/* Specified by the module's documentation */
#define UPDATE_RATE_MS 1000

#define DEG2RAD     0.0174532925
#define DEG2MILIRAD 17.4532925 /* 1 degree -> 1 milliradian conversion parameter */
#define KMH2MMS     277.7778   /* 1 km/h -> mm/s conversion parameter */


typedef struct {
	sensor_event_t evtGps;
	int filedes;
	char buff[1024];
	char stack[512] __attribute__((aligned(8)));
} pa6h_ctx_t;


int pa6h_update(nmea_t *message, pa6h_ctx_t *ctx)
{
	switch (message->type) {
		case nmea_gga:
			ctx->evtGps.gps.lat = message->msg.gga.lat * 1e7;
			ctx->evtGps.gps.lon = message->msg.gga.lon * 1e7;
			ctx->evtGps.gps.hdop = (unsigned int)(message->msg.gga.hdop * 1e2);
			ctx->evtGps.gps.fix = message->msg.gga.fix;
			ctx->evtGps.gps.alt = message->msg.gga.h_asl * 1e3;
			ctx->evtGps.gps.altEllipsoid = message->msg.gga.h_wgs * 1e3;
			ctx->evtGps.gps.satsNb = message->msg.gga.sats;
			break;

		case nmea_gsa:
			ctx->evtGps.gps.hdop = (unsigned int)(message->msg.gsa.hdop * 1e2);
			ctx->evtGps.gps.vdop = (unsigned int)(message->msg.gsa.vdop * 1e2);
			break;

		case nmea_rmc:
			break;

		case nmea_vtg:
			ctx->evtGps.gps.heading = message->msg.vtg.track * DEG2MILIRAD;     /* degrees -> milliradians */
			ctx->evtGps.gps.groundSpeed = message->msg.vtg.speed_kmh * KMH2MMS; /* kmh->mm/s */
			ctx->evtGps.gps.velNorth = cos(message->msg.vtg.track * DEG2RAD) * ctx->evtGps.gps.groundSpeed;
			ctx->evtGps.gps.velEast = sin(message->msg.vtg.track * DEG2RAD) * ctx->evtGps.gps.groundSpeed;
			break;

		default:
			break;
	}
	gettime(&(ctx->evtGps.timestamp), NULL);

	return 0;
}


static void pa6h_threadPublish(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	pa6h_ctx_t *ctx = info->ctx;
	nmea_t message;
	char **lines;
	int i, nlines = 0, nbytes = 0, ret = 0;
	unsigned char doUpdate = 0;

	while (1) {
		memset(ctx->buff, 0, sizeof(ctx->buff));
		usleep(1000 * UPDATE_RATE_MS);

		nbytes = read(ctx->filedes, ctx->buff, sizeof(ctx->buff) - 1);
		if (nbytes <= 0) {
			continue;
		}

		nlines = nmea_countlines(ctx->buff);
		lines = nmea_getlines(ctx->buff, nlines);
		if (lines == NULL) {
			continue;
		}

		for (i = 0; i < nlines; i++) {
			if (nmea_assertChecksum(lines[i]) == EOK) {
				ret = nmea_interpreter(lines[i], &message);
				if (ret != nmea_broken && ret != nmea_unknown) {
					pa6h_update(&message, ctx);
					doUpdate = 1;
				}
			}
		}
		if (doUpdate == 1) {
			sensors_publish(info->id, &ctx->evtGps);
			doUpdate = 0;
		}
	}
	free(lines);
}


static int pa6h_start(sensor_info_t *info)
{
	int err;
	pa6h_ctx_t *ctx = info->ctx;

	ctx->evtGps.type = SENSOR_TYPE_GPS;
	ctx->evtGps.gps.devId = info->id;

	err = beginthread(pa6h_threadPublish, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err < 0) {
		free(ctx);
	}
	else {
		printf("%s launched sensor\n", PA6H_STR);
	}

	return err;
}


static int pa6h_parse(const char *args, const char **path)
{
	int err = EOK;

	if (!(args == NULL || strchr(args, ':'))) {
		*path = args;
	}
	else {
		fprintf(
			stderr,
			"%s Wrong arguments\n"
			"Please specify the path to source device instance, for example: /dev/uart0\n",
			PA6H_STR);
		err = -EINVAL;
	}

	return err;
}


static int pa6h_alloc(sensor_info_t *info, const char *args)
{
	int cnt = 0, err;
	const char *path;
	pa6h_ctx_t *ctx;

	ctx = malloc(sizeof(pa6h_ctx_t));
	if (ctx == NULL) {
		return -ENOMEM;
	}

	info->types = SENSOR_TYPE_GPS;

	err = pa6h_parse(args, &path);
	if (err != EOK) {
		free(ctx);
		return err;
	}

	ctx->filedes = open(path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
	while (ctx->filedes < 0) {
		usleep(10 * 1000);
		cnt++;

		if (cnt > 10000) {
				fprintf(stderr, "%s Can't open %s: %s\n", PA6H_STR, path, strerror(errno));
				err = -errno;
				free(ctx);
				return err;
			}
		ctx->filedes = open(path, O_RDONLY | O_NOCTTY | O_NONBLOCK);
	}
	info->ctx = ctx;

	return EOK;
}


void __attribute__((constructor)) pa6h_register(void)
{
	static sensor_drv_t sensor = {
		.name = "pa6h",
		.alloc = pa6h_alloc,
		.start = pa6h_start
	};

	sensors_register(&sensor);
}
