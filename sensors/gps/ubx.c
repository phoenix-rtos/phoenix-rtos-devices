/*
 * Phoenix-RTOS
 *
 * Driver for GPS module ubx
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
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>

#include "common.h"
#include "../sensors.h"
#include "nmea.h"
#include "ubx.h"

#define ubx_STR "ubx:"

/* Specified by the module's documentation */
#define UPDATE_RATE_MS 100

/* ubx has: pos_CEP = 3m, vel_CEP = 0.1m/s. We use RMS values which are 0.849 * CEP (https://en.wikipedia.org/wiki/Circular_error_probable) */
#define UBX_POS_ACCURACY 2.547f
#define UBX_VEL_ACCURACY 0.0849f

#define REC_BUF_SZ 1024
#define INBOX_SIZE 3


typedef struct {
	sensor_event_t evtGps;
	int filedes;
	gps_receiver_t receiver;
	char stack[2048] __attribute__((aligned(8)));
} ubx_ctx_t;


static void ubx_threadPublish(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	struct __errno_t errnoNew;
	ubx_ctx_t *ctx = info->ctx;
	nmea_t message;
	unsigned int i, inbocCap, update;

	/* Redirecting errno to keep backward compatibility (in case of errno not working correctly) */
	_errno_new(&errnoNew);

	while (1) {
		inbocCap = gps_recv(ctx->filedes, &ctx->receiver);

		update = 0;
		for (i = 0; i < inbocCap; i++) {
			nmea_interpreter(ctx->receiver.inbox[i].msg, &message);
			update |= gps_updateEvt(&message, &ctx->evtGps, UBX_POS_ACCURACY, UBX_VEL_ACCURACY);
		}

		if (update != 0) {
			sensors_publish(info->id, &ctx->evtGps);
		}
	}
}


static int ubx_start(sensor_info_t *info)
{
	int err;
	ubx_ctx_t *ctx = info->ctx;

	ctx->evtGps.type = SENSOR_TYPE_GPS;
	ctx->evtGps.gps.devId = info->id;

	err = beginthread(ubx_threadPublish, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err < 0) {
		free(ctx);
	}
	else {
		printf("%s launched sensor\n", ubx_STR);
	}

	return err;
}


static int ubx_parse(const char *args, const char **path)
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
			ubx_STR);
		err = -EINVAL;
	}

	return err;
}


static int ubx_alloc(sensor_info_t *info, const char *args)
{
	int cnt = 0, err = -1;
	const char *path;
	ubx_ctx_t *ctx;
	struct termios termBackup;

	ctx = malloc(sizeof(ubx_ctx_t));
	if (ctx == NULL) {
		return -ENOMEM;
	}

	info->types = SENSOR_TYPE_GPS;

	err = ubx_parse(args, &path);
	if (err != EOK) {
		free(ctx);
		return err;
	}

	/* Preparing receiver */
	err = gps_recvInit(&ctx->receiver, REC_BUF_SZ, INBOX_SIZE);
	if (err < 0) {
		free(ctx);
		return -1;
	}

	/* Opening serial device */
	do {
		ctx->filedes = open(path, O_RDWR | O_NOCTTY);

		if (ctx->filedes < 0) {
			usleep(10 * 1000);
			cnt++;

			if (cnt > 10000) {
				fprintf(stderr, "%s Can't open %s: %s\n", ubx_STR, path, strerror(errno));
				err = -errno;
				gps_recvDone(&ctx->receiver);
				free(ctx);
				return err;
			}
		}
	} while (ctx->filedes < 0);

	if (isatty(ctx->filedes) != 1) {
		fprintf(stderr, "%s %s not a tty\n", ubx_STR, path);
		close(ctx->filedes);
		gps_recvDone(&ctx->receiver);
		free(ctx);
		return -1;
	}

	/* First serial setup backups termios settings */
	if (gps_serialSetup(ctx->filedes, B9600, &termBackup) < 0) {
		fprintf(stderr, "%s cannot set baud %d\n", ubx_STR, 9600);
		close(ctx->filedes);
		gps_recvDone(&ctx->receiver);
		free(ctx);
		return -1;
	}
	usleep(100 * 1000);

	/* baudrate setting to 115200 */
	write(ctx->filedes, UBX_PREMADE_BAUD_115200, sizeof(UBX_PREMADE_BAUD_115200));
	usleep(100 * 1000);

	/* First serial setup backups termios settings */
	if (gps_serialSetup(ctx->filedes, B115200, NULL) < 0) {
		fprintf(stderr, "%s cannot set baud %d\n", ubx_STR, 115200);
		close(ctx->filedes);
		gps_recvDone(&ctx->receiver);
		free(ctx);
		return -1;
	}
	usleep(100 * 1000);

	/* output messages selection */
	write(ctx->filedes, UBX_PREMADE_RMC_OFF, sizeof(UBX_PREMADE_RMC_OFF));
	write(ctx->filedes, UBX_PREMADE_GSV_OFF, sizeof(UBX_PREMADE_GSV_OFF));
	write(ctx->filedes, UBX_PREMADE_GLL_OFF, sizeof(UBX_PREMADE_GLL_OFF));
	write(ctx->filedes, UBX_PREMADE_GSA_ON, sizeof(UBX_PREMADE_GSA_ON));
	write(ctx->filedes, UBX_PREMADE_GGA_ON, sizeof(UBX_PREMADE_GGA_ON));
	write(ctx->filedes, UBX_PREMADE_VTG_ON, sizeof(UBX_PREMADE_VTG_ON));

	/* set output rate to 10Hz */
	write(ctx->filedes, UBX_PREMADE_FIX_10HZ, sizeof(UBX_PREMADE_FIX_10HZ));
	usleep(100 * 1000);

	info->ctx = ctx;

	return EOK;
}


void __attribute__((constructor)) ubx_register(void)
{
	static sensor_drv_t sensor = {
		.name = "ubx",
		.alloc = ubx_alloc,
		.start = ubx_start
	};

	sensors_register(&sensor);
}
