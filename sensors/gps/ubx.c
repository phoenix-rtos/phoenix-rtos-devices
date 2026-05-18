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
#include <poll.h>

#include <libsensors/sensor.h>
#include <libsensors/gps/receiver.h>
#include <libsensors/gps/nmea.h>

#include <board_config.h>

#include "ubx.h"

#define ubx_STR "ubx:"

/* Specified by the module's documentation */
#define UPDATE_RATE_MS 100

/* ubx has: pos_CEP = 3m, vel_CEP = 0.1m/s. We use RMS values which are 0.849 * CEP (https://en.wikipedia.org/wiki/Circular_error_probable) */
#define UBX_POS_ACCURACY 2.547f
#define UBX_VEL_ACCURACY 0.0849f

#define REC_BUF_SZ 512

#ifndef UBX_DEFAULT_BAUDRATE
#define UBX_DEFAULT_BAUDRATE 115200
#endif


typedef struct {
	sensor_event_t evtGps;
	int filedes;
	gps_receiver_t receiver;
	char stack[2048] __attribute__((aligned(8)));
	handle_t tid;
	volatile int run;
} ubx_ctx_t;


static void ubx_threadPublish(void *data)
{
	/* calculate time needed to fill 50 % of buffer to set it as read interval */
	static const unsigned int sleepPeriod = ((REC_BUF_SZ / 2) * 1e6) / (UBX_DEFAULT_BAUDRATE / 10);

	sensor_info_t *info = (sensor_info_t *)data;
	struct __errno_t errnoNew;
	ubx_ctx_t *ctx = info->ctx;
	const char *frameStr;
	nmea_t message = { 0 };
	int ret = 0;

	/* reset NMEA scanner context */
	memset(&ctx->receiver.pCtx, 0, sizeof(nmea_scanCtx_t));

	/* Redirecting errno to keep backward compatibility (in case of errno not working correctly) */
	_errno_new(&errnoNew);

	while (ctx->run) {
		ret = read(ctx->filedes, ctx->receiver.buf, ctx->receiver.bufSz);
		while ((ret > 0) && (nmea_scan(&ctx->receiver.pCtx, ctx->receiver.buf, ret, &frameStr) > 0)) {
			nmea_interpreter(frameStr, &message);
			if (gps_updateEvt(&message, &ctx->evtGps, UBX_POS_ACCURACY, UBX_VEL_ACCURACY) != 0) {
				sensors_publish(ctx->evtGps.gps.devId, &ctx->evtGps);
			}
		}

		if (ret == ctx->receiver.bufSz) {
			/* buffer was full, there might be more data to read already */
			continue;
		}

		/* sleep to wait for more data to arrive */
		usleep(sleepPeriod);
	}

	endthread();
}


static int ubx_start(sensor_info_t *info)
{
	int err;
	ubx_ctx_t *ctx = info->ctx;

	ctx->evtGps.type = SENSOR_TYPE_GPS;
	ctx->evtGps.gps.devId = info->id;
	ctx->run = 1;

	err = beginthreadex(ubx_threadPublish, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info, &ctx->tid);
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
	static const speed_t baudTable[] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };

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
	err = gps_recvInit(&ctx->receiver, REC_BUF_SZ);
	if (err < 0) {
		free(ctx);
		return -1;
	}

	/* Opening serial device */
	do {
		ctx->filedes = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);

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

	/* Detect baudrate (NMEA frames are expected) */
	for (uint8_t i = 0; i < (sizeof(baudTable) / sizeof(speed_t)); i++) {
		if (gps_serialSetup(ctx->filedes, baudTable[i], (i == 0) ? &termBackup : NULL) < 0) {
			fprintf(stderr, "%s cannot set default baud\n", ubx_STR);
			close(ctx->filedes);
			gps_recvDone(&ctx->receiver);
			free(ctx);
			return -1;
		}
		const char *msg;
		int detected = 0;

		/* scan for NMEA frames for 1 s */
		cnt = 0;
		memset(&ctx->receiver.pCtx, 0, sizeof(nmea_scanCtx_t));
		while ((cnt++ < 10) && !detected) {
			usleep(100 * 1000);
			err = read(ctx->filedes, ctx->receiver.buf, ctx->receiver.bufSz);
			if (err <= 0) {
				continue;
			}

			while (nmea_scan(&ctx->receiver.pCtx, ctx->receiver.buf, err, &msg) > 0) {
				detected = 1;
			}
		}

		if (detected == 1) {
			printf("%s detected NMEA device at %d baudrate\n", ubx_STR, baudTable[i]);
			break;
		}

		if (i == (sizeof(baudTable) / sizeof(speed_t) - 1)) {
			fprintf(stderr, "%s failed to detect baudrate\n", ubx_STR);
			close(ctx->filedes);
			gps_recvDone(&ctx->receiver);
			free(ctx);
			return -1;
		}
	}


	/* baudrate setting to 115200 */
	write(ctx->filedes, UBX_PREMADE_BAUD_115200, sizeof(UBX_PREMADE_BAUD_115200));
	usleep(100 * 1000);

	/* First serial setup backups termios settings */
	if (gps_serialSetup(ctx->filedes, UBX_DEFAULT_BAUDRATE, NULL) < 0) {
		fprintf(stderr, "%s cannot set baud %d\n", ubx_STR, UBX_DEFAULT_BAUDRATE);
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


static int ubx_dealloc(sensor_info_t *info)
{
	ubx_ctx_t *ctx;

	if (!info || !info->ctx) {
		return -EINVAL;
	}

	ctx = (ubx_ctx_t *)info->ctx;
	ctx->run = 0;

	(void)close(ctx->filedes);

	if (ctx->tid) {
		(void)threadJoin(ctx->tid, (time_t)-1);
	}

	gps_recvDone(&ctx->receiver);
	free(ctx);
	info->ctx = NULL;

	return 0;
}


void __attribute__((constructor)) ubx_register(void)
{
	static sensor_drv_t sensor = {
		.name = "ubx",
		.alloc = ubx_alloc,
		.start = ubx_start,
		.dealloc = ubx_dealloc,
	};

	sensors_register(&sensor);
}
