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
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>

#include "../sensors.h"
#include "common.h"
#include "nmea.h"
#include "pmtk.h"

#define PA6H_STR "pa6h:"

/* Specified by the module's documentation */
#define UPDATE_RATE_MS 100

/* PA6H has: pos_CEP = 3m, vel_CEP = 0.1m/s. We use RMS values which are 0.849 * CEP (https://en.wikipedia.org/wiki/Circular_error_probable) */
#define PA6H_POS_ACCURACY 2.547f
#define PA6H_VEL_ACCURACY 0.0849f


#define REC_BUF_SZ 2048
#define INBOX_SIZE 32


typedef struct {
	sensor_event_t evtGps;
	int filedes;
	gps_receiver_t receiver;
	char stack[1024] __attribute__((aligned(8)));
} pa6h_ctx_t;


/*
* With assumption that at buf[0] is '$' validates if message is pmtk acknowledgment message coming from msgId message.
* Returns status flag if message is pmtk ack and source id matches `msgId`. Returns -1 otherwise.
*/
static int pa6h_pmtkAckCheck(const char *buf, size_t bufSz, unsigned int msgId)
{
	/*
	* Pmtk ack message template:
	* $PMTK001,cmd,o*cs
	*
	* cmd - three digit source command id
	* o   - one digit output
	* cs  - two digit checksum
	*
	* message length is 17 bytes, all buffers are +1 for nul character
	*/
	unsigned int val;

	if (bufSz <= PMTK_ACK_MSG_LEN || msgId > 999 || msgId == 0) {
		return pmtk_ack_none;
	}

	/* validate general static tokens. Assures that atoi/strtod will work on data chunks of correct length/semantics */
	if (buf[8] != ',' || buf[12] != ',' || buf[14] != '*') {
		return pmtk_ack_none;
	}

	/* ACK message header check */
	if (strncmp(&buf[0], PMTK_ACK_HEADER, sizeof(PMTK_HEADER) - 1) != 0) {
		return pmtk_ack_none;
	}

	/* source message ID check */
	val = strtoul(&buf[9], (char **)NULL, 10);
	if (val != msgId || (val == 0 && errno == EINVAL)) {
		return pmtk_ack_none;
	}

	/* status check */
	switch (buf[13]) {
		case '0':
			return pmtk_ack_invalid;
		case '1':
			return pmtk_ack_nosupport;
		case '2':
			return pmtk_ack_failed;
		case '3':
			return pmtk_ack_success;
		default:
			return pmtk_ack_none;
	}
}


/*
* Sends PMTK message of `msgId` id and preformated `payload`  to `fd` and then listens for ackownelgment.
* Waits for ack message for ackWait microseconds (exits immediately if ackWait = 0).
* On success returns 0, otherwise -1;
*/
static int pa6h_paramSet(int fd, unsigned int msgId, const char *payload, enum pmtk_ack *ack, time_t ackWait, gps_receiver_t *recv)
{
	/* Default inter-read sleep set to 10 ms */
	static const time_t pollSleep = 10 * 1000;

	char buf[128];
	time_t endTime, currTime;
	int i, capacity, pos, status = pmtk_ack_none;

	/* Compose message */
	pos = snprintf(buf, sizeof(buf), "%s%03u,%s\r\n", PMTK_HEADER, msgId, payload);
	if (pos < 0 || pos >= sizeof(buf)) {
		fprintf(stderr, "%s cannot compose message\n", PA6H_STR);
		return -1;
	}

	/* Flush fd i/o */
	if (tcflush(fd, TCIOFLUSH) < 0) {
		fprintf(stderr, "%s tcflush failed.\n", PA6H_STR);
		return -1;
	}

	/* Send message */
	if (write(fd, buf, pos) < pos) {
		fprintf(stderr, "%s msg write failed.\n", PA6H_STR);
		return -1;
	}
	fsync(fd);
	tcdrain(fd);

	/* Poll for ack status */
	gettime(&endTime, NULL);
	currTime = endTime;
	endTime += ackWait;
	while (currTime < endTime && status == pmtk_ack_none) {
		capacity = gps_recv(fd, recv);
		for (i = 0; i < capacity && status == pmtk_ack_none; i++) {
			status = pa6h_pmtkAckCheck(recv->inbox[i].msg, recv->inbox[i].sz, msgId);
		}

		if (status == pmtk_ack_none) {
			usleep(pollSleep);
			gettime(&currTime, NULL);
		}
	}

	*ack = status;

	return 0;
}


static void pa6h_threadPublish(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	struct __errno_t errnoNew;
	pa6h_ctx_t *ctx = info->ctx;
	nmea_t message;
	unsigned int i, inbocCap, update;

	/* errno is currently not thread-safe. Rediricting to local errno structure */
	_errno_new(&errnoNew);

	while (1) {
		inbocCap = gps_recv(ctx->filedes, &ctx->receiver);

		update = 0;
		for (i = 0; i < inbocCap; i++) {
			nmea_interpreter(ctx->receiver.inbox[i].msg, &message);
			update |= gps_updateEvt(&message, &ctx->evtGps, PA6H_POS_ACCURACY, PA6H_VEL_ACCURACY);
		}

		if (update != 0) {
			sensors_publish(info->id, &ctx->evtGps);
		}

		usleep(UPDATE_RATE_MS * 1000);
	}
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
	const time_t ackWait = 2 * 1000 * 1000;
	const int repeat = 10;

	int cnt = 0, err = -1;
	const char *path;
	pa6h_ctx_t *ctx;
	struct termios termBackup;
	enum pmtk_ack ack;

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

	err = gps_recvInit(&ctx->receiver, REC_BUF_SZ, INBOX_SIZE);
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
				fprintf(stderr, "%s Can't open %s: %s\n", PA6H_STR, path, strerror(errno));
				err = -errno;
				gps_recvDone(&ctx->receiver);
				free(ctx);
				return err;
			}
		}
	} while (ctx->filedes < 0);

	if (isatty(ctx->filedes) != 1) {
		fprintf(stderr, "%s %s not a tty\n", PA6H_STR, path);
		close(ctx->filedes);
		gps_recvDone(&ctx->receiver);
		free(ctx);
		return -1;
	}

	/* First serial setup backups termios settings */
	if (gps_serialSetup(ctx->filedes, B9600, &termBackup) < 0) {
		fprintf(stderr, "%s cannot set baud %d\n", PA6H_STR, 9600);
		close(ctx->filedes);
		gps_recvDone(&ctx->receiver);
		free(ctx);
		return -1;
	}

	/* Request baudrate change to 115200 */
	for (cnt = 0; cnt < repeat; cnt++) {
		/* PA6H does not acknowledge baudrate change success with PMTK_ACK. Discarding read `ack` value */
		err = pa6h_paramSet(ctx->filedes, PMTK_ID_SET_NMEA_BAUDRATE, PMTK_SET_NMEA_BAUDRATE_115200, &ack, 0, &ctx->receiver);
		usleep(10 * 1000);
	}
	if (err < 0) {
		fprintf(stderr, "%s cannot send baud %d request\n", PA6H_STR, 115200);
		close(ctx->filedes);
		gps_recvDone(&ctx->receiver);
		free(ctx);
		return -1;
	}

	if (gps_serialSetup(ctx->filedes, B115200, NULL) < 0) {
		fprintf(stderr, "%s cannot set baud %d\n", PA6H_STR, 115200);
		close(ctx->filedes);
		gps_recvDone(&ctx->receiver);
		tcsetattr(ctx->filedes, TCSANOW, &termBackup);
		free(ctx);
		return -1;
	}

	cnt = 0;
	ack = pmtk_ack_none;
	/* Reduce incoming messages only to GPGGA, GPGSA, GPRMC */
	while (pa6h_paramSet(ctx->filedes, PMTK_ID_SET_NMEA_OUTPUT, PMTK_SET_NMEA_OUTPUT_GSA_GGA_VTG, &ack, ackWait, &ctx->receiver) == 0 && cnt < repeat && ack != pmtk_ack_success) {
		cnt++;
	}
	if (ack != pmtk_ack_success) {
		fprintf(stderr, "%s cannot set output semantic\n", PA6H_STR);
		tcsetattr(ctx->filedes, TCSANOW, &termBackup);
		close(ctx->filedes);
		gps_recvDone(&ctx->receiver);
		free(ctx);
		return -1;
	}

	/* Increase message output rate to 10Hz */
	cnt = 0;
	ack = pmtk_ack_none;
	while (pa6h_paramSet(ctx->filedes, PMTK_ID_SET_NMEA_UPDATERATE, PMTK_SET_NMEA_UPDATERATE_100, &ack, ackWait, &ctx->receiver) == 0 && cnt < repeat && ack != pmtk_ack_success) {
		cnt++;
	}
	if (ack != pmtk_ack_success) {
		fprintf(stderr, "%s cannot set message out. rate\n", PA6H_STR);
		tcsetattr(ctx->filedes, TCSANOW, &termBackup);
		close(ctx->filedes);
		gps_recvDone(&ctx->receiver);
		free(ctx);
		return -1;
	}


	/* Increase position fix rate to 5Hz */
	cnt = 0;
	ack = pmtk_ack_none;
	while (pa6h_paramSet(ctx->filedes, PMTK_ID_API_SET_FIX_CTL, PMTK_API_SET_FIX_CTL_200, &ack, ackWait, &ctx->receiver) == 0 && cnt < repeat && ack != pmtk_ack_success) {
		cnt++;
	}
	if (ack != pmtk_ack_success) {
		fprintf(stderr, "%s cannot set fix rate\n", PA6H_STR);
		tcsetattr(ctx->filedes, TCSANOW, &termBackup);
		close(ctx->filedes);
		gps_recvDone(&ctx->receiver);
		free(ctx);
		return -1;
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
