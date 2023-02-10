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
#include "nmea.h"
#include "pmtk.h"

#define PA6H_STR "pa6h:"

/* Specified by the module's documentation */
#define UPDATE_RATE_MS 100

#define DEG2RAD     0.0174532925
#define DEG2MILIRAD 17.4532925 /* 1 degree -> 1 milliradian conversion parameter */
#define KMH2MMS     277.7778   /* 1 km/h -> mm/s conversion parameter */


#define REC_BUF_SZ 2048
#define INBOX_SIZE 32


typedef struct {
	char *msg;
	size_t sz;
} pa6h_msg_t;


typedef struct {
	sensor_event_t evtGps;
	int filedes;
	char buff[1024];
	char stack[512] __attribute__((aligned(8)));
} pa6h_ctx_t;


/*
* Sets correct termios parameters for gps device under `fd` with baudrate specified with `baud`.
* Original termios structure read from `fd` is copied to `backup` (if it is not NULL).
* Returns 0 on success and -1 on fail. Prints its own error messages.
*/
static int pa6h_serialSetup(int fd, speed_t baud, struct termios *backup)
{
	struct termios attr;

	if (tcgetattr(fd, &attr) < 0) {
		fprintf(stderr, "%s tcgetattr failed\n", PA6H_STR);
		return -1;
	}

	/* Backup termios if requested */
	if (backup != NULL) {
		*backup = attr;
	}

	attr.c_iflag = 0;
	attr.c_oflag = ONLCR;
	attr.c_cflag = CS8 | CLOCAL;
	attr.c_lflag = 0;

	/* These parameters are not based upon `picocom` ones */
	attr.c_cc[VMIN] = 1;
	attr.c_cc[VTIME] = 0;

	if (cfsetispeed(&attr, baud) < 0 || cfsetospeed(&attr, baud) < 0) {
		fprintf(stderr, "%s failed to set baudrate", PA6H_STR);
		return -1;
	}

	if (tcsetattr(fd, TCSANOW, &attr) < 0) {
		fprintf(stderr, "%s tcsetattr failed.\n", PA6H_STR);
		return -1;
	}

	tcflush(fd, TCIOFLUSH);

	return 0;
}


/*
* Performs buffered read of gps incoming messages, and saves them to the `inbox`
*
* Buffering is inter-read split safe. Message that was only partially read in the first call
* will be joined with its remainder in the next call (assuming no buffer overflow occurs).
*/
static int pa6h_receiver(int fd, pa6h_msg_t *inbox, size_t inboxSz)
{
	static char buf[REC_BUF_SZ] = { 0 };
	static char *remStart = buf;
	static int pos = 0;
	static size_t remLen;

	size_t inboxFill = 0;
	char *tokenStart = NULL, *tokenEnd = NULL, *nextStartToken, checksum;
	int i, ret, val, len;

	remLen = &buf[pos] - remStart;
	pos = 0;

	/* NMEA/PMTK messages are no longer than 128 bytes. Discard partials longer than sizeof(buf)/2 */
	if (remLen < (sizeof(buf) / 2)) {
		/* move partial at the end to the beginning of buffer */
		if (remStart != buf) {
			memmove(buf, remStart, remLen);
		}
		pos = remLen;
	}

	ret = read(fd, &buf[pos], sizeof(buf) - 1 - pos);
	remStart = buf;
	nextStartToken = NULL;

	if (ret >= 0) {
		/* update position and nul terminate the buffer */
		pos += ret;
		buf[pos] = '\0';

		/* find beginning of first message */
		tokenStart = strchr(buf, '$');
		tokenEnd = tokenStart;
		while (tokenStart != NULL) {
			remStart = tokenStart;

			/* Seek current message`s end. Avoid multiple search for the same 'tokenEnd' if 'tokenStart' is behind it */
			if (tokenStart >= tokenEnd) {
				tokenEnd = strchr(tokenStart + 1, '*');
				if (tokenEnd == NULL) {
					break;
				}
				nextStartToken = strchr(tokenEnd, '$');
			}

			/* Ensure space for checksum and nul character not interfering with next message. Lack '/r/n' at the end is discarded */
			len = ((nextStartToken == NULL) ? &buf[pos] : nextStartToken) - tokenEnd;
			if (len < 3) {
				break;
			}

			/* Read and validate checksum. Save message to inbox if there is space there */
			val = strtol(tokenEnd + 1, (char **)NULL, 16);
			if (!(val == 0 && errno == EINVAL) && val <= 0xff && inboxFill < inboxSz) {

				checksum = 0;
				for (i = 1; i < tokenEnd - tokenStart; i++) {
					checksum ^= tokenStart[i];
				}

				if (checksum == val) {
					inbox[inboxFill].msg = tokenStart;
					inbox[inboxFill].sz = tokenEnd - tokenStart + 4; /* +1 for '*', +2 for checksum, +1 for nul */
					inboxFill++;
				}
			}

			/* find next starting token */
			tokenStart = strchr(tokenStart + 1, '$');
		}
	}

	/* Discard buffer remainings if last message was complete */
	if (tokenStart == NULL && tokenEnd != NULL) {
		remStart = buf;
		pos = 0;
	}

	for (i = 0; i < inboxFill; i++) {
		inbox[i].msg[inbox[i].sz] = 0;
	}

	return inboxFill;
}


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
static int pa6h_paramSet(int fd, unsigned int msgId, const char *payload, enum pmtk_ack *ack, time_t ackWait)
{
	/* Default inter-read sleep set to 10 ms */
	static const time_t pollSleep = 10 * 1000;

	char buf[128];
	pa6h_msg_t inbox[INBOX_SIZE];
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
		capacity = pa6h_receiver(fd, inbox, INBOX_SIZE);
		for (i = 0; i < capacity && status == pmtk_ack_none; i++) {
			status = pa6h_pmtkAckCheck(inbox[i].msg, inbox[i].sz, msgId);
		}

		if (status == pmtk_ack_none) {
			usleep(pollSleep);
			gettime(&currTime, NULL);
		}
	}

	*ack = status;

	return 0;
}


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
