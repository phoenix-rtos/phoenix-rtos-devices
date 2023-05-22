/*
 * Phoenix-RTOS
 *
 * Text-driven GPS receivers common code
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <string.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <unistd.h>

#include <sys/time.h>

#include "../sensors.h"
#include "common.h"
#include "nmea.h"


#define DEG2RAD     0.0174532925
#define DEG2MILIRAD 17.4532925 /* 1 degree -> 1 milliradian conversion parameter */
#define KMH2MMS     277.7778   /* 1 km/h -> mm/s conversion parameter */


/*
* Sets correct termios parameters for gps device under `fd` with baudrate specified with `baud`.
* Original termios structure read from `fd` is copied to `backup` (if it is not NULL).
* Returns 0 on success and -1 on fail. Prints its own error messages.
*/
int gps_serialSetup(int fd, speed_t baud, struct termios *backup)
{
	struct termios attr;

	if (tcgetattr(fd, &attr) < 0) {
		fprintf(stderr, "gps: tcgetattr failed\n");
		return -1;
	}

	/* Backup termios if requested */
	if (backup != NULL) {
		*backup = attr;
	}

	attr.c_iflag = 0;
	attr.c_oflag = ONLCR;
	attr.c_cflag = CS8 | CLOCAL;
	attr.c_lflag &= ~ECHO;

	/* These parameters are not based upon `picocom` ones */
	attr.c_cc[VMIN] = 1;
	attr.c_cc[VTIME] = 0;

	if (cfsetispeed(&attr, baud) < 0 || cfsetospeed(&attr, baud) < 0) {
		fprintf(stderr, "gps: failed to set baudrate");
		return -1;
	}

	if (tcsetattr(fd, TCSANOW, &attr) < 0) {
		fprintf(stderr, "gps: tcsetattr failed.\n");
		return -1;
	}

	tcflush(fd, TCIOFLUSH);

	return 0;
}


int gps_recv(int fd, gps_receiver_t *rcv)
{
	size_t inboxFill = 0;
	char *tokenStart = NULL, *tokenEnd = NULL, *nextStartToken, checksum;
	int i, ret, val, len;

	rcv->remLen = &rcv->buf[rcv->pos] - rcv->remStart;
	rcv->pos = 0;

	/* NMEA/PMTK messages are no longer than 128 bytes. Discard partials longer than sizeof(buf)/2 */
	if (rcv->remLen < (rcv->bufSz / 2)) {
		/* move partial at the end to the beginning of buffer */
		if (rcv->remStart != rcv->buf) {
			memmove(rcv->buf, rcv->remStart, rcv->remLen);
		}
		rcv->pos = rcv->remLen;
	}

	ret = read(fd, &rcv->buf[rcv->pos], rcv->bufSz - 1 - rcv->pos);
	rcv->remStart = rcv->buf;
	nextStartToken = NULL;

	if (ret >= 0) {
		/* update position and nul terminate the buffer */
		rcv->pos += ret;
		rcv->buf[rcv->pos] = '\0';

		/* find beginning of first message */
		tokenStart = strchr(rcv->buf, '$');
		tokenEnd = tokenStart;
		while (tokenStart != NULL) {
			rcv->remStart = tokenStart;

			/* Seek current message`s end. Avoid multiple search for the same 'tokenEnd' if 'tokenStart' is behind it */
			if (tokenStart >= tokenEnd) {
				tokenEnd = strchr(tokenStart + 1, '*');
				if (tokenEnd == NULL) {
					break;
				}
				nextStartToken = strchr(tokenEnd, '$');
			}

			/* Ensure space for checksum and nul character not interfering with next message. Lack '/r/n' at the end is discarded */
			len = ((nextStartToken == NULL) ? &rcv->buf[rcv->pos] : nextStartToken - 1) - tokenEnd;
			if (len < 3) {
				break;
			}

			/* Read and validate checksum. Save message to inbox if there is space there */
			errno = EOK;
			val = strtol(tokenEnd + 1, (char **)NULL, 16);
			if (!(val == 0 && errno == EINVAL) && val <= 0xff && inboxFill < rcv->inboxLen) {

				checksum = 0;
				for (i = 1; i < tokenEnd - tokenStart; i++) {
					checksum ^= tokenStart[i];
				}

				if (checksum == val) {
					rcv->inbox[inboxFill].msg = tokenStart;
					rcv->inbox[inboxFill].sz = tokenEnd - tokenStart + 4; /* +1 for '*', +2 for checksum, +1 for nul */
					inboxFill++;
				}
			}

			/* find next starting token */
			tokenStart = strchr(tokenStart + 1, '$');
		}
	}

	/* Discard buffer remainings if last message was complete */
	if (tokenStart == NULL && tokenEnd != NULL) {
		rcv->remStart = rcv->buf;
		rcv->pos = 0;
	}

	for (i = 0; i < inboxFill; i++) {
		rcv->inbox[i].msg[rcv->inbox[i].sz - 1] = 0;
	}

	return inboxFill;
}


/* Returns 1 if `gpsEvt` was updated with data from `message`. Otherwise returns 0. */
int gps_updateEvt(nmea_t *message, sensor_event_t *evtGps, float posStdev, float velStdev)
{
	/* timestamp update happens only on GPGGA message as it contains position */
	switch (message->type) {
		case nmea_gga:
			evtGps->gps.lat = message->msg.gga.lat * 1e9;
			evtGps->gps.lon = message->msg.gga.lon * 1e9;
			evtGps->gps.hdop = (unsigned int)(message->msg.gga.hdop * 1e2);
			evtGps->gps.fix = message->msg.gga.fix;
			evtGps->gps.alt = message->msg.gga.h_asl * 1e3;
			evtGps->gps.altEllipsoid = message->msg.gga.h_wgs * 1e3;
			evtGps->gps.satsNb = message->msg.gga.sats;
			evtGps->gps.eph = evtGps->gps.hdop * posStdev;
			evtGps->gps.utc = message->msg.gga.utc * 1000000;

			gettime(&evtGps->timestamp, NULL);
			break;

		case nmea_gsa:
			evtGps->gps.hdop = (unsigned int)(message->msg.gsa.hdop * 1e2);
			evtGps->gps.vdop = (unsigned int)(message->msg.gsa.vdop * 1e2);

			evtGps->gps.eph = evtGps->gps.hdop * posStdev;
			evtGps->gps.epv = evtGps->gps.vdop * posStdev;
			break;

		case nmea_rmc:
			break;

		case nmea_vtg:
			evtGps->gps.heading = message->msg.vtg.track * DEG2MILIRAD;     /* degrees -> milliradians */
			evtGps->gps.groundSpeed = message->msg.vtg.speed_kmh * KMH2MMS; /* kmh->mm/s */
			evtGps->gps.velNorth = cos(message->msg.vtg.track * DEG2RAD) * evtGps->gps.groundSpeed;
			evtGps->gps.velEast = sin(message->msg.vtg.track * DEG2RAD) * evtGps->gps.groundSpeed;

			/* This is not 100% correct error estimation but the only we have */
			evtGps->gps.evel = evtGps->gps.hdop * velStdev;
			break;

		default:
			return 0;
	}

	return 1;
}


void gps_recvDone(gps_receiver_t *rcv)
{
	if (rcv != NULL) {
		free(rcv->buf);
		free(rcv->inbox);
	}
}


int gps_recvInit(gps_receiver_t *rcv, size_t buffSz, size_t inboxLen)
{
	if (rcv == NULL) {
		return -1;
	}

	rcv->buf = malloc(buffSz);
	if (rcv->buf == NULL) {
		return -1;
	}

	rcv->inbox = calloc(inboxLen, sizeof(gps_msg_t));
	if (rcv->inbox == NULL) {
		free(rcv->buf);
		return -1;
	}

	rcv->bufSz = buffSz;
	rcv->inboxLen = inboxLen;
	rcv->remStart = rcv->buf;
	rcv->pos = 0;
	rcv->remLen = 0;

	return 0;
}
