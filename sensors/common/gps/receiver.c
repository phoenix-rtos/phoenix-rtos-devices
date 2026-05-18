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

#include <libsensors/sensor.h>

#include <libsensors/gps/receiver.h>
#include <libsensors/gps/nmea.h>


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

	/* open in raw mode */
	attr.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	attr.c_oflag &= ~(OPOST);
	attr.c_lflag &= ~(ICANON | ECHONL | ISIG | ECHO | IEXTEN);

	/* no parity bits, single stop bit, 8 bits per byte */
	attr.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
	attr.c_cflag |= CS8 | CREAD | CLOCAL;

	/* enable non-blocking mode */
	attr.c_cc[VMIN] = 0;
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


/* Returns 1 if `gpsEvt` was updated with data from `message`. Otherwise returns 0. */
int gps_updateEvt(const nmea_t *message, sensor_event_t *evtGps, float posStdev, float velStdev)
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
			return 1;

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

	return 0;
}


void gps_recvDone(gps_receiver_t *rcv)
{
	if (rcv != NULL) {
		free(rcv->buf);
	}
}


int gps_recvInit(gps_receiver_t *rcv, size_t buffSz)
{
	if (rcv == NULL) {
		return -1;
	}

	rcv->buf = malloc(buffSz);
	if (rcv->buf == NULL) {
		return -1;
	}

	rcv->bufSz = buffSz;

	memset(&rcv->pCtx, 0, sizeof(nmea_scanCtx_t));

	return 0;
}
