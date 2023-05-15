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


#ifndef _GPS_COMMON_H_
#define _GPS_COMMON_H_


#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <stdlib.h>

#include "nmea.h"
#include "../sensors.h"


typedef struct {
	char *msg;
	size_t sz;
} gps_msg_t;


typedef struct {
	/* NMEA text reading buffer */
	char *buf;
	size_t bufSz;

	/* receiver specific variables */
	char *remStart;
	size_t remLen;
	int pos;

	/* nmea messages inbox */
	gps_msg_t *inbox;
	size_t inboxLen;
} gps_receiver_t;


/*
* Sets correct termios parameters for gps device under `fd` with baudrate specified with `baud`.
* Original termios structure read from `fd` is copied to `backup` (if it is not NULL).
* Returns 0 on success and -1 on fail. Prints its own error messages.
*/
int gps_serialSetup(int fd, speed_t baud, struct termios *backup);


/*
* Performs buffered read of gps incoming messages, and saves them to the `inbox`
*
* Buffering is inter-read split safe. Message that was only partially read in the first call
* will be joined with its remainder in the next call (assuming no buffer overflow occurs).
*/
int gps_recv(int fd, gps_receiver_t *rcv);


/*
 * Returns 1 if `gpsEvt` was updated with data from `message`. Otherwise returns 0.
 * `posStdev` - position accuracy of sensor (from datasheet)
 * ``velStdev` - velocity accuracy of sensor (from dataseet)
 */
int gps_updateEvt(nmea_t *message, sensor_event_t *evtGps, float posStdev, float velStdev);


/* Gps receiver instance deinitialziation */
void gps_recvDone(gps_receiver_t *rcv);


/* Gps receiver instance initialization */
int gps_recvInit(gps_receiver_t *rcv, size_t buffSz, size_t inboxLen);


#endif
