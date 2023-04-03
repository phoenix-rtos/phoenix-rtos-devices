/*
 * Phoenix-RTOS
 *
 * Simsensor reader reads and parses data form special scenario file
 * than it pushes it to event_queue
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef SIMSENSOR_READER_H
#define SIMSENSOR_READER_H

#include <time.h>

#include "event_queue.h"


typedef struct {
	FILE *scenarioFile;
	int sensorTypes;

	time_t timeHorizon;

	char *lineBuf; /* Buffer for storing line from file */
	size_t bufLen;

	time_t timeLast;
	time_t offset;
} simsens_reader_t;


extern int reader_open(simsens_reader_t *rd, const char *path, int sensorTypes, time_t timeHorizon);


extern void reader_close(simsens_reader_t *reader);


extern int32_t reader_read(simsens_reader_t *rd, event_queue_t *queue);


#endif
