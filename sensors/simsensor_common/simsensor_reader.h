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
	ssize_t headerLen;
	int sensorTypes;

	time_t timeHorizon;

	char *lineBuf; /* Buffer for storing line from file */
	size_t bufLen;

	time_t timeLast;
	long long timeOffset;
} simsens_reader_t;


/*
 * Initiates `simsens_reader_t` structure.
 * `path` specifies file with test scenario
 * `sensorType` tells what types of readings are parsed from test file. Argument can be a logical or of different types
 * `timeHorizon` - max difference between first and last reader timestamp in single `reader_read` execution
 */
extern int reader_open(simsens_reader_t *rd, const char *path, int sensorTypes, time_t timeHorizon);


/* Frees elements form `reader`  */
extern void reader_close(simsens_reader_t *reader);


/* Parses the file. Stores results in `queue`. */
extern int reader_read(simsens_reader_t *rd, event_queue_t *queue);


#endif
