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


/* Object of simsensor reader */
typedef struct simsens_reader_t simsens_reader_t;


simsens_reader_t *reader_open(const char *path, int sensor_types, time_t time_horizon);


void reader_close(simsens_reader_t *reader);


int32_t reader_read(simsens_reader_t *rd, event_queue_t *queue);


#endif
