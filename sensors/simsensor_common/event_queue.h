/*
 * Phoenix-RTOS
 *
 * Queue for buffering sensor event data from reader to simsensors
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef EVENT_QUEUE_H
#define EVENT_QUEUE_H

#include <libsensors.h>


typedef struct event_queue_t event_queue_t;


/* Initiates event queue. Returns ready event_queue_t object. */
extern event_queue_t *eventQueue_init(unsigned int elem_cnt);


extern void eventQueue_free(event_queue_t *q);


/* Adds new sensor event to queue. If succeeded returns 0. In other case -1. */
extern int eventQueue_enqueue(event_queue_t *q, const sensor_event_t *event);


extern int eventQueue_canEnqueue(const event_queue_t *q);


/* Returns and removes the last element from queue. If there is no element in queue, then returns NULL. */
extern int eventQueue_dequeue(event_queue_t *q, sensor_event_t *event);


#endif
