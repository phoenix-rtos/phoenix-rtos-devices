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

#include <stdbool.h>
#include <time.h>

#include <libsensors.h>


typedef struct {
	sensor_event_t *queue;
	unsigned int capacity; /* queue max length */
	unsigned int len;      /* current length of queue */

	sensor_event_t *head;
	sensor_event_t *tail;
} event_queue_t;


/* Initiates `q` as an event queue. If succeeded returns 0. In other case -1. */
extern int eventQueue_init(event_queue_t *q, unsigned int cnt);


/* Deallocates queue pointed by `q` */
extern void eventQueue_free(event_queue_t *q);


/* Adds new sensor event to queue. If succeeded returns 0. In other case -1. */
extern int eventQueue_enqueue(event_queue_t *q, const sensor_event_t *event);


/* Returns information if the queue `q` is full. If `q` is NULL, then the behaviour is undefined. */
extern bool eventQueue_full(const event_queue_t *q);


/* Returns information if the queue `q` is empty. If `q` is NULL, then the behaviour is undefined. */
extern bool eventQueue_empty(const event_queue_t *q);


/* Removes the last element from queue `q` and saves it into `event`. Returns 0 on success, or -1 if queue is empty */
extern int eventQueue_dequeue(event_queue_t *q, sensor_event_t *event);


#endif
