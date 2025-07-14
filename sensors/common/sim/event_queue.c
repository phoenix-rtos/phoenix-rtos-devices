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

#include "event_queue.h"

#include <stdlib.h>


int eventQueue_init(event_queue_t *q, unsigned int cnt)
{
	if (cnt == 0 || q == NULL) {
		return -1;
	}

	q->queue = malloc(sizeof(sensor_event_t) * cnt);
	if (q->queue == NULL) {
		return -1;
	}

	q->capacity = cnt;
	q->len = 0;

	q->head = q->queue;
	q->tail = q->queue;

	return 0;
}


void eventQueue_free(event_queue_t *q)
{
	if (q != NULL) {
		free(q->queue);
		q->queue = NULL;
	}
}


static inline sensor_event_t *eventQueue_next(event_queue_t *q, sensor_event_t *elem)
{
	return (elem == q->queue + q->capacity - 1) ? q->queue : elem + 1;
}


int eventQueue_enqueue(event_queue_t *q, const sensor_event_t *event)
{
	if (q == NULL || event == NULL) {
		return -1;
	}

	if (q->len == q->capacity) {
		return -1;
	}

	*q->head = *event;
	q->head = eventQueue_next(q, q->head);
	q->len++;

	return 0;
}


bool eventQueue_full(const event_queue_t *q)
{
	if (q == NULL) {
		return true;
	}

	return q->len >= q->capacity;
}


extern bool eventQueue_empty(const event_queue_t *q)
{
	if (q == NULL) {
		return true;
	}

	return q->len == 0;
}


int eventQueue_dequeue(event_queue_t *q, sensor_event_t *event)
{
	if (q == NULL || event == NULL) {
		return -1;
	}

	if (q->len == 0) {
		return -1;
	}

	*event = *q->tail;
	q->tail = eventQueue_next(q, q->tail);
	q->len--;

	return 0;
}
