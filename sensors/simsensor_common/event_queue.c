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


struct event_queue_t {
	sensor_event_t *queue;
	unsigned int queue_sz;
	unsigned int act_elem_cnt;

	sensor_event_t *head;
	sensor_event_t *tail;
};


event_queue_t *eventQueue_init(unsigned int elem_cnt)
{
	event_queue_t *result;

	if (elem_cnt == 0) {
		return NULL;
	}

	result = malloc(sizeof(event_queue_t));
	if (result == NULL) {
		return NULL;
	}

	result->queue = malloc(sizeof(sensor_event_t) * elem_cnt);
	if (result->queue == NULL) {
		free(result);
		return NULL;
	}

	result->queue_sz = elem_cnt;
	result->act_elem_cnt = 0;

	result->head = result->queue;
	result->tail = result->queue;

	return result;
}


void eventQueue_free(event_queue_t *q)
{
	if (q == NULL) {
		return;
	}

	free(q->queue);
	free(q);
}


static sensor_event_t *eventQueue_nextElement(event_queue_t *q, sensor_event_t *elementPtr)
{
	if (elementPtr == q->queue + q->queue_sz) {
		return q->queue;
	}

	return elementPtr + 1;
}


int eventQueue_enqueue(event_queue_t *q, const sensor_event_t *event)
{
	if (q == NULL || event == NULL) {
		return -1;
	}

	if (q->act_elem_cnt == q->queue_sz) {
		return -1;
	}

	*q->head = *event;
	q->head = eventQueue_nextElement(q, q->head);
	q->act_elem_cnt++;

	return 0;
}


inline int eventQueue_canEnqueue(const event_queue_t *q)
{
	return q->act_elem_cnt != q->queue_sz;
}


int eventQueue_dequeue(event_queue_t *q, sensor_event_t *event)
{
	if (q == NULL || event == NULL) {
		return -1;
	}

	if (q->act_elem_cnt == 0) {
		return -1;
	}

	*event = *q->tail;
	q->tail = eventQueue_nextElement(q, q->tail);
	q->act_elem_cnt--;

	return 0;
}
