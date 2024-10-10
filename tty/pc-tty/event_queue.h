/*
 * Phoenix-RTOS
 *
 * Simple circular buffer for enqueueing PS/2 events
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _EVENT_QUEUE_H_
#define _EVENT_QUEUE_H_

#include <sys/threads.h>

#define TTYPC_EQBUF_SIZE 258u /* should be a multiple of 3 for mouse events */

typedef struct _event_queue_t {
	unsigned char buf[TTYPC_EQBUF_SIZE];
	unsigned int cnt;
	unsigned int r;
	unsigned int w;
	handle_t mutex;
	handle_t waitq;
} event_queue_t;


/* Initializes event queue */
extern int event_queue_init(event_queue_t *eq);


/* Puts a byte to event queue. If the queue has notify_cnt bytes or more, wake
   the readers. If the queue is full, does nothing and returns 0. If event was
   put, returns 1. */
extern int event_queue_put(event_queue_t *eq, unsigned char event, unsigned int notify_cnt);


/* Reads data from event queue */
extern int event_queue_get(event_queue_t *eq, void *dest, unsigned int size, unsigned int flags);


/* Retrieves number of bytes stored in event queue */
extern int event_queue_count(event_queue_t *eq);


/* Destroys event queue */
extern void event_queue_destroy(event_queue_t *eq);


#endif
