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

#define TTYPC_EQBUF_SIZE 256u

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


/* Puts a byte into event queue */
extern int event_queue_put(event_queue_t *eq, unsigned char event, unsigned int notify_cnt);


/* Reads data from event queue */
extern int event_queue_get(event_queue_t *eq, char *dest, unsigned int size, unsigned int minsize, unsigned int flags);


/* Reads 3-byte mouse packets from event queue */
extern int event_queue_get_mouse(event_queue_t *meq, char *dest, unsigned int size, unsigned int flags);


/* Retrieves number of bytes stored in event queue */
extern int event_queue_count(const event_queue_t *eq);


/* Destroys event queue */
extern void event_queue_destroy(const event_queue_t *eq);


#endif
