/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Ethernet driver ring buffer management
 *
 * Copyright 2013 Phoenix Systems
 *
 * Author: Jacek Popko
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _RINGBUF_H
#define	_RINGBUF_H

#include <esirisc/emac.h>
#include <lwip/pbuf.h>

typedef struct
{
	unsigned head;
	unsigned tail;
	unsigned size_mask;
	unsigned max_use;
	struct pbuf *data[];
} ringbuf_t;


static inline int ringbuf_full(ringbuf_t *rb)
{
	assert(rb != NULL);
	return (rb->head == rb->tail - 1) || (rb->tail == 0 && rb->head == rb->size_mask);
}

static inline int ringbuf_empty(ringbuf_t *rb)
{
	assert(rb != NULL);
	return (rb->head == rb->tail);
}

extern ringbuf_t *ringbuf_alloc(unsigned size);
extern int ringbuf_put(ringbuf_t *rb, struct pbuf *d);
extern struct pbuf *ringbuf_get(ringbuf_t *rb);
extern int ringbuf_fill(ringbuf_t *rb, esi_emac_t *emacregs);
extern struct pbuf *ringbuf_drain(ringbuf_t *rb, struct pbuf *replacement);

#endif

