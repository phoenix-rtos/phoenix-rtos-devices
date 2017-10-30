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

#include <hal/if.h>
#include "ringbuf.h"
#include <lib/assert.h>

int ringbuf_put(ringbuf_t *rb, struct pbuf *d)
{
	int used;

	if (ringbuf_full(rb))
	{
		return 0;
	}

	rb->data[rb->head] = d;
	rb->head = (rb->head + 1) & rb->size_mask;

	used = (rb->head - rb->tail) & rb->size_mask;
	if (used > rb->max_use)
		rb->max_use = used;

	return 1;
}


struct pbuf *ringbuf_get(ringbuf_t *rb)
{
	struct pbuf* ret;

	if (ringbuf_empty(rb))
	{
		return NULL;
	}

	ret = rb->data[rb->tail];
	rb->tail = (rb->tail + 1) & rb->size_mask;

	return ret;
}


int ringbuf_fill(ringbuf_t *rb, esi_emac_t *emacregs)
{
	struct pbuf *p;
	unsigned len;
	int i, used;

	if (ringbuf_full(rb))
		return 0;

	len = emacregs->rx_length;
	for (p = rb->data[rb->head]; p != NULL && len > 0; p = p->next)
				for (i = 0; i < p->len && len > 0; len--)
					((u8 *)p->payload)[i++] = emacregs->rx_data;

	rb->head = (rb->head + 1) & rb->size_mask;

	used = (rb->head - rb->tail) & rb->size_mask;
	if (used > rb->max_use)
		rb->max_use = used;

	return 1;
}


struct pbuf *ringbuf_drain(ringbuf_t *rb, struct pbuf *replacement)
{
	struct pbuf *p;

	if (ringbuf_empty(rb))
		return NULL;

	p = rb->data[rb->tail];
	rb->data[rb->tail] = replacement;
	rb->tail = (rb->tail + 1) & rb->size_mask;
	return p;
}


ringbuf_t *ringbuf_alloc(unsigned size)
{
	ringbuf_t *rb;

	size = size & ~(size - 1); /* round down to value which can be easily bit-masked */

	if ((rb = vm_kmalloc(sizeof(ringbuf_t) + size * sizeof(struct pbuf *))))
	{
		rb->head = rb->tail = rb->max_use = 0;
		rb->size_mask = size - 1;
		hal_memset(rb->data, 0, size * sizeof(struct pbuf *));
	}
	return rb;
}