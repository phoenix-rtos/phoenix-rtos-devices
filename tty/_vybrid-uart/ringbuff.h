/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * eSi-UART serial port driver
 *
 * Copyright 2012-2013 Phoenix Systems
 *
 * Author: Jacek Popko
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _UART_RINGBUFF_H_
#define _UART_RINGBUFF_H_

typedef struct {
	volatile unsigned head;
	volatile unsigned tail;
	unsigned size_mask;
	u16 data[];
} ringbuff_t;


static inline int ringbuff_full(ringbuff_t *rb)
{
	assert (rb->head <= rb->size_mask && rb->tail <= rb->size_mask);
	return ((rb->head + 1) & rb->size_mask) == rb->tail;
}


static inline int ringbuff_empty(ringbuff_t *rb)
{
	assert (rb->head <= rb->size_mask && rb->tail <= rb->size_mask);
	return (rb->head == rb->tail);
}


static inline void ringbuff_put(ringbuff_t *rb, u16 c)
{
	assert(((rb->head + 1) & rb->size_mask) != rb->tail);
	rb->data[rb->head] = c;
	rb->head = (rb->head + 1) & rb->size_mask;
}


static inline u16 ringbuff_get(ringbuff_t *rb)
{
	u16 c;
	
	assert(rb->head != rb->tail);
	c = rb->data[rb->tail];
	rb->tail = (rb->tail + 1) & rb->size_mask;

	return c;
}


static inline u16 ringbuff_next(ringbuff_t *rb)
{
	u16 c;
	
	assert(rb->head != rb->tail);
	c = rb->data[rb->tail];
	return c;
}


static inline void ringbuff_init(ringbuff_t *rb, unsigned size)
{
	rb->head = 0;
	rb->tail = 0;
	rb->size_mask = size - 1;
	assert(hal_cpuGetLastBit(size) == hal_cpuGetFirstBit(size));
}


static inline void ringbuff_makeEmpty(ringbuff_t *rb)
{
	assert (rb->head <= rb->size_mask && rb->tail <= rb->size_mask);
	rb->tail = rb->head;
}


#endif
