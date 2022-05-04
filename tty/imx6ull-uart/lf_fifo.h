/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Lock-free SPSC Lamport FIFO
 *
 * Copyright 2022 Phoenix Systems
 * Author: Ziemowit Leszczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _LF_FIFO_H
#define _LF_FIFO_H

#include <stdint.h>

typedef struct lf_fifo_s lf_fifo_t;

struct lf_fifo_s {
	unsigned int head;
	unsigned int tail;
	unsigned int size_mask;
	uint8_t *data;
};


/* NOTE: size must be a power of 2 */
static inline void lf_fifo_init(lf_fifo_t *f, uint8_t *data, unsigned int size)
{
	f->head = 0;
	f->tail = 0;
	f->size_mask = size - 1;
	f->data = data;
}


/* returns 1 if element has been pushed 0 otherwise */
static inline int lf_fifo_push(lf_fifo_t *f, uint8_t byte)
{
	unsigned int head = __atomic_load_n(&f->head, __ATOMIC_SEQ_CST);
	unsigned int tail = __atomic_load_n(&f->tail, __ATOMIC_SEQ_CST);

	if (((head + 1) & f->size_mask) == tail) {
		return 0;
	}

	f->data[head] = byte;
	__atomic_store_n(&f->head, (head + 1) & f->size_mask, __ATOMIC_SEQ_CST);

	return 1;
}


/* returns 1 if element has been popped 0 otherwise */
static inline int lf_fifo_pop(lf_fifo_t *f, uint8_t *byte)
{
	unsigned int head = __atomic_load_n(&f->head, __ATOMIC_SEQ_CST);
	unsigned int tail = __atomic_load_n(&f->tail, __ATOMIC_SEQ_CST);

	if (head == tail) {
		return 0;
	}

	*byte = f->data[tail];
	__atomic_store_n(&f->tail, (tail + 1) & f->size_mask, __ATOMIC_SEQ_CST);

	return 1;
}

#endif
