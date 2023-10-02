/*
 * Phoenix-RTOS
 *
 * libtty
 *
 * Lock-free SPSC FIFO assuming push operations happen inside of interrupt handler.
 * In case of overflow old data is discarded.
 *
 * Copyright 2022-2023 Phoenix Systems
 * Author: Ziemowit Leszczynski, Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef LIBTTY_LF_FIFO_H
#define LIBTTY_LF_FIFO_H


#include <stdint.h>
#include <stdatomic.h>


#if ATOMIC_INT_LOCK_FREE != 2
#error "Atomic int is not lock free."
#endif


/* TODO: consider migrating to weakrb */
/* TODO: move into corelibs */


typedef struct {
	atomic_uint headPos;
	/* LSB indicates if push (0) or pop (1) last updated tail.
	 * Without this information pop could read an old byte
	 * discarding a new one in case of an overrun by multiple of fifo size. */
	atomic_uint tail;
	unsigned int sizeMask;
	uint8_t *data;
} lf_fifo_t;


/* NOTE: size must be a power of 2 */
/* NOTE: effective capacity is size - 1 */
static inline void lf_fifo_init(lf_fifo_t *f, uint8_t *data, unsigned int size)
{
	atomic_init(&f->headPos, 0);
	atomic_init(&f->tail, 0);
	f->sizeMask = size - 1;
	f->data = data;
}


/* return 0 in case of overrun, 1 otherwise */
static inline int lf_fifo_push(lf_fifo_t *f, uint8_t byte)
{
	unsigned int headPos = atomic_load_explicit(&f->headPos, memory_order_seq_cst);
	unsigned int tail = atomic_load_explicit(&f->tail, memory_order_seq_cst);
	unsigned int tailPos = tail >> 1;
	int ret = 1;

	f->data[headPos] = byte;

	headPos = (headPos + 1) & f->sizeMask;
	if (headPos == tailPos) {
		atomic_store_explicit(&f->tail, ((headPos + 1) & f->sizeMask) << 1, memory_order_seq_cst);
		ret = 0;
	}

	atomic_store_explicit(&f->headPos, headPos, memory_order_seq_cst);

	return ret;
}


/* returns 1 if element has been popped 0 otherwise */
static inline int lf_fifo_pop(lf_fifo_t *f, uint8_t *byte)
{
	unsigned int headPos = atomic_load_explicit(&f->headPos, memory_order_seq_cst);
	unsigned int tail = atomic_fetch_or_explicit(&f->tail, 1, memory_order_seq_cst) | 1;
	unsigned int tailPos = tail >> 1;
	unsigned int newTail = (((tailPos + 1) & f->sizeMask) << 1) | 1;

	if (headPos == tailPos) {
		return 0;
	}

	*byte = f->data[tailPos];

	return atomic_compare_exchange_strong_explicit(&f->tail, &tail, newTail, memory_order_seq_cst, memory_order_seq_cst) ? 1 : 0;
}


static inline int lf_fifo_empty(lf_fifo_t *f)
{
	unsigned int headPos = atomic_load_explicit(&f->headPos, memory_order_seq_cst);
	unsigned int tailPos = atomic_fetch_or_explicit(&f->tail, 1, memory_order_seq_cst) >> 1;

	return headPos == tailPos;
}


#endif
