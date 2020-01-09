/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ehci/dma.c
 *
 * Copyright 2018 Phoenix Systems
 * Copyright 2007 Pawel Pisarczyk
 * Author: Jan Sikorski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/mman.h>

#include <stdint.h>
#include <string.h>

#include "dma.h"


static struct {
	dma_buf_t *buffer;
} dma_common;


static dma_buf_t *dma_allocBuffer(void)
{
	char *p;
	dma_buf_t *buf;

	buf = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_UNCACHED, OID_NULL, 0);

	if (buf == MAP_FAILED)
		return NULL;

	buf->next = NULL;
	buf->freesz = _PAGE_SIZE - 64;

	for (p = buf->start; p < (char *)buf + _PAGE_SIZE; p += 64)
		*(void **)p = p + 64;

	*(void **)(p - 64) = NULL;
	buf->free = (void **)buf->start;
	return buf;
}


static void *dma_alloc64From(dma_buf_t *buf)
{
	void *result;

	if (buf->free == NULL) {
		if (buf->next == NULL) {
			buf->next = dma_allocBuffer();
			return dma_alloc64From(buf->next);
		}

		return dma_alloc64From(buf->next);
	}

	buf->freesz -= 64;
	result = buf->free;
	buf->free = *buf->free;
	memset(result, 0, 64);
	return result;
}


void dma_free64(void *ptr)
{
	dma_buf_t *buf = (void *)((uintptr_t)ptr & ~(_PAGE_SIZE - 1));
	*(void **)ptr = buf->free;
	buf->free = ptr;
	buf->freesz += 64;
}


void *dma_alloc64(void)
{
	if (dma_common.buffer == NULL) {
		dma_common.buffer = dma_allocBuffer();

		if (dma_common.buffer == NULL)
			return NULL;
	}

	return dma_alloc64From(dma_common.buffer);
}
