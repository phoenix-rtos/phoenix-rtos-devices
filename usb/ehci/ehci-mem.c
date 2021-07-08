/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ehci/mem.c
 *
 * Copyright 2018, 2021 Phoenix Systems
 * Copyright 2007 Pawel Pisarczyk
 * Author: Jan Sikorski, Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/mman.h>

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "ehci-mem.h"

static struct {
	ehci_buf_t *buffer;
} ehci_mem_common;


void *ehci_allocPage(void)
{
	char *addr, *tmp;

	addr = (char *)mmap(NULL, EHCI_PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_UNCACHED, OID_NULL, 0);
	if (addr == NULL)
		return NULL;

	if (va2pa(addr) % EHCI_PAGE_SIZE == 0)
		return addr;

	/* On NOMMU architecture for now, page allignment is not guaranteed */
	munmap(addr, EHCI_PAGE_SIZE);

	addr = (char *)mmap(NULL, 2 * EHCI_PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_UNCACHED, OID_NULL, 0);
	if (addr == NULL)
		return NULL;
	tmp = (char *)(((uintptr_t)addr + (EHCI_PAGE_SIZE - 1)) & ~(EHCI_PAGE_SIZE - 1));

	/* unmap memory prefix */
	if (tmp != addr)
		munmap(addr, tmp - addr);

	/* unmap memory sufix */
	if (tmp + EHCI_PAGE_SIZE != addr)
		munmap(tmp + EHCI_PAGE_SIZE, (addr + 2 * EHCI_PAGE_SIZE) - (tmp + EHCI_PAGE_SIZE));

	return tmp;
}

void ehci_freePage(void *ptr)
{
	munmap(ptr, EHCI_PAGE_SIZE);
}

static ehci_buf_t *ehci_allocBuffer(void)
{
	char *p;
	ehci_buf_t *buf;

	buf = ehci_allocPage();

	if (buf == MAP_FAILED)
		return NULL;

	buf->next = NULL;
	buf->freesz = EHCI_PAGE_SIZE - 64;

	for (p = buf->start; p < (char *)buf + EHCI_PAGE_SIZE; p += 64)
		*(void **)p = p + 64;

	*(void **)(p - 64) = NULL;
	buf->free = (void **)buf->start;
	return buf;
}


static void *ehci_allocFrom(ehci_buf_t *buf)
{
	void *result;

	if (buf->free == NULL) {
		if (buf->next == NULL) {
			buf->next = ehci_allocBuffer();
			return ehci_allocFrom(buf->next);
		}

		return ehci_allocFrom(buf->next);
	}

	buf->freesz -= 64;
	result = buf->free;
	buf->free = *buf->free;
	memset(result, 0, 64);
	return result;
}


void ehci_free(void *ptr)
{
	ehci_buf_t *buf = (void *)((uintptr_t)ptr & ~(EHCI_PAGE_SIZE - 1));

	*(void **)ptr = buf->free;
	buf->free = ptr;
	buf->freesz += 64;
}


 void *ehci_alloc(void)
{
	if (ehci_mem_common.buffer == NULL) {
		ehci_mem_common.buffer = ehci_allocBuffer();

		if (ehci_mem_common.buffer == NULL)
			return NULL;
	}

	return ehci_allocFrom(ehci_mem_common.buffer);
}
