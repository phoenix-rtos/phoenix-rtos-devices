/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ehci/mem.h
 *
 * Copyright 2018, 2021 Phoenix Systems
 * Copyright 2007 Pawel Pisarczyk
 * Author: Jan Sikorski, Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#ifndef _EHCI_MEM_H
#define _EHCI_MEM_H

#define EHCI_PAGE_SIZE	4096


typedef struct ehci_buf {
	union {
		struct {
			struct ehci_buf *next;
			unsigned freesz;
			void **free;
		};
		char padding[64];
	};

	char start[];
} ehci_buf_t;


void ehci_free(void *ptr);

void *ehci_alloc(void);

void *ehci_allocPage(void);

void ehci_freePage(void *ptr);

#endif
