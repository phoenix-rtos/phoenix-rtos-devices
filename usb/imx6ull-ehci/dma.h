/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ehci/dma.h
 *
 * Copyright 2018 Phoenix Systems
 * Copyright 2007 Pawel Pisarczyk
 * Author: Jan Sikorski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#ifndef _IMX6ULL_EHCI_DMA_H_
#define _IMX6ULL_EHCI_DMA_H_


typedef struct dma_buf {
	union {
		struct {
			struct dma_buf *next;
			unsigned freesz;
			void **free;
		};
		char padding[64];
	};

	char start[];
} dma_buf_t;


extern void dma_free64(void *ptr);


extern void *dma_alloc64(void);

#endif
