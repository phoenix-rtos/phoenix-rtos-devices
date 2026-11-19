/*
 * Phoenix-RTOS
 *
 * STM32 DMA manager for use with xPDMA driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _LIBXPDMA_MGR_H_
#define _LIBXPDMA_MGR_H_

#include <stdint.h>
#include <stddef.h>

#include <libxpdma.h>


/* Initialize DMA manager */
int libxpdma_mgr_init(void);


int libxpdma_mgr_getMemoryParams(void **dmaMemPtr, size_t *dmaMemSz);


void *libxpdma_mgr_malloc(size_t size, size_t alignment, uint32_t flags);


int libxpdma_mgr_acquireChannels(unsigned int perType, unsigned int num, uint32_t flags, libdma_chnAlloc_t chns_out[LIBDMA_DIRECTIONS]);


void libxpdma_mgr_releaseChannel(uint8_t dma, uint8_t chn);


/* Get function pointers for a locally running DMA manager */
int libxpdma_mgr_getLocal(const libxpdma_mgrFunctions_t **mgrFunctions_out);

#endif /* _LIBXPDMA_MGR_H_ */
