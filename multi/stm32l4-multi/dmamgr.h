/*
 * Phoenix-RTOS
 *
 * STM32 DMA manager API
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DMAMGR_H_
#define _DMAMGR_H_

#include "stm32l4-multi.h"
#include "common.h"

#define STM32MULTI_HANDLE_DMAMGR 1


int dmamgr_init(void);


int dmamgr_getMemoryParams(void **dmaMemPtr, size_t *dmaMemSz);


void *dmamgr_malloc(size_t size, size_t alignment);


int dmamgr_acquireChannels(unsigned int dma, const uint8_t requests[2], uint32_t flags, uint8_t chns_out[2], uint32_t *listbufs_out[2]);


void dmamgr_releaseChannel(unsigned int dma, uint8_t chn);


#endif /* _DMAMGR_H_ */
