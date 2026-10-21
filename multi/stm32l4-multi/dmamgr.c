/*
 * Phoenix-RTOS
 *
 * STM32 DMA manager
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/threads.h>

#include "stm32l4-multi.h"
#include "common.h"
#include "dmamgr.h"
#include <libxpdma.h>
#include <xpdma_defs.h>

#define DMA_SYSPAGE_MAP_NAME "dmamem"

/* Type of buffer used for making linked lists, will be allocated in non-cached memory */
typedef uint32_t libdma_listbufs_t[LIBDMA_NUM_CONTROLLERS][LIBDMA_NUM_CHANNELS][XPDMA_LISTBUF_LEN];

struct {
	struct dma_ctrl {
		uint32_t chanFree; /* Bitmap of free channels */
		handle_t chanFreeLock;
	} ctrl[LIBDMA_NUM_CONTROLLERS];
	bool initialized;
	libdma_listbufs_t *listbufs;
	void *dmaMemPtr;
	size_t dmaMemSz;
#if LIBDMA_HAS_DATA_CACHE
	handle_t dmaAllocMutex;
	void *dmaAllocPtr;
	size_t dmaAllocSz;
#endif
} dmamgr_common;


static void dmamgr_setLBAR(unsigned int dma, unsigned int chn, void *addr)
{
	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_dmaLinkBaseAddr;
	pctl.dmaLinkBaseAddr.dev = dma_setup[dma].pctl;
	pctl.dmaLinkBaseAddr.channel = chn;
	pctl.dmaLinkBaseAddr.addr = ((uint32_t)addr) & 0xffff0000;
	platformctl(&pctl);
}


static inline uint32_t *dmamgr_getListbuf(unsigned int dma, unsigned int chn)
{
	if ((dma >= LIBDMA_NUM_CONTROLLERS) || (chn >= LIBDMA_NUM_CHANNELS)) {
		return NULL;
	}

	return (*dmamgr_common.listbufs)[dma][chn];
}


static int dmamgr_findChannel(uint8_t request, int dir, uint32_t *chanFreePtr, uint32_t flags)
{
	if (request == dma_req_invalid) {
		return LIBDMA_NUM_CHANNELS;
	}

	uint32_t chanFreeMasked = *chanFreePtr;
	bool mask2D = false;
	if ((dir == dma_per2mem) && ((flags & LIBXPDMA_ACQUIRE_2D_PER2MEM) != 0)) {
		mask2D = true;
	}
	else if ((dir == dma_mem2per) && ((flags & LIBXPDMA_ACQUIRE_2D_MEM2PER) != 0)) {
		mask2D = true;
	}

	if (mask2D) {
		chanFreeMasked &= LIBDMA_2D_CAPABLE_MASK | (~((1u << LIBDMA_NUM_CHANNELS) - 1));
	}

	int chn = __builtin_ctz(chanFreeMasked);
	if (chn >= LIBDMA_NUM_CHANNELS) {
		return -EBUSY;
	}

	*chanFreePtr &= ~(1u << chn);
	return chn;
}


int dmamgr_acquireChannels(unsigned int dma, const uint8_t requests[2], uint32_t flags, uint8_t chns_out[2], uint32_t *listbufs_out[2])
{
	if (!dmamgr_common.initialized) {
		return -ENXIO;
	}

	struct dma_ctrl *ctrl = &dmamgr_common.ctrl[dma];
	mutexLock(ctrl->chanFreeLock);

	/* Find enough unused channels necessary for peripheral */
	uint32_t chanFree = ctrl->chanFree;
	for (size_t dir = 0; dir < 2; dir++) {
		int acqResult = dmamgr_findChannel(requests[dir], dir, &chanFree, flags);
		if (acqResult < 0) {
			mutexUnlock(ctrl->chanFreeLock);
			return acqResult;
		}

		chns_out[dir] = acqResult;
		listbufs_out[dir] = dmamgr_getListbuf(dma, acqResult);
	}

	ctrl->chanFree = chanFree;
	mutexUnlock(ctrl->chanFreeLock);
	return 0;
}


void dmamgr_releaseChannel(unsigned int dma, uint8_t chn)
{
	if (!dmamgr_common.initialized) {
		return;
	}

	struct dma_ctrl *ctrl = &dmamgr_common.ctrl[dma];
	mutexLock(ctrl->chanFreeLock);
	ctrl->chanFree |= (1UL << chn);
	mutexUnlock(ctrl->chanFreeLock);
}


void *dmamgr_malloc(size_t size, size_t alignment)
{
#if LIBDMA_HAS_DATA_CACHE
	if (!dmamgr_common.initialized) {
		return NULL;
	}

	mutexLock(dmamgr_common.dmaAllocMutex);
	size_t misalign = (alignment == 0) ? 0 : ((uintptr_t)dmamgr_common.dmaAllocPtr & (alignment - 1));
	misalign = (misalign != 0) ? (alignment - misalign) : 0;

	if ((dmamgr_common.dmaAllocSz < misalign) || (size > (dmamgr_common.dmaAllocSz - misalign))) {
		mutexUnlock(dmamgr_common.dmaAllocMutex);
		return NULL;
	}

	dmamgr_common.dmaAllocSz -= misalign;
	dmamgr_common.dmaAllocPtr += misalign;
	void *ret = dmamgr_common.dmaAllocPtr;
	dmamgr_common.dmaAllocSz -= size;
	dmamgr_common.dmaAllocPtr += size;
	mutexUnlock(dmamgr_common.dmaAllocMutex);
	return ret;
#else
	return NULL;
#endif
}


static int dma_getDmaMemory(void)
{
	void *dmaMemPtr = NULL;
	size_t dmaMemSz = 0;

#if LIBDMA_HAS_DATA_CACHE
	/* Get memory that is DMA capable (non-cached).
	 * We can't do this using mmap(), because this is a NOMMU target so the MAP_UNCACHED flag does nothing.
	 * Instead, we check the system's memory maps if there's a map with a pre-defined name (`DMA_SYSPAGE_MAP_NAME`).
	 */
	meminfo_t mi;
	mi.page.mapsz = -1;
	mi.entry.kmapsz = -1;
	mi.entry.mapsz = -1;
	mi.maps.mapsz = 0;
	mi.maps.map = NULL;
	meminfo(&mi);

	mi.maps.map = malloc(mi.maps.mapsz * sizeof(mapinfo_t));
	if (mi.maps.map == NULL) {
		return -ENOMEM;
	}

	meminfo(&mi);
	for (int i = 0; i < mi.maps.mapsz; i++) {
		if (strcmp(mi.maps.map[i].name, DMA_SYSPAGE_MAP_NAME) == 0) {
			dmaMemPtr = (void *)mi.maps.map[i].vstart;
			dmaMemSz = mi.maps.map[i].vend - mi.maps.map[i].vstart;
			break;
		}
	}

	free(mi.maps.map);
#else
	dmaMemSz = sizeof(*dmamgr_common.listbufs);
	dmaMemPtr = malloc(dmaMemSz);
#endif

	if (dmaMemPtr == NULL) {
		return -ENODEV;
	}

	dmamgr_common.dmaMemPtr = dmaMemPtr;
	dmamgr_common.dmaMemSz = dmaMemSz;
	return EOK;
}


int dmamgr_getMemoryParams(void **dmaMemPtr, size_t *dmaMemSz)
{
	if (!dmamgr_common.initialized) {
		return -ENXIO;
	}

	*dmaMemPtr = dmamgr_common.dmaMemPtr;
	*dmaMemSz = dmamgr_common.dmaMemSz;
	return 0;
}


int dmamgr_init(void)
{
	if (dmamgr_common.initialized) {
		return EOK;
	}

	int ret;
	ret = dma_getDmaMemory();
	if (ret < 0) {
		return ret;
	}

	if (dmamgr_common.dmaMemSz < sizeof(*dmamgr_common.listbufs)) {
		return -ENOMEM;
	}

	dmamgr_common.listbufs = dmamgr_common.dmaMemPtr;

#if LIBDMA_HAS_DATA_CACHE
	mutexCreate(&dmamgr_common.dmaAllocMutex);
	dmamgr_common.dmaAllocPtr = dmamgr_common.dmaMemPtr + sizeof(*dmamgr_common.listbufs);
	dmamgr_common.dmaAllocSz = dmamgr_common.dmaMemSz - sizeof(*dmamgr_common.listbufs);
#endif

	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_dmaPermissions;
	pctl.dmaPermissions.secure = 1;
	pctl.dmaPermissions.privileged = -1;
	pctl.dmaPermissions.lock = 0;

	for (size_t dma = 0; dma < LIBDMA_NUM_CONTROLLERS; dma++) {
		devClk(dma_setup[dma].pctl, 1);
		mutexCreate(&dmamgr_common.ctrl[dma].chanFreeLock);
		pctl.dmaPermissions.dev = dma_setup[dma].pctl;
		for (size_t chn = 0; chn < LIBDMA_NUM_CHANNELS; chn++) {
			pctl.dmaPermissions.channel = chn;
			platformctl(&pctl);
			/* Set base address for channel's linked list buffer */
			dmamgr_setLBAR(dma, chn, dmamgr_getListbuf(dma, chn));
			*(libdma_channelBase(dma, chn) + xpdma_cxcr) = (1 << 1); /* Reset channel */
		}

		dmamgr_common.ctrl[dma].chanFree = ~0u; /* NOTE: bits after (1 << LIBDMA_NUM_CHANNELS) must always be 1 */
	}

	/* Must be static const, because the pointer needs to remain valid */
	static const libxpdma_mgrFunctions_t mgrFunctions = {
		.acquireChannels = dmamgr_acquireChannels,
		.releaseChannel = dmamgr_releaseChannel,
		.getMemoryParams = dmamgr_getMemoryParams,
		.malloc = LIBDMA_HAS_DATA_CACHE ? dmamgr_malloc : NULL,
	};

	libxpdma_init(&mgrFunctions);
	dmamgr_common.initialized = true;
	return EOK;
}
