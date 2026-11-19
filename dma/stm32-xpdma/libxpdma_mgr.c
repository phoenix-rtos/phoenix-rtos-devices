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


#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/platform.h>

#include "libxpdma_mgr.h"
#include "xpdma_defs.h"

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
#if LIBDMA_HAS_DATA_CACHE
	char *dmaMemPtr;
	size_t dmaMemSz;
	handle_t dmaAllocMutex;
	char *dmaAllocPtr;
	size_t dmaAllocSz;
#endif
} dmamgr_common = {
	.initialized = false,
};


static int libxpdma_devClk(int dev, int state)
{
	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_devclk;
	pctl.devclk.dev = dev;
	pctl.devclk.state = state;
#if defined(__CPU_STM32N6) || defined(__CPU_STM32U3)
	pctl.devclk.lpState = state;
#endif

	return platformctl(&pctl);
}


static int libxpdma_mgr_setLBAR(unsigned int dma, unsigned int chn, void *addr)
{
	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_dmaLinkBaseAddr;
	pctl.dmaLinkBaseAddr.dev = dma_setup[dma].pctl;
	pctl.dmaLinkBaseAddr.channel = chn;
	pctl.dmaLinkBaseAddr.addr = ((uint32_t)addr) & 0xffff0000;
	return platformctl(&pctl);
}


static inline uint32_t *libxpdma_mgr_getListbuf(unsigned int dma, unsigned int chn)
{
	return (*dmamgr_common.listbufs)[dma][chn];
}


static int libxpdma_mgr_findChannel(uint8_t request, int dir, uint32_t *chanFreePtr, uint32_t flags)
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


int libxpdma_mgr_acquireChannels(unsigned int perType, unsigned int num, uint32_t flags, libdma_chnAlloc_t chns_out[LIBDMA_DIRECTIONS])
{
	uint8_t dma;
	uint8_t requests[LIBDMA_DIRECTIONS];
	uint8_t portPer;
	if (!dmamgr_common.initialized) {
		return -ENXIO;
	}

	int ret = libxpdma_lookupPeripheral(perType, num, requests, &dma, &portPer);
	if (ret < 0) {
		return ret;
	}

	if ((flags & (LIBXPDMA_ACQUIRE_FORCE_HPDMA | LIBXPDMA_ACQUIRE_FORCE_GPDMA)) != 0) {
		bool findHPDMA = (flags & LIBXPDMA_ACQUIRE_FORCE_HPDMA) != 0;
		for (dma = 0; dma < LIBDMA_NUM_CONTROLLERS; dma++) {
			if (dma_setup[dma].isHPDMA == findHPDMA) {
				break;
			}
		}

		if (dma >= LIBDMA_NUM_CONTROLLERS) {
			return -EINVAL;
		}
	}

	struct dma_ctrl *ctrl = &dmamgr_common.ctrl[dma];
	mutexLock(ctrl->chanFreeLock);

	/* Find enough unused channels necessary for peripheral */
	uint32_t chanFree = ctrl->chanFree;
	for (size_t dir = 0; dir < LIBDMA_DIRECTIONS; dir++) {
		int acqResult = libxpdma_mgr_findChannel(requests[dir], dir, &chanFree, flags);
		if (acqResult < 0) {
			mutexUnlock(ctrl->chanFreeLock);
			return acqResult;
		}

		chns_out[dir].dma = dma;
		chns_out[dir].idx = (uint8_t)acqResult;
		chns_out[dir].request = requests[dir];
		chns_out[dir].portPer = portPer;
		if (acqResult < LIBDMA_NUM_CHANNELS) {
			chns_out[dir].listbuf = libxpdma_mgr_getListbuf(dma, acqResult);
		}
		else {
			chns_out[dir].listbuf = NULL;
		}
	}

	ctrl->chanFree = chanFree;
	mutexUnlock(ctrl->chanFreeLock);
	return 0;
}


void libxpdma_mgr_releaseChannel(uint8_t dma, uint8_t chn)
{
	if (!dmamgr_common.initialized) {
		return;
	}

	if ((dma >= LIBDMA_NUM_CONTROLLERS) || (chn >= LIBDMA_NUM_CHANNELS)) {
		return;
	}

	struct dma_ctrl *ctrl = &dmamgr_common.ctrl[dma];
	mutexLock(ctrl->chanFreeLock);
	ctrl->chanFree |= (1UL << chn);
	mutexUnlock(ctrl->chanFreeLock);
}


void *libxpdma_mgr_malloc(size_t size, size_t alignment, uint32_t flags)
{
#if LIBDMA_HAS_DATA_CACHE
	if (!dmamgr_common.initialized) {
		return NULL;
	}

	/* Currently only allocating in AHBSRAM, which is lower performance than main system memory */
	if ((flags & LIBXPDMA_MALLOC_FASTMEM) != 0) {
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


static int libxpdma_mgr_getDmaMemory(void)
{
#if LIBDMA_HAS_DATA_CACHE
	char *dmaMemPtr = NULL;
	size_t dmaMemSz = 0;

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
			dmaMemPtr = (char *)mi.maps.map[i].vstart;
			dmaMemSz = mi.maps.map[i].vend - mi.maps.map[i].vstart;
			break;
		}
	}

	free(mi.maps.map);

	if (dmaMemPtr == NULL) {
		return -ENODEV;
	}

	dmamgr_common.dmaMemPtr = dmaMemPtr;
	dmamgr_common.dmaMemSz = dmaMemSz;
#else
	/* By aligning every list buffer to its size we ensure that it never crosses the 64 KB boundary imposed by hardware */
	static libdma_listbufs_t staticallyAllocatedListbufs __attribute__((aligned(XPDMA_LISTBUF_LEN * sizeof(uint32_t))));
	dmamgr_common.listbufs = &staticallyAllocatedListbufs;
#endif

	return EOK;
}


int libxpdma_mgr_getMemoryParams(void **dmaMemPtr, size_t *dmaMemSz)
{
	if (!dmamgr_common.initialized) {
		return -ENXIO;
	}

#if LIBDMA_HAS_DATA_CACHE
	*dmaMemPtr = dmamgr_common.dmaMemPtr;
	*dmaMemSz = dmamgr_common.dmaMemSz;
#else
	*dmaMemPtr = NULL;
	*dmaMemSz = 0;
#endif
	return 0;
}


int libxpdma_mgr_init(void)
{
	if (dmamgr_common.initialized) {
		return EOK;
	}

	int ret = libxpdma_mgr_getDmaMemory();
	if (ret < 0) {
		return ret;
	}

#if LIBDMA_HAS_DATA_CACHE
	if (dmamgr_common.dmaMemSz < sizeof(*dmamgr_common.listbufs)) {
		return -ENOMEM;
	}

	dmamgr_common.listbufs = (libdma_listbufs_t *)dmamgr_common.dmaMemPtr;

	ret = mutexCreate(&dmamgr_common.dmaAllocMutex);
	if (ret < 0) {
		return ret;
	}

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
		ret = (ret < 0) ? ret : libxpdma_devClk(dma_setup[dma].pctl, 1);
		ret = (ret < 0) ? ret : mutexCreate(&dmamgr_common.ctrl[dma].chanFreeLock);
		pctl.dmaPermissions.dev = dma_setup[dma].pctl;
		for (size_t chn = 0; chn < LIBDMA_NUM_CHANNELS; chn++) {
			pctl.dmaPermissions.channel = chn;
			ret = (ret < 0) ? ret : platformctl(&pctl);
			/* Set base address for channel's linked list buffer */
			ret = (ret < 0) ? ret : libxpdma_mgr_setLBAR(dma, chn, libxpdma_mgr_getListbuf(dma, chn));
			if (ret >= 0) {
				*(libdma_channelBase(dma, chn) + xpdma_cxcr) = (1 << 1); /* Reset channel */
			}
		}

		dmamgr_common.ctrl[dma].chanFree = ~0u; /* NOTE: bits after (1 << LIBDMA_NUM_CHANNELS) must always be 1 */
	}

	if (ret < 0) {
		for (size_t dma = 0; dma < LIBDMA_NUM_CONTROLLERS; dma++) {
			libxpdma_devClk(dma_setup[dma].pctl, 0);
			if (dmamgr_common.ctrl[dma].chanFreeLock != 0) {
				resourceDestroy(dmamgr_common.ctrl[dma].chanFreeLock);
			}
		}

#if LIBDMA_HAS_DATA_CACHE
		resourceDestroy(dmamgr_common.dmaAllocMutex);
#endif
		return ret;
	}

	dmamgr_common.initialized = true;

	return EOK;
}


static const libxpdma_mgrFunctions_t mgrFunctions = {
	.acquireChannels = libxpdma_mgr_acquireChannels,
	.releaseChannel = libxpdma_mgr_releaseChannel,
	.getMemoryParams = libxpdma_mgr_getMemoryParams,
	.malloc = LIBDMA_HAS_DATA_CACHE ? libxpdma_mgr_malloc : NULL,
};


int libxpdma_mgr_getLocal(const libxpdma_mgrFunctions_t **mgrFunctions_out)
{
	if (!dmamgr_common.initialized) {
		return -ENXIO;
	}

	*mgrFunctions_out = &mgrFunctions;

	return 0;
}
