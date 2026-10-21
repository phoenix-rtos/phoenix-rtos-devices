/*
 * Phoenix-RTOS
 *
 * STM32 xPDMA driver connector to DMA manager.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/msg.h>

#include <stm32l4-multi.h>
#include "libxpdma.h"

#define LOOKUP_RETRIES 20

#define CHECK(x) \
	do { \
		int _ret = x; \
		if (_ret < 0) { \
			return _ret; \
		} \
	} while (0)


static oid_t mgrConnector_multidrv;


static int mgrConnector_multiLookup(const char *path, oid_t *oid)
{
	size_t i;
	for (i = 0; i < LOOKUP_RETRIES; i++) {
		if (lookup(path, NULL, oid) >= 0) {
			break;
		}

		usleep(100000);
	}

	return (i == LOOKUP_RETRIES) ? -ENOENT : 0;
}


int mgrConnector_getMemoryParams(void **dmaMemPtr, size_t *dmaMemSz)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = mgrConnector_multidrv,
	};
	multi_i_t *i = (multi_i_t *)msg.i.raw;
	multi_o_t *o = (multi_o_t *)msg.o.raw;

	i->type = stm32_dmamgr_getMemoryParams;
	CHECK(msgSend(mgrConnector_multidrv.port, &msg));
	CHECK(msg.o.err);

	*dmaMemPtr = o->dmamgr_getMemoryParams.dmaMemPtr;
	*dmaMemSz = o->dmamgr_getMemoryParams.dmaMemSz;
	return 0;
}


int mgrConnector_acquireChannels(unsigned int dma, const uint8_t requests[LIBDMA_DIRECTIONS], uint32_t flags, uint8_t chns_out[LIBDMA_DIRECTIONS], uint32_t *listbufs_out[LIBDMA_DIRECTIONS])
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = mgrConnector_multidrv,
	};
	multi_i_t *i = (multi_i_t *)msg.i.raw;
	multi_o_t *o = (multi_o_t *)msg.o.raw;

	i->type = stm32_dmamgr_acquireChn;
	i->dmamgr_acquireChn.dma = dma;
	i->dmamgr_acquireChn.requests[0] = requests[0];
	i->dmamgr_acquireChn.requests[1] = requests[1];
	i->dmamgr_acquireChn.flags = flags;
	CHECK(msgSend(mgrConnector_multidrv.port, &msg));
	CHECK(msg.o.err);

	chns_out[0] = o->dmamgr_acquireChn.chns[0];
	chns_out[1] = o->dmamgr_acquireChn.chns[1];
	listbufs_out[0] = o->dmamgr_acquireChn.listbufs[0];
	listbufs_out[1] = o->dmamgr_acquireChn.listbufs[1];
	return 0;
}


void mgrConnector_releaseChannel(unsigned int dma, uint8_t chn)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = mgrConnector_multidrv,
	};
	multi_i_t *i = (multi_i_t *)msg.i.raw;

	i->type = stm32_dmamgr_releaseChn;
	i->dmamgr_releaseChn.dma = dma;
	i->dmamgr_releaseChn.chn = chn;
	msgSend(mgrConnector_multidrv.port, &msg);
}


void *mgrConnector_malloc(size_t size, size_t alignment)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = mgrConnector_multidrv,
	};
	multi_i_t *i = (multi_i_t *)msg.i.raw;
	multi_o_t *o = (multi_o_t *)msg.o.raw;

	i->type = stm32_dmamgr_malloc;
	i->dmamgr_malloc.size = size;
	i->dmamgr_malloc.alignment = alignment;
	if (msgSend(mgrConnector_multidrv.port, &msg) < 0) {
		return NULL;
	}

	if (msg.o.err != 0) {
		return NULL;
	}

	return o->dmamgr_malloc;
}


/* Must be static const, because the pointer needs to remain valid */
static const libxpdma_mgrFunctions_t mgrFunctions = {
	.acquireChannels = mgrConnector_acquireChannels,
	.releaseChannel = mgrConnector_releaseChannel,
	.getMemoryParams = mgrConnector_getMemoryParams,
	.malloc = LIBDMA_HAS_DATA_CACHE ? mgrConnector_malloc : NULL,
};


int libxpdma_mgrConnect(const char *path, const libxpdma_mgrFunctions_t **mgrFunctions_out)
{
	CHECK(mgrConnector_multiLookup(path, &mgrConnector_multidrv));
	*mgrFunctions_out = &mgrFunctions;
	return 0;
}
