/*
 * Phoenix-RTOS
 *
 * STM32 xPDMA driver connector external to DMA manager
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


static int mgrConnector_getMemoryParams(void **dmaMemPtr, size_t *dmaMemSz)
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


static int mgrConnector_acquireChannels(unsigned int perType, unsigned int num, uint32_t flags, libdma_chnAlloc_t chns_out[LIBDMA_DIRECTIONS])
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = mgrConnector_multidrv,
	};
	multi_i_t *i = (multi_i_t *)msg.i.raw;
	multi_o_t *o = (multi_o_t *)msg.o.raw;

	i->type = stm32_dmamgr_acquireChn;
	i->dmamgr_acquireChn.perType = perType;
	i->dmamgr_acquireChn.num = num;
	i->dmamgr_acquireChn.flags = flags;
	CHECK(msgSend(mgrConnector_multidrv.port, &msg));
	CHECK(msg.o.err);

	for (size_t i = 0; i < 2; i++) {
		chns_out[i].dma = o->dmamgr_acquireChn.chns[i].dma;
		chns_out[i].idx = o->dmamgr_acquireChn.chns[i].idx;
		chns_out[i].request = o->dmamgr_acquireChn.chns[i].request;
		chns_out[i].portPer = o->dmamgr_acquireChn.chns[i].portPer;
		chns_out[i].listbuf = o->dmamgr_acquireChn.chns[i].listbuf;
	}

	return 0;
}


static void mgrConnector_releaseChannel(uint8_t dma, uint8_t chn)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = mgrConnector_multidrv,
	};
	multi_i_t *i = (multi_i_t *)msg.i.raw;

	i->type = stm32_dmamgr_releaseChn;
	i->dmamgr_releaseChn.dma = dma;
	i->dmamgr_releaseChn.idx = chn;
	msgSend(mgrConnector_multidrv.port, &msg);
}


static void *mgrConnector_malloc(size_t size, size_t alignment, uint32_t flags)
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
	i->dmamgr_malloc.flags = flags;
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
