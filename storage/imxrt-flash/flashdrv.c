/*
 * Phoenix-RTOS
 *
 * i.MX RT Flash driver
 *
 * Copyright 2019-2024 Phoenix Systems
 * Author: Hubert Buczynski, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "fspi.h"
#include "flashdrv.h"
#include "nor.h"


#define LOG_INFO(fmt, ...) printf("imxrt-flash: " fmt "\n", ##__VA_ARGS__);


static inline int get_sectorIdFromAddress(flash_context_t *ctx, uint32_t addr)
{
	return addr / ctx->properties.sector_size;
}


static inline addr_t get_sectorAddress(flash_context_t *ctx, uint32_t addr)
{
	return addr & ~(ctx->properties.sector_size - 1);
}


static int flash_isValidAddress(flash_context_t *context, addr_t addr, size_t size)
{
	if ((addr + size) <= context->properties.size) {
		return 0;
	}

	return 1;
}


ssize_t flash_directRead(flash_context_t *ctx, uint32_t offset, void *buff, size_t size)
{
	if (flash_isValidAddress(ctx, offset, size)) {
		return -1;
	}

	return nor_readData(&ctx->fspi, ctx->port, offset, buff, size, ctx->timeout);
}


ssize_t flash_directWrite(flash_context_t *ctx, uint32_t offset, const void *buff, size_t size)
{
	int err;
	size_t chunk, len = size;

	while (len) {
		chunk = ctx->properties.page_size - (offset & (ctx->properties.page_size - 1));
		if (chunk > len) {
			chunk = len;
		}

		err = nor_pageProgram(&ctx->fspi, ctx->port, offset, buff, chunk, ctx->timeout);
		if (err < 0) {
			return err;
		}

		offset += chunk;
		len -= chunk;
		buff = (char *)buff + chunk;
	}

	return size - len;
}


static ssize_t bufferSync(flash_context_t *ctx, uint32_t dstAddr, int isLast)
{
	ssize_t res = 0;

	const int sectorLast = get_sectorIdFromAddress(ctx, ctx->prevAddr);
	const int sectorCurr = get_sectorIdFromAddress(ctx, dstAddr);

	if (sectorCurr != sectorLast) {
		res = flash_sync(ctx);
		if (res < 0) {
			return res;
		}

		if (ctx->properties.sector_size <= dstAddr) {
			ctx->prevAddr = (uint32_t)-1;
		}

		if (isLast != 0) {
			return res;
		}

		if (ctx->properties.size < dstAddr + ctx->properties.sector_size) {
			return -EIO;
		}

		res = nor_readData(&ctx->fspi, ctx->port, get_sectorAddress(ctx, dstAddr), ctx->buff, ctx->properties.sector_size, ctx->timeout);
		if (res < 0) {
			return res;
		}

		ctx->prevAddr = dstAddr;
	}

	return res;
}


ssize_t flash_bufferedRead(flash_context_t *ctx, uint32_t srcAddr, void *dstPtr, size_t size)
{
	if ((ctx->prevAddr == (uint32_t)-1) || (ctx->isDirty == 0)) {
		return flash_directRead(ctx, srcAddr, dstPtr, size);
	}

	uint32_t bufSize = ctx->properties.sector_size;
	uint32_t bufAddr = ctx->prevAddr & ~(bufSize - 1);
	uint32_t bufEndAddr = bufAddr + bufSize;

	/* Source area does not overlap the buffer window, read the entire buffer directly from flash */
	if ((srcAddr >= bufEndAddr) || (bufAddr >= srcAddr + size)) {
		return flash_directRead(ctx, srcAddr, dstPtr, size);
	}

	/* Determine the overlapping range */
	uint32_t overlapStart = (srcAddr > bufAddr) ? srcAddr : bufAddr;
	uint32_t overlapEnd = (srcAddr + size < bufEndAddr) ? srcAddr + size : bufEndAddr;
	uint32_t overlapSize = overlapEnd - overlapStart;

	/* Read the non-overlapping part from flash */
	if (srcAddr < bufAddr) {
		ssize_t res = flash_directRead(ctx, srcAddr, dstPtr, bufAddr - srcAddr);
		if (res < 0) {
			return res;
		}
	}

	/* Copy the overlapping part from the buffer */
	memcpy((uint8_t *)dstPtr + (overlapStart - srcAddr), ctx->buff + (overlapStart - bufAddr), overlapSize);

	/* Read the remaining non-overlapping part from flash */
	if (srcAddr + size > bufEndAddr) {
		ssize_t res = flash_directRead(ctx, bufEndAddr, (uint8_t *)dstPtr + (bufEndAddr - srcAddr), (srcAddr + size) - bufEndAddr);
		if (res < 0) {
			return res;
		}
	}

	return size;
}


ssize_t flash_bufferedWrite(flash_context_t *ctx, uint32_t dstAddr, const void *srcPtr, size_t size)
{
	int res;
	uint32_t ofs;
	size_t chunkSz, doneBytes;

	if (flash_isValidAddress(ctx, dstAddr, size)) {
		return -1;
	}

	doneBytes = 0;
	while (doneBytes < size) {
		res = bufferSync(ctx, dstAddr, 0);
		if (res < 0) {
			return res;
		}

		chunkSz = size - doneBytes;
		ofs = dstAddr & (ctx->properties.sector_size - 1);

		if (chunkSz > ctx->properties.sector_size - ofs) {
			chunkSz = ctx->properties.sector_size - ofs;
		}

		memcpy((char *)ctx->buff + ofs, srcPtr, chunkSz);
		ctx->isDirty = 1;

		dstAddr += chunkSz;
		srcPtr = (const char *)srcPtr + chunkSz;
		doneBytes += chunkSz;
	}

	if (doneBytes > 0) {
		res = bufferSync(ctx, dstAddr, 1);
		if (res < 0) {
			return res;
		}
	}

	return doneBytes;
}


int flash_chipErase(flash_context_t *ctx)
{
	return nor_eraseChip(&ctx->fspi, ctx->port, ctx->timeout);
}


int flash_sectorErase(flash_context_t *ctx, uint32_t offset)
{
	offset &= ~(ctx->properties.sector_size - 1);

	return nor_eraseSector(&ctx->fspi, ctx->port, offset, ctx->timeout);
}


int flash_sync(flash_context_t *ctx)
{
	ssize_t res;
	uint32_t ofs, pos, sectorAddr;

	/* Initial sector value check, nothing to do */
	if (ctx->prevAddr == (uint32_t)-1) {
		return EOK;
	}

	if (ctx->isDirty == 0) {
		return EOK;
	}

	sectorAddr = get_sectorAddress(ctx, ctx->prevAddr);

	/* all 'ones' in buffer means sector is erased ... */
	for (pos = 0; pos < ctx->properties.sector_size; ++pos) {
		if (*(ctx->buff + pos) != NOR_ERASED_STATE) {
			break;
		}
	}

	/* ... then erase may be skipped */
	if (pos != ctx->properties.sector_size) {
		res = nor_eraseSector(&ctx->fspi, ctx->port, sectorAddr, ctx->timeout);
		if (res < 0) {
			return res;
		}
	}

	for (ofs = 0; ofs < ctx->properties.sector_size; ofs += ctx->properties.page_size) {
		/* Just erased NOR page contains all 'ones' */
		for (pos = 0; pos < ctx->properties.page_size; ++pos) {
			if (*(ctx->buff + ofs + pos) != NOR_ERASED_STATE) {
				break;
			}
		}

		/* then if buffer is the same skip single page program */
		if (pos == ctx->properties.page_size) {
			continue;
		}

		res = nor_pageProgram(&ctx->fspi, ctx->port, sectorAddr + ofs, ctx->buff + ofs, ctx->properties.page_size, ctx->timeout);
		if (res < 0) {
			return res;
		}
	}

	/* FIXME: unable to call e.g.: hal_cpuInvCache(hal_cpuDCache, sectorAddr, ctx->properties.sector_size); from userspace,
	 * but FlexSPI driver use IP bus for read and write operations, so it's not necessary to invalidate cache, until it comes
	 * to AHB reads during XIP which is cached, and the cache should be invalidated.
	 */

	ctx->isDirty = 0;

	return EOK;
}


/* Init functions */

static int flash_defineFlexSPI(flash_context_t *ctx)
{
	switch (ctx->address) {
		case FLASH_EXT_DATA_ADDRESS:
			ctx->fspi.instance = 0;
			ctx->fspi.base = (void *)FLEXSPI1_BASE;
			ctx->fspi.ahbAddr = (void *)FLEXSPI1_AHB_ADDR;
			ctx->fspi.slPortMask = 1;
			ctx->timeout = 10000;
			ctx->port = 0;
			break;

		case FLASH_INTERNAL_DATA_ADDRESS:
			ctx->fspi.instance = 1;
			ctx->fspi.base = (void *)FLEXSPI2_BASE;
			ctx->fspi.ahbAddr = (void *)FLEXSPI2_AHB_ADDR;
			ctx->fspi.slPortMask = 1;
			ctx->timeout = 10000;
			ctx->port = 0;
			break;

		default:
			return -ENODEV;
	}

	return EOK;
}


int flash_init(flash_context_t *ctx)
{
	void *buff;
	const struct nor_info *pInfo;
	int res = EOK;

	ctx->prevAddr = (uint32_t)-1;
	ctx->isDirty = 0;
	ctx->buff = NULL;

	res = flash_defineFlexSPI(ctx);
	if (res < 0) {
		return res;
	}

	res = nor_probe(&ctx->fspi, ctx->port, &pInfo, &ctx->properties.pVendor);
	if (res < 0) {
		return res;
	}

	LOG_INFO("detected %s %s (0x%x)", ctx->properties.pVendor, pInfo->name, pInfo->jedecId);

	buff = malloc(pInfo->sectorSz);
	if (buff == NULL) {
		return -ENOMEM;
	}

	memset(buff, NOR_ERASED_STATE, pInfo->sectorSz);

	ctx->properties.size = pInfo->totalSz;
	ctx->properties.page_size = pInfo->pageSz;
	ctx->properties.sector_size = pInfo->sectorSz;
	ctx->fspi.slFlashSz[ctx->port] = pInfo->totalSz;
	ctx->buff = buff;

	return res;
}


void flash_contextDestroy(flash_context_t *ctx)
{
	(int)flash_sync(ctx);
	free(ctx->buff);
}
