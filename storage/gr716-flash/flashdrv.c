/*
 * Phoenix-RTOS
 *
 * GR716 Flash driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/minmax.h>

#include "flashdrv.h"
#include "spimctrl.h"
#include "nor/nor.h"


/* Helper functions */


static inline int flash_getSectorIdFromAddress(flash_context_t *ctx, addr_t addr)
{
	return addr / ctx->properties->sectorSz;
}


static inline addr_t flash_getSectorAddress(flash_context_t *ctx, addr_t addr)
{
	return addr & ~(ctx->properties->sectorSz - 1u);
}


static int flash_isValidAddress(size_t memsz, addr_t offs, size_t len)
{
	if ((offs < memsz) && ((offs + len) <= memsz)) {
		return 1;
	}

	return 0;
}


static ssize_t flash_directSectorWrite(flash_context_t *ctx, addr_t offs, const uint8_t *src)
{
	int res = nor_eraseSector(&ctx->spimctrl, offs, ctx->properties->tSE);
	if (res < 0) {
		return res;
	}

	for (addr_t pos = 0; pos < ctx->properties->sectorSz; pos += ctx->properties->pageSz) {
		res = nor_pageProgram(&ctx->spimctrl, offs + pos, src + pos, ctx->properties->pageSz, ctx->properties->tPP);
		if (res < 0) {
			return res;
		}
	}

	return (ssize_t)ctx->properties->sectorSz;
}


/* Device driver interface */


ssize_t flash_readData(flash_context_t *ctx, addr_t offs, void *buff, size_t len)
{
	int res;
	size_t doneBytes = 0;

	if (flash_isValidAddress(ctx->properties->totalSz, offs, len) == 0) {
		return -EINVAL;
	}

	if (len == 0u) {
		return 0;
	}

	if (flash_getSectorAddress(ctx, offs) == ctx->sectorBufAddr) {
		doneBytes = min(ctx->sectorBufAddr + ctx->properties->sectorSz - offs, len);

		(void)memcpy(buff, ctx->sectorBuf + offs - ctx->sectorBufAddr, doneBytes);

		if (doneBytes == len) {
			return (ssize_t)doneBytes;
		}

		offs += doneBytes;
		buff += doneBytes;
		len -= doneBytes;
	}

	res = nor_readData(&ctx->spimctrl, offs, buff, len);

	return (res < 0) ? res : (ssize_t)(doneBytes + res);
}


ssize_t flash_directWrite(flash_context_t *ctx, addr_t offs, const void *buff, size_t len)
{
	ssize_t res;
	size_t chunk, doneBytes = 0;

	if (flash_isValidAddress(ctx->properties->totalSz, offs, len) == 0) {
		return -EINVAL;
	}

	for (doneBytes = 0; doneBytes < len; doneBytes += chunk) {
		chunk = min(len - doneBytes, ctx->properties->pageSz);

		/* Invalidate cache if needed */
		if (flash_getSectorAddress(ctx, offs + doneBytes) == ctx->sectorBufAddr) {
			ctx->sectorBufAddr = (addr_t)-1;
		}

		res = nor_pageProgram(&ctx->spimctrl, offs + doneBytes, buff + doneBytes, chunk, ctx->properties->tPP);
		if (res < 0) {
			return res;
		}
	}

	return (ssize_t)doneBytes;
}


ssize_t flash_bufferedWrite(flash_context_t *ctx, addr_t offs, const void *buff, size_t len)
{
	ssize_t res;
	addr_t sectorOfs, currAddr;
	const uint8_t *src = buff;
	size_t doneBytes = 0, chunk;

	if (flash_isValidAddress(ctx->properties->totalSz, offs, len) == 0) {
		return -EINVAL;
	}

	if (len == 0u) {
		return 0;
	}

	while (doneBytes < len) {
		currAddr = flash_getSectorAddress(ctx, offs);

		sectorOfs = offs - currAddr;
		chunk = min(ctx->properties->sectorSz - sectorOfs, len - doneBytes);

		if (currAddr != ctx->sectorBufAddr) {
			if ((sectorOfs == 0) && (chunk == ctx->properties->sectorSz)) {
				/* Whole sector to write */
				res = flash_directSectorWrite(ctx, offs, src);
				if (res < 0) {
					return res;
				}
			}
			else {
				res = flash_sync(ctx);
				if (res < 0) {
					return res;
				}

				ctx->sectorBufAddr = currAddr;
				res = nor_readData(&ctx->spimctrl, ctx->sectorBufAddr, ctx->sectorBuf, ctx->properties->sectorSz);
				if (res < 0) {
					ctx->sectorBufAddr = (addr_t)-1;
					return res;
				}
			}
		}

		if (currAddr == ctx->sectorBufAddr) {
			/* Sector to write to in cache */
			(void)memcpy(ctx->sectorBuf + sectorOfs, src, chunk);
			ctx->sectorBufDirty = 1;
		}

		src += chunk;
		offs += chunk;
		doneBytes += chunk;
	}

	return doneBytes;
}


int flash_sync(flash_context_t *ctx)
{
	int res;

	if (ctx == NULL) {
		return -EINVAL;
	}

	if ((ctx->sectorBufAddr == (addr_t)-1) || (ctx->sectorBufDirty == 0)) {
		return EOK;
	}

	res = flash_directSectorWrite(ctx, ctx->sectorBufAddr, ctx->sectorBuf);

	if (res == EOK) {
		ctx->sectorBufDirty = 0;
	}

	return res < 0 ? res : EOK;
}


int flash_sectorErase(flash_context_t *ctx, addr_t offs)
{
	if (flash_isValidAddress(ctx->properties->totalSz, offs, ctx->properties->sectorSz) == 0) {
		return -EINVAL;
	}

	offs = flash_getSectorAddress(ctx, offs);

	if (offs == ctx->sectorBufAddr) {
		ctx->sectorBufAddr = (addr_t)-1;
	}

	return nor_eraseSector(&ctx->spimctrl, offs, ctx->properties->tSE);
}


int flash_chipErase(flash_context_t *ctx)
{
	ctx->sectorBufAddr = (addr_t)-1;

	return nor_eraseChip(&ctx->spimctrl, ctx->properties->tCE);
}


/* Initialization */


int flash_init(flash_context_t *ctx, int instance)
{
	uint8_t *buff;
	int res;

	ctx->sectorBufAddr = (addr_t)-1;
	ctx->sectorBufDirty = 0;
	ctx->sectorBuf = NULL;

	res = spimctrl_init(&ctx->spimctrl, instance);
	if (res != EOK) {
		return res;
	}

	res = nor_probe(&ctx->spimctrl, &ctx->properties, &ctx->vendor);
	if (res != EOK) {
		return res;
	}

	(void)printf("gr716-flashdrv: detected %s %s (0x%x)\n", ctx->vendor, ctx->properties->name, ctx->properties->jedecId);

	buff = malloc(ctx->properties->sectorSz);
	if (buff == NULL) {
		return -ENOMEM;
	}

	(void)memset(buff, NOR_ERASED_STATE, ctx->properties->sectorSz);

	ctx->sectorBuf = buff;

	return EOK;
}


void flash_contextDestroy(flash_context_t *ctx)
{
	(void)flash_sync(ctx);
	free(ctx->sectorBuf);
}
