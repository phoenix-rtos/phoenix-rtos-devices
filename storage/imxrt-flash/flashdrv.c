/*
 * Phoenix-RTOS
 *
 * i.MX RT Flash driver
 *
 * Copyright 2019-2022 Phoenix Systems
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


static int flash_isValidAddress(flash_context_t *context, addr_t addr, size_t size)
{
	if ((addr + size) <= context->properties.size) {
		return 0;
	}

	return 1;
}


ssize_t flash_readData(flash_context_t *ctx, uint32_t offset, void *buff, size_t size)
{
	if (flash_isValidAddress(ctx, offset, size)) {
		return -1;
	}

	return nor_readData(&ctx->fspi, ctx->port, offset, buff, size, 0);
}


ssize_t flash_directBytesWrite(flash_context_t *ctx, uint32_t offset, const void *buff, size_t size)
{
	int err;
	size_t chunk, len = size;

	while (len) {
		chunk = ctx->properties.page_size - (offset & (ctx->properties.page_size - 1));
		if (chunk > len) {
			chunk = len;
		}

		err = nor_pageProgram(&ctx->fspi, ctx->port, offset, buff, chunk, 0);
		if (err < 0) {
			return err;
		}

		offset += chunk;
		len -= chunk;
		buff = (char *)buff + chunk;
	}

	return size - len;
}


ssize_t flash_bufferedPagesWrite(flash_context_t *ctx, uint32_t offset, const void *buff, size_t size)
{
	uint32_t pageAddr;
	uint16_t sector_id;
	size_t savedBytes = 0;

	if (size % ctx->properties.page_size) {
		return -1;
	}

	if (flash_isValidAddress(ctx, offset, size)) {
		return -1;
	}

	while (savedBytes < size) {
		pageAddr = offset + savedBytes;
		sector_id = pageAddr / ctx->properties.sector_size;

		/* If sector_id has changed, data from previous sector have to be saved and new sector is read. */
		if (sector_id != ctx->sectorID) {
			flash_sync(ctx);

			if (flash_readData(ctx, ctx->properties.sector_size * sector_id, ctx->buff, ctx->properties.sector_size) <= 0) {
				return savedBytes;
			}

			if (nor_eraseSector(&ctx->fspi, ctx->port, ctx->properties.sector_size * sector_id, 0) < 0) {
				return savedBytes;
			}

			ctx->sectorID = sector_id;
			ctx->counter = offset - ctx->properties.sector_size * ctx->sectorID;
		}

		memcpy(ctx->buff + ctx->counter, (char *)buff + savedBytes, ctx->properties.page_size);

		savedBytes += ctx->properties.page_size;
		ctx->counter += ctx->properties.page_size;

		/* Save filled buffer */
		if (ctx->counter >= ctx->properties.sector_size) {
			flash_sync(ctx);
		}
	}

	return size;
}


int flash_chipErase(flash_context_t *ctx)
{
	return nor_eraseChip(&ctx->fspi, ctx->port, 0);
}


int flash_sectorErase(flash_context_t *ctx, uint32_t offset)
{

	offset &= ~(ctx->properties.sector_size - 1);

	return nor_eraseSector(&ctx->fspi, ctx->port, offset, 0);
}


void flash_sync(flash_context_t *ctx)
{
	int i;
	uint32_t dstAddr;
	const uint32_t *src;
	const uint32_t pagesNumber = ctx->properties.sector_size / ctx->properties.page_size;

	if (ctx->counter == 0) {
		return;
	}

	for (i = 0; i < pagesNumber; ++i) {
		dstAddr = ctx->sectorID * ctx->properties.sector_size + i * ctx->properties.page_size;
		src = (const uint32_t *)(ctx->buff + i * ctx->properties.page_size);

		nor_pageProgram(&ctx->fspi, ctx->port, dstAddr, src, ctx->properties.page_size, 0);
	}

	ctx->counter = 0;
	ctx->sectorID = -1;
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
			ctx->port = 0;
			break;

		case FLASH_INTERNAL_DATA_ADDRESS:
			ctx->fspi.instance = 1;
			ctx->fspi.base = (void *)FLEXSPI2_BASE;
			ctx->fspi.ahbAddr = (void *)FLEXSPI2_AHB_ADDR;
			ctx->fspi.slPortMask = 1;
			ctx->port = 0;
			break;

		default:
			return -ENODEV;
	}

	return EOK;
}


int flash_init(flash_context_t *ctx)
{
	const struct nor_info *pInfo;
	int res = EOK;

	ctx->sectorID = -1;
	ctx->counter = 0;
	ctx->buff = NULL;

	res = flash_defineFlexSPI(ctx);
	if (res < 0) {
		return res;
	}

	res = nor_probe(&ctx->fspi, ctx->port, &pInfo, &ctx->properties.pVendor);
	if (res < 0) {
		return res;
	}

	ctx->properties.size = pInfo->totalSz;
	ctx->properties.page_size = pInfo->pageSz;
	ctx->properties.sector_size = pInfo->sectorSz;
	ctx->fspi.slFlashSz[ctx->port] = pInfo->totalSz;

	LOG_INFO("imxrt-flash: detected %s %s (0x%x)", ctx->properties.pVendor, pInfo->name, pInfo->jedecId);

	return res;
}


void flash_contextDestroy(flash_context_t *ctx)
{
	flash_sync(ctx);
	free(ctx->buff);
}
