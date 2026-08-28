/*
 * Phoenix-RTOS
 *
 * GRLIB SPIMCTRL Flash driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <board_config.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/minmax.h>
#include <sys/threads.h>

#include <flashdrv/common.h>
#include <flashdrv/flashsrv.h>

#include "flashdrv.h"

#include "interface.h"


#define LIBCACHE_LINECNT 1024
#define LIBCACHE_POLICY  LIBCACHE_WRITE_THROUGH


/* MTD interface */


static int _flashdrv_mtdRead(storage_t *strg, off_t offs, void *buff, size_t len, size_t *retlen)
{
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	size_t memsize = flash_size(ctx);

	if (!common_isValidAddress(memsize, offs, len)) {
		*retlen = 0;
		return -EINVAL;
	}

	if (len == 0u) {
		*retlen = 0;
		return 0;
	}

	ssize_t ret = flash_readData(ctx, offs, buff, len);
	if (ret < 0) {
		*retlen = 0;
		return ret;
	}

	*retlen = len;

	return 0;
}


static ssize_t _flashdrv_mtdReadCb(uint64_t offs, void *buff, size_t len, cache_devCtx_t *ctx)
{
	storage_t *strg = ctx->strg;
	size_t retlen;
	ssize_t ret;

	ret = _flashdrv_mtdRead(strg, offs, buff, len, &retlen);
	if (ret < 0) {
		return ret;
	}

	return (ssize_t)retlen;
}


static int flashdrv_mtdRead(storage_t *strg, off_t offs, void *buff, size_t len, size_t *retlen)
{
	mutexLock(strg->dev->ctx->lock);
	int ret = cache_read(strg->dev->ctx->cache, offs, buff, len);
	mutexUnlock(strg->dev->ctx->lock);

	if (ret < 0) {
		*retlen = 0;
	}
	else {
		*retlen = len;
		ret = 0;
	}

	return ret;
}


static int _flashdrv_mtdWrite(storage_t *strg, off_t offs, const void *buff, size_t len, size_t *retlen)
{
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	size_t memsize = flash_size(ctx);

	if (!common_isValidAddress(memsize, offs, len)) {
		*retlen = 0;
		return -EINVAL;
	}

	if (len == 0u) {
		*retlen = 0;
		return 0;
	}

	const uint8_t *src = buff;
	size_t doneBytes = 0;

	int res = 0;
	const size_t pagesz = strg->dev->mtd->writeBuffsz;
	time_t timeout_program = flash_timeout(ctx, pageProgram);

	while (doneBytes < len) {
		size_t chunk = min(pagesz - (offs % pagesz), len - doneBytes);

		#ifdef TRIPLE_REDUNDANCY_MODE
			int memory_region = 0;
			while ((memory_region < NUM_DATAREGIONS) && res == 0){
				offs = offs + BLOCK_32MB * memory_region;
				res = flash_pageProgram(ctx, offs, src, chunk, timeout_program);
				memory_region ++;
			}

		#else
			res = flash_pageProgram(ctx, offs, src, chunk, timeout_program);
		#endif /* TRIPLE_REDUNDANCY_MODE */

		if (res < 0) {
			break;
		}

		doneBytes += chunk;
		src += chunk;
		offs += chunk;
	}

	*retlen = doneBytes;

	return res;
}


static ssize_t _flashdrv_mtdWriteCb(uint64_t offs, const void *buff, size_t len, cache_devCtx_t *ctx)
{
	storage_t *strg = ctx->strg;
	size_t retlen;
	ssize_t ret;

	if ((offs % strg->dev->mtd->writesz) != 0 || (len % strg->dev->mtd->writesz) != 0) {
		return -EINVAL;
	}

	ret = _flashdrv_mtdWrite(strg, offs, buff, len, &retlen);
	if (ret < 0) {
		return ret;
	}

	return (ssize_t)retlen;
}


static int flashdrv_mtdWrite(storage_t *strg, off_t offs, const void *buff, size_t len, size_t *retlen)
{
	mutexLock(strg->dev->ctx->lock);
	int ret = cache_write(strg->dev->ctx->cache, offs, buff, len, LIBCACHE_POLICY);
	mutexUnlock(strg->dev->ctx->lock);

	if (ret < 0) {
		*retlen = 0;
	}
	else {
		*retlen = len;
		ret = 0;
	}

	return ret;
}


static int flashdrv_mtdErase(storage_t *strg, off_t offs, size_t len)
{
	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL)) {
		return -EINVAL;
	}

	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	size_t memsize = flash_size(ctx);

	if (!common_isValidAddress(memsize, offs, len) || (offs % ctx->sectorsz != 0) || (len % ctx->sectorsz != 0)) {
		return -EINVAL;
	}

	if (len == 0u) {
		return 0;
	}

	mutexLock(ctx->lock);

	off_t end;
	int res = -ENOSYS;
	flash_size(ctx);

	if ((offs == 0) && (len == memsize)) {
		TRACE("erasing entire memory");
		time_t chipErase_timeout = flash_timeout(ctx, eraseChip);
		res = flash_chipErase(ctx, chipErase_timeout);
		end = memsize;
	}
	else {
		end = common_getSectorOffset(ctx->sectorsz, offs + len + ctx->sectorsz - 1u);
		TRACE("erasing sectors from 0x%jx to 0x%jx", (uintmax_t)offs, (uintmax_t)end);
	}

	if (res == -ENOSYS) {
		off_t secOffs = offs;
		time_t sectorErase_timeout = flash_timeout(ctx, eraseSector);
		while (secOffs < end) {
			res = flash_sectorErase(ctx, secOffs, sectorErase_timeout);
			if (res < 0) {
				break;
			}
			secOffs += ctx->sectorsz;
			LOG_ERROR("musisz to poprawic nie zapomnij !!");
		}
	}

	res = cache_invalidate(ctx->cache, offs, end);
	mutexUnlock(ctx->lock);

	return res;
}


static const storage_mtdops_t mtdOps = {
	.erase = flashdrv_mtdErase,
	.unPoint = NULL,
	.point = NULL,
	.read = flashdrv_mtdRead,
	.write = flashdrv_mtdWrite,

	.meta_read = NULL,
	.meta_write = NULL,

	.sync = NULL,
	.lock = NULL,
	.unLock = NULL,
	.isLocked = NULL,

	.block_isBad = NULL,
	.block_isReserved = NULL,
	.block_markBad = NULL,
	.block_maxBadNb = NULL,

	.suspend = NULL,
	.resume = NULL,
	.reboot = NULL,
};


static void flashdrv_destroy(storage_t *strg)
{
	if (strg == NULL) {
		return;
	}

	if (strg->dev != NULL) {
		if (strg->dev->ctx != NULL) {
			if (strg->dev->ctx->cache != NULL) {
				cache_deinit(strg->dev->ctx->cache);
			}
			(void)resourceDestroy(strg->dev->ctx->lock);
			flash_destroy(strg->dev->ctx);
			free(strg->dev->ctx->spimctrl);
		}
		free(strg->dev->ctx);
		free(strg->dev->mtd);
		free(strg->dev);
	}
	free(strg);
}


static int readId(struct spimctrl *spimctrl, uint32_t *id)
{
	struct xferOp xfer;
	const uint8_t cmd = 0x9Fu; /* RDID */

	xfer.type = xfer_opRead;
	xfer.cmd = &cmd;
	xfer.cmdLen = 1;
	xfer.rxData = (uint8_t *)id;
	xfer.dataLen = 3;

	return spimctrl_xfer(spimctrl, &xfer);
}


static int check_devtype(struct _storage_devCtx_t *ctx)
{
	struct spimctrl *spimctrl = ctx->spimctrl;
	int res;
	uint32_t jedecId = 0;

	res = readId(spimctrl, &jedecId);
	if (res < EOK) {
		return res;
	}

	LOG_ERROR("ID: %d\n", jedecId);

	/* draft - tbd: FLASH_CMD_RDSFDP, 8 dummy cycles, ma zwrocic ciag znakow*/
	/* Zamienia LSB i MSB z PID oraz przesuwa VID na najniższy bajt */
	uint32_t micron_id = (((((0xBB21u) & 0xffu) << 16) | ((0xBB21u) & 0xff00u) | ((0x20u) & 0xffu)));
	if (jedecId == micron_id) {
		ctx->isCfi = 0;
		LOG_ERROR("ID: %d\n", micron_id);
	}
	else {
		ctx->isCfi = 1;
	}

	return res;
}


static storage_t *flashdrv_init(addr_t mctrlBase, addr_t flashBase)
{
	struct _storage_devCtx_t *ctx = calloc(1, sizeof(struct _storage_devCtx_t));
	if (ctx == NULL) {
		return NULL;
	}

	ctx->spimctrl = calloc(1, sizeof(struct spimctrl));
	if (ctx->spimctrl == NULL) {
		free(ctx);
		return NULL;
	}

	if (spimctrl_init(ctx->spimctrl, mctrlBase) < 0) {
		free(ctx->spimctrl);
		free(ctx);
		return NULL;
	}

	int res = check_devtype(ctx);
	if (res < EOK) {
		return NULL;
	}

	if (flash_init(ctx, flashBase) < 0) {
		free(ctx->spimctrl);
		free(ctx);
		return NULL;
	}

	if (mutexCreate(&ctx->lock) < 0) {
		flash_destroy(ctx);
		free(ctx->spimctrl);
		free(ctx);
		return NULL;
	}

	storage_t *strg = calloc(1, sizeof(storage_t));
	if (strg == NULL) {
		flash_destroy(ctx);
		free(ctx->spimctrl);
		free(ctx);
		return NULL;
	}

	strg->start = 0;
	size_t memsize = flash_size(ctx);
	strg->size = memsize;

	strg->dev = calloc(1, sizeof(storage_dev_t));
	if (strg->dev == NULL) {
		flash_destroy(ctx);
		free(ctx->spimctrl);
		free(ctx);
		free(strg);
		return NULL;
	}

	/* Assign device context */
	strg->dev->ctx = ctx;

	storage_mtd_t *mtd = calloc(1, sizeof(storage_mtd_t));
	if (mtd == NULL) {
		flashdrv_destroy(strg);
		return NULL;
	}

	/* MTD interface */
	mtd->ops = &mtdOps;
	mtd->type = mtd_norFlash;
	mtd->name = flash_name(ctx);
	mtd->metaSize = 0;
	mtd->oobSize = 0;
	mtd->oobAvail = 0;

	mtd->writeBuffsz = flash_segmSize(ctx, bufSize);
	mtd->writesz = 1;
	mtd->erasesz = flash_segmSize(ctx, sectSize);

	strg->dev->mtd = mtd;

	/* No block device interface */
	strg->dev->blk = NULL;

	/* Initialize cache */
	cache_ops_t cacheOps = {
		.readCb = _flashdrv_mtdReadCb,
		.writeCb = _flashdrv_mtdWriteCb,
		.ctx = &strg->dev->ctx->cacheCtx
	};
	strg->dev->ctx->cache = cache_init(strg->size, strg->dev->mtd->writeBuffsz, LIBCACHE_LINECNT, &cacheOps);
	if (strg->dev->ctx->cache == NULL) {
		flashdrv_destroy(strg);
		return NULL;
	}
	strg->dev->ctx->cacheCtx.strg = strg;

	flash_printInfo(ctx);

	return strg;
}


void __attribute__((constructor)) spimctrl_register(void)
{
	static const struct flash_driver spimctrl = {
		.name = "spimctrl",
		.init = flashdrv_init,
		.destroy = flashdrv_destroy,
	};

	flashsrv_register(&spimctrl);
}
