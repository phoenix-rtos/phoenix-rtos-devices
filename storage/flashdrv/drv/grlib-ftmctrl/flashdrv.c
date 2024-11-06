/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * Copyright 2023 Phoenix Systems
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
#include "flash.h"
#include "ftmctrl.h"


#define LIBCACHE_LINECNT 1024
#define LIBCACHE_POLICY  LIBCACHE_WRITE_BACK


/* MTD interface */


static int _flashdrv_mtdRead(storage_t *strg, off_t offs, void *buff, size_t len, size_t *retlen)
{
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	if (!common_isValidAddress(CFI_SIZE(ctx->cfi.chipSz), offs, len)) {
		*retlen = 0;
		return -EINVAL;
	}

	if (len == 0u) {
		*retlen = 0;
		return 0;
	}

	ftmctrl_WrEn(ctx->ftmctrl);
	ftmctrl_flash_read(ctx, offs, buff, len);
	ftmctrl_WrDis(ctx->ftmctrl);

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
	if (!common_isValidAddress(CFI_SIZE(ctx->cfi.chipSz), offs, len)) {
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
	const size_t writeBuffsz = strg->dev->mtd->writeBuffsz;

	while (doneBytes < len) {
		size_t chunk = min(writeBuffsz - (offs % writeBuffsz), len - doneBytes);

		ftmctrl_WrEn(ctx->ftmctrl);
		res = ftmctrl_flash_writeBuffer(ctx, offs, src, chunk, CFI_TIMEOUT_MAX_PROGRAM(ctx->cfi.toutTypical.bufWrite, ctx->cfi.toutMax.bufWrite));
		ftmctrl_WrDis(ctx->ftmctrl);

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
	if (!common_isValidAddress(CFI_SIZE(ctx->cfi.chipSz), offs, len) || ((offs & (ctx->sectorsz - 1)) != 0) || (len % ctx->sectorsz != 0)) {
		return -EINVAL;
	}

	if (len == 0u) {
		return 0;
	}

	mutexLock(ctx->lock);
	ftmctrl_WrEn(ctx->ftmctrl);

	off_t end;
	int res = -ENOSYS;
	if ((offs == 0) && (len == CFI_SIZE(ctx->cfi.chipSz))) {
		TRACE("erasing entire memory");
		res = ftmctrl_flash_chipErase(ctx, CFI_TIMEOUT_MAX_ERASE(ctx->cfi.toutTypical.chipErase, ctx->cfi.toutMax.chipErase));
		end = CFI_SIZE(ctx->cfi.chipSz);
	}
	else {
		end = common_getSectorOffset(ctx->sectorsz, offs + len + ctx->sectorsz - 1u);
		TRACE("erasing sectors from 0x%jx to 0x%jx", (uintmax_t)offs, (uintmax_t)end);
	}

	if (res == -ENOSYS) {
		off_t secOffs = offs;
		while (secOffs < end) {
			res = ftmctrl_flash_sectorErase(ctx, secOffs, CFI_TIMEOUT_MAX_ERASE(ctx->cfi.toutTypical.blkErase, ctx->cfi.toutMax.blkErase));
			if (res < 0) {
				break;
			}
			secOffs += ctx->sectorsz;
		}
	}

	ftmctrl_WrDis(ctx->ftmctrl);
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
			(void)munmap(strg->dev->ctx->ftmctrl, _PAGE_SIZE);
			ftmctrl_flash_destroy(strg->dev->ctx);
		}
		free(strg->dev->ctx);
		free(strg->dev->mtd);
		free(strg->dev);
	}
	free(strg);
	strg = NULL;
}


static storage_t *flashdrv_init(addr_t mctrlBase, addr_t flashBase)
{
	struct _storage_devCtx_t *ctx = malloc(sizeof(struct _storage_devCtx_t));
	if (ctx == NULL) {
		return NULL;
	}
	ctx->ftmctrl = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, mctrlBase);
	if (ctx->ftmctrl == MAP_FAILED) {
		free(ctx);
		return NULL;
	}

	ftmctrl_WrEn(ctx->ftmctrl);
	int res = ftmctrl_flash_init(ctx, flashBase);
	ftmctrl_WrDis(ctx->ftmctrl);
	if (res < 0) {
		(void)munmap(ctx->ftmctrl, _PAGE_SIZE);
		free(ctx);
		return NULL;
	}

	if (mutexCreate(&ctx->lock) < 0) {
		(void)munmap(ctx->ftmctrl, _PAGE_SIZE);
		ftmctrl_flash_destroy(ctx);
		free(ctx);
		return NULL;
	}

	storage_t *strg = calloc(1, sizeof(storage_t));
	if (strg == NULL) {
		(void)munmap(ctx->ftmctrl, _PAGE_SIZE);
		ftmctrl_flash_destroy(ctx);
		free(ctx);
		return NULL;
	}

	strg->start = 0;
	strg->size = CFI_SIZE(ctx->cfi.chipSz);

	strg->dev = calloc(1, sizeof(storage_dev_t));
	if (strg->dev == NULL) {
		(void)munmap(ctx->ftmctrl, _PAGE_SIZE);
		ftmctrl_flash_destroy(ctx);
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
	mtd->name = ctx->dev->name;
	mtd->metaSize = 0;
	mtd->oobSize = 0;
	mtd->oobAvail = 0;

	uint8_t shift = ((ctx->dev->chipWidth == 16) && (ftmctrl_portWidth(ctx->ftmctrl) == 8)) ? 1 : 0;
	mtd->writeBuffsz = CFI_SIZE(ctx->cfi.bufSz) >> shift;
	mtd->writesz = 1;
	mtd->erasesz = ctx->sectorsz;

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

	ftmctrl_flash_printInfo(ctx);

	return strg;
}


void __attribute__((constructor)) ftmctrl_register(void)
{
	static const struct flash_driver ftmctrl = {
		.name = "ftmctrl",
		.init = flashdrv_init,
		.destroy = flashdrv_destroy,
	};

	flashsrv_register(&ftmctrl);
}
