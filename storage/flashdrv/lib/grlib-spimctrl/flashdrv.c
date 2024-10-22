// /*
//  * Phoenix-RTOS
//  *
//  * GRLIB FTMCTRL Flash driver
//  *
//  * Copyright 2023 Phoenix Systems
//  * Author: Lukasz Leczkowski
//  *
//  * This file is part of Phoenix-RTOS.
//  *
//  * %LICENSE%
//  */


#include <board_config.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/minmax.h>
#include <sys/threads.h>

#include <flashsrv.h>
#include "flashdrv.h"
#include "nor.h"
#include "spimctrl.h"


#define LIBCACHE_LINECNT 1024
#define LIBCACHE_POLICY  LIBCACHE_WRITE_BACK


/* Helper functions */


static int fdrv_isValidAddress(size_t memsz, off_t offs, size_t len)
{
	if ((offs < memsz) && ((offs + len) <= memsz)) {
		return 1;
	}

	return 0;
}


/* MTD interface */


static int _flashdrv_mtdRead(storage_t *strg, off_t offs, void *buff, size_t len, size_t *retlen)
{
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	if (fdrv_isValidAddress(CFI_SIZE(ctx->cfi.chipSz), offs, len) == 0) {
		*retlen = 0;
		return -EINVAL;
	}

	if (len == 0u) {
		*retlen = 0;
		return EOK;
	}

	nor_readData(ctx->spimctrl, offs, buff, len);

	*retlen = len;

	return EOK;
}


// static ssize_t _flashdrv_mtdReadCb(uint64_t offs, void *buff, size_t len, cache_devCtx_t *ctx)
// {
// 	storage_t *strg = ctx->strg;
// 	size_t retlen;
// 	ssize_t ret;

// 	ret = _flashdrv_mtdRead(strg, offs, buff, len, &retlen);
// 	if (ret < 0) {
// 		return ret;
// 	}

// 	return (ssize_t)retlen;
// }


// static int flashdrv_mtdRead(storage_t *strg, off_t offs, void *buff, size_t len, size_t *retlen)
// {
// 	mutexLock(strg->dev->ctx->lock);
// 	int ret = cache_read(strg->dev->ctx->cache, offs, buff, len);
// 	mutexUnlock(strg->dev->ctx->lock);

// 	if (ret < 0) {
// 		*retlen = 0;
// 	}
// 	else {
// 		*retlen = len;
// 		ret = 0;
// 	}

// 	return ret;
// }


// static int _flashdrv_mtdWrite(storage_t *strg, off_t offs, const void *buff, size_t len, size_t *retlen)
// {
// 	struct _storage_devCtx_t *ctx = strg->dev->ctx;
// 	if (fdrv_isValidAddress(CFI_SIZE(ctx->cfi.chipSz), offs, len) == 0) {
// 		*retlen = 0;
// 		return -EINVAL;
// 	}

// 	if (len == 0u) {
// 		*retlen = 0;
// 		return EOK;
// 	}

// 	const uint8_t *src = buff;
// 	size_t doneBytes = 0;

// 	int res = EOK;
// 	const size_t writeBuffsz = strg->dev->mtd->writeBuffsz;

// 	while (doneBytes < len) {
// 		size_t chunk = min(writeBuffsz - (offs % writeBuffsz), len - doneBytes);

// 		ftmctrl_WrEn(ctx->ftmctrl);
// 		res = flash_writeBuffer(ctx, offs, src, chunk, CFI_TIMEOUT_MAX_PROGRAM(ctx->cfi.toutTypical.bufWrite, ctx->cfi.toutMax.bufWrite));
// 		ftmctrl_WrDis(ctx->ftmctrl);

// 		if (res < 0) {
// 			break;
// 		}

// 		doneBytes += chunk;
// 		src += chunk;
// 		offs += chunk;
// 	}

// 	*retlen = doneBytes;

// 	return res;
// }


// static ssize_t _flashdrv_mtdWriteCb(uint64_t offs, const void *buff, size_t len, cache_devCtx_t *ctx)
// {
// 	storage_t *strg = ctx->strg;
// 	size_t retlen;
// 	ssize_t ret;

// 	if ((offs % strg->dev->mtd->writesz) != 0 || (len % strg->dev->mtd->writesz) != 0) {
// 		return -EINVAL;
// 	}

// 	ret = _flashdrv_mtdWrite(strg, offs, buff, len, &retlen);
// 	if (ret < 0) {
// 		return ret;
// 	}

// 	return (ssize_t)retlen;
// }


// static int flashdrv_mtdWrite(storage_t *strg, off_t offs, const void *buff, size_t len, size_t *retlen)
// {
// 	mutexLock(strg->dev->ctx->lock);
// 	int ret = cache_write(strg->dev->ctx->cache, offs, buff, len, LIBCACHE_POLICY);
// 	mutexUnlock(strg->dev->ctx->lock);

// 	if (ret < 0) {
// 		*retlen = 0;
// 	}
// 	else {
// 		*retlen = len;
// 		ret = 0;
// 	}

// 	return ret;
// }


// static int flashdrv_mtdErase(storage_t *strg, off_t offs, size_t len)
// {
// 	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL)) {
// 		return -EINVAL;
// 	}

// 	struct _storage_devCtx_t *ctx = strg->dev->ctx;
// 	if ((fdrv_isValidAddress(CFI_SIZE(ctx->cfi.chipSz), offs, len) == 0) || ((offs & (ctx->sectorsz - 1)) != 0) || (len % ctx->sectorsz != 0)) {
// 		return -EINVAL;
// 	}

// 	if (len == 0u) {
// 		return EOK;
// 	}

// 	mutexLock(ctx->lock);
// 	ftmctrl_WrEn(ctx->ftmctrl);

// 	off_t end;
// 	int res = -ENOSYS;
// 	if ((offs == 0) && (len == CFI_SIZE(ctx->cfi.chipSz))) {
// 		TRACE("erasing entire memory");
// 		res = flash_chipErase(ctx, CFI_TIMEOUT_MAX_ERASE(ctx->cfi.toutTypical.chipErase, ctx->cfi.toutMax.chipErase));
// 		end = CFI_SIZE(ctx->cfi.chipSz);
// 	}
// 	else {
// 		end = flash_getSectorOffset(ctx, offs + len + ctx->sectorsz - 1u);
// 		TRACE("erasing sectors from 0x%llx to 0x%llx", offs, end);
// 	}

// 	if (res == -ENOSYS) {
// 		off_t secOffs = offs;
// 		while (secOffs < end) {
// 			res = flash_sectorErase(ctx, secOffs, CFI_TIMEOUT_MAX_ERASE(ctx->cfi.toutTypical.blkErase, ctx->cfi.toutMax.blkErase));
// 			if (res < 0) {
// 				break;
// 			}
// 			secOffs += ctx->sectorsz;
// 		}
// 	}

// 	ftmctrl_WrDis(ctx->ftmctrl);
// 	res = cache_invalidate(ctx->cache, offs, end);
// 	mutexUnlock(ctx->lock);

// 	return res;
// }


// static const storage_mtdops_t mtdOps = {
// 	.erase = flashdrv_mtdErase,
// 	.unPoint = NULL,
// 	.point = NULL,
// 	.read = flashdrv_mtdRead,
// 	.write = flashdrv_mtdWrite,

// 	.meta_read = NULL,
// 	.meta_write = NULL,

// 	.sync = NULL,
// 	.lock = NULL,
// 	.unLock = NULL,
// 	.isLocked = NULL,

// 	.block_isBad = NULL,
// 	.block_isReserved = NULL,
// 	.block_markBad = NULL,
// 	.block_maxBadNb = NULL,

// 	.suspend = NULL,
// 	.resume = NULL,
// 	.reboot = NULL,
// };


static void flashdrv_destroy(storage_t *strg)
{
}


static storage_t *flashdrv_init(void)
{
	return NULL;
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
