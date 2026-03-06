/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash device interface.
 * Implements the storage MTD operations using the low-level flashdrv API.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/mman.h>
#include <sys/minmax.h>
#include <sys/threads.h>

#include "nandfctrl2-flashdev.h"
#include "nandfctrl2-flashdrv.h"


/* clang-format off */
#define LOG(str_, ...)       do { printf("nandfctrl2-flash: " str_ "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(str_, ...) LOG("error: %s:%d: " str_, __FILE__, __LINE__, ##__VA_ARGS__)
/* clang-format on */


/* ============================== MTD operations ============================== */


static int flashmtd_erase(struct _storage_t *strg, off_t offs, size_t size)
{
	const storage_mtd_t *mtd = strg->dev->mtd;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	uint32_t startBlock, endBlock, pagesPerBlock;
	uint32_t eb, pageID;
	int erased = 0;
	int res;

	if (((offs % mtd->erasesz) != 0) || ((size % mtd->erasesz) != 0)) {
		return -EINVAL;
	}

	startBlock = (uint32_t)(offs / mtd->erasesz);
	endBlock = (uint32_t)((offs + size) / mtd->erasesz);
	pagesPerBlock = (uint32_t)(mtd->erasesz / mtd->writesz);

	mutexLock(ctx->lock);
	for (eb = startBlock; eb < endBlock; eb++) {
		pageID = eb * pagesPerBlock;

		res = flashdrv_isbad(ctx->dma, pageID);
		if (res != 0) {
			LOG("erase: skipping bad block %u", eb);
			continue;
		}

		res = flashdrv_erase(ctx->dma, pageID);
		if (res < 0) {
			LOG_ERROR("erase failed on block %u, marking bad", eb);
			res = flashdrv_markbad(ctx->dma, pageID);
			if (res < 0) {
				mutexUnlock(ctx->lock);
				return res;
			}
		}
		else {
			erased++;
		}
	}
	mutexUnlock(ctx->lock);

	return erased;
}


static int flashmtd_read(struct _storage_t *strg, off_t offs, void *data, size_t len, size_t *retlen)
{
	const storage_mtd_t *mtd = strg->dev->mtd;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	flashdrv_eccStatus_t eccStatus;
	uint32_t paddr;
	size_t tempsz = 0, chunksz;
	off_t pageOffs;
	int err, ret = 0;

	paddr = (uint32_t)(offs / mtd->writesz);
	pageOffs = offs % mtd->writesz;

	mutexLock(ctx->lock);
	while (tempsz < len) {
		err = flashdrv_read(ctx->dma, paddr, ctx->databuf, &eccStatus);
		if (err == -EBADMSG) {
			ret = -EBADMSG;
		}
		else if ((err == -EUCLEAN) && (ret != -EBADMSG)) {
			ret = -EUCLEAN;
		}
		else if (err < 0) {
			ret = err;
			break;
		}

		chunksz = min(len - tempsz, (size_t)(mtd->writesz - pageOffs));
		memcpy((uint8_t *)data + tempsz, (const uint8_t *)ctx->databuf + pageOffs, chunksz);
		pageOffs = 0;
		tempsz += chunksz;
		paddr++;
	}
	*retlen = tempsz;
	mutexUnlock(ctx->lock);

	return ret;
}


static int flashmtd_write(struct _storage_t *strg, off_t offs, const void *data, size_t len, size_t *retlen)
{
	const storage_mtd_t *mtd = strg->dev->mtd;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	uint32_t paddr;
	size_t tempsz = 0;
	int res = 0;

	if (((offs % mtd->writesz) != 0) || ((len % mtd->writesz) != 0)) {
		return -EINVAL;
	}

	paddr = (uint32_t)(offs / mtd->writesz);

	mutexLock(ctx->lock);
	while (tempsz < len) {
		memcpy(ctx->databuf, (const uint8_t *)data + tempsz, mtd->writesz);

		res = flashdrv_write(ctx->dma, paddr, ctx->databuf, NULL);
		if (res < 0) {
			res = -EIO;
			break;
		}

		tempsz += mtd->writesz;
		paddr++;
	}
	*retlen = tempsz;
	mutexUnlock(ctx->lock);

	return (res < 0) ? res : 0;
}


static int flashmtd_metaRead(struct _storage_t *strg, off_t offs, void *data, size_t len, size_t *retlen)
{
	const storage_mtd_t *mtd = strg->dev->mtd;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	flashdrv_eccStatus_t eccStatus;
	uint32_t paddr;
	size_t tempsz = 0, chunksz;
	int err, ret = 0;

	if ((offs % mtd->writesz) != 0) {
		return -EINVAL;
	}

	paddr = (uint32_t)(offs / mtd->writesz);

	mutexLock(ctx->lock);
	while (tempsz < len) {
		/* Read spare (OOB) only - data=NULL triggers spare-only read  */
		err = flashdrv_read(ctx->dma, paddr, NULL, &eccStatus);
		if (err == -EBADMSG) {
			ret = -EBADMSG;
		}
		else if ((err == -EUCLEAN) && (ret != -EBADMSG)) {
			ret = -EUCLEAN;
		}
		else if (err < 0) {
			ret = err;
			break;
		}

		chunksz = min(len - tempsz, (size_t)mtd->oobSize);
		memcpy((uint8_t *)data + tempsz, meta->metadata, chunksz);
		tempsz += chunksz;
		paddr++;
	}
	*retlen = tempsz;
	mutexUnlock(ctx->lock);

	return ret;
}


static int flashmtd_metaWrite(struct _storage_t *strg, off_t offs, const void *data, size_t len, size_t *retlen)
{
	const storage_mtd_t *mtd = strg->dev->mtd;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	flashdrv_meta_t *meta = ctx->metabuf;
	size_t tempsz = 0, chunksz;
	uint32_t paddr;
	int res = 0;

	if ((offs % mtd->writesz) != 0) {
		return -EINVAL;
	}

	paddr = (uint32_t)(offs / mtd->writesz);

	mutexLock(ctx->lock);
	while (tempsz < len) {
		chunksz = min(len - tempsz, (size_t)mtd->oobSize);

		/* Copy user OOB data into the DMA-accessible metabuf */
		memset(meta->metadata, 0xff, mtd->oobSize);
		memcpy(meta->metadata, (const uint8_t *)data + tempsz, chunksz);

		/* Write metadata bytes to spare area end (EDAC disabled in driver) */
		res = flashdrv_write(ctx->dma, paddr, NULL, meta->metadata);
		if (res < 0) {
			res = -EIO;
			break;
		}

		tempsz += chunksz;
		paddr++;
	}
	*retlen = tempsz;
	mutexUnlock(ctx->lock);

	return (res < 0) ? res : 0;
}


static int flashmtd_blockIsBad(struct _storage_t *strg, off_t offs)
{
	const storage_mtd_t *mtd = strg->dev->mtd;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	int ret;

	if ((offs % mtd->erasesz) != 0) {
		return -EINVAL;
	}

	mutexLock(ctx->lock);
	ret = flashdrv_isbad(ctx->dma, (uint32_t)(offs / mtd->writesz));
	mutexUnlock(ctx->lock);

	return ret;
}


static int flashmtd_blockMarkBad(struct _storage_t *strg, off_t offs)
{
	const storage_mtd_t *mtd = strg->dev->mtd;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	int ret;

	if ((offs % mtd->erasesz) != 0) {
		return -EINVAL;
	}

	mutexLock(ctx->lock);
	ret = flashdrv_markbad(ctx->dma, (uint32_t)(offs / mtd->writesz));
	mutexUnlock(ctx->lock);

	return ret;
}


static int flashmtd_blockMaxBitflips(struct _storage_t *strg, off_t offs)
{
	const storage_mtd_t *mtd = strg->dev->mtd;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	uint32_t paddr;
	size_t tempsz;
	int err;
	int maxBitflips = 0;

	if ((offs % mtd->erasesz) != 0) {
		return -EINVAL;
	}

	paddr = (uint32_t)(offs / mtd->writesz);

	mutexLock(ctx->lock);
	for (tempsz = 0; tempsz < (size_t)mtd->erasesz; tempsz += mtd->writesz) {
		flashdrv_eccStatus_t eccStatus;
		/* Read page data + fill meta with ECC status */
		err = flashdrv_read(ctx->dma, paddr, ctx->databuf, &eccStatus);
		if (err < 0 && (err != -EUCLEAN) && (err != -EBADMSG)) {
			break;
		}

		if (eccStatus.worstBitflips > maxBitflips) {
			maxBitflips = eccStatus.worstBitflips;
		}
		paddr++;
	}
	mutexUnlock(ctx->lock);

	return (err < 0 && err != -EUCLEAN && err != -EBADMSG) ? -EIO : maxBitflips;
}


static const storage_mtdops_t mtdOps = {
	.erase = flashmtd_erase,
	.unPoint = NULL,
	.point = NULL,
	.read = flashmtd_read,
	.write = flashmtd_write,
	.meta_read = flashmtd_metaRead,
	.meta_write = flashmtd_metaWrite,
	.sync = NULL,
	.lock = NULL,
	.unLock = NULL,
	.isLocked = NULL,
	.block_isBad = flashmtd_blockIsBad,
	.block_isReserved = NULL,
	.block_markBad = flashmtd_blockMarkBad,
	.block_maxBadNb = NULL,
	.block_maxBitflips = flashmtd_blockMaxBitflips,
	.suspend = NULL,
	.resume = NULL,
	.reboot = NULL,
};


/* ============================== Initialisation ============================== */


int flashdev_done(storage_t *strg)
{
	if ((strg == NULL) || (strg->dev == NULL)) {
		return -EINVAL;
	}

	if (strg->dev->ctx != NULL) {
		if (strg->dev->ctx->metabuf != NULL) {
			(void)munmap(strg->dev->ctx->metabuf, _PAGE_SIZE);
		}
		if (strg->dev->ctx->databuf != NULL) {
			const flashdrv_info_t *info = flashdrv_info(strg->dev->ctx->target);
			size_t sz = (info != NULL) ? (2u * info->writesz) : (2u * _PAGE_SIZE);
			(void)munmap(strg->dev->ctx->databuf, sz);
		}
		if (strg->dev->ctx->dma != NULL) {
			flashdrv_dmadestroy(strg->dev->ctx->dma);
		}
		resourceDestroy(strg->dev->ctx->lock);
		free(strg->dev->ctx);
	}

	if (strg->dev->mtd != NULL) {
		free(strg->dev->mtd);
	}

	free(strg->dev);
	return 0;
}


int flashdev_init(storage_t *strg, unsigned int target)
{
	const flashdrv_info_t *info;
	int err;

	if (strg == NULL) {
		return -EINVAL;
	}

	info = flashdrv_info(target);
	if (info == NULL) {
		return -EINVAL;
	}

	/* Set size/start for the root (non-partition) storage object */
	if (strg->parent == NULL) {
		strg->start = 0;
		strg->size = info->size;
	}

	strg->dev = malloc(sizeof(storage_dev_t));
	if (strg->dev == NULL) {
		return -ENOMEM;
	}
	memset(strg->dev, 0, sizeof(storage_dev_t));

	strg->dev->ctx = malloc(sizeof(storage_devCtx_t));
	if (strg->dev->ctx == NULL) {
		free(strg->dev);
		return -ENOMEM;
	}
	memset(strg->dev->ctx, 0, sizeof(storage_devCtx_t));
	strg->dev->ctx->target = target;

	/* DMA descriptor chain for this device */
	strg->dev->ctx->dma = flashdrv_dmanew(target);
	if (strg->dev->ctx->dma == NULL) {
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
	}

	/* Uncached, contiguous data buffer (2 * writesz for alignment headroom) */
	strg->dev->ctx->databuf = mmap(NULL, 2u * info->writesz, PROT_READ | PROT_WRITE,
			MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (strg->dev->ctx->databuf == MAP_FAILED) {
		strg->dev->ctx->databuf = NULL;
		err = -ENOMEM;
		goto err_databuf;
	}

	/*
	 * Uncached, contiguous metadata buffer.
	 * Contains a flashdrv_meta_t with metadata[] at offset 0 for DMA safety.
	 * We allocate a full page to ensure page alignment.
	 */
	strg->dev->ctx->metabuf = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
			MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (strg->dev->ctx->metabuf == MAP_FAILED) {
		strg->dev->ctx->metabuf = NULL;
		err = -ENOMEM;
		goto err_metabuf;
	}

	err = mutexCreate(&strg->dev->ctx->lock);
	if (err < 0) {
		goto err_mutex;
	}

	strg->dev->mtd = malloc(sizeof(storage_mtd_t));
	if (strg->dev->mtd == NULL) {
		err = -ENOMEM;
		goto err_mtd;
	}

	strg->dev->mtd->ops = &mtdOps;
	strg->dev->mtd->type = mtd_nandFlash;
	strg->dev->mtd->name = info->name;
	strg->dev->mtd->metaSize = info->sparesz;
	strg->dev->mtd->oobSize = info->spareavail;
	strg->dev->mtd->oobAvail = info->spareavail;
	strg->dev->mtd->writesz = info->writesz;
	strg->dev->mtd->writeBuffsz = info->writesz;
	strg->dev->mtd->erasesz = info->erasesz;

	strg->dev->blk = NULL; /* NAND does not expose a block device */

	return 0;

err_mtd:
	resourceDestroy(strg->dev->ctx->lock);
err_mutex:
	(void)munmap(strg->dev->ctx->metabuf, _PAGE_SIZE);
err_metabuf:
	(void)munmap(strg->dev->ctx->databuf, 2u * info->writesz);
err_databuf:
	flashdrv_dmadestroy(strg->dev->ctx->dma);
	free(strg->dev->ctx);
	free(strg->dev);
	return err;
}
