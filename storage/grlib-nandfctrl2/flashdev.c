/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash device interface
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/minmax.h>

#include "nandfctrl2-flashdev.h"
#include "nandfctrl2-flashdrv.h"


int flashdev_readRaw(struct _storage_t *strg, off_t offs, void *buff, size_t len, size_t *retlen)
{
	int ret = 0;
	size_t doneBytes = 0;
	const size_t rawPagesz = strg->dev->mtd->writesz + strg->dev->mtd->metaSize;
	uint32_t page = offs / rawPagesz;
	off_t pageOffs = offs % rawPagesz;

	mutexLock(strg->dev->ctx->lock);
	while (doneBytes < len) {
		ret = flashdrv_readRaw(strg->dev->ctx->die, page, strg->dev->ctx->databuf);
		if (ret < 0) {
			break;
		}

		size_t chunk = min(rawPagesz - pageOffs, len - doneBytes);
		memcpy((uint8_t *)buff + doneBytes, (uint8_t *)strg->dev->ctx->databuf + pageOffs, chunk);
		doneBytes += chunk;
		pageOffs = 0;
		++page;
	}
	mutexUnlock(strg->dev->ctx->lock);

	*retlen = doneBytes;

	return ret;
}


int flashdev_writeRaw(struct _storage_t *strg, off_t offs, const void *buff, size_t len, size_t *retlen)
{
	int ret = 0;
	size_t doneBytes = 0;
	const size_t rawPagesz = strg->dev->mtd->writesz + strg->dev->mtd->metaSize;
	uint32_t page = offs / rawPagesz;
	off_t pageOffs = offs % rawPagesz;

	mutexLock(strg->dev->ctx->lock);
	while (doneBytes < len) {
		size_t chunk = min(rawPagesz - pageOffs, len - doneBytes);

		if (chunk != rawPagesz) {
			memset(strg->dev->ctx->databuf, 0xffU, rawPagesz);
		}
		memcpy((uint8_t *)strg->dev->ctx->databuf + pageOffs, buff, chunk);

		ret = flashdrv_writeRaw(strg->dev->ctx->die, page, strg->dev->ctx->databuf);
		if (ret < 0) {
			break;
		}
		doneBytes += chunk;
		pageOffs = 0;
		++page;
	}
	mutexUnlock(strg->dev->ctx->lock);

	*retlen = doneBytes;

	return ret;
}


/* MTD interface */


static int flashmtd_erase(struct _storage_t *strg, off_t offs, size_t size)
{
	if ((offs % strg->dev->mtd->erasesz) != 0 || (size % strg->dev->mtd->erasesz) != 0) {
		return -EINVAL;
	}

	uint32_t erased = 0;
	uint32_t startBlock = offs / strg->dev->mtd->erasesz;
	uint32_t endBlock = (offs + size) / strg->dev->mtd->erasesz;

	mutexLock(strg->dev->ctx->lock);
	for (uint32_t block = startBlock; block < endBlock; ++block) {
		int ret = flashdrv_isbad(strg->dev->ctx->die, block);
		if (ret != 0) {
			printf("%s: skipping bad block: %u\n", __func__, block);
			continue;
		}

		ret = flashdrv_erase(strg->dev->ctx->die, block);
		if (ret < 0) {
			printf("%s: error while erasing block: %u - marking as badblock\n", __func__, block);
			ret = flashdrv_markbad(strg->dev->ctx->die, block);
			if (ret < 0) {
				mutexUnlock(strg->dev->ctx->lock);
				return ret; /* can't mark as badblock, unrecoverable error */
			}
		}
		else {
			++erased;
		}
	}
	mutexUnlock(strg->dev->ctx->lock);

	return erased;
}


static int flashmtd_read(struct _storage_t *strg, off_t offs, void *data, size_t len, size_t *retlen)
{
	int ret = 0;
	size_t doneBytes = 0;
	const size_t writesz = strg->dev->mtd->writesz;
	uint32_t page = offs / writesz;
	off_t pageOffs = offs % writesz;

	mutexLock(strg->dev->ctx->lock);
	while (doneBytes < len) {
		flashdrv_eccStatus_t eccStatus;

		int err = flashdrv_readPage(strg->dev->ctx->die, page, strg->dev->ctx->databuf, &eccStatus);
		if ((err != -EBADMSG) && (err != -EUCLEAN) && (err < 0)) {
			ret = -EIO;
			break;
		}

		if (err == -EBADMSG) {
			/* Continue reading, let the client handle -EBADMSG */
			ret = -EBADMSG;
		}
		else if (err == -EUCLEAN && ret != -EBADMSG) {
			/* Not a critical error, continue reading, return -EUCLEAN, but don't overwrite -EBADMSG */
			ret = -EUCLEAN;
		}

		size_t chunk = min(writesz - pageOffs, len - doneBytes);
		memcpy((uint8_t *)data + doneBytes, (uint8_t *)strg->dev->ctx->databuf + pageOffs, chunk);
		doneBytes += chunk;
		pageOffs = 0;
		++page;
	}
	*retlen = doneBytes;
	mutexUnlock(strg->dev->ctx->lock);

	return ret;
}


static int flashmtd_write(struct _storage_t *strg, off_t offs, const void *data, size_t len, size_t *retlen)
{
	const size_t writesz = strg->dev->mtd->writesz;

	if ((offs % writesz) != 0 || (len % writesz) != 0) {
		return -EINVAL;
	}

	int ret = 0;
	size_t doneBytes = 0;
	uint32_t page = offs / writesz;

	mutexLock(strg->dev->ctx->lock);
	while (doneBytes < len) {
		memcpy(strg->dev->ctx->databuf, (uint8_t *)data + doneBytes, writesz);

		ret = flashdrv_writePage(strg->dev->ctx->die, page, strg->dev->ctx->databuf);
		if (ret < 0) {
			break;
		}

		doneBytes += writesz;
		++page;
	}
	mutexUnlock(strg->dev->ctx->lock);
	*retlen = doneBytes;

	return (ret < 0) ? ret : 0;
}


static int flashmtd_metaRead(struct _storage_t *strg, off_t offs, void *data, size_t len, size_t *retlen)
{
	const size_t writesz = strg->dev->mtd->writesz;

	if ((offs % writesz) != 0) {
		return -EINVAL;
	}

	int ret = 0;
	size_t doneBytes = 0;
	uint32_t page = offs / writesz;

	mutexLock(strg->dev->ctx->lock);
	while (doneBytes < len) {
		size_t chunksz = min(len - doneBytes, strg->dev->mtd->oobAvail);
		ret = flashdrv_metaRead(strg->dev->ctx->die, page, strg->dev->ctx->metabuf, chunksz);
		if (ret < 0) {
			break;
		}

		memcpy((uint8_t *)data + doneBytes, strg->dev->ctx->metabuf, chunksz);
		doneBytes += chunksz;
		++page;
	}
	mutexUnlock(strg->dev->ctx->lock);
	*retlen = doneBytes;

	return (ret < 0) ? ret : 0;
}


static int flashmtd_metaWrite(struct _storage_t *strg, off_t offs, const void *data, size_t len, size_t *retlen)
{
	if ((offs % strg->dev->mtd->writesz) != 0) {
		return -EINVAL;
	}

	int ret = 0;
	size_t doneBytes = 0;
	uint32_t page = offs / strg->dev->mtd->writesz;

	mutexLock(strg->dev->ctx->lock);
	while (doneBytes < len) {
		size_t chunksz = min(len - doneBytes, strg->dev->mtd->oobAvail);
		memcpy(strg->dev->ctx->metabuf, (const uint8_t *)data + doneBytes, chunksz);
		ret = flashdrv_metaWrite(strg->dev->ctx->die, page, strg->dev->ctx->metabuf, chunksz);
		if (ret < 0) {
			break;
		}

		doneBytes += chunksz;
		page++;
	}
	*retlen = doneBytes;
	mutexUnlock(strg->dev->ctx->lock);

	return (ret < 0) ? ret : 0;
}


static int flashmtd_blockIsBad(struct _storage_t *strg, off_t offs)
{
	if ((offs % strg->dev->mtd->erasesz) != 0) {
		return -EINVAL;
	}

	mutexLock(strg->dev->ctx->lock);
	int ret = flashdrv_isbad(strg->dev->ctx->die, (uint32_t)(offs / strg->dev->mtd->erasesz));
	mutexUnlock(strg->dev->ctx->lock);

	return ret;
}


static int flashmtd_blockMarkBad(struct _storage_t *strg, off_t offs)
{
	if ((offs % strg->dev->mtd->erasesz) != 0) {
		return -EINVAL;
	}

	mutexLock(strg->dev->ctx->lock);
	int ret = flashdrv_markbad(strg->dev->ctx->die, (uint32_t)(offs / strg->dev->mtd->erasesz));
	mutexUnlock(strg->dev->ctx->lock);

	return ret;
}


static int flashmtd_blockMaxBitflips(struct _storage_t *strg, off_t offs)
{
	storage_mtd_t *mtd = strg->dev->mtd;

	if ((offs % mtd->erasesz) != 0) {
		return -EINVAL;
	}

	int ret = 0;
	size_t tempsz = 0;
	unsigned int maxBitFlips = 0;
	uint32_t page = offs / mtd->writesz;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;

	mutexLock(ctx->lock);

	/* Read one block, page by page */
	while (tempsz < mtd->erasesz) {
		flashdrv_eccStatus_t eccStatus;
		ret = flashdrv_readPage(ctx->die, page, ctx->databuf, &eccStatus);
		if (ret == -EBADMSG) {
			/* Block is corrupted. No point in counting bitflips. */
			break;
		}
		else if ((ret < 0) && (ret != -EUCLEAN)) {
			/* Fatal hardware/DMA error */
			ret = -EIO;
			break;
		}

		if (eccStatus.eccState == flash_ecc_corrected) {
			maxBitFlips = max(maxBitFlips, eccStatus.worstBitflips);
		}

		tempsz += mtd->writesz;
		page++;
	}

	mutexUnlock(ctx->lock);

	return ((ret == -EBADMSG) || (ret == -EIO)) ? ret : (int)maxBitFlips;
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


/* Init/deinit functions */


void flashdev_done(storage_t *strg)
{
	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL)) {
		return;
	}

	if (strg->dev->ctx->databuf != NULL) {
		(void)munmap(strg->dev->ctx->databuf, 2 * strg->dev->mtd->writesz);
	}
	if (strg->dev->ctx->metabuf != NULL) {
		(void)munmap(strg->dev->ctx->metabuf, _PAGE_SIZE);
	}
	if (strg->dev->ctx->die != NULL) {
		flashdrv_dieFree(strg->dev->ctx->die);
	}
	(void)resourceDestroy(strg->dev->ctx->lock);

	free(strg->dev->ctx);
	free(strg->dev->mtd);
	free(strg->dev);
}


int flashdev_init(storage_t *strg, unsigned int target)
{
	/* Initialize device structure */
	strg->dev = malloc(sizeof(storage_dev_t));
	if (strg->dev == NULL) {
		return -ENOMEM;
	}

	/* Initialize device context */
	strg->dev->ctx = malloc(sizeof(storage_devCtx_t));
	if (strg->dev->ctx == NULL) {
		free(strg->dev);
		return -ENOMEM;
	}

	strg->dev->ctx->die = flashdrv_dieAlloc(target);
	if (strg->dev->ctx->die == NULL) {
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
	}

	const flashdrv_info_t *info = &strg->dev->ctx->die->info;

	/* Define start and size only for root flash memory */
	if (strg->parent == NULL) {
		strg->start = 0;
		strg->size = info->size;
	}

	strg->dev->ctx->databuf = mmap(NULL, 2 * info->writesz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (strg->dev->ctx->databuf == MAP_FAILED) {
		flashdrv_dieFree(strg->dev->ctx->die);
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
	}

	strg->dev->ctx->metabuf = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (strg->dev->ctx->metabuf == MAP_FAILED) {
		(void)munmap(strg->dev->ctx->databuf, 2 * info->writesz);
		flashdrv_dieFree(strg->dev->ctx->die);
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
	}

	if (mutexCreate(&strg->dev->ctx->lock) < 0) {
		(void)munmap(strg->dev->ctx->databuf, 2 * info->writesz);
		(void)munmap(strg->dev->ctx->metabuf, _PAGE_SIZE);
		flashdrv_dieFree(strg->dev->ctx->die);
		free(strg->dev->ctx);
		free(strg->dev);
		return -EINVAL;
	}

	/* NAND flash driver supports MTD interface */
	strg->dev->mtd = malloc(sizeof(storage_mtd_t));
	if (strg->dev->mtd == NULL) {
		(void)resourceDestroy(strg->dev->ctx->lock);
		(void)munmap(strg->dev->ctx->databuf, 2 * info->writesz);
		(void)munmap(strg->dev->ctx->metabuf, _PAGE_SIZE);
		flashdrv_dieFree(strg->dev->ctx->die);
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
	}

	strg->dev->mtd->ops = &mtdOps;
	strg->dev->mtd->type = mtd_nandFlash;
	strg->dev->mtd->name = info->name;
	strg->dev->mtd->metaSize = info->sparesz;
	strg->dev->mtd->oobSize = info->sparesz;
	strg->dev->mtd->oobAvail = info->spareavail;
	strg->dev->mtd->writesz = info->writesz;
	strg->dev->mtd->writeBuffsz = info->writesz;
	strg->dev->mtd->erasesz = info->erasesz;

	/* NAND does not support block device yet */
	strg->dev->blk = NULL;

	return 0;
}
