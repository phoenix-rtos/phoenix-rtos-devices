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


#define ECC_BITFLIP_THRESHOLD 5 /* Min number of bitflips for page rewrite */


/* MTD interface */


static int flashmtd_erase(struct _storage_t *strg, off_t offs, size_t size)
{
	if ((offs % strg->dev->mtd->erasesz) != 0 || (size % strg->dev->mtd->erasesz) != 0) {
		return -EINVAL;
	}

	uint32_t startBlock = offs / strg->dev->mtd->erasesz;
	uint32_t endBlock = (offs + size) / strg->dev->mtd->erasesz;

	uint32_t erased = 0;

	mutexLock(strg->dev->ctx->lock);
	for (uint32_t block = startBlock; block < endBlock; ++block) {
		int ret = flashdrv_isbad(strg->dev->ctx->die, block);
		if (ret != 0) {
			printf("flashmtd_erase: skipping bad block: %u\n", block);
			continue;
		}

		ret = flashdrv_erase(strg->dev->ctx->die, block);
		if (ret < 0) {
			printf("flashmtd_erase: error while erasing block: %u - marking as badblock\n", block);
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
	size_t tempsz = 0;
	uint32_t paddr = offs / strg->dev->mtd->writesz;

	offs %= strg->dev->mtd->writesz;

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < len) {
		flashdrv_eccStatus_t eccStatus;

		/* TODO: should we skip badblocks ? */
		int err = flashdrv_read(strg->dev->ctx->die, paddr, strg->dev->ctx->databuf, &eccStatus);
		if (err < 0) {
			ret = -EIO;
			break;
		}

		if (eccStatus.eccState == flash_ecc_uncorrectable) {
			/* Continue reading, let the client handle -EBADMSG */
			ret = -EBADMSG;
		}
		else if ((eccStatus.eccState == flash_ecc_corrected) && (eccStatus.worstBitflips >= ECC_BITFLIP_THRESHOLD) && (ret != -EBADMSG)) {
			/* Page degraded but still readable - signal for rewrite, don't overwrite -EBADMSG */
			ret = -EUCLEAN;
		}

		size_t chunksz = min(len - tempsz, strg->dev->mtd->writesz - offs);
		memcpy((unsigned char *)data + tempsz, (unsigned char *)strg->dev->ctx->databuf + offs, chunksz);
		offs = 0;
		tempsz += chunksz;
		paddr++;
	}
	*retlen = tempsz;
	mutexUnlock(strg->dev->ctx->lock);

	return ret;
}


static int flashmtd_write(struct _storage_t *strg, off_t offs, const void *data, size_t len, size_t *retlen)
{
	if ((offs % strg->dev->mtd->writesz) != 0 || (len % strg->dev->mtd->writesz) != 0) {
		return -EINVAL;
	}

	int ret = 0;
	size_t tempsz = 0;

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < len) {
		/* TODO: should we skip badblocks ? */
		memcpy(strg->dev->ctx->databuf, (uint8_t *)data + tempsz, strg->dev->mtd->writesz);

		uint32_t page = offs / strg->dev->mtd->writesz;
		ret = flashdrv_write(strg->dev->ctx->die, page, strg->dev->ctx->databuf);
		if (ret < 0) {
			ret = -EIO;
			break;
		}

		tempsz += strg->dev->mtd->writesz;
		offs += strg->dev->mtd->writesz;
	}
	*retlen = tempsz;
	mutexUnlock(strg->dev->ctx->lock);

	return ret < 0 ? ret : 0;
}


static int flashmtd_metaRead(struct _storage_t *strg, off_t offs, void *data, size_t len, size_t *retlen)
{
	if ((offs % strg->dev->mtd->writesz) != 0) {
		return -EINVAL;
	}

	int ret = 0;
	size_t tempsz = 0;
	uint32_t paddr = offs / strg->dev->mtd->writesz;

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < len) {
		size_t chunksz = min(len - tempsz, (size_t)strg->dev->mtd->oobSize);
		/* TODO: should we skip badblocks ? */
		int err = nandfctrl2_readmeta(strg->dev->ctx->die, paddr, strg->dev->ctx->metabuf, chunksz);
		if (err < 0) {
			ret = err;
			break;
		}

		memcpy((unsigned char *)data + tempsz, strg->dev->ctx->metabuf, chunksz);
		tempsz += chunksz;
		paddr++;
	}
	*retlen = tempsz;
	mutexUnlock(strg->dev->ctx->lock);

	return ret;
}


static int flashmtd_metaWrite(struct _storage_t *strg, off_t offs, const void *data, size_t len, size_t *retlen)
{
	if ((offs % strg->dev->mtd->writesz) != 0) {
		return -EINVAL;
	}

	int ret = 0;
	size_t tempsz = 0;

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < len) {
		size_t chunksz = min(len - tempsz, (size_t)strg->dev->mtd->oobSize);
		memcpy(strg->dev->ctx->metabuf, (const unsigned char *)data + tempsz, chunksz);
		ret = nandfctrl2_writemeta(strg->dev->ctx->die, offs / strg->dev->mtd->writesz, strg->dev->ctx->metabuf, chunksz);
		if (ret < 0) {
			break;
		}

		tempsz += chunksz;
		offs += strg->dev->mtd->writesz;
	}
	*retlen = tempsz;
	mutexUnlock(strg->dev->ctx->lock);

	return ret < 0 ? ret : 0;
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

	int err = 0;
	size_t tempsz = 0;
	unsigned int maxBitFlips = 0;
	uint32_t paddr = offs / mtd->writesz;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;

	mutexLock(ctx->lock);

	/* Read one block, page by page */
	while (tempsz < mtd->erasesz) {
		flashdrv_eccStatus_t eccStatus;
		err = flashdrv_read(ctx->die, paddr, ctx->databuf, &eccStatus);
		if (err < 0) {
			err = -EIO;
			break;
		}

		if (eccStatus.eccState == flash_ecc_corrected) {
			maxBitFlips = max(maxBitFlips, eccStatus.worstBitflips);
		}

		tempsz += mtd->writesz;
		paddr++;
	}

	mutexUnlock(ctx->lock);

	return (err < 0) ? err : maxBitFlips;
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

	/* Free device context */
	if (strg->dev->ctx->metabuf != NULL) {
		munmap(strg->dev->ctx->metabuf, strg->dev->mtd->oobSize);
	}
	if (strg->dev->ctx->databuf != NULL) {
		munmap(strg->dev->ctx->databuf, 2 * strg->dev->mtd->writesz);
	}
	if (strg->dev->ctx->die != NULL) {
		flashdrv_dieFree(strg->dev->ctx->die);
	}
	resourceDestroy(strg->dev->ctx->lock);

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

	strg->dev->ctx->metabuf = mmap(NULL, info->spareavail, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (strg->dev->ctx->metabuf == MAP_FAILED) {
		munmap(strg->dev->ctx->databuf, 2 * info->writesz);
		flashdrv_dieFree(strg->dev->ctx->die);
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
	}

	if (mutexCreate(&strg->dev->ctx->lock) < 0) {
		munmap(strg->dev->ctx->metabuf, info->spareavail);
		munmap(strg->dev->ctx->databuf, 2 * info->writesz);
		flashdrv_dieFree(strg->dev->ctx->die);
		free(strg->dev->ctx);
		free(strg->dev);
		return -EINVAL;
	}

	/* NAND flash driver supports MTD interface */
	strg->dev->mtd = malloc(sizeof(storage_mtd_t));
	if (strg->dev->mtd == NULL) {
		resourceDestroy(strg->dev->ctx->lock);
		munmap(strg->dev->ctx->metabuf, info->spareavail);
		munmap(strg->dev->ctx->databuf, 2 * info->writesz);
		flashdrv_dieFree(strg->dev->ctx->die);
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
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

	/* NAND does not support block device yet */
	strg->dev->blk = NULL;

	return 0;
}
