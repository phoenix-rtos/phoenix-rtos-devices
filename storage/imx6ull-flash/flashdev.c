/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND flash device interface
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/minmax.h>

#include "imx6ull-flashdev.h"
#include "imx6ull-flashdrv.h"

#define ECC_BITFLIP_THRESHOLD 10
#define ECCN_BITFLIP_STRENGHT 14
#define ECC0_BITFLIP_STRENGHT 16


/* Auxiliary functions */

/* Function derived from the previous code phoenix-rtos-filesystems/jffs/phoenix-rtos/mtd.c */
static int _flashmtd_errRead(int ret, flashdrv_meta_t *meta, int status, void *data)
{
	int max_bitflip = 0;
	unsigned *blk;
	int i, j, err = 0;
	/* check current read status. if it is EBADMSG we have nothing to do here since
	 * read is marked as uncorrectable */
	if (status == -EBADMSG) {
		return status;
	}

	/* check status for every block */
	for (i = 0; i < sizeof(meta->errors) / sizeof(meta->errors[0]); i++) {
		/* no error or all ones */
		if (meta->errors[i] == 0x0 || meta->errors[i] == 0xff) {
			continue;
		}
		/* uncorrectable errors but maybe block is erased and we can still use it */
		/* TODO: check if this actually works */
		else if (meta->errors[i] == 0xfe) {
			err = 0;
			if (i == 0) {
				blk = (unsigned *)meta->metadata;
				for (j = 0; j < sizeof(meta->metadata) / sizeof(unsigned); ++j) {
					err += (sizeof(unsigned) * 8) - __builtin_popcount(*blk + j);
				}

				if (err > ECC0_BITFLIP_STRENGHT) {
					return -EBADMSG;
				}

				memset(blk, 0xff, sizeof(meta->metadata));
				max_bitflip = max(max_bitflip, err);
			}
			else {
				blk = data + ((i - 1) * 512);
				for (j = 0; j < 128; j++) {
					err += (sizeof(unsigned) * 8) - __builtin_popcount(*(blk + j));
				}

				if (err > ECCN_BITFLIP_STRENGHT) {
					return -EBADMSG;
				}

				memset(blk, 0xff, 512);
				max_bitflip = max(max_bitflip, err);
			}
		}
		/* correctable errors */
		else {
			max_bitflip = max(max_bitflip, (int)meta->errors[i]);
		}
	}

	return max_bitflip >= ECC_BITFLIP_THRESHOLD ? -EUCLEAN : status;
}


/* MTD interface */

static int flashmtd_erase(struct _storage_t *strg, off_t offs, size_t size)
{
	int res;
	uint32_t pageID;
	uint32_t eb, startBlock, endBlock, erased = 0; /* values in erase block units */
	unsigned int pagesPerBlock;

	if ((offs % strg->dev->mtd->erasesz) != 0 || (size % strg->dev->mtd->erasesz) != 0) {
		return -EINVAL;
	}

	startBlock = offs / strg->dev->mtd->erasesz;
	endBlock = (offs + size) / strg->dev->mtd->erasesz;
	pagesPerBlock = strg->dev->mtd->erasesz / strg->dev->mtd->writesz;

	mutexLock(strg->dev->ctx->lock);
	for (eb = startBlock; eb < endBlock; ++eb) {
		pageID = eb * pagesPerBlock;

		res = flashdrv_isbad(strg->dev->ctx->dma, pageID);
		if (res != 0) {
			printf("flashmtd_erase: skipping bad block: %u\n", eb);
			continue;
		}

		res = flashdrv_erase(strg->dev->ctx->dma, pageID);
		if (res < 0) {
			printf("flashmtd_erase: error while erasing block: %u - marking as badblock", eb);
			res = flashdrv_markbad(strg->dev->ctx->dma, pageID);
			if (res < 0) {
				mutexUnlock(strg->dev->ctx->lock);
				return res; /* can't mark as badblock, urecoverable error */
			}
		}
		else {
			++erased;
		}
	}
	mutexUnlock(strg->dev->ctx->lock);

	return erased;
}


static ssize_t flashmtd_read(struct _storage_t *strg, off_t offs, void *data, size_t len)
{
	int res, err = 0;
	size_t tempsz = 0, chunksz;

	mutexLock(strg->dev->ctx->lock);
	if (offs % strg->dev->mtd->writesz) {
		res = flashdrv_read(strg->dev->ctx->dma, offs / strg->dev->mtd->writesz, strg->dev->ctx->databuf, strg->dev->ctx->metabuf);
		if (res == flash_uncorrectable) {
			mutexUnlock(strg->dev->ctx->lock);
			return res;
		}

		err = _flashmtd_errRead(res, strg->dev->ctx->metabuf, err, strg->dev->ctx->databuf);
		/* TODO: handle error from _flashmtd_errRead */

		chunksz = (strg->dev->mtd->writesz - (offs % strg->dev->mtd->writesz));
		if (chunksz > len) {
			chunksz = len;
		}

		memcpy(data, (unsigned char *)strg->dev->ctx->databuf + (offs % strg->dev->mtd->writesz), chunksz);
		tempsz += chunksz;
		offs += chunksz;
	}

	while (tempsz < len) {
		res = flashdrv_read(strg->dev->ctx->dma, offs / strg->dev->mtd->writesz, strg->dev->ctx->databuf, strg->dev->ctx->metabuf);
		if (res == flash_uncorrectable) {
			mutexUnlock(strg->dev->ctx->lock);
			break;
		}

		err = _flashmtd_errRead(res, strg->dev->ctx->metabuf, err, strg->dev->ctx->databuf);
		/* TODO: handle error from _flashmtd_errRead */

		chunksz = ((len - tempsz) > strg->dev->mtd->writesz) ? strg->dev->mtd->writesz : (len - tempsz);
		memcpy((unsigned char *)data + tempsz, strg->dev->ctx->databuf, chunksz);
		tempsz += chunksz;
		offs += chunksz;
	}

	mutexUnlock(strg->dev->ctx->lock);

	return tempsz > 0 ? tempsz : err;
}


static ssize_t flashmtd_write(struct _storage_t *strg, off_t offs, const void *data, size_t len)
{
	ssize_t res = 0;
	blkcnt_t pageID;
	size_t tempsz = 0;

	if ((offs % strg->dev->mtd->writesz) != 0 || (len % strg->dev->mtd->writesz) != 0) {
		return -EINVAL;
	}

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < len) {
		/* TODO: should we skip badblocks ? */
		memcpy(strg->dev->ctx->databuf, (unsigned char *)data + tempsz, strg->dev->mtd->writesz);

		pageID = offs / strg->dev->mtd->writesz;
		res = flashdrv_write(strg->dev->ctx->dma, pageID, strg->dev->ctx->databuf, NULL);
		if (res < 0) {
			break;
		}

		tempsz += strg->dev->mtd->writesz;
		offs += strg->dev->mtd->writesz;
	}
	mutexUnlock(strg->dev->ctx->lock);

	return tempsz > 0 ? tempsz : res;
}


static ssize_t flashmtd_metaRead(struct _storage_t *strg, off_t offs, void *data, size_t len)
{
	int res = 0, err = 0;
	size_t tempsz = 0, chunksz;
	flashdrv_meta_t *meta = strg->dev->ctx->metabuf;

	if ((offs % strg->dev->mtd->writesz) != 0) {
		return -EINVAL;
	}

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < len) {
		memset(meta->errors, 0, sizeof(meta->errors));
		/* TODO: should we skip badblocks ? */
		res = flashdrv_read(strg->dev->ctx->dma, offs / strg->dev->mtd->writesz, NULL, strg->dev->ctx->metabuf);
		err = _flashmtd_errRead(res, strg->dev->ctx->metabuf, err, strg->dev->ctx->databuf);
		/* TODO: handle error from _flashmtd_errRead */

		chunksz = len > strg->dev->mtd->oobSize ? strg->dev->mtd->oobSize : len;
		memcpy((unsigned char *)data + tempsz, strg->dev->ctx->metabuf, chunksz);

		tempsz += chunksz;
		offs += strg->dev->mtd->writesz;
	}
	mutexUnlock(strg->dev->ctx->lock);

	return tempsz > 0 ? tempsz : err;
}


static ssize_t flashmtd_metaWrite(struct _storage_t *strg, off_t offs, const void *data, size_t len)
{
	int res = EOK;
	size_t tempsz = 0, chunksz;

	if ((offs % strg->dev->mtd->writesz) != 0) {
		return -EINVAL;
	}

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < len) {
		chunksz = len > strg->dev->mtd->oobSize ? strg->dev->mtd->oobSize : len;
		memset(strg->dev->ctx->metabuf, 0xff, strg->dev->mtd->oobSize);
		memcpy(strg->dev->ctx->metabuf, (unsigned char *)data + tempsz, chunksz);

		res = flashdrv_write(strg->dev->ctx->dma, offs / strg->dev->mtd->writesz, NULL, strg->dev->ctx->metabuf);
		if (res < 0) {
			break;
		}

		tempsz += chunksz;
		offs += strg->dev->mtd->writesz;
	}
	mutexUnlock(strg->dev->ctx->lock);

	return tempsz > 0 ? tempsz : res;
}


static int flashmtd_blockIsBad(struct _storage_t *strg, off_t offs)
{
	int ret;

	if ((offs % strg->dev->mtd->erasesz) != 0) {
		return -EINVAL;
	}

	mutexLock(strg->dev->ctx->lock);
	/* address in pages, divide by pagesize */
	ret = flashdrv_isbad(strg->dev->ctx->dma, (offs / strg->dev->mtd->writesz));
	mutexUnlock(strg->dev->ctx->lock);

	return ret;
}


static int flashmtd_blockMarkBad(struct _storage_t *strg, off_t offs)
{
	int ret;

	if ((offs % strg->dev->mtd->erasesz) != 0) {
		return -EINVAL;
	}

	mutexLock(strg->dev->ctx->lock);
	/* address in pages, divide by pagesize */
	ret = flashdrv_markbad(strg->dev->ctx->dma, (offs / strg->dev->mtd->writesz));
	mutexUnlock(strg->dev->ctx->lock);

	return ret;
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

	.suspend = NULL,
	.resume = NULL,
	.reboot = NULL,
};


/* Initialization functions */

int flashdev_done(storage_t *strg)
{
	/* Free device context */
	munmap(strg->dev->ctx->metabuf, strg->dev->mtd->writesz);
	munmap(strg->dev->ctx->databuf, 2 * strg->dev->mtd->writesz);
	flashdrv_dmadestroy(strg->dev->ctx->dma);
	resourceDestroy(strg->dev->ctx->lock);
	free(strg->dev->ctx);

	free(strg->dev->mtd);
	free(strg->dev);

	return EOK;
}


int flashdev_init(storage_t *strg)
{
	const flashdrv_info_t *info = flashdrv_info();

	if (strg == NULL || info == NULL) {
		return -EINVAL;
	}

	/* Define start and size only for root flash memory */
	if (strg->parent == NULL) {
		strg->start = 0;
		strg->size = info->size;
	}

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

	strg->dev->ctx->dma = flashdrv_dmanew();
	if (strg->dev->ctx->dma == MAP_FAILED) {
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
	}

	strg->dev->ctx->databuf = mmap(NULL, 2 * info->writesz, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_CONTIGUOUS, 0);
	if (strg->dev->ctx->databuf == MAP_FAILED) {
		flashdrv_dmadestroy(strg->dev->ctx->dma);
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
	}

	strg->dev->ctx->metabuf = mmap(NULL, info->writesz, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_CONTIGUOUS, 0);
	if (strg->dev->ctx->metabuf == MAP_FAILED) {
		munmap(strg->dev->ctx->databuf, 2 * info->writesz);
		flashdrv_dmadestroy(strg->dev->ctx->dma);
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
	}

	if (mutexCreate(&strg->dev->ctx->lock) < 0) {
		munmap(strg->dev->ctx->metabuf, info->writesz);
		munmap(strg->dev->ctx->databuf, 2 * info->writesz);
		flashdrv_dmadestroy(strg->dev->ctx->dma);
		free(strg->dev->ctx);
		free(strg->dev);
		return -EINVAL;
	}

	/* NAND flash driver supports MTD interface */
	strg->dev->mtd = malloc(sizeof(storage_mtd_t));
	if (strg->dev->mtd == NULL) {
		resourceDestroy(strg->dev->ctx->lock);
		munmap(strg->dev->ctx->metabuf, info->writesz);
		munmap(strg->dev->ctx->databuf, 2 * info->writesz);
		flashdrv_dmadestroy(strg->dev->ctx->dma);
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
	}

	strg->dev->mtd->ops = &mtdOps;
	strg->dev->mtd->type = mtd_nandFlash;
	strg->dev->mtd->name = info->name;
	strg->dev->mtd->metaSize = info->metasz;
	strg->dev->mtd->oobSize = info->oobsz;
	strg->dev->mtd->oobAvail = info->oobavail;
	strg->dev->mtd->writesz = info->writesz;
	strg->dev->mtd->writeBuffsz = info->writesz;
	strg->dev->mtd->erasesz = info->erasesz;

	/* NAND does not support block device yet */
	strg->dev->blk = NULL;

	return EOK;
}
