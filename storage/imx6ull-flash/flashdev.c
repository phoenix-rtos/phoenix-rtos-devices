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
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/minmax.h>

#include "imx6ull-flashdev.h"
#include "imx6ull-flashdrv.h"

#define ECC_GF                13  /* Galois field */
#define ECC_BITFLIP_THRESHOLD 10  /* Min number of bitflips for page rewrite */
#define ECC0_BITFLIP_STRENGHT 16  /* ECC strength for metadata chunk */
#define ECCN_BITFLIP_STRENGHT 14  /* ECC strength for data chunk */
#define ECCN_DATA_SIZE        512 /* Data chunk size */

#define CHUNK_IS_META(chunkidx) ((chunkidx) == 0)


/* Auxiliary functions */
static inline int chunk_is_uncorrectable(unsigned int chunkidx, unsigned int flips)
{
	if (CHUNK_IS_META(chunkidx)) {
		return flips > ECC0_BITFLIP_STRENGHT;
	}
	else {
		return flips > ECCN_BITFLIP_STRENGHT;
	}
}


static unsigned int _flashmtd_checkErased(const void *buff, size_t boffs, size_t blen)
{
	const uint8_t *buff8 = buff;
	const uint32_t *buff32 = buff;
	unsigned int ret = 0;
	uint32_t data32;
	uint8_t data8;

	buff8 += boffs / CHAR_BIT;
	boffs %= CHAR_BIT;

	/* Check first byte */
	if (boffs > 0) {
		data8 = *buff8++;
		data8 |= (uint8_t)(0xff << (CHAR_BIT - boffs));

		/* Is it also last byte? */
		if (boffs + blen < CHAR_BIT) {
			data8 |= (uint8_t)(0xff >> (boffs + blen));
			blen = 0;
		}
		else {
			blen -= CHAR_BIT - boffs;
		}

		ret += CHAR_BIT - __builtin_popcount(data8);
	}

	/* Check bytes until 32-bit aligned address */
	while ((blen > CHAR_BIT) && (((uintptr_t)buff8) % sizeof(data32))) {
		data8 = *buff8++;
		blen -= CHAR_BIT;
		ret += CHAR_BIT - __builtin_popcount(data8);
	}

	/* Check 32-bit words */
	buff32 = (const uint32_t *)buff8;
	while (blen > CHAR_BIT * sizeof(data32)) {
		data32 = *buff32++;
		blen -= CHAR_BIT * sizeof(data32);

		if (data32 == 0xffffffff) {
			continue;
		}
		ret += CHAR_BIT * sizeof(data32) - __builtin_popcount(data32);
	}

	/* Check rest of the bytes */
	buff8 = (const uint8_t *)buff32;
	while (blen > CHAR_BIT) {
		data8 = *buff8++;
		blen -= CHAR_BIT;
		ret += CHAR_BIT - __builtin_popcount(data8);
	}

	/* Check last byte */
	if (blen > 0) {
		data8 = *buff8;
		data8 |= (uint8_t)(0xff >> blen);
		ret += CHAR_BIT - __builtin_popcount(data8);
	}

	return ret;
}


static void _flashmtd_chunkCorrect(struct _storage_t *strg, unsigned int chunkidx, unsigned int nflips)
{
	flashdrv_meta_t *meta = strg->dev->ctx->metabuf;
	unsigned char *data = strg->dev->ctx->databuf;

	if (CHUNK_IS_META(chunkidx)) {
		memset(meta, 0xff, strg->dev->mtd->oobSize);
	}
	else {
		memset(data + (chunkidx - 1) * ECCN_DATA_SIZE, 0xff, ECCN_DATA_SIZE);
	}
}


static int _flashmtd_chunkFlips(struct _storage_t *strg, flashdrv_meta_t *meta, uint32_t pageaddr, unsigned int chunkidx)
{
	storage_mtd_t *mtd = strg->dev->mtd;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	const size_t rawsz = (mtd->writesz + mtd->metaSize + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1);
	const size_t metalen = strg->dev->mtd->oobSize * CHAR_BIT + ECC_GF * ECC0_BITFLIP_STRENGHT;
	const size_t datalen = ECCN_DATA_SIZE * CHAR_BIT + ECC_GF * ECCN_BITFLIP_STRENGHT;
	void *raw = NULL;
	size_t boffs, blen;
	int ret;

	switch (meta->errors[chunkidx]) {
		case flash_no_errors:
		case flash_erased:
			ret = 0;
			break;

		case flash_uncorrectable:
			/* BCH reports chunk as uncorrectable in case of an erased page with bitflips within that chunk area */
			/* Check if that's the case by counting the bitflips with an assumption that the page is fully erased */
			/* Read raw page in order to check full chunk - both data and its ECC area */
			raw = mmap(NULL, rawsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
			if (raw == MAP_FAILED) {
				ret = -ENOMEM;
				raw = NULL;
				break;
			}

			/* Read raw page */
			ret = flashdrv_readraw(ctx->dma, pageaddr, raw, mtd->writesz + mtd->metaSize);
			if (ret < 0) {
				break;
			}

			if (CHUNK_IS_META(chunkidx)) {
				boffs = 0;
				blen = metalen;
			}
			else {
				boffs = metalen + (chunkidx - 1) * datalen;
				blen = datalen;
			}

			ret = _flashmtd_checkErased(raw, boffs, blen);

			/* Let the user see this chunk as correctly erased */
			if (!chunk_is_uncorrectable(chunkidx, ret)) {
				_flashmtd_chunkCorrect(strg, chunkidx, ret);
			}
			break;

		default:
			/* On any other case metadata error field contains number of corrected bits */
			ret = meta->errors[chunkidx];
			break;
	}

	if (raw != NULL) {
		munmap(raw, rawsz);
	}

	return ret;
}


/* Returns maximum number of bitlips or number smaller than 0 in case of error */
static int _flashmtd_blockMaxflips(struct _storage_t *strg, uint32_t pageaddr)
{
	int ret, i;
	unsigned int maxflips = 0;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	flashdrv_meta_t *meta = ctx->metabuf;
	const int nchunks = sizeof(meta->errors) / sizeof(meta->errors[0]);

	for (i = 0; i < nchunks; i++) {
		ret = _flashmtd_chunkFlips(strg, meta, pageaddr, i);
		if (ret < 0) {
			return ret;
		}
		maxflips = max(ret, maxflips);
	}

	return maxflips;
}


static int _flashmtd_checkECC(struct _storage_t *strg, uint32_t pageaddr, int nchunks)
{
	int ret = 0, chunkidx, nflips;
	unsigned int maxflips = 0;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	flashdrv_meta_t *meta = ctx->metabuf;

	for (chunkidx = 0; chunkidx < nchunks; chunkidx++) {
		nflips = _flashmtd_chunkFlips(strg, meta, pageaddr, chunkidx);
		if (nflips < 0) {
			return nflips;
		}

		if (chunk_is_uncorrectable(chunkidx, nflips)) {
			/* Continue reading, later return -EBADMSG */
			ret = -EBADMSG;
		}

		maxflips = max(nflips, maxflips);
	}

	if (ret < 0) {
		/* flash_uncorrectable has been detected on some chunks, return -EBADMSG */
		return ret;
	}

	if (maxflips >= ECC_BITFLIP_THRESHOLD) {
		/* Bitflips correctable, page degradated, but not a critical error */
		return -EUCLEAN;
	}

	/* Bitflips correctable, not worth raising an error */
	return 0;
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
			printf("flashmtd_erase: error while erasing block: %u - marking as badblock\n", eb);
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


static int flashmtd_read(struct _storage_t *strg, off_t offs, void *data, size_t len, size_t *retlen)
{
	uint32_t paddr;
	int err, ret = EOK;
	size_t tempsz = 0, chunksz;
	flashdrv_meta_t *meta = strg->dev->ctx->metabuf;
	const int nchunks = sizeof(meta->errors) / sizeof(meta->errors[0]);

	paddr = offs / strg->dev->mtd->writesz;
	offs %= strg->dev->mtd->writesz;

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < len) {
		/* TODO: should we skip badblocks ? */
		err = flashdrv_read(strg->dev->ctx->dma, paddr, strg->dev->ctx->databuf, meta);
		if (err < 0) {
			ret = -EIO;
			break;
		}

		err = _flashmtd_checkECC(strg, paddr, nchunks);
		if (err == -EBADMSG) {
			/* Continue reading, let the client handle -EBADMSG */
			ret = -EBADMSG;
		}
		else if (err == -EUCLEAN && ret != -EBADMSG) {
			/* Not a critical error, continue reading, return -EUCLEAN, but don't overwrite -EBADMSG */
			ret = -EUCLEAN;
		}
		else if (err < 0) {
			ret = err;
			break;
		}

		chunksz = min(len - tempsz, strg->dev->mtd->writesz - offs);
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
			res = -EIO;
			break;
		}

		tempsz += strg->dev->mtd->writesz;
		offs += strg->dev->mtd->writesz;
	}
	*retlen = tempsz;
	mutexUnlock(strg->dev->ctx->lock);

	return res < 0 ? res : EOK;
}


static int flashmtd_metaRead(struct _storage_t *strg, off_t offs, void *data, size_t len, size_t *retlen)
{
	uint32_t paddr;
	int err, ret = EOK;
	size_t tempsz = 0, chunksz;
	flashdrv_meta_t *meta = strg->dev->ctx->metabuf;

	if ((offs % strg->dev->mtd->writesz) != 0) {
		return -EINVAL;
	}
	paddr = offs / strg->dev->mtd->writesz;

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < len) {
		/* TODO: should we skip badblocks ? */
		err = flashdrv_read(strg->dev->ctx->dma, paddr, NULL, meta);
		if (err < 0) {
			ret = -EIO;
			break;
		}

		/* Check only metadata chunk (DATA0) */
		err = _flashmtd_checkECC(strg, paddr, 1);
		if (err == -EBADMSG) {
			/* Continue reading, let the client handle -EBADMSG */
			ret = -EBADMSG;
		}
		else if (err == -EUCLEAN && ret != -EBADMSG) {
			/* Not a critical error, continue reading, return -EUCLEAN, but don't overwrite -EBADMSG */
			ret = -EUCLEAN;
		}
		else if (err < 0) {
			ret = err;
			break;
		}

		chunksz = min(len - tempsz, strg->dev->mtd->oobSize);
		memcpy((unsigned char *)data + tempsz, meta->metadata, chunksz);
		tempsz += chunksz;
		paddr++;
	}
	*retlen = tempsz;
	mutexUnlock(strg->dev->ctx->lock);

	return ret;
}


static int flashmtd_metaWrite(struct _storage_t *strg, off_t offs, const void *data, size_t len, size_t *retlen)
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
			res = -EIO;
			break;
		}

		tempsz += chunksz;
		offs += strg->dev->mtd->writesz;
	}
	*retlen = tempsz;
	mutexUnlock(strg->dev->ctx->lock);

	return res < 0 ? res : EOK;
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


static int flashmtd_blockMaxBitflips(struct _storage_t *strg, off_t offs)
{
	storage_mtd_t *mtd = strg->dev->mtd;
	struct _storage_devCtx_t *ctx = strg->dev->ctx;
	flashdrv_meta_t *meta = (flashdrv_meta_t *)ctx->metabuf;
	size_t tempsz = 0;
	uint32_t paddr;
	int err;
	int maxBitFlips = 0;

	if ((offs % mtd->erasesz) != 0) {
		return -EINVAL;
	}

	paddr = offs / mtd->writesz;

	mutexLock(ctx->lock);

	/* Read one block, page by page */
	while (tempsz < mtd->erasesz) {
		err = flashdrv_read(ctx->dma, paddr, ctx->databuf, meta);
		if (err < 0) {
			break;
		}

		err = _flashmtd_blockMaxflips(strg, paddr);
		if (err < 0) {
			printf("flashmtd_checkECC: returned error: %d\n", err);
			break;
		}

		maxBitFlips = max(maxBitFlips, err);
		tempsz += mtd->writesz;
		paddr++;
	}

	mutexUnlock(ctx->lock);

	return (err < 0) ? -EIO : maxBitFlips;
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

	strg->dev->ctx->databuf = mmap(NULL, 2 * info->writesz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (strg->dev->ctx->databuf == MAP_FAILED) {
		flashdrv_dmadestroy(strg->dev->ctx->dma);
		free(strg->dev->ctx);
		free(strg->dev);
		return -ENOMEM;
	}

	strg->dev->ctx->metabuf = mmap(NULL, info->writesz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
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
