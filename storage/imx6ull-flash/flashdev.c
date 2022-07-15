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


/* Auxiliary functions */

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


static int _flashmtd_checkECC(struct _storage_t *strg, uint32_t paddr, unsigned int chunks)
{
	/* Metadata and data (with their ECC) chunk sizes in bits */
	const size_t mlen = strg->dev->mtd->oobSize * CHAR_BIT + ECC_GF * ECC0_BITFLIP_STRENGHT;
	const size_t dlen = ECCN_DATA_SIZE * CHAR_BIT + ECC_GF * ECCN_BITFLIP_STRENGHT;
	flashdrv_meta_t *meta = strg->dev->ctx->metabuf;
	unsigned char *data = strg->dev->ctx->databuf;
	unsigned int i, flips, maxflips = 0;
	size_t boffs, blen, rawsz;
	void *raw = NULL;
	int err;

	for (i = 0; i < chunks; i++) {
		switch (meta->errors[i]) {
			case flash_no_errors:
			case flash_erased:
				break;

			case flash_uncorrectable:
				/* BCH reports chunk as uncorrectable in case of an erased page with bitflips within that chunk area */
				/* Check if that's the case by counting the bitflips with an assumption that the page is fully erased */

				/* Read raw page in order to check full chunk - both data and its ECC area */
				if (raw == NULL) {
					/* Map raw page buffer */
					rawsz = (strg->dev->mtd->writesz + strg->dev->mtd->metaSize + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1);
					raw = mmap(NULL, rawsz, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_CONTIGUOUS, 0);
					if (raw == MAP_FAILED) {
						return -ENOMEM;
					}

					/* Read raw page */
					err = flashdrv_readraw(strg->dev->ctx->dma, paddr, raw, strg->dev->mtd->writesz + strg->dev->mtd->metaSize);
					if (err < 0) {
						munmap(raw, rawsz);
						return err;
					}
				}

				/* Metadata chunk */
				if (i == 0) {
					boffs = 0;
					blen = mlen;
				}
				/* Data chunk */
				else {
					boffs = mlen + (i - 1) * dlen;
					blen = dlen;
				}

				flips = _flashmtd_checkErased(raw, boffs, blen);

				/* Nothing to do if there're no bitflips */
				if (flips == 0) {
					break;
				}

				/* Handle metadata chunk bitflips */
				if (i == 0) {
					/* Too many metadata bitflips, return error */
					if (flips > ECC0_BITFLIP_STRENGHT) {
						munmap(raw, rawsz);
						return -EBADMSG;
					}

					/* Correct metadata chunk */
					memset(meta, 0xff, strg->dev->mtd->oobSize);
				}
				/* Handle data chunk bitflips */
				else {
					/* Too many data bitflips, return error */
					if (flips > ECCN_BITFLIP_STRENGHT) {
						munmap(raw, rawsz);
						return -EBADMSG;
					}

					/* Correct data chunk */
					memset(data + (i - 1) * ECCN_DATA_SIZE, 0xff, ECCN_DATA_SIZE);
				}

				/* Chunk corrected, update max number of bitflips */
				maxflips = max(flips, maxflips);
				break;

			default:
				/* Chunk corrected by BCH, update max number of bitflips */
				maxflips = max(meta->errors[i], maxflips);
				break;
		}
	}

	if (raw != NULL) {
		munmap(raw, rawsz);
	}

	/* Return -EUCLEAN if the page has to be rewritten */
	return (maxflips >= ECC_BITFLIP_THRESHOLD) ? -EUCLEAN : EOK;
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

		err = _flashmtd_checkECC(strg, paddr, sizeof(meta->errors) / sizeof(meta->errors[0]));
		if (err < 0) {
			ret = err;
			/* -EUCLEAN isn't a fatal error (indicates dangerous page degradation but all bitflips were successfully corrected) */
			if (err != -EUCLEAN) {
				break;
			}
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
		if (err < 0) {
			ret = err;
			/* -EUCLEAN isn't a fatal error (indicates dangerous page degradation but all bitflips were successfully corrected) */
			if (err != -EUCLEAN) {
				break;
			}
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
