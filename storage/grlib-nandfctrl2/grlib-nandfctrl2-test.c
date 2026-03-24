/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 flash tests using MTD interface.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/mman.h>

#include <board_config.h>
#include <storage/storage.h>

#include "nandfctrl2-flashdev.h"
#include "nandfctrl2-flashdrv.h"


#define TEST_BLOCKS_NEEDED 2U


/* clang-format off */
#define LOG(fmt, ...)        do { printf("grlib-nandfctrl2-test: " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(fmt, ...)  do { printf("grlib-nandfctrl2-test:%d: error: " fmt "\n", __LINE__, ##__VA_ARGS__); } while (0)
/* clang-format on */


typedef struct {
	const char *name;
	int (*fn)(void *arg);
} testcase_t;


typedef struct {
	storage_t strg;

	uint32_t blockCount;
	uint32_t pagesPerBlock;
	uint32_t rawPageSz;
	uint32_t tagOffs;

	uint32_t goodBlocks;
	uint32_t badBlocks;
	uint32_t selectedBlocks[TEST_BLOCKS_NEEDED];
	unsigned int selectedCnt;

	size_t dataMapSz;
	size_t rawMapSz;
	size_t metaMapSz;

	uint8_t *dataA;
	uint8_t *dataB;
	uint8_t *metaA;
	uint8_t *metaB;
	uint8_t *raw;
} test_ctx_t;


static void destroyContext(test_ctx_t *ctx);


static size_t alignUp(size_t value, size_t alignment)
{
	return (value + alignment - 1U) & ~(alignment - 1U);
}


static void *bufAlloc(size_t size, size_t *mapSz)
{
	void *ptr;

	*mapSz = alignUp(size, _PAGE_SIZE);
	ptr = mmap(NULL, *mapSz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	return (ptr == MAP_FAILED) ? NULL : ptr;
}


static void bufFree(void *ptr, size_t mapSz)
{
	if (ptr != NULL) {
		(void)munmap(ptr, mapSz);
	}
}


static uint32_t patternStep(uint32_t x)
{
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return x;
}


static void fillPattern(uint8_t *buff, size_t len, uint32_t seed)
{
	uint32_t state = seed | 1U;

	for (size_t i = 0; i < len; ++i) {
		state = patternStep(state + (uint32_t)i + 0x9e3779b9U);
		buff[i] = (uint8_t)(state >> ((i & 0x3U) * 8U));
	}
}


static uint32_t makeSeed(uint32_t page, uint32_t salt)
{
	return 0xa5a55a5aU ^ salt ^ (page * 0x45d9f3bU);
}


static int checkValue(volatile uint8_t *buff, size_t len, uint8_t value, const char *what)
{
	for (size_t i = 0; i < len; ++i) {
		if (buff[i] != value) {
			LOG_ERROR("%s mismatch at byte %zu: expected 0x%02x, got 0x%02x", what, i, value, buff[i]);
			return -EFAULT;
		}
	}

	return 0;
}


static int checkRangeValue(const uint8_t *buff, size_t offs, size_t len, uint8_t value, const char *what)
{
	for (size_t i = 0; i < len; ++i) {
		if (buff[offs + i] != value) {
			LOG_ERROR("%s mismatch at byte %zu: expected 0x%02x, got 0x%02x", what, offs + i, value, buff[offs + i]);
			return -EFAULT;
		}
	}

	return 0;
}


static int checkEqual(const uint8_t *lhs, const uint8_t *rhs, size_t len, const char *what)
{
	for (size_t i = 0; i < len; ++i) {
		if (lhs[i] != rhs[i]) {
			LOG_ERROR("%s mismatch at byte %zu: expected 0x%02x, got 0x%02x", what, i, rhs[i], lhs[i]);
			return -EFAULT;
		}
	}

	return 0;
}


/* Helper: byte offset of a block */
static off_t blockOffs(const test_ctx_t *ctx, uint32_t block)
{
	return (off_t)block * ctx->strg.dev->mtd->erasesz;
}


/* Helper: byte offset of a page */
static off_t pageOffs(const test_ctx_t *ctx, uint32_t page)
{
	return (off_t)page * ctx->strg.dev->mtd->writesz;
}


static off_t rawPageOffs(const test_ctx_t *ctx, uint32_t page)
{
	return (off_t)page * ctx->rawPageSz;
}


static int checkGoodBlockMarker(test_ctx_t *ctx, uint32_t block, const char *stage)
{
	const storage_mtdops_t *ops = ctx->strg.dev->mtd->ops;

	int err = ops->block_isBad(&ctx->strg, blockOffs(ctx, block));
	if (err != 0) {
		LOG_ERROR("block %" PRIu32 " unexpectedly marked bad %s", block, stage);
		return (err < 0) ? err : -EFAULT;
	}

	/* Cross-check with raw BBM read */
	size_t retlen;
	uint32_t page = block * ctx->pagesPerBlock;
	err = flashdev_readRaw(&ctx->strg, rawPageOffs(ctx, page) + ctx->strg.dev->mtd->writesz, ctx->raw, 1U, &retlen);
	if (err < 0) {
		LOG_ERROR("failed to read BBM for block %" PRIu32 " %s: %d", block, stage, err);
		return err;
	}

	if (ctx->raw[0] == 0x00U) {
		LOG_ERROR("good block %" PRIu32 " BBM corrupted %s", block, stage);
		return -EFAULT;
	}

	return 0;
}


static int testScanBadBlocks(void *arg)
{
	test_ctx_t *ctx = arg;
	const storage_mtdops_t *ops = ctx->strg.dev->mtd->ops;

	ctx->goodBlocks = 0U;
	ctx->badBlocks = 0U;
	ctx->selectedCnt = 0U;

	for (uint32_t block = ctx->blockCount; block > 0U; --block) {
		uint32_t blk = block - 1U;
		uint32_t page = blk * ctx->pagesPerBlock;
		int isBad = ops->block_isBad(&ctx->strg, blockOffs(ctx, blk));

		if (isBad < 0) {
			LOG_ERROR("block_isBad() failed for block %" PRIu32 ": %d", blk, isBad);
			return isBad;
		}

		size_t retlen;
		/* Cross-check with raw BBM */
		int err = flashdev_readRaw(&ctx->strg, rawPageOffs(ctx, page) + ctx->strg.dev->mtd->writesz, ctx->raw, 1U, &retlen);
		if (err < 0) {
			LOG_ERROR("raw BBM read failed for block %" PRIu32 ": %d", blk, err);
			return err;
		}

		if (isBad != 0) {
			ctx->badBlocks++;
			if (ctx->raw[0] != 0x00U) {
				LOG_ERROR("block %" PRIu32 " reported as bad but BBM is 0x%02x", blk, ctx->raw[0]);
				return -EFAULT;
			}
		}
		else {
			ctx->goodBlocks++;
			if (ctx->raw[0] == 0x00U) {
				LOG_ERROR("good block %" PRIu32 " has a bad-block marker", blk);
				return -EFAULT;
			}

			if (ctx->selectedCnt < TEST_BLOCKS_NEEDED) {
				ctx->selectedBlocks[ctx->selectedCnt++] = blk;
			}
		}
	}

	LOG("die %u scan complete: good=%" PRIu32 ", bad=%" PRIu32, ctx->strg.dev->ctx->die->target, ctx->goodBlocks, ctx->badBlocks);

	if (ctx->selectedCnt < TEST_BLOCKS_NEEDED) {
		LOG_ERROR("need at least %u good blocks, found %u", TEST_BLOCKS_NEEDED, ctx->selectedCnt);
		return -ENOSPC;
	}

	LOG("selected test blocks: %" PRIu32 ", %" PRIu32, ctx->selectedBlocks[0], ctx->selectedBlocks[1]);
	return 0;
}


static int testEraseReadback(void *arg)
{
	test_ctx_t *ctx = arg;
	uint32_t block = ctx->selectedBlocks[0];
	const storage_mtdops_t *ops = ctx->strg.dev->mtd->ops;
	const storage_mtd_t *mtd = ctx->strg.dev->mtd;

	int err = ops->erase(&ctx->strg, blockOffs(ctx, block), mtd->erasesz);
	if (err < 0) {
		LOG_ERROR("erase failed for block %" PRIu32 ": %d", block, err);
		return err;
	}

	err = checkGoodBlockMarker(ctx, block, "after erase");
	if (err < 0) {
		return err;
	}

	for (uint32_t i = 0; i < ctx->pagesPerBlock; ++i) {
		uint32_t page = block * ctx->pagesPerBlock + i;
		size_t retlen;

		/* Raw read to verify the page is fully erased (including spare) */
		err = flashdev_readRaw(&ctx->strg, rawPageOffs(ctx, page), ctx->raw, ctx->rawPageSz, &retlen);
		if (err < 0) {
			LOG_ERROR("raw read failed for page %" PRIu32 " after erase: %d", page, err);
			return err;
		}

		err = checkValue(ctx->raw, mtd->writesz, 0xffU, "erased raw page");
		if (err < 0) {
			LOG_ERROR("page %" PRIu32 " is not fully erased", page);
			return err;
		}

		/* MTD read to verify ECC engine sees an erased page */
		memset(ctx->dataA, 0x00, mtd->writesz);
		err = ops->read(&ctx->strg, pageOffs(ctx, page), ctx->dataA, mtd->writesz, &retlen);
		if (err < 0) {
			LOG_ERROR("MTD read failed for erased page %" PRIu32 ": %d", page, err);
			return err;
		}

		if (retlen != mtd->writesz) {
			LOG_ERROR("MTD read short for erased page %" PRIu32 ": got %zu", page, retlen);
			return -EIO;
		}

		err = checkValue(ctx->dataA, mtd->writesz, 0xffU, "erased page data");
		if (err < 0) {
			LOG_ERROR("erased page %" PRIu32 " was not corrected to 0xff", page);
			return err;
		}
	}

	return 0;
}


static int testPageProgramRead(void *arg)
{
	test_ctx_t *ctx = arg;
	const storage_mtdops_t *ops = ctx->strg.dev->mtd->ops;
	const storage_mtd_t *mtd = ctx->strg.dev->mtd;
	uint32_t block = ctx->selectedBlocks[0];

	int err = ops->erase(&ctx->strg, blockOffs(ctx, block), mtd->erasesz);
	if (err < 0) {
		LOG_ERROR("erase failed for block %" PRIu32 ": %d", block, err);
		return err;
	}

	for (uint32_t pageIdx = 0; pageIdx < ctx->pagesPerBlock; pageIdx++) {
		uint32_t page = block * ctx->pagesPerBlock + pageIdx;

		/* Verify tag area is untouched after data-only write */
		memset(ctx->metaA, 0x5a, mtd->oobAvail);
		size_t retlen = 0;
		err = ops->meta_read(&ctx->strg, pageOffs(ctx, page), ctx->metaA, mtd->oobAvail, &retlen);
		if (err < 0) {
			LOG_ERROR("meta read failed for page %" PRIu32 ": %d", page, err);
			return err;
		}

		if (retlen != mtd->oobAvail) {
			LOG_ERROR("meta read short for page %" PRIu32 ": got %zu", page, retlen);
			return -EIO;
		}

		err = checkValue(ctx->metaA, mtd->oobAvail, 0xffU, "untouched tag area");
		if (err < 0) {
			LOG_ERROR("tag area changed on page %" PRIu32 " during data program", page);
			return err;
		}
	}

	for (uint32_t pageIdx = 0; pageIdx < ctx->pagesPerBlock; pageIdx++) {
		uint32_t page = block * ctx->pagesPerBlock + pageIdx;

		fillPattern(ctx->dataA, mtd->writesz, makeSeed(page, 0x1010U));

		size_t retlen = 0;
		err = ops->write(&ctx->strg, pageOffs(ctx, page), ctx->dataA, mtd->writesz, &retlen);
		if (err < 0) {
			LOG_ERROR("write failed for page %" PRIu32 ": %d", page, err);
			return err;
		}

		if (retlen != mtd->writesz) {
			LOG_ERROR("write short for page %" PRIu32 ": got %zu", page, retlen);
			return -EIO;
		}
	}

	err = checkGoodBlockMarker(ctx, block, "after full-block program");
	if (err < 0) {
		return err;
	}

	for (uint32_t pageIdx = 0; pageIdx < ctx->pagesPerBlock; pageIdx++) {
		uint32_t page = block * ctx->pagesPerBlock + pageIdx;

		fillPattern(ctx->dataB, mtd->writesz, makeSeed(page, 0x1010U));
		memset(ctx->dataA, 0x00, mtd->writesz);

		size_t retlen = 0;
		err = ops->read(&ctx->strg, pageOffs(ctx, page), ctx->dataA, mtd->writesz, &retlen);
		if (err < 0) {
			LOG_ERROR("read failed for page %" PRIu32 ": %d", page, err);
			return err;
		}

		if (retlen != mtd->writesz) {
			LOG_ERROR("read short for page %" PRIu32 ": got %zu", page, retlen);
			return -EIO;
		}

		err = checkEqual(ctx->dataA, ctx->dataB, mtd->writesz, "program/read data");
		if (err < 0) {
			LOG_ERROR("data verification failed for page %" PRIu32, page);
			return err;
		}

		/* Verify tag area is untouched after data-only write */
		memset(ctx->metaA, 0x5a, mtd->oobAvail);
		retlen = 0;
		err = ops->meta_read(&ctx->strg, pageOffs(ctx, page), ctx->metaA, mtd->oobAvail, &retlen);
		if (err < 0) {
			LOG_ERROR("meta read failed for page %" PRIu32 ": %d", page, err);
			return err;
		}

		if (retlen != mtd->oobAvail) {
			LOG_ERROR("meta read short for page %" PRIu32 ": got %zu", page, retlen);
			return -EIO;
		}

		err = checkValue(ctx->metaA, mtd->oobAvail, 0xffU, "untouched tag area");
		if (err < 0) {
			LOG_ERROR("tag area changed on page %" PRIu32 " during data program", page);
			return err;
		}
	}

	return 0;
}


static int testMetadataAccess(void *arg)
{
	test_ctx_t *ctx = arg;
	const storage_mtdops_t *ops = ctx->strg.dev->mtd->ops;
	const storage_mtd_t *mtd = ctx->strg.dev->mtd;
	uint32_t block = ctx->selectedBlocks[0];
	uint32_t page = block * ctx->pagesPerBlock;
	int err;

	err = ops->erase(&ctx->strg, blockOffs(ctx, block), mtd->erasesz);
	if (err < 0) {
		LOG_ERROR("erase failed for metadata test on block %" PRIu32 ": %d", block, err);
		return err;
	}

	fillPattern(ctx->metaA, mtd->oobAvail, makeSeed(page, 0x10U));

	memset(ctx->metaB, 0x00, mtd->oobAvail);

	size_t retlen = 0;
	err = ops->meta_write(&ctx->strg, pageOffs(ctx, page), ctx->metaA, mtd->oobAvail, &retlen);
	if (err < 0) {
		LOG_ERROR("meta write failed for page %" PRIu32 ": %d", page, err);
		return err;
	}

	if (retlen != mtd->oobAvail) {
		LOG_ERROR("meta write short for page %" PRIu32 ": got %zu", page, retlen);
		return -EIO;
	}

	retlen = 0;
	err = ops->meta_read(&ctx->strg, pageOffs(ctx, page), ctx->metaB, mtd->oobAvail, &retlen);
	if (err < 0) {
		LOG_ERROR("meta read failed for page %" PRIu32 ": %d", page, err);
		return err;
	}

	if (retlen != mtd->oobAvail) {
		LOG_ERROR("meta read short for page %" PRIu32 ": got %zu", page, retlen);
		return -EIO;
	}

	err = checkEqual(ctx->metaB, ctx->metaA, mtd->oobAvail, "meta roundtrip");
	if (err < 0) {
		LOG_ERROR("metadata verification failed for page %" PRIu32, page);
		return err;
	}

	/* Cross-check: verify raw layout shows tag at expected offset */
	err = flashdev_readRaw(&ctx->strg, rawPageOffs(ctx, page), ctx->raw, ctx->rawPageSz, &retlen);
	if (err < 0) {
		LOG_ERROR("raw read failed for metadata test page %" PRIu32 ": %d", page, err);
		return err;
	}

	err = checkValue(ctx->raw, ctx->tagOffs, 0xffU, "bytes before tag area");
	if (err < 0) {
		return err;
	}

	err = checkEqual(ctx->raw + ctx->tagOffs, ctx->metaA, mtd->oobAvail, "raw tag contents");
	if (err < 0) {
		return err;
	}

	/* Verify data area is still erased after metadata-only write */
	memset(ctx->dataA, 0x00, mtd->writesz);
	retlen = 0;
	err = ops->read(&ctx->strg, pageOffs(ctx, page), ctx->dataA, mtd->writesz, &retlen);
	/* EUCLEAN/EBADMSG possible - page and ECC is erased but metadata is written. */
	if ((err != -EBADMSG) && (err != -EUCLEAN) && (err < 0)) {
		LOG_ERROR("MTD read failed for metadata-only page %" PRIu32 ": %d", page, err);
		return err;
	}

	err = checkValue(ctx->dataA, mtd->writesz, 0xffU, "metadata-only page data");
	if (err < 0) {
		return err;
	}

	return checkGoodBlockMarker(ctx, block, "after metadata write");
}


static int testRawAccess(void *arg)
{
	test_ctx_t *ctx = arg;
	const storage_mtdops_t *ops = ctx->strg.dev->mtd->ops;
	const storage_mtd_t *mtd = ctx->strg.dev->mtd;
	uint32_t block = ctx->selectedBlocks[0];
	uint32_t pageData = block * ctx->pagesPerBlock;
	uint32_t pageTag = pageData + 1U;
	const uint32_t dataOffs = 128U;
	const size_t dataLen = 256U;

	int err = ops->erase(&ctx->strg, blockOffs(ctx, block), mtd->erasesz);
	if (err < 0) {
		LOG_ERROR("erase failed for raw access test on block %" PRIu32 ": %d", block, err);
		return err;
	}

	/* Raw data write at arbitrary offset within data area */
	size_t retlen;
	fillPattern(ctx->dataA, dataLen, makeSeed(pageData, 0x3030U));
	err = flashdev_writeRaw(&ctx->strg, rawPageOffs(ctx, pageData) + dataOffs, ctx->dataA, dataLen, &retlen);
	if (err < 0) {
		LOG_ERROR("raw data write failed for page %" PRIu32 ": %d", pageData, err);
		return err;
	}

	err = flashdev_readRaw(&ctx->strg, rawPageOffs(ctx, pageData), ctx->raw, ctx->rawPageSz, &retlen);
	if (err < 0) {
		LOG_ERROR("raw full-page read failed for page %" PRIu32 ": %d", pageData, err);
		return err;
	}

	err = checkValue(ctx->raw, dataOffs, 0xffU, "raw data prefix");
	if (err < 0) {
		return err;
	}

	err = checkEqual(ctx->raw + dataOffs, ctx->dataA, dataLen, "raw programmed data span");
	if (err < 0) {
		return err;
	}

	err = checkRangeValue(ctx->raw, dataOffs + dataLen, ctx->rawPageSz - (dataOffs + dataLen), 0xffU, "raw data suffix");
	if (err < 0) {
		return err;
	}

	/* Raw tag write */
	fillPattern(ctx->metaA, mtd->oobAvail, makeSeed(pageTag, 0x3131U));
	err = flashdev_writeRaw(&ctx->strg, rawPageOffs(ctx, pageTag) + ctx->tagOffs, ctx->metaA, mtd->oobAvail, &retlen);
	if (err < 0) {
		LOG_ERROR("raw tag write failed for page %" PRIu32 ": %d", pageTag, err);
		return err;
	}

	err = flashdev_readRaw(&ctx->strg, rawPageOffs(ctx, pageTag), ctx->raw, ctx->rawPageSz, &retlen);
	if (err < 0) {
		LOG_ERROR("raw read failed for page %" PRIu32 ": %d", pageTag, err);
		return err;
	}

	err = checkValue(ctx->raw, ctx->tagOffs, 0xffU, "raw tag prefix");
	if (err < 0) {
		return err;
	}

	err = checkEqual(ctx->raw + ctx->tagOffs, ctx->metaA, mtd->oobAvail, "raw tag data");
	if (err < 0) {
		return err;
	}

	return checkGoodBlockMarker(ctx, block, "after raw accesses");
}


static int testErasedPageBitflipCorrection(void *arg)
{
	test_ctx_t *ctx = arg;
	const storage_mtdops_t *ops = ctx->strg.dev->mtd->ops;
	const storage_mtd_t *mtd = ctx->strg.dev->mtd;
	uint32_t block = ctx->selectedBlocks[0];
	uint32_t page = block * ctx->pagesPerBlock;
	const uint8_t flipped = 0xfeU;

	int err = ops->erase(&ctx->strg, blockOffs(ctx, block), mtd->erasesz);
	if (err < 0) {
		LOG_ERROR("erase failed for erased-page bitflip test on block %" PRIu32 ": %d", block, err);
		return err;
	}

	size_t retlen;

	/* Inject a single bitflip via raw write */
	err = flashdev_writeRaw(&ctx->strg, rawPageOffs(ctx, page) + 17U, &flipped, sizeof(flipped), &retlen);
	if (err < 0) {
		LOG_ERROR("raw bitflip injection failed for page %" PRIu32 ": %d", page, err);
		return err;
	}

	/* Verify the bitflip is present in raw data */
	err = flashdev_readRaw(&ctx->strg, rawPageOffs(ctx, page) + 17U, ctx->raw, 1U, &retlen);
	if (err < 0) {
		LOG_ERROR("raw verification read failed for page %" PRIu32 ": %d", page, err);
		return err;
	}

	if (ctx->raw[0] != flipped) {
		LOG_ERROR("raw bitflip injection not visible on page %" PRIu32, page);
		return -EFAULT;
	}

	/* MTD read should correct the bitflip and return clean data */
	memset(ctx->dataA, 0x00, mtd->writesz);
	err = ops->read(&ctx->strg, pageOffs(ctx, page), ctx->dataA, mtd->writesz, &retlen);
	if (err == -EBADMSG) {
		LOG_ERROR("erased page %" PRIu32 " bitflip was reported as uncorrectable", page);
		return -EFAULT;
	}
	if ((err < 0) && (err != -EUCLEAN)) {
		LOG_ERROR("MTD read failed for erased page with bitflip %" PRIu32 ": %d", page, err);
		return err;
	}

	err = checkValue(ctx->dataA, mtd->writesz, 0xffU, "corrected erased page");
	if (err < 0) {
		LOG_ERROR("erased page %" PRIu32 " was not corrected back to 0xff", page);
		return err;
	}

	return checkGoodBlockMarker(ctx, block, "after erased-page bitflip correction");
}


static int testMarkBad(void *arg)
{
	test_ctx_t *ctx = arg;
	const storage_mtdops_t *ops = ctx->strg.dev->mtd->ops;
	const storage_mtd_t *mtd = ctx->strg.dev->mtd;
	uint32_t block = ctx->selectedBlocks[0];

	int err = ops->erase(&ctx->strg, blockOffs(ctx, block), mtd->erasesz);
	if (err < 0) {
		LOG_ERROR("erase failed for block %" PRIu32 ": %d", block, err);
		return err;
	}

	err = checkGoodBlockMarker(ctx, block, "before markBad");
	if (err < 0) {
		return err;
	}

	/* Mark the block as bad */
	err = ops->block_markBad(&ctx->strg, blockOffs(ctx, block));
	if (err < 0) {
		LOG_ERROR("block_markBad failed for block %" PRIu32 ": %d", block, err);
		return err;
	}

	/* Verify block is now reported as bad */
	err = ops->block_isBad(&ctx->strg, blockOffs(ctx, block));
	if (err == 0) {
		LOG_ERROR("block %" PRIu32 " not reported as bad after markBad", block);
		return -EFAULT;
	}
	if (err < 0) {
		LOG_ERROR("block_isBad failed for block %" PRIu32 ": %d", block, err);
		return err;
	}

	/* Cross-check: raw BBM byte should be 0x00 */
	size_t retlen;
	uint32_t page = block * ctx->pagesPerBlock;
	err = flashdev_readRaw(&ctx->strg, rawPageOffs(ctx, page) + ctx->strg.dev->mtd->writesz, ctx->raw, 1U, &retlen);
	if (err < 0) {
		LOG_ERROR("raw BBM read failed for block %" PRIu32 ": %d", block, err);
		return err;
	}

	if (ctx->raw[0] != 0x00U) {
		LOG_ERROR("BBM byte is 0x%02x after markBad, expected 0x00", ctx->raw[0]);
		return -EFAULT;
	}

	/* Restore the block: use flashdrv_erase directly as MTD erase skips bad blocks */
	err = flashdrv_erase(ctx->strg.dev->ctx->die, block);
	if (err < 0) {
		LOG_ERROR("flashdrv_erase failed for block %" PRIu32 ": %d", block, err);
		return err;
	}

	/* Verify block is good again after raw erase */
	return checkGoodBlockMarker(ctx, block, "after markBad restore");
}


static int testBlockIsolation(void *arg)
{
	test_ctx_t *ctx = arg;
	const storage_mtdops_t *ops = ctx->strg.dev->mtd->ops;
	const storage_mtd_t *mtd = ctx->strg.dev->mtd;
	uint32_t blockA = ctx->selectedBlocks[0];
	uint32_t blockB = ctx->selectedBlocks[1];
	uint32_t pageA = blockA * ctx->pagesPerBlock;
	uint32_t pageB = blockB * ctx->pagesPerBlock;

	int err = ops->erase(&ctx->strg, blockOffs(ctx, blockA), mtd->erasesz);
	if (err < 0) {
		LOG_ERROR("erase failed for block %" PRIu32 ": %d", blockA, err);
		return err;
	}

	err = ops->erase(&ctx->strg, blockOffs(ctx, blockB), mtd->erasesz);
	if (err < 0) {
		LOG_ERROR("erase failed for block %" PRIu32 ": %d", blockB, err);
		return err;
	}

	fillPattern(ctx->dataA, mtd->writesz, makeSeed(pageA, 0x4040U));
	fillPattern(ctx->dataB, mtd->writesz, makeSeed(pageB, 0x5050U));

	size_t retlen = 0;
	err = ops->write(&ctx->strg, pageOffs(ctx, pageA), ctx->dataA, mtd->writesz, &retlen);
	if (err < 0) {
		LOG_ERROR("write failed for page %" PRIu32 ": %d", pageA, err);
		return err;
	}

	retlen = 0;
	err = ops->write(&ctx->strg, pageOffs(ctx, pageB), ctx->dataB, mtd->writesz, &retlen);
	if (err < 0) {
		LOG_ERROR("write failed for page %" PRIu32 ": %d", pageB, err);
		return err;
	}

	/* Read back block A */
	memset(ctx->raw, 0x00, mtd->writesz);
	retlen = 0;
	err = ops->read(&ctx->strg, pageOffs(ctx, pageA), ctx->raw, mtd->writesz, &retlen);
	if (err < 0) {
		LOG_ERROR("read failed for page %" PRIu32 ": %d", pageA, err);
		return err;
	}
	if (checkEqual(ctx->raw, ctx->dataA, mtd->writesz, "block A data") < 0) {
		return -EFAULT;
	}

	/* Read back block B */
	memset(ctx->raw, 0x00, mtd->writesz);
	retlen = 0;
	err = ops->read(&ctx->strg, pageOffs(ctx, pageB), ctx->raw, mtd->writesz, &retlen);
	if (err < 0) {
		LOG_ERROR("read failed for page %" PRIu32 ": %d", pageB, err);
		return err;
	}
	if (checkEqual(ctx->raw, ctx->dataB, mtd->writesz, "block B data") < 0) {
		return -EFAULT;
	}

	/* Erase block A, verify block B is intact */
	err = ops->erase(&ctx->strg, blockOffs(ctx, blockA), mtd->erasesz);
	if (err < 0) {
		LOG_ERROR("re-erase failed for block %" PRIu32 ": %d", blockA, err);
		return err;
	}

	memset(ctx->raw, 0x00, mtd->writesz);
	retlen = 0;
	err = ops->read(&ctx->strg, pageOffs(ctx, pageB), ctx->raw, mtd->writesz, &retlen);
	if (err < 0) {
		LOG_ERROR("read failed for page %" PRIu32 " after erasing block %" PRIu32 ": %d", pageB, blockA, err);
		return err;
	}

	if (checkEqual(ctx->raw, ctx->dataB, mtd->writesz, "block B persistence") < 0) {
		LOG_ERROR("erasing block %" PRIu32 " affected block %" PRIu32, blockA, blockB);
		return -EFAULT;
	}

	err = checkGoodBlockMarker(ctx, blockA, "after isolation test erase");
	if (err < 0) {
		return err;
	}
	err = checkGoodBlockMarker(ctx, blockB, "after isolation test");
	if (err < 0) {
		return err;
	}

	return 0;
}


static int parseDieTarget(int argc, char **argv, unsigned int *target)
{
	char *end = NULL;
	unsigned long value;
	bool set = false;

	*target = 0U;

	for (;;) {
		int opt = getopt(argc, argv, "d:h");
		if (opt == -1) {
			break;
		}

		switch (opt) {
			case 'd':
				value = strtoul(optarg, &end, 0);
				if ((end == optarg) || (*end != '\0') || (value >= NAND_DIE_CNT)) {
					return -EINVAL;
				}
				*target = (unsigned int)value;
				set = true;
				break;

			case 'h':
			default:
				return -EINVAL;
		}
	}

	if (optind < argc) {
		value = strtoul(argv[optind], &end, 0);
		if ((end == argv[optind]) || (*end != '\0') || (value >= NAND_DIE_CNT)) {
			return -EINVAL;
		}
		*target = (unsigned int)value;
		set = true;
	}

	return set ? 0 : -EINVAL;
}


static void printUsage(const char *prog)
{
	printf("Usage: %s -d <die>\n", prog);
	printf("  die: 0..%u\n", NAND_DIE_CNT - 1U);
	printf("The test is destructive on good blocks selected on the chosen die.\n");
}


static int initContext(test_ctx_t *ctx, unsigned int target)
{
	const flashdrv_info_t *info;
	int err;

	LOG("initContext: target=%u", target);

	memset(ctx, 0, sizeof(*ctx));

	err = flashdev_init(&ctx->strg, target);
	if (err < 0) {
		LOG_ERROR("failed to initialize MTD for die %u: %d", target, err);
		return err;
	}

	info = &ctx->strg.dev->ctx->die->info;

	ctx->blockCount = (uint32_t)(info->size / info->erasesz);
	ctx->pagesPerBlock = info->pagesPerBlock;
	ctx->rawPageSz = info->writesz + info->sparesz;
	ctx->tagOffs = ctx->rawPageSz - info->spareavail;

	ctx->dataA = bufAlloc(info->writesz, &ctx->dataMapSz);
	ctx->dataB = bufAlloc(info->writesz, &ctx->dataMapSz);
	ctx->metaA = bufAlloc(info->spareavail, &ctx->metaMapSz);
	ctx->metaB = bufAlloc(info->spareavail, &ctx->metaMapSz);
	ctx->raw = bufAlloc(ctx->rawPageSz, &ctx->rawMapSz);

	if ((ctx->dataA == NULL) || (ctx->dataB == NULL) || (ctx->metaA == NULL) || (ctx->metaB == NULL) || (ctx->raw == NULL)) {
		LOG_ERROR("failed to allocate DMA buffers");
		destroyContext(ctx);
		return -ENOMEM;
	}

	return 0;
}


static void destroyContext(test_ctx_t *ctx)
{
	bufFree(ctx->raw, ctx->rawMapSz);
	bufFree(ctx->metaB, ctx->metaMapSz);
	bufFree(ctx->metaA, ctx->metaMapSz);
	bufFree(ctx->dataB, ctx->dataMapSz);
	bufFree(ctx->dataA, ctx->dataMapSz);
	flashdev_done(&ctx->strg);
}


int main(int argc, char **argv)
{
	static const testcase_t tests[] = {
		{ "bad-block scan", testScanBadBlocks },
		{ "erase readback", testEraseReadback },
		{ "metadata access", testMetadataAccess },
		{ "page program/read", testPageProgramRead },
		{ "raw access", testRawAccess },
		{ "erased-page bitflip correction", testErasedPageBitflipCorrection },
		{ "mark bad", testMarkBad },
		{ "block isolation", testBlockIsolation },
	};

	test_ctx_t ctx;
	unsigned int target;
	unsigned int i;
	int err;

	err = parseDieTarget(argc, argv, &target);
	if (err < 0) {
		printUsage(argv[0]);
		return 1;
	}

	LOG("initializing controller for die %u", target);
	err = flashdrv_init();
	if (err < 0) {
		LOG_ERROR("flashdrv_init() failed: %d", err);
		return 1;
	}

	err = initContext(&ctx, target);
	if (err < 0) {
		return 1;
	}

	err = flashdrv_reset(ctx.strg.dev->ctx->die);
	if (err < 0) {
		LOG_ERROR("flashdrv_reset() failed for die %u: %d", target, err);
		destroyContext(&ctx);
		return 1;
	}

	const storage_mtd_t *mtd = ctx.strg.dev->mtd;

	LOG("die %u: %s, size=%" PRIu64 ", writesz=%zu, oobSize=%zu, oobAvail=%zu, erasesz=%zu, pages/block=%u",
			target,
			mtd->name,
			(uint64_t)ctx.strg.size,
			mtd->writesz,
			mtd->oobSize,
			mtd->oobAvail,
			mtd->erasesz,
			ctx.pagesPerBlock);

	for (i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) {
		LOG("running test %u/%zu: %s", i + 1U, sizeof(tests) / sizeof(tests[0]), tests[i].name);
		err = tests[i].fn(&ctx);
		if (err < 0) {
			LOG_ERROR("test failed: %s (%d)", tests[i].name, err);
			break;
		}
		LOG("test passed: %s", tests[i].name);
	}

	if (err == 0) {
		LOG("all tests passed on die %u", target);
	}

	/* Erase all blocks used during testing */
	for (unsigned int b = 0; b < ctx.selectedCnt; ++b) {
		uint32_t block = ctx.selectedBlocks[b];
		err = mtd->ops->erase(&ctx.strg, blockOffs(&ctx, block), mtd->erasesz);
		if (err < 0) {
			LOG_ERROR("cleanup erase failed for block %" PRIu32 ": %d", block, err);
			break;
		}
		LOG("cleanup: erased block %" PRIu32, block);
	}

	destroyContext(&ctx);
	return (err == 0) ? 0 : 1;
}
