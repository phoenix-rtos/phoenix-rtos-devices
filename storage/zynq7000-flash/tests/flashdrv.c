/*
 * Phoenix-RTOS
 *
 * Nor Flash driver tests for zynq-7000
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <flashdrv.h>
#include <flashcfg.h>

#include "unity_fixture.h"

#define SIZE_DEFAULT_BUFF    0x10000
#define VALUE_DEFAULT_SAMPLE 0xa5

struct {
	uint8_t *txBuff;
	uint8_t *rxBuff;
	size_t buffSz;

	const flash_info_t *info;
} common;


/* Auxiliary functions */

static inline void test_initBuffs(size_t sz)
{
	int i;

	TEST_ASSERT_GREATER_OR_EQUAL_INT32(sz, common.buffSz);

	for (i = 0; i < sz; i++)
		common.txBuff[i] = i;

	memset(common.rxBuff, VALUE_DEFAULT_SAMPLE, common.buffSz);
}


static void test_writeSyncVerify(addr_t offs, size_t sz)
{
	/* Prepare sample data to write */
	test_initBuffs(sz);

	/* Write */
	TEST_ASSERT_GREATER_OR_EQUAL_INT32_MESSAGE(offs + sz, CFI_SIZE_FLASH(common.info->cfi.chipSize), "wrong offset");
	TEST_ASSERT_EQUAL(sz, flashdrv_write(offs, common.txBuff, sz));

	/* Synchronize */
	TEST_ASSERT_EQUAL(EOK, flashdrv_sync());

	/* Verify */
	TEST_ASSERT_EQUAL(sz, flashdrv_read(offs, common.rxBuff, sz, 0));
	TEST_ASSERT_EQUAL_UINT8_ARRAY(common.txBuff, common.rxBuff, sz);
	if (sz < common.buffSz)
		TEST_ASSERT_EQUAL(VALUE_DEFAULT_SAMPLE, common.rxBuff[sz]);
}


/* Setup */

TEST_GROUP(test_flashdrv);


TEST_SETUP(test_flashdrv)
{
	int i;
	size_t sz;

	/* Initialize flash memory */
	TEST_ASSERT_EQUAL(EOK, flashdrv_init());

	common.info = flashdrv_flashInfo();
	common.buffSz = SIZE_DEFAULT_BUFF;

	/* Find max sector size */
	for (i = 0; i < common.info->cfi.regsCount; ++i) {
		sz = CFI_SIZE_SECTION(common.info->cfi.regs[i].size);

		if (sz > common.buffSz)
			common.buffSz = sz;
	}

	/* Initialize buffers */
	common.rxBuff = malloc(common.buffSz);
	common.txBuff = malloc(common.buffSz);

	TEST_ASSERT_NOT_NULL(common.rxBuff);
	TEST_ASSERT_NOT_NULL(common.txBuff);
}


TEST_TEAR_DOWN(test_flashdrv)
{
	free(common.rxBuff);
	free(common.txBuff);

	TEST_ASSERT_EQUAL(EOK, flashdrv_done());
}


/* Test info about flash */

TEST(test_flashdrv, info_validate)
{
	TEST_ASSERT_NOT_NULL(common.info->name);
	TEST_ASSERT_NOT_EQUAL(0, common.info->cfi.chipSize);
	TEST_ASSERT_NOT_EQUAL(0, common.info->cfi.pageSize);
	TEST_ASSERT_NOT_EQUAL(0, common.info->cfi.regsCount);
}


/* Test read operation */

TEST(test_flashdrv, read_toInvalBuff)
{
	TEST_ASSERT_EQUAL(-EINVAL, flashdrv_read(0, NULL, 0x1000, 0));
}


TEST(test_flashdrv, read_fromInvalOffs)
{
	TEST_ASSERT_EQUAL(-EINVAL, flashdrv_read(CFI_SIZE_FLASH(common.info->cfi.chipSize), common.txBuff, 0x1, 0));
}


/* Test erase operations */

TEST(test_flashdrv, erase_invalOffs)
{
	int i = 0;
	addr_t offs;
	size_t sz = 0, regSz = 0;

	/* Check wrong section alligment */
	for (i = 0; i < common.info->cfi.regsCount; ++i) {
		regSz += CFI_SIZE_REGION(common.info->cfi.regs[i].size, common.info->cfi.regs[i].count);
		sz = CFI_SIZE_SECTION(common.info->cfi.regs[i].size);

		/* Offset in the quarter of sector */
		offs = regSz - sz + (sz / 4);
		TEST_ASSERT_EQUAL(-EINVAL, flashdrv_sectorErase(offs));
	}
}


TEST(test_flashdrv, erase_outOfMem)
{
	/* Erase out of flash memory range */
	TEST_ASSERT_EQUAL(-EINVAL, flashdrv_sectorErase(CFI_SIZE_FLASH(common.info->cfi.chipSize)));
}


TEST(test_flashdrv, erase_sector)
{
	addr_t offs;
	size_t sz;

	/* Sample offset in nor flash memory */
	sz = CFI_SIZE_SECTION(common.info->cfi.regs[0].size);
	offs = (common.info->cfi.regs[0].count - 1) * sz;

	/* Prepare sample data to write */
	test_initBuffs(sz);

	TEST_ASSERT_EQUAL(sz, flashdrv_write(offs, common.txBuff, sz));
	TEST_ASSERT_EQUAL(EOK, flashdrv_sectorErase(offs));
	TEST_ASSERT_EQUAL(sz, flashdrv_read(offs, common.rxBuff, sz, 0));
	TEST_ASSERT_EACH_EQUAL_UINT8(0xff, common.rxBuff, sz);
}


TEST(test_flashdrv, erase_chip)
{
	int i;
	addr_t offs[3];
	static const size_t sz = 0x1000;

	offs[0] = 0;
	offs[1] = CFI_SIZE_FLASH(common.info->cfi.chipSize) / 2;
	offs[2] = CFI_SIZE_FLASH(common.info->cfi.chipSize) - sz;

	/* Write data at begin, in the middle and at the end of memory */
	for (i = 0; i < (sizeof(offs) / sizeof(offs[0])); ++i)
		test_writeSyncVerify(offs[i], sz);

	/* Erase the whole memory */
	TEST_ASSERT_EQUAL(EOK, flashdrv_chipErase());

	/* Check if data is erased at begin, in the middle and at the end of memory */
	for (i = 0; i < (sizeof(offs) / sizeof(offs[0])); ++i) {
		TEST_ASSERT_EQUAL(sz, flashdrv_read(offs[i], common.rxBuff, sz, 0));
		TEST_ASSERT_EACH_EQUAL_UINT8(0xff, common.rxBuff, sz);
	}
}


/* Test write & read operations */

TEST(test_flashdrv, write_outOfRange)
{
	TEST_ASSERT_EQUAL(-EINVAL, flashdrv_write(CFI_SIZE_FLASH(common.info->cfi.chipSize), common.txBuff, 0x1));
}


TEST(test_flashdrv, write_fromInvalBuff)
{
	TEST_ASSERT_EQUAL(-EINVAL, flashdrv_write(0, NULL, 0x1000));
}


TEST(test_flashdrv, write_flashSync)
{
	addr_t offs;
	size_t sectorSz, sz;

	/* Set sample offset in nor flash memory */
	sectorSz = CFI_SIZE_SECTION(common.info->cfi.regs[0].size);
	offs = (common.info->cfi.regs[0].count - 1) * sectorSz;
	sz = sectorSz - (sectorSz / 4);

	/* Data size is not equal sector size, using flashdrv_sync data should be flashed */
	test_writeSyncVerify(offs, sz);
}


TEST(test_flashdrv, write_flashNonSync)
{
	addr_t offs;
	size_t sectorSz, sz;

	/* Sample offset in nor flash memory */
	sectorSz = CFI_SIZE_SECTION(common.info->cfi.regs[0].size);
	sz = sectorSz - sectorSz / 4;
	offs = (common.info->cfi.regs[0].count - 1) * sectorSz;

	/* Prepare sample data to write */
	test_initBuffs(sz);

	/* Data size is not equal sector size, without flashdrv_sync data should not be flashed */
	TEST_ASSERT_GREATER_OR_EQUAL_INT32_MESSAGE(offs + sz, CFI_SIZE_FLASH(common.info->cfi.chipSize), "wrong offset");
	TEST_ASSERT_EQUAL(sz, flashdrv_write(offs, common.txBuff, sz));
	TEST_ASSERT_EQUAL(sz, flashdrv_read(offs, common.rxBuff, sz, 0));
	TEST_ASSERT_EQUAL(-EINVAL, memcmp(common.rxBuff, common.txBuff, sz) != 0 ? -EINVAL : 0);
}


TEST(test_flashdrv, write_smallSectorSz)
{
	int i;
	addr_t offs;
	size_t sz = 0, minSz = (size_t)-1;

	/* Find min section size */
	for (i = 0; i < common.info->cfi.regsCount; ++i) {
		sz = CFI_SIZE_SECTION(common.info->cfi.regs[i].size);

		if (sz < minSz)
			minSz = sz;
	}

	/* Sample offset in nor flash memory */
	offs = 14 * minSz;

	/* Data size might not be equaled sector size, using flashdrv_sync data should be flashed */
	test_writeSyncVerify(offs, sz);
}


TEST(test_flashdrv, write_bigSectorSz)
{
	int i;
	addr_t offs;
	size_t sz = 0, maxSz = 0;

	/* Find max sector size */
	for (i = 0; i < common.info->cfi.regsCount; ++i) {
		sz = CFI_SIZE_SECTION(common.info->cfi.regs[i].size);
		if (sz > maxSz)
			maxSz = sz;
	}

	/* Sample offset in nor flash memory */
	offs = 20 * maxSz;

	/* Data size might not be equaled sector size, using flashdrv_sync data should be flashed */
	test_writeSyncVerify(offs, sz);
}


TEST(test_flashdrv, write_smallChunks)
{
	int i;
	static const int offs = 0x100;
	static const size_t sz = 0x20;

	/* Check different length of input buffer to verify correct interpretation of dummy bytes */
	for (i = 1; i < sz; i++)
		test_writeSyncVerify(offs, i);
}


TEST(test_flashdrv, write_bigChunk)
{
	size_t flashSz = CFI_SIZE_FLASH(common.info->cfi.chipSize);
	addr_t offs = flashSz / 4;

	/* Test SIZE_DEFAULT_BUFF data length */
	test_writeSyncVerify(offs, SIZE_DEFAULT_BUFF);
}


TEST(test_flashdrv, write_pageAtBegin)
{
	/* Write and verify page size at the beginning of memory */
	test_writeSyncVerify(0, CFI_SIZE_PAGE(common.info->cfi.pageSize));
}


TEST(test_flashdrv, write_atEnd)
{
	static const size_t sz = 0x1000;
	addr_t offs = CFI_SIZE_FLASH(common.info->cfi.chipSize) - sz;

	/* Write and verify page size at the end of memory */
	test_writeSyncVerify(offs, sz);
}


TEST(test_flashdrv, write_betweenSectors)
{
	size_t sz = CFI_SIZE_SECTION(common.info->cfi.regs[0].size);
	addr_t offs = sz * 2 + sz / 2;

	test_writeSyncVerify(offs, sz);
}


TEST(test_flashdrv, write_betweenRegions)
{
	addr_t offs;
	size_t sz;

	/* Verify writing data between regions */
	if (common.info->cfi.regsCount != 1) {
		sz = CFI_SIZE_SECTION(common.info->cfi.regs[0].size);
		offs = CFI_SIZE_REGION(common.info->cfi.regs[0].size, common.info->cfi.regs[0].count) - (sz / 2);

		test_writeSyncVerify(offs, sz);
	}
}


TEST_GROUP_RUNNER(test_flashdrv)
{
	RUN_TEST_CASE(test_flashdrv, info_validate);

	RUN_TEST_CASE(test_flashdrv, read_toInvalBuff);
	RUN_TEST_CASE(test_flashdrv, read_fromInvalOffs);

	RUN_TEST_CASE(test_flashdrv, erase_invalOffs);
	RUN_TEST_CASE(test_flashdrv, erase_outOfMem);
	RUN_TEST_CASE(test_flashdrv, erase_sector);
	RUN_TEST_CASE(test_flashdrv, erase_chip);

	RUN_TEST_CASE(test_flashdrv, write_outOfRange);
	RUN_TEST_CASE(test_flashdrv, write_fromInvalBuff);
	RUN_TEST_CASE(test_flashdrv, write_flashSync);
	RUN_TEST_CASE(test_flashdrv, write_flashNonSync);
	RUN_TEST_CASE(test_flashdrv, write_smallSectorSz);
	RUN_TEST_CASE(test_flashdrv, write_bigSectorSz);
	RUN_TEST_CASE(test_flashdrv, write_smallChunks);
	RUN_TEST_CASE(test_flashdrv, write_bigChunk);
	RUN_TEST_CASE(test_flashdrv, write_pageAtBegin);
	RUN_TEST_CASE(test_flashdrv, write_atEnd);
	RUN_TEST_CASE(test_flashdrv, write_betweenSectors);
	RUN_TEST_CASE(test_flashdrv, write_betweenRegions);
}


void runner(void)
{
	RUN_TEST_GROUP(test_flashdrv);
}


int main(int argc, char *argv[])
{
	UnityMain(argc, (const char **)argv, runner);
	return 0;
}
