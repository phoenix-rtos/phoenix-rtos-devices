/*
 * Phoenix-RTOS
 *
 * Nor Flash driver tests for zynq-7000
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski, Malgorzata Wrobel
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

	idtree_t strgs;
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


static void test_writeSyncVerify(storage_t *strg, addr_t offs, size_t sz)
{
	/* Prepare sample data to write */
	test_initBuffs(sz);

	/* Write */
	TEST_ASSERT_EQUAL(sz, strg->dev->blk->ops->write(strg, offs, common.txBuff, sz));

	/* Synchronize */
	TEST_ASSERT_EQUAL(EOK, strg->dev->blk->ops->sync(strg));

	/* Verify */
	TEST_ASSERT_EQUAL(sz, strg->dev->blk->ops->read(strg, offs, common.rxBuff, sz));
	TEST_ASSERT_EQUAL_UINT8_ARRAY(common.txBuff, common.rxBuff, sz);
	if (sz < common.buffSz)
		TEST_ASSERT_EQUAL(VALUE_DEFAULT_SAMPLE, common.rxBuff[sz]);
}


/* Setup */

TEST_GROUP(test_flashblk);


TEST_SETUP(test_flashblk)
{
	int fdrvRes;
	storage_t *strg;

	common.txBuff = NULL;
	common.rxBuff = NULL;
	common.buffSz = SIZE_DEFAULT_BUFF;
	idtree_init(&common.strgs);

	/* Initialize flash memory */
	do {
		strg = calloc(1, sizeof(storage_t));
		TEST_ASSERT_NOT_NULL(strg);

		fdrvRes = flashdrv_init(strg);
		TEST_ASSERT_GREATER_OR_EQUAL(0, fdrvRes);

		TEST_ASSERT_NOT_NULL(strg->dev);
		TEST_ASSERT_NOT_NULL(strg->dev->blk);
		TEST_ASSERT_NOT_NULL(strg->dev->blk->ops);
		TEST_ASSERT_NOT_NULL(strg->dev->blk->ops->sync);
		TEST_ASSERT_NOT_NULL(strg->dev->blk->ops->write);
		TEST_ASSERT_NOT_NULL(strg->dev->blk->ops->read);

		idtree_alloc(&common.strgs, &strg->node);

		/* Find max sector size */
		if (strg->size > common.buffSz)
			common.buffSz = strg->size;
	} while (fdrvRes > 0);

	/* Initialize buffers */
	common.rxBuff = malloc(common.buffSz);
	common.txBuff = malloc(common.buffSz);

	TEST_ASSERT_NOT_NULL(common.rxBuff);
	TEST_ASSERT_NOT_NULL(common.txBuff);
}


TEST_TEAR_DOWN(test_flashblk)
{
	rbnode_t *node;
	storage_t *strg;

	for (node = lib_rbMinimum(common.strgs.root); node != NULL; node = lib_rbNext(node)) {
		strg = lib_treeof(storage_t, node, node);

		flashdrv_done(strg);
		idtree_remove(&common.strgs, &strg->node);

		free(strg);
	}

	free(common.rxBuff);
	free(common.txBuff);
}


/* Test info about flash */

TEST(test_flashblk, info_blk)
{
	rbnode_t *node;
	storage_t *strg;

	for (node = lib_rbMinimum(common.strgs.root); node != NULL; node = lib_rbNext(node)) {
		strg = lib_treeof(storage_t, node, node);

		TEST_ASSERT_NOT_EQUAL(0, strg->size);
	}
}


/* Test write & read operations */


TEST(test_flashblk, write_flashSync)
{
	rbnode_t *node;
	storage_t *strg;

	for (node = lib_rbMinimum(common.strgs.root); node != NULL; node = lib_rbNext(node)) {
		strg = lib_treeof(storage_t, node, node);

		/* Data size and offset are not aligned to typicall values */
		test_writeSyncVerify(strg, 417, 373);
	}
}


TEST(test_flashblk, write_flashNonSync)
{
	addr_t offs = 317;
	size_t sz = 47;

	rbnode_t *node;
	storage_t *strg;

	for (node = lib_rbMinimum(common.strgs.root); node != NULL; node = lib_rbNext(node)) {
		strg = lib_treeof(storage_t, node, node);

		/* Prepare sample data to write */
		test_initBuffs(sz);

		/* Data size is not equal sector size, without sync data should not be flashed */
		TEST_ASSERT_EQUAL(sz, strg->dev->blk->ops->write(strg, offs, common.txBuff, sz));

		/* Read data stored in cache */
		TEST_ASSERT_EQUAL(sz, strg->dev->blk->ops->read(strg, offs, common.rxBuff, sz));

		/* Verify */
		TEST_ASSERT_EQUAL(0, memcmp(common.rxBuff, common.txBuff, sz) != 0 ? -EINVAL : 0);

		/* Synchronize */
		TEST_ASSERT_EQUAL(EOK, strg->dev->blk->ops->sync(strg));

		/* Read data from flash memory via cache */
		TEST_ASSERT_EQUAL(sz, strg->dev->blk->ops->read(strg, offs, common.rxBuff, sz));

		/* Verify */
		TEST_ASSERT_EQUAL(0, memcmp(common.rxBuff, common.txBuff, sz) != 0 ? -EINVAL : 0);
	}
}


TEST(test_flashblk, write_smallSectorSz)
{
	addr_t offs;
	rbnode_t *node;
	storage_t *strg;
	static const size_t sz = 0x1000; /* The smallest sector size for NOR flash memories */

	for (node = lib_rbMinimum(common.strgs.root); node != NULL; node = lib_rbNext(node)) {
		strg = lib_treeof(storage_t, node, node);

		/* Sample offset in nor flash memory */
		offs = 10 * sz;

		/* Data size might not be equaled sector size, using flashdrv_sync data should be flashed */
		test_writeSyncVerify(strg, offs, sz);
	}
}


TEST(test_flashblk, write_bigSectorSz)
{
	addr_t offs;
	rbnode_t *node;
	storage_t *strg;
	static const size_t sz = 0x10000;

	for (node = lib_rbMinimum(common.strgs.root); node != NULL; node = lib_rbNext(node)) {
		strg = lib_treeof(storage_t, node, node);

		/* Sample offset in nor flash memory */
		offs = 7 * sz;
		/* Skip small regions */
		if (offs > strg->size)
			continue;

		/* Data size might not be equaled sector size, using flashdrv_sync data should be flashed */
		test_writeSyncVerify(strg, offs, sz);
	}
}


TEST(test_flashblk, write_smallChunks)
{
	int i;
	static const int offs = 0x100;
	static const size_t sz = 0x20;

	rbnode_t *node;
	storage_t *strg;

	for (node = lib_rbMinimum(common.strgs.root); node != NULL; node = lib_rbNext(node)) {
		strg = lib_treeof(storage_t, node, node);

		/* Check different length of input buffer to verify correct interpretation of dummy bytes */
		for (i = 1; i < sz; i++)
			test_writeSyncVerify(strg, offs, i);
	}
}


TEST(test_flashblk, write_bigChunk)
{
	addr_t offs;
	rbnode_t *node;
	storage_t *strg;

	for (node = lib_rbMinimum(common.strgs.root); node != NULL; node = lib_rbNext(node)) {
		strg = lib_treeof(storage_t, node, node);
		offs = strg->size / 4;

		/* Test SIZE_DEFAULT_BUFF data length */
		test_writeSyncVerify(strg, offs, SIZE_DEFAULT_BUFF);
	}
}


TEST(test_flashblk, write_pageAtBegin)
{
	rbnode_t *node;
	storage_t *strg;

	for (node = lib_rbMinimum(common.strgs.root); node != NULL; node = lib_rbNext(node)) {
		strg = lib_treeof(storage_t, node, node);

		/* Write and verify page size at the beginning of memory */
		test_writeSyncVerify(strg, 0, 0x200);
	}
}


TEST(test_flashblk, write_atEnd)
{
	addr_t offs;
	rbnode_t *node;
	storage_t *strg;
	static const size_t sz = 0x1000;

	for (node = lib_rbMinimum(common.strgs.root); node != NULL; node = lib_rbNext(node)) {
		strg = lib_treeof(storage_t, node, node);
		offs = strg->size - sz;

		/* Write and verify page size at the end of memory */
		test_writeSyncVerify(strg, offs, sz);
	}
}


TEST(test_flashblk, write_betweenSectors)
{
	size_t sz;
	rbnode_t *node;
	storage_t *strg;

	for (node = lib_rbMinimum(common.strgs.root); node != NULL; node = lib_rbNext(node)) {
		strg = lib_treeof(storage_t, node, node);

		sz = 0x1000;
		test_writeSyncVerify(strg, sz / 2, sz);

		sz = 0x10000;
		test_writeSyncVerify(strg, sz / 2, sz);
	}
}


TEST_GROUP_RUNNER(test_flashblk)
{
	RUN_TEST_CASE(test_flashblk, info_blk);

	RUN_TEST_CASE(test_flashblk, write_flashSync);
	RUN_TEST_CASE(test_flashblk, write_flashNonSync);
	RUN_TEST_CASE(test_flashblk, write_smallSectorSz);
	RUN_TEST_CASE(test_flashblk, write_bigSectorSz);
	RUN_TEST_CASE(test_flashblk, write_smallChunks);
	RUN_TEST_CASE(test_flashblk, write_bigChunk);
	RUN_TEST_CASE(test_flashblk, write_pageAtBegin);
	RUN_TEST_CASE(test_flashblk, write_atEnd);
	RUN_TEST_CASE(test_flashblk, write_betweenSectors);
}


void runner(void)
{
	RUN_TEST_GROUP(test_flashblk);
}


int main(int argc, char *argv[])
{
	UnityMain(argc, (const char **)argv, runner);
	return 0;
}
