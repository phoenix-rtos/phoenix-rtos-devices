/*
 * Phoenix-RTOS
 *
 * Zynq Flash forwarder storage device
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jakub Klimek
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include "flashfwd.h"


static int flashfwd_mtdErase(storage_t *strg, off_t offs, size_t size)
{
	devlimit_data_t *limitData = (devlimit_data_t *)strg->dev->ctx;
	int err;

	err = limitData->limitF(limitData, 0U, 0U);
	if (err != EOK) {
		return err;
	}

	return limitData->targetStrg->dev->mtd->ops->erase(limitData->targetStrg, offs, size);
}


static int flashfwd_mtdRead(storage_t *strg, off_t offs, void *data, size_t len, size_t *retlen)
{
	devlimit_data_t *limitData = (devlimit_data_t *)strg->dev->ctx;
	int err;

	err = limitData->limitF(limitData, 0U, len);
	if (err != EOK) {
		return err;
	}

	return limitData->targetStrg->dev->mtd->ops->read(limitData->targetStrg, offs, data, len, retlen);
}


static int flashfwd_mtdWrite(storage_t *strg, off_t offs, const void *data, size_t len, size_t *retlen)
{
	devlimit_data_t *limitData = (devlimit_data_t *)strg->dev->ctx;
	int err;

	err = limitData->limitF(limitData, len, 0U);
	if (err != EOK) {
		return err;
	}

	return limitData->targetStrg->dev->mtd->ops->write(limitData->targetStrg, offs, data, len, retlen);
}


static ssize_t flashfwd_blkRead(storage_t *strg, off_t start, void *data, size_t size)
{
	devlimit_data_t *limitData = (devlimit_data_t *)strg->dev->ctx;
	int err;

	err = limitData->limitF(limitData, 0U, size);
	if (err != EOK) {
		return err;
	}

	return limitData->targetStrg->dev->blk->ops->read(limitData->targetStrg, start, data, size);
}


static ssize_t flashfwd_blkWrite(storage_t *strg, off_t start, const void *data, size_t size)
{
	devlimit_data_t *limitData = (devlimit_data_t *)strg->dev->ctx;
	int err;

	err = limitData->limitF(limitData, size, 0U);
	if (err != EOK) {
		return err;
	}

	return limitData->targetStrg->dev->blk->ops->write(limitData->targetStrg, start, data, size);
}


static int flashfwd_blkSync(storage_t *strg)
{
	devlimit_data_t *limitData = (devlimit_data_t *)strg->dev->ctx;
	int err;

	err = limitData->limitF(limitData, 0U, 0U);
	if (err != EOK) {
		return err;
	}

	return limitData->targetStrg->dev->blk->ops->sync(limitData->targetStrg);
}


const storage_mtdops_t mtdFwOps = {
	.erase = flashfwd_mtdErase,
	.unPoint = NULL,
	.point = NULL,
	.read = flashfwd_mtdRead,
	.write = flashfwd_mtdWrite,

	.meta_read = NULL,
	.meta_write = NULL,

	.sync = NULL,
	.lock = NULL,
	.unLock = NULL,
	.isLocked = NULL,

	.block_isBad = NULL,
	.block_isReserved = NULL,
	.block_markBad = NULL,
	.block_maxBadNb = NULL,

	.suspend = NULL,
	.resume = NULL,
	.reboot = NULL,
};


const storage_blkops_t blkFwOps = {
	.read = flashfwd_blkRead,
	.write = flashfwd_blkWrite,
	.sync = flashfwd_blkSync
};
