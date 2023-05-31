/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL ECSPI NOR flash driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <sys/threads.h>

#include "imx6ull-flashnor-drv.h"


struct _storage_devCtx_t {
	const flashnor_ops_t *ops;
	int ndev;
};


static int flashnor_mtdErase(storage_t *storage, off_t offs, size_t size)
{
	int err;

	/* Offs alignment is not checked as meterfs sometimes issues unaligned erases. */
	if ((size % storage->dev->mtd->erasesz) != 0) {
		return -EINVAL;
	}

	err = storage->dev->ctx->ops->erase(storage->dev->ctx->ndev, offs, size);
	if (err < 0) {
		return err;
	}
	return EOK;
}


static int flashnor_mtdRead(storage_t *storage, off_t offs, void *data, size_t len, size_t *retlen)
{
	int res;

	res = storage->dev->ctx->ops->read(storage->dev->ctx->ndev, offs, data, len);
	if (res >= 0) {
		*retlen = res;
	}

	return (res < 0) ? res : EOK;
}


static int flashnor_mtdWrite(storage_t *storage, off_t offs, const void *data, size_t len, size_t *retlen)
{
	int res;

	res = storage->dev->ctx->ops->write(storage->dev->ctx->ndev, offs, data, len);
	if (res >= 0) {
		*retlen = res;
	}

	return (res < 0) ? res : EOK;
}


void flashnor_drvDone(storage_t *storage)
{
	if (storage->parent == NULL) { /* Only parent has to clean up. */
		free(storage->dev->ctx);

		free(storage->dev->mtd);
		free(storage->dev->blk);
		free(storage->dev);
	}
}


int flashnor_drvInit(const flashnor_info_t *info, storage_t *storage)
{
	static const storage_mtdops_t mtdOps = {
		.erase = flashnor_mtdErase,
		.unPoint = NULL,
		.point = NULL,
		.read = flashnor_mtdRead,
		.write = flashnor_mtdWrite,

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

	if (storage->parent == NULL) {
		storage->start = 0;
		storage->size = info->devInfo->size;

		storage->dev = malloc(sizeof(storage_dev_t));
		if (storage->dev == NULL) {
			return -ENOMEM;
		}

		storage->dev->mtd = malloc(sizeof(storage_mtd_t));
		if (storage->dev->mtd == NULL) {
			free(storage->dev);
			return -ENOMEM;
		}

		storage->dev->mtd->type = mtd_norFlash;
		storage->dev->mtd->name = info->devInfo->name;
		storage->dev->mtd->metaSize = 0;
		storage->dev->mtd->oobAvail = 0;
		storage->dev->mtd->oobSize = 0;
		storage->dev->mtd->writesz = 1;
		storage->dev->mtd->writeBuffsz = info->devInfo->writeBuffsz;
		storage->dev->mtd->erasesz = info->devInfo->erasesz;

		/* Block device not yet implemented. */
		storage->dev->blk = NULL;

		storage->dev->ctx = malloc(sizeof(struct _storage_devCtx_t));
		if (storage->dev->ctx == NULL) {
			free(storage->dev->mtd);
			free(storage->dev);
			return -ENOMEM;
		}
		storage->dev->ctx->ndev = info->ndev;
		storage->dev->ctx->ops = info->ops;

		storage->dev->mtd->ops = &mtdOps;
	}
	else {
		storage->dev = storage->parent->dev;
	}

	return EOK;
}
