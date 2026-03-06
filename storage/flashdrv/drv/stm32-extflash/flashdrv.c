/*
 * Phoenix-RTOS
 *
 * STM32 external Flash driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <flashdrv/flashsrv.h>
#include <stm32l4-multi.h>

#define LOOKUP_RETRIES 20

/* This code is basically just an adapter that interfaces flashdrv with stm32l4-multi.
 * No mutexing or argument verification takes place here, because we assume all of it will be
 * performed by multiserver according to its needs. */

typedef struct _storage_devCtx_t {
	oid_t multisrv;
	unsigned int devNum;
	extFlashDef_t flashDef;
} devCtx_t;


#define CHECK(x) \
	do { \
		int _ret = x; \
		if (_ret < 0) { \
			return _ret; \
		} \
	} while (0)


/* Communication with multiserver */

static int flash_getConfig(devCtx_t *ctx)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = ctx->multisrv,
	};
	multi_i_t *i = (multi_i_t *)msg.i.raw;
	multi_o_t *o = (multi_o_t *)msg.o.raw;

	i->type = extFlash_def;
	i->extFlash_op.devNum = ctx->devNum;
	CHECK(msgSend(ctx->multisrv.port, &msg));
	CHECK(msg.o.err);

	memcpy(&ctx->flashDef, &o->extFlash_def, sizeof(ctx->flashDef));
	return 0;
}


static int flash_write(const devCtx_t *ctx, addr_t addr, const void *data, size_t size)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = ctx->multisrv,
	};
	multi_i_t *i = (multi_i_t *)msg.i.raw;

	msg.i.data = data;
	msg.i.size = size;
	i->type = extFlash_write;
	i->extFlash_op.devNum = ctx->devNum;
	i->extFlash_op.addr = addr;
	CHECK(msgSend(ctx->multisrv.port, &msg));
	return msg.o.err;
}


static int flash_read(const devCtx_t *ctx, addr_t addr, void *data, size_t size)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = ctx->multisrv,
	};
	multi_i_t *i = (multi_i_t *)msg.i.raw;

	msg.o.data = data;
	msg.o.size = size;
	i->type = extFlash_read;
	i->extFlash_op.devNum = ctx->devNum;
	i->extFlash_op.addr = addr;
	CHECK(msgSend(ctx->multisrv.port, &msg));
	return msg.o.err;
}


/* MTD interface */


static int flashdrv_mtdRead(storage_t *strg, off_t offs, void *buff, size_t len, size_t *retlen)
{
	int ret = flash_read(strg->dev->ctx, offs, buff, len);
	if (ret < 0) {
		*retlen = 0;
	}
	else {
		*retlen = len;
		ret = 0;
	}

	return ret;
}


static int flashdrv_mtdWrite(storage_t *strg, off_t offs, const void *buff, size_t len, size_t *retlen)
{
	int ret = flash_write(strg->dev->ctx, offs, buff, len);
	if (ret < 0) {
		*retlen = 0;
	}
	else {
		*retlen = len;
		ret = 0;
	}

	return ret;
}


static int flashdrv_mtdErase(storage_t *strg, off_t offs, size_t len)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = strg->dev->ctx->multisrv,
	};
	multi_i_t *i = (multi_i_t *)msg.i.raw;

	i->type = extFlash_erase;
	i->extFlash_op.devNum = strg->dev->ctx->devNum;
	i->extFlash_op.addr = offs;
	i->extFlash_op.eraseSize = len;
	CHECK(msgSend(strg->dev->ctx->multisrv.port, &msg));
	return msg.o.err;
}


static const storage_mtdops_t mtdOps = {
	.erase = flashdrv_mtdErase,
	.unPoint = NULL,
	.point = NULL,
	.read = flashdrv_mtdRead,
	.write = flashdrv_mtdWrite,

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


static void flashdrv_destroy(storage_t *strg)
{
	if (strg == NULL) {
		return;
	}

	if (strg->dev != NULL) {
		free(strg->dev->ctx);
		free(strg->dev->mtd);
		free(strg->dev);
	}

	free(strg);
}


static int flashdrv_multiLookup(oid_t *oid)
{
	int i;
	for (i = 0; i < LOOKUP_RETRIES; i++) {
		if (lookup("devfs/multi", NULL, oid) >= 0) {
			break;
		}

		if (lookup("/dev/multi", NULL, oid) >= 0) {
			break;
		}

		usleep(100000);
	}

	return (i == LOOKUP_RETRIES) ? -ENOENT : 0;
}


static storage_t *flashdrv_init(addr_t mctrlBase, addr_t flashBase)
{
	struct _storage_devCtx_t *ctx = calloc(1, sizeof(struct _storage_devCtx_t));
	if (ctx == NULL) {
		LOG_ERROR("out of memory");
		return NULL;
	}

	(void)flashBase;
	ctx->devNum = mctrlBase;
	if (flashdrv_multiLookup(&ctx->multisrv) < 0) {
		LOG_ERROR("multiserver not found");
		return NULL;
	}

	int ret = flash_getConfig(ctx);
	if (ret < 0) {
		LOG_ERROR("flash_getConfig failed: %d", ret);
		free(ctx);
		return NULL;
	}

	storage_t *strg = calloc(1, sizeof(storage_t));
	if (strg == NULL) {
		free(ctx);
		return NULL;
	}

	strg->start = 0;
	strg->size = ctx->flashDef.totalSize;

	strg->dev = calloc(1, sizeof(storage_dev_t));
	if (strg->dev == NULL) {
		free(ctx);
		free(strg);
		return NULL;
	}

	/* Assign device context */
	strg->dev->ctx = ctx;
	storage_mtd_t *mtd = calloc(1, sizeof(storage_mtd_t));
	if (mtd == NULL) {
		flashdrv_destroy(strg);
		return NULL;
	}

	/* MTD interface */
	mtd->ops = &mtdOps;
	mtd->type = mtd_norFlash;
	mtd->name = ctx->flashDef.name;
	mtd->metaSize = 0;
	mtd->oobSize = 0;
	mtd->oobAvail = 0;
	mtd->writeBuffsz = ctx->flashDef.pageSize;
	mtd->writesz = ctx->flashDef.writeSize;
	mtd->erasesz = ctx->flashDef.eraseSize;

	strg->dev->mtd = mtd;

	/* No block device interface */
	strg->dev->blk = NULL;

	return strg;
}


void __attribute__((constructor)) spimctrl_register(void)
{
	static const struct flash_driver spimctrl = {
		.name = "stm32-extflash",
		.init = flashdrv_init,
		.destroy = flashdrv_destroy,
	};

	flashsrv_register(&spimctrl);
}
