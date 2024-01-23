/*
 * Phoenix-RTOS
 *
 * SD Card libstorage-based driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "sdstorage_dev.h"

#include <errno.h>
#include <limits.h>
#include <sys/list.h>
#include <sys/msg.h>
#include <sys/minmax.h>
#include <sys/types.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <posix/utils.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <cache.h>
#include <mtd/mtd.h>
#include <storage/storage.h>
#include <mbr.h>

#include "sdcard.h"

#define LOG_TAG "sdstorage_dev"
/* clang-format off */
#define LOG_ERROR(str, ...) do { fprintf(stderr, LOG_TAG " error: " str "\n", ##__VA_ARGS__); } while (0)
#define TRACE(str, ...)     do { if (0) fprintf(stderr, LOG_TAG " trace: " str "\n", ##__VA_ARGS__); } while (0)
/* clang-format on */

#define BLK_CACHE_SECSIZE   (2 * SDCARD_BLOCKLEN) /* Size of cache sector (must be multiple of SDCARD_BLOCKLEN) */
#define BLK_CACHE_SECNUM    16                    /* Maximum number of cached sectors in a region */
#define MTD_DEFAULT_ERASESZ 0x10000

#define MTD_DEV_FORMAT   "mmcmtd%u"
#define BLOCK_DEV_FORMAT "mmcblk%u"

typedef uint32_t blockSize_t;

typedef struct {
	blockSize_t offsetBl;
	blockSize_t sizeBl;
} sdcard_partition_t;

struct cache_devCtx_s {
	unsigned int id;
	/* cache_deinit tries to flush cache and if writing fails it stops freeing up resources.
	 * As a workaround, noFlushShutdown makes all operations "succeed" and behave as no-op.
	 */
	bool noFlushShutdown;
};

typedef struct _storage_devCtx_t {
	unsigned int id;
	handle_t lock;
	int cachePolicy;
	cachectx_t *cache;
	cache_devCtx_t devCtxForCache;
} storage_devCtx_t;

static struct {
	bool commonInit;
	handle_t lock;
	mbr_t mbr_temp;
	int defaultCachePolicy;
} sdcard_common = { .commonInit = false };

#define PRESENCE_THREAD_STACK_SIZE 1024
static char presenceThreadStack[PRESENCE_THREAD_STACK_SIZE] __attribute__((aligned(8)));


static size_t calculateSizeWithSaturation(blockSize_t sizeBlocks)
{
	size_t sizeBytes;
	if (sizeof(sizeBytes) > 5) {
		sizeBytes = (uint64_t)sizeBlocks * SDCARD_BLOCKLEN;
	}
	else {
		if (sizeBlocks <= (UINT32_MAX / SDCARD_BLOCKLEN)) {
			sizeBytes = sizeBlocks * SDCARD_BLOCKLEN;
		}
		else {
			sizeBytes = UINT32_MAX;
		}
	}

	return sizeBytes;
}


static int sdcard_readCb(uint64_t offs, void *buff, size_t len, cache_devCtx_t *ctx)
{
	if (ctx->noFlushShutdown) {
		return len;
	}

	if ((offs % SDCARD_BLOCKLEN) != 0 || (len % SDCARD_BLOCKLEN) != 0) {
		LOG_ERROR("read bad offset or size");
		return -EINVAL;
	}

	uint32_t lba = offs / SDCARD_BLOCKLEN;
	len = min(len, SDCARD_MAX_TRANSFER);
	int ret = sdcard_transferBlocks(ctx->id, sdio_read, lba, buff, len);
	return ret < 0 ? ret : len;
}


static int sdcard_writeCb(uint64_t offs, const void *buff, size_t len, cache_devCtx_t *ctx)
{
	if (ctx->noFlushShutdown) {
		return len;
	}

	if ((offs % SDCARD_BLOCKLEN) != 0 || (len % SDCARD_BLOCKLEN) != 0) {
		LOG_ERROR("write bad offset or size");
		return -EINVAL;
	}

	uint32_t lba = offs / SDCARD_BLOCKLEN;
	len = min(len, SDCARD_MAX_TRANSFER);

	int ret = sdcard_transferBlocks(ctx->id, sdio_write, lba, (void *)buff, len);
	return ret < 0 ? ret : len;
}


static ssize_t sdstorage_cachedRead(struct _storage_t *strg, off_t start, void *data, size_t size)
{
	ssize_t res;
	storage_devCtx_t *ctx = strg->dev->ctx;
	mutexLock(ctx->lock);
	res = cache_read(ctx->cache, start, data, size);
	mutexUnlock(ctx->lock);

	return res;
}


static ssize_t sdstorage_cachedWrite(struct _storage_t *strg, off_t start, const void *data, size_t size)
{
	ssize_t res;
	storage_devCtx_t *ctx = strg->dev->ctx;
	mutexLock(ctx->lock);
	res = cache_write(ctx->cache, start, data, size, ctx->cachePolicy);
	mutexUnlock(ctx->lock);

	return res;
}


static int sdstorage_cachedFlush(struct _storage_t *strg)
{
	ssize_t res;
	storage_devCtx_t *ctx = strg->dev->ctx;
	mutexLock(ctx->lock);
	res = cache_flush(ctx->cache, strg->start, strg->start + strg->size);
	mutexUnlock(ctx->lock);

	return res;
}


const static storage_blkops_t blkOps = {
	.read = sdstorage_cachedRead,
	.write = sdstorage_cachedWrite,
	.sync = sdstorage_cachedFlush,
};


static int sdstorage_mtdErase(struct _storage_t *strg, off_t offs, size_t size)
{
	int res;
	if ((offs % SDCARD_BLOCKLEN != 0) || size % SDCARD_BLOCKLEN != 0) {
		LOG_ERROR("erase bad offset or size");
		return -EINVAL;
	}

	mutexLock(strg->dev->ctx->lock);
	res = sdcard_writeFF(strg->dev->ctx->id, offs / SDCARD_BLOCKLEN, size / SDCARD_BLOCKLEN);
	if (res < 0) {
		mutexUnlock(strg->dev->ctx->lock);
		return res;
	}

	/* Invalidate block device cache for coherence */
	res = cache_invalidate(strg->dev->ctx->cache, offs, offs + size);

	mutexUnlock(strg->dev->ctx->lock);

	return res < 0 ? res : EOK;
}


static int sdstorage_mtdRead(struct _storage_t *strg, off_t offs, void *data, size_t len, size_t *retlen)
{
	int res = sdstorage_cachedRead(strg, offs, data, len);
	if (res >= 0) {
		*retlen = len;
	}

	return res < 0 ? res : EOK;
}


static int sdstorage_mtdWrite(struct _storage_t *strg, off_t offs, const void *data, size_t len, size_t *retlen)
{
	int res = sdstorage_cachedWrite(strg, offs, data, len);
	if (res >= 0) {
		*retlen = len;
	}

	return res < 0 ? res : EOK;
}


const static storage_mtdops_t mtdOps = {
	.erase = sdstorage_mtdErase,
	.unPoint = NULL,
	.point = NULL,
	.read = sdstorage_mtdRead,
	.write = sdstorage_mtdWrite,

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


static storage_devCtx_t *sdstorage_devCtxAlloc(void)
{
	storage_devCtx_t *dev = malloc(sizeof(storage_devCtx_t));
	if (dev == NULL) {
		return NULL;
	}

	if (mutexCreate(&dev->lock)) {
		free(dev);
		return NULL;
	}

	dev->cache = NULL;
	dev->devCtxForCache.noFlushShutdown = false;
	dev->cachePolicy = sdcard_common.defaultCachePolicy;

	return dev;
}


static void sdstorage_devCtxFree(storage_devCtx_t *dev, bool noFlush)
{
	if (dev == NULL) {
		return;
	}

	if (dev->cache != NULL) {
		dev->devCtxForCache.noFlushShutdown = noFlush;
		cache_deinit(dev->cache);
	}

	resourceDestroy(dev->lock);
	free(dev);
}


static int sdstorage_createDeviceFile(oid_t *oid, const char *pathFmt, unsigned int parentID, unsigned int partID)
{
	int ret;
	char path[32];
	int pos = snprintf(path, sizeof(path), pathFmt, parentID);
	if (pos >= sizeof(path)) {
		ret = -ENAMETOOLONG;
		LOG_ERROR("failed to build file path, err: %d", ret);
		return ret;
	}

	if (partID != 0) {
		pos += snprintf(path + pos, sizeof(path) - pos, "p%u", partID);
		if (pos >= sizeof(path)) {
			ret = -ENAMETOOLONG;
			LOG_ERROR("failed to build file path, err: %d", ret);
			return ret;
		}
	}

	ret = create_dev(oid, path);
	if (ret < 0) {
		LOG_ERROR("failed to create a device file, err: %d", ret);
		return ret;
	}

	return EOK;
}


static int sdstorage_createDevices(const storage_t *strg, oid_t *oid)
{
	int ret;
	unsigned int parentID = strg->dev->ctx->id, partID = 0;

	/* Find id for a new partition */
	if (strg->parent != NULL) {
		storage_t *part = strg->parent->parts;
		if (part != NULL) {
			do {
				partID++;
				part = part->next;
			} while (part != strg->parent->parts);
		}
	}

	/* Add mtdchar device */
	if (strg->dev->mtd != NULL) {
		oid->id &= ~DEVTYPE_MASK;
		oid->id |= DEVTYPE_MTD;

		ret = sdstorage_createDeviceFile(oid, "/dev/" MTD_DEV_FORMAT, parentID, partID);
		if (ret < 0) {
			return ret;
		}
	}

	/* Add block device */
	if (strg->dev->blk != NULL) {
		oid->id &= ~DEVTYPE_MASK;
		oid->id |= DEVTYPE_BLOCK;

		ret = sdstorage_createDeviceFile(oid, "/dev/" BLOCK_DEV_FORMAT, parentID, partID);
		if (ret < 0) {
			return ret;
		}
	}

	return EOK;
}


static int sdstorage_addPartition(storage_t *parent, sdcard_partition_t part)
{
	int ret;

	storage_t *strg = malloc(sizeof(storage_t));
	if (strg == NULL) {
		return -ENOMEM;
	}

	strg->start = parent->start + (off_t)part.offsetBl * SDCARD_BLOCKLEN;
	strg->size = calculateSizeWithSaturation(part.sizeBl);
	strg->parent = parent;
	strg->dev = parent->dev;
	strg->parts = NULL;

	oid_t oid;
	ret = storage_add(strg, &oid);
	if (ret < 0) {
		free(strg);
		return ret;
	}

	ret = sdstorage_createDevices(strg, &oid);
	if (ret < 0) {
		storage_remove(strg);
		free(strg);
		return ret;
	}

	return 0;
}


/* Returns number of valid MBR partitions or < 0 if an error occurred while reading.
 * If MBR is malformed it does not count as an error and 0 is returned.
 */
static int sdstorage_checkMBR(unsigned int slot, sdcard_partition_t parts[4])
{
	mbr_t *mbr = &sdcard_common.mbr_temp;
	if (sdcard_transferBlocks(slot, sdio_read, 0, mbr, SDCARD_BLOCKLEN) < 0) {
		LOG_ERROR("mbr read failed");
		return -EIO;
	}

	if (mbr_deserialize(mbr) < 0) {
		return 0;
	}

	int partNum = 0;
	for (int i = 0; i < 4; i++) {
		if (mbr->pent[i].sectors != 0) {
			parts[partNum].offsetBl = mbr->pent[i].start;
			parts[partNum].sizeBl = mbr->pent[i].sectors;
			partNum++;
		}
	}

	return partNum;
}


int sdstorage_handleInsertion(unsigned int slot)
{
	storage_t *strg;
	oid_t oid;
	int ret;

	mutexLock(sdcard_common.lock);
	for (int i = 0; i < 5; i++) {
		ret = sdcard_initCard(slot, 0);
		if (ret == 0) {
			break;
		}

		usleep(1000);
	}

	if (ret < 0) {
		LOG_ERROR("initialization failed");
		mutexUnlock(sdcard_common.lock);
		return ret;
	}

	sdcard_partition_t parts[4];
	int nParts = sdstorage_checkMBR(slot, parts);
	if (nParts < 0) {
		LOG_ERROR("check MBR failed");
		mutexUnlock(sdcard_common.lock);
		return -EIO;
	}

	strg = malloc(sizeof(storage_t));
	if (strg == NULL) {
		mutexUnlock(sdcard_common.lock);
		return -ENOMEM;
	}

	strg->dev = malloc(sizeof(storage_dev_t));
	if (strg->dev == NULL) {
		free(strg);
		mutexUnlock(sdcard_common.lock);
		return -ENOMEM;
	}

	strg->dev->blk = malloc(sizeof(storage_blk_t));
	if (strg->dev->blk == NULL) {
		free(strg->dev);
		free(strg);
		mutexUnlock(sdcard_common.lock);
		return -ENOMEM;
	}

	strg->dev->mtd = malloc(sizeof(storage_mtd_t));
	if (strg->dev->mtd == NULL) {
		free(strg->dev->blk);
		free(strg->dev);
		free(strg);
		mutexUnlock(sdcard_common.lock);
		return -ENOMEM;
	}

	strg->dev->ctx = sdstorage_devCtxAlloc();
	if (strg->dev->ctx == NULL) {
		free(strg->dev->mtd);
		free(strg->dev->blk);
		free(strg->dev);
		free(strg);
		mutexUnlock(sdcard_common.lock);
		return -ENOMEM;
	}

	blockSize_t sizeBlocks = sdcard_getSizeBlocks(slot);
	size_t sizeBytes = calculateSizeWithSaturation(sizeBlocks);

	strg->dev->ctx->devCtxForCache.id = slot;

	cache_ops_t cacheOps;
	cacheOps.readCb = sdcard_readCb;
	cacheOps.writeCb = sdcard_writeCb;
	cacheOps.ctx = &strg->dev->ctx->devCtxForCache;
	strg->dev->ctx->cache = cache_init(sizeBytes, BLK_CACHE_SECSIZE, BLK_CACHE_SECNUM, &cacheOps);
	if (strg->dev->ctx->cache == NULL) {
		LOG_ERROR("cache init failed");
		sdstorage_devCtxFree(strg->dev->ctx, false);
		free(strg->dev->mtd);
		free(strg->dev->blk);
		free(strg->dev);
		free(strg);
		mutexUnlock(sdcard_common.lock);
		return -ENOMEM;
	}

	strg->dev->ctx->id = slot;
	strg->parent = NULL;
	strg->start = 0;
	strg->size = sizeBytes;
	strg->dev->blk->ops = &blkOps;

	uint32_t eraseSize = sdcard_getEraseSizeBlocks(slot) * SDCARD_BLOCKLEN;
	strg->dev->mtd->type = mtd_norFlash;
	strg->dev->mtd->name = "SD CARD";
	strg->dev->mtd->erasesz = (eraseSize < MTD_DEFAULT_ERASESZ) ? MTD_DEFAULT_ERASESZ : eraseSize;
	strg->dev->mtd->writesz = 1;
	strg->dev->mtd->writeBuffsz = SDCARD_BLOCKLEN;
	strg->dev->mtd->metaSize = 0;
	strg->dev->mtd->oobSize = 0;
	strg->dev->mtd->oobAvail = 0;
	strg->dev->mtd->ops = &mtdOps;

	ret = storage_add(strg, &oid);
	if (ret < 0) {
		sdstorage_devCtxFree(strg->dev->ctx, false);
		free(strg->dev->mtd);
		free(strg->dev->blk);
		free(strg->dev);
		free(strg);
		mutexUnlock(sdcard_common.lock);
		return ret;
	}

	ret = sdstorage_createDevices(strg, &oid);
	if (ret < 0) {
		storage_remove(strg);
		sdstorage_devCtxFree(strg->dev->ctx, false);
		free(strg->dev->mtd);
		free(strg->dev->blk);
		free(strg->dev);
		free(strg);
		mutexUnlock(sdcard_common.lock);
		return ret;
	}

	for (int i = 0; i < nParts; i++) {
		ret = sdstorage_addPartition(strg, parts[i]);
		if (ret < 0) {
			LOG_ERROR("storage add part %d failed: %d", i + 1, ret);
		}
		else {
			TRACE("storage add part %d", i + 1);
		}
	}

	mutexUnlock(sdcard_common.lock);
	return EOK;
}


static int sdstorage_freeStorage(storage_t *strg)
{
	if (strg == NULL) {
		return 0;
	}

	int ret;
	if (strg->fs != NULL) {
		TRACE("remove fs");
		if (strg->fs->mnt != NULL) {
			msg_t msg = { 0 };
			msg.type = mtSetAttr;
			msg.i.attr.type = atDev;
			msg.i.attr.oid = *strg->fs->mnt;
			msg.i.data = strg->fs->mnt;
			msg.i.size = sizeof(*strg->fs->mnt);

			if (msgSend(strg->fs->mnt->port, &msg) < 0) {
				return -EIO;
			}

			free(strg->fs->mnt);
			strg->fs->mnt = NULL;
		}

		ret = storage_umountfs(strg);
		if (ret < 0) {
			return ret;
		}
	}

	TRACE("remove storage");
	ret = storage_remove(strg);
	if (ret < 0) {
		return ret;
	}

	free(strg);
	return 0;
}


static int sdstorage_removeDevice(const char *format, unsigned int slot, unsigned int partition)
{
	int ret;
	char path[32];
	int pos = snprintf(path, sizeof(path), format, slot);
	if (pos >= sizeof(path)) {
		ret = -ENAMETOOLONG;
		LOG_ERROR("failed to build file path, err: %d", ret);
		return ret;
	}

	if (partition != 0) {
		pos += snprintf(path + pos, sizeof(path) - pos, "p%u", partition);
		if (pos >= sizeof(path)) {
			ret = -ENAMETOOLONG;
			LOG_ERROR("failed to build file path, err: %d", ret);
			return ret;
		}
	}

	ret = remove(path);
	/* remove() sometimes returns -1 even though the removal was successful */
	return (ret == -1) ? 0 : ret;
}


int sdstorage_handleRemoval(unsigned int slot)
{
	oid_t oid;
	char temp[32];
	int ret;

	ret = snprintf(temp, sizeof(temp), "/dev/" BLOCK_DEV_FORMAT, slot);
	if (ret >= sizeof(temp)) {
		ret = -ENAMETOOLONG;
		LOG_ERROR("failed to build file path, err: %d", ret);
		return ret;
	}

	ret = lookup(temp, NULL, &oid);
	if (ret < 0) {
		TRACE("dev lookup failed: %d", ret);
		return ret;
	}

	mutexLock(sdcard_common.lock);
	storage_t *strg = storage_get(GET_STORAGE_ID(oid.id));
	if (strg == NULL || strg->dev == NULL || strg->dev->ctx == NULL) {
		mutexUnlock(sdcard_common.lock);
		return EOK;
	}

	storage_dev_t *dev = strg->dev;
	int totalPartitions = 0;
	while (strg->parts != NULL) {
		ret = sdstorage_freeStorage(strg->parts);
		if (ret < 0) {
			mutexUnlock(sdcard_common.lock);
			return ret;
		}

		totalPartitions++;
	}

	ret = storage_remove(strg);
	if (ret < 0) {
		TRACE("remove main failed %d", ret);
		mutexUnlock(sdcard_common.lock);
		return ret;
	}

	sdstorage_devCtxFree(dev->ctx, true);
	free(dev->mtd);
	free(dev->blk);
	free(dev);

	for (int i = 0; i <= totalPartitions; i++) {
		ret = sdstorage_removeDevice("/dev/" MTD_DEV_FORMAT, slot, i);
		if (ret < 0) {
			TRACE("fail mtd %d", ret);
			mutexUnlock(sdcard_common.lock);
			return ret;
		}

		ret = sdstorage_removeDevice("/dev/" BLOCK_DEV_FORMAT, slot, i);
		if (ret < 0) {
			TRACE("fail blk %d", ret);
			mutexUnlock(sdcard_common.lock);
			return ret;
		}
	}

	mutexUnlock(sdcard_common.lock);
	return EOK;
}


static void sdstorage_presenceThread(void *arg)
{
	sdcard_presenceThread(sdstorage_handleInsertion, sdstorage_handleRemoval);
	endthread();
}


int sdstorage_runPresenceDetection(void)
{
	sdcard_handlePresence(sdstorage_handleInsertion, NULL);
	return beginthread(sdstorage_presenceThread, 3, presenceThreadStack, PRESENCE_THREAD_STACK_SIZE, NULL);
}


int sdstorage_initHost(unsigned int slot)
{
	if (!sdcard_common.commonInit) {
		if (mutexCreate(&sdcard_common.lock) < 0) {
			LOG_ERROR("Can't create mutex");
			return -ENOMEM;
		}

		sdcard_common.defaultCachePolicy = LIBCACHE_WRITE_THROUGH;
		sdcard_common.commonInit = true;
	}

	return sdcard_initHost(slot);
}


void sdstorage_setDefaultCachePolicy(int cachePolicy)
{
	sdcard_common.defaultCachePolicy = cachePolicy;
}
