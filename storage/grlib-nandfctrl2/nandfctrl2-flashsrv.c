/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash server.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <endian.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <libjffs2.h>
#include <ptable.h>
#include <storage/storage.h>

/* Per-project board config may override PTABLE_NCOPIES and NAND_DIE_CNT */
#include <board_config.h>

#include "nandfctrl2-flashsrv.h"
#include "nandfctrl2-flashdrv.h"
#include "nandfctrl2-flashdev.h"


/* clang-format off */
#define LOG(str_, ...)       do { printf("grlib-nandfctrl2-flash: " str_ "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(str_, ...) LOG(__FILE__ ":%d error: " str_, __LINE__, ##__VA_ARGS__)
#define TRACE(str_, ...)     do { if (0) LOG(__FILE__ ":%d TRACE: " str_, __LINE__, ##__VA_ARGS__); } while (0)
/* clang-format on */


/* Format for root NAND device path (die N = /dev/mtdN) */
#define PATH_ROOT_FMT  "/dev/mtd%u"
#define PATH_ROOT_STRG "/dev/mtd0"

/* Maximum per-die path length  ("/dev/mtd0" + null) */
#define PATH_MAX_LEN 32

/* Number of partition table copies saved on flash */
#ifndef PTABLE_NCOPIES
#define PTABLE_NCOPIES 4
#endif


/* ============================== Auxiliary ============================== */

static int flash_oidResolve(const char *devPath, oid_t *oid)
{
	int res;
	oid_t dir;
	char temp[PATH_MAX_LEN];

	res = lookup("devfs", NULL, &dir);
	if (res >= 0) {
		if (strncmp("/dev/", devPath, 5) != 0) {
			return -EINVAL;
		}

		res = snprintf(temp, sizeof(temp), "devfs/%s", devPath + 5);
		if (res >= (int)sizeof(temp)) {
			LOG_ERROR("path too long: %s", devPath);
			return -ENAMETOOLONG;
		}
	}
	else {
		strncpy(temp, devPath, sizeof(temp));
		temp[sizeof(temp) - 1] = '\0';
	}

	return lookup(temp, NULL, oid);
}


/* Create symlink manually (root may not be available yet) */
static int flashsrv_devfsSymlink(const char *name, const char *target)
{
	oid_t dir;
	msg_t msg = { 0 };
	int len1, len2;
	int ret;
	void *idata;

	ret = lookup("devfs", NULL, &dir);
	if (ret < 0) {
		return ret;
	}

	msg.type = mtCreate;
	msg.oid = dir;
	msg.i.create.type = otSymlink;
	msg.i.create.mode = S_IFLNK | ACCESSPERMS;

	len1 = strlen(name);
	len2 = strlen(target);

	msg.i.size = len1 + len2 + 2;
	idata = calloc(1, msg.i.size);
	if (idata == NULL) {
		return -ENOMEM;
	}

	memcpy(idata, name, len1);
	memcpy((char *)idata + len1 + 1, target, len2);
	msg.i.data = idata;

	ret = msgSend(dir.port, &msg);
	free(idata);

	return (ret != EOK) ? -EIO : msg.o.err;
}


/* ============================== Partition table ============================== */

/* ptable buffer must have at least mtd->writesz bytes */
static int flashsrv_ptableRead(storage_t *strg, ptable_t *ptable)
{
	storage_mtd_t *mtd = strg->dev->mtd;
	size_t retlen;
	off_t offs;
	uint32_t count;
	int err, i;

	for (i = 0; i < PTABLE_NCOPIES; i++) {
		offs = (off_t)strg->size - (off_t)(i + 1) * mtd->erasesz;
		if (mtd->ops->block_isBad(strg, offs) != 0) {
			continue;
		}

		err = mtd->ops->read(strg, offs, ptable, mtd->writesz, &retlen);
		if (((err < 0) && (err != -EUCLEAN)) || (retlen != mtd->writesz)) {
			continue;
		}

		count = le32toh(ptable->count);
		if (ptable_size(count) > mtd->writesz) {
			continue;
		}

		if (ptable_deserialize(ptable, strg->size, mtd->erasesz) < 0) {
			continue;
		}

		return EOK;
	}

	return -ENOENT;
}


static int flashsrv_ptableWrite(storage_t *strg, const ptable_t *ptable)
{
	storage_mtd_t *mtd = strg->dev->mtd;
	ptable_t *ptab;
	size_t retlen;
	off_t offs;
	uint32_t size;
	int err, i, n = 0;

	size = ptable_size(ptable->count);
	if (size > mtd->writesz) {
		return -EINVAL;
	}

	ptab = malloc(mtd->writesz);
	if (ptab == NULL) {
		return -ENOMEM;
	}
	memcpy(ptab, ptable, size - sizeof(ptable_magic));

	if (ptable_serialize(ptab, strg->size, mtd->erasesz) < 0) {
		free(ptab);
		return -EINVAL;
	}

	for (i = 0; i < PTABLE_NCOPIES; i++) {
		offs = (off_t)strg->size - (off_t)(i + 1) * mtd->erasesz;
		if (mtd->ops->block_isBad(strg, offs) != 0) {
			continue;
		}

		if (mtd->ops->erase(strg, offs, mtd->erasesz) != 1) {
			continue;
		}

		err = mtd->ops->write(strg, offs, ptab, mtd->writesz, &retlen);
		if ((err < 0) || (retlen != mtd->writesz)) {
			continue;
		}
		n++;
	}
	free(ptab);

	return (n > 0) ? EOK : -EIO;
}


/* ============================== Device control ============================== */

static int flashsrv_devInfo(id_t id, flash_o_devctl_t *odevctl)
{
	storage_t *strg = storage_get(id);
	const flashdrv_info_t *devInfo;

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL)) {
		return -EINVAL;
	}

	devInfo = flashdrv_info(strg->dev->ctx->target);
	if (devInfo == NULL) {
		return -EINVAL;
	}

	odevctl->info.size = devInfo->size;
	odevctl->info.writesz = devInfo->writesz;
	odevctl->info.metasz = devInfo->sparesz;
	odevctl->info.oobsz = devInfo->spareavail;
	odevctl->info.erasesz = devInfo->erasesz;

	return EOK;
}


static int flashsrv_devErase(id_t id, const flash_i_devctl_t *idevctl)
{
	size_t offs = idevctl->erase.address;
	size_t size = idevctl->erase.size;
	storage_t *strg = storage_get(id);

	TRACE("DevErase: off:%zu, size: %zu", offs, size);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (strg->dev->mtd->ops == NULL) ||
			(strg->dev->mtd->ops->erase == NULL) || ((offs + size) > strg->size)) {
		return -EINVAL;
	}

	if (size == 0) {
		size = strg->size - offs;
	}

	return strg->dev->mtd->ops->erase(strg, strg->start + offs, size);
}


static int flashsrv_devReadMeta(id_t id, flash_i_devctl_t *idevctl, void *data)
{
	storage_t *strg = storage_get(id);
	off_t offs = idevctl->read.address;
	size_t retlen, size = idevctl->read.size;
	int res;

	TRACE("META read off: %lld, size: %zu, ptr: %p", offs, size, data);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (strg->dev->mtd->ops == NULL) ||
			(strg->dev->mtd->ops->meta_read == NULL) ||
			((offs + (off_t)size) > (off_t)strg->size) || (data == NULL)) {
		return -EINVAL;
	}

	res = strg->dev->mtd->ops->meta_read(strg, strg->start + offs, data, size, &retlen);
	if (res >= 0) {
		res = (int)retlen;
	}

	return res;
}


static int flashsrv_devWriteMeta(id_t id, flash_i_devctl_t *idevctl, const void *data)
{
	int res;
	size_t retlen;
	off_t offs = idevctl->write.address;
	size_t size = idevctl->write.size;
	storage_t *strg = storage_get(id);

	TRACE("META write off: %lld, size: %zu, ptr: %p", offs, size, data);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (strg->dev->mtd->ops == NULL) ||
			(strg->dev->mtd->ops->meta_write == NULL) ||
			((offs + (off_t)size) > (off_t)strg->size) || (data == NULL)) {
		return -EINVAL;
	}

	res = strg->dev->mtd->ops->meta_write(strg, strg->start + offs, data, size, &retlen);
	if (res >= 0) {
		res = (int)retlen;
	}

	return res;
}


static ssize_t flashsrv_devWriteRaw(id_t id, flash_i_devctl_t *idevctl, const char *data)
{
	int res = EOK;
	size_t rawPagesz, rawEraseBlockSz, rawPartsz, tempsz = 0;
	size_t rawsz = idevctl->write.size;
	size_t rawoffs = idevctl->write.address;
	storage_t *strg = storage_get(id);

	TRACE("RAW write off: %zu, size: %zu, ptr: %p", rawoffs, rawsz, data);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (data == NULL)) {
		return -EINVAL;
	}

	rawEraseBlockSz = (strg->dev->mtd->erasesz / strg->dev->mtd->writesz) * (strg->dev->mtd->writesz + strg->dev->mtd->metaSize);
	rawPartsz = (strg->size / strg->dev->mtd->erasesz) * rawEraseBlockSz;

	if ((rawoffs + rawsz) >= rawPartsz) {
		return -EINVAL;
	}

	rawPagesz = strg->dev->mtd->writesz + strg->dev->mtd->metaSize;
	if (((rawsz % rawPagesz) != 0) || ((rawoffs % rawPagesz) != 0)) {
		return -EINVAL;
	}

	rawoffs += (strg->start / strg->dev->mtd->erasesz) * rawEraseBlockSz;

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < rawsz) {
		memcpy(strg->dev->ctx->databuf, data + tempsz, rawPagesz);

		res = flashdrv_writeraw(strg->dev->ctx->dma, (uint32_t)(rawoffs / rawPagesz), strg->dev->ctx->databuf, rawPagesz);
		if (res < 0) {
			LOG_ERROR("raw write error %d", res);
			break;
		}

		rawoffs += rawPagesz;
		tempsz += rawPagesz;
	}
	mutexUnlock(strg->dev->ctx->lock);

	return (tempsz > 0) ? (ssize_t)tempsz : (ssize_t)res;
}


static ssize_t flashsrv_devReadRaw(id_t id, flash_i_devctl_t *idevctl, char *data)
{
	int res = EOK;
	size_t rawPagesz, rawEraseBlockSz, rawPartsz, tempsz = 0;
	size_t rawsz = idevctl->read.size;
	size_t rawoffs = idevctl->read.address;
	storage_t *strg = storage_get(id);

	TRACE("RAW read off: %zu, size: %zu", rawoffs, rawsz);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (data == NULL)) {
		return -EINVAL;
	}

	rawEraseBlockSz = (strg->dev->mtd->erasesz / strg->dev->mtd->writesz) *
			(strg->dev->mtd->writesz + strg->dev->mtd->metaSize);
	rawPartsz = (strg->size / strg->dev->mtd->erasesz) * rawEraseBlockSz;

	if ((rawoffs + rawsz) >= rawPartsz) {
		return -EINVAL;
	}

	rawPagesz = strg->dev->mtd->writesz + strg->dev->mtd->metaSize;
	if (((rawsz % rawPagesz) != 0) || ((rawoffs % rawPagesz) != 0)) {
		return -EINVAL;
	}

	rawoffs += (strg->start / strg->dev->mtd->erasesz) * rawEraseBlockSz;

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < rawsz) {
		res = flashdrv_readraw(strg->dev->ctx->dma, (uint32_t)(rawoffs / rawPagesz),
				strg->dev->ctx->databuf, rawPagesz);
		if (res < 0) {
			LOG_ERROR("raw read error %d", res);
			break;
		}

		memcpy(data + tempsz, strg->dev->ctx->databuf, rawPagesz);
		rawoffs += rawPagesz;
		tempsz += rawPagesz;
	}
	mutexUnlock(strg->dev->ctx->lock);

	return (tempsz > 0) ? (ssize_t)tempsz : (ssize_t)res;
}


static int flashsrv_devIsbad(id_t id, const flash_i_devctl_t *idevctl)
{
	size_t addr = idevctl->badblock.address;
	storage_t *strg = storage_get(id);

	TRACE("DevIsbad: off:%zu", addr);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (strg->dev->mtd->ops == NULL) ||
			(strg->dev->mtd->ops->block_isBad == NULL) || (addr >= strg->size)) {
		return -EINVAL;
	}

	return strg->dev->mtd->ops->block_isBad(strg, strg->start + addr);
}


static int flashsrv_devMarkbad(id_t id, const flash_i_devctl_t *idevctl)
{
	size_t addr = idevctl->badblock.address;
	storage_t *strg = storage_get(id);

	TRACE("DevMarkbad: off:%zu", addr);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (strg->dev->mtd->ops == NULL) ||
			(strg->dev->mtd->ops->block_markBad == NULL) || (addr >= strg->size)) {
		return -EINVAL;
	}

	return strg->dev->mtd->ops->block_markBad(strg, strg->start + addr);
}


static int flashsrv_devMaxBitflips(id_t id, const flash_i_devctl_t *idevctl)
{
	size_t addr = idevctl->maxbitflips.address;
	storage_t *strg = storage_get(id);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (strg->dev->mtd->ops == NULL) ||
			(strg->dev->mtd->ops->block_maxBitflips == NULL) || (addr >= strg->size)) {
		return -EINVAL;
	}

	return strg->dev->mtd->ops->block_maxBitflips(strg, strg->start + addr);
}


static int flashsrv_devPtableRead(id_t id, ptable_t *ptable)
{
	storage_t *strg = storage_get(id);

	if ((strg == NULL) || (strg->parent != NULL)) {
		return -EINVAL;
	}

	return flashsrv_ptableRead(strg, ptable);
}


static int flashsrv_devPtableWrite(id_t id, const ptable_t *ptable)
{
	storage_t *strg = storage_get(id);

	if ((strg == NULL) || (strg->parent != NULL)) {
		return -EINVAL;
	}

	return flashsrv_ptableWrite(strg, ptable);
}


/* ============================== Message handling ============================== */

static ssize_t flashsrv_read(oid_t *oid, off_t offs, char *data, size_t size)
{
	int res;
	size_t retlen;
	storage_t *strg = storage_get(oid->id);

	TRACE("Read off: %lld, size: %zu", offs, size);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (strg->dev->mtd->ops == NULL) ||
			(strg->dev->mtd->ops->read == NULL) ||
			((offs + (off_t)size) > (off_t)strg->size) || (data == NULL)) {
		return -EINVAL;
	}

	res = strg->dev->mtd->ops->read(strg, strg->start + offs, data, size, &retlen);
	if ((res < 0) && (res != -EUCLEAN)) {
		return res;
	}

	return (ssize_t)retlen;
}


static ssize_t flashsrv_write(oid_t *oid, off_t offs, const char *data, size_t size)
{
	int res;
	size_t retlen;
	storage_t *strg = storage_get(oid->id);

	TRACE("Write off: %lld, size: %zu, ptr: %p", offs, size, data);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (strg->dev->mtd->ops == NULL) ||
			(strg->dev->mtd->ops->write == NULL) ||
			((offs + (off_t)size) > (off_t)strg->size) || (data == NULL)) {
		return -EINVAL;
	}

	res = strg->dev->mtd->ops->write(strg, strg->start + offs, data, size, &retlen);
	if (res >= 0) {
		res = (int)retlen;
	}

	return (ssize_t)res;
}


static int flashsrv_fileAttrGet(oid_t *oid, int type, long long *attr)
{
	storage_t *strg = storage_get(oid->id);

	if (strg == NULL) {
		return -EINVAL;
	}

	switch (type) {
		case atSize:
			*attr = (long long)strg->size;
			break;

		case atDev:
			*attr = (long long)strg->start;
			break;

		default:
			return -EINVAL;
	}

	return EOK;
}


static void flashsrv_devCtrl(msg_t *msg)
{
	flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg->i.raw;
	flash_o_devctl_t *odevctl = (flash_o_devctl_t *)msg->o.raw;

	switch (idevctl->type) {
		case flashsrv_devctl_info:
			msg->o.err = flashsrv_devInfo(msg->oid.id, odevctl);
			break;

		case flashsrv_devctl_erase:
			msg->o.err = flashsrv_devErase(msg->oid.id, idevctl);
			break;

		case flashsrv_devctl_writeraw:
			msg->o.err = (int)flashsrv_devWriteRaw(msg->oid.id, idevctl, msg->i.data);
			break;

		case flashsrv_devctl_writemeta:
			msg->o.err = flashsrv_devWriteMeta(msg->oid.id, idevctl, msg->i.data);
			break;

		case flashsrv_devctl_readraw:
			msg->o.err = (int)flashsrv_devReadRaw(msg->oid.id, idevctl, msg->o.data);
			break;

		case flashsrv_devctl_readmeta:
			msg->o.err = flashsrv_devReadMeta(msg->oid.id, idevctl, msg->o.data);
			break;

		case flashsrv_devctl_isbad:
			msg->o.err = flashsrv_devIsbad(msg->oid.id, idevctl);
			break;

		case flashsrv_devctl_markbad:
			msg->o.err = flashsrv_devMarkbad(msg->oid.id, idevctl);
			break;

		case flashsrv_devctl_maxbitflips:
			msg->o.err = flashsrv_devMaxBitflips(msg->oid.id, idevctl);
			break;

		case flashsrv_devctl_readptable:
			msg->o.err = flashsrv_devPtableRead(msg->oid.id, msg->o.data);
			break;

		case flashsrv_devctl_writeptable:
			msg->o.err = flashsrv_devPtableWrite(msg->oid.id, msg->i.data);
			break;

		default:
			msg->o.err = -EINVAL;
			break;
	}
}


static void flashsrv_msgHandler(void *arg, msg_t *msg)
{
	mount_i_msg_t *imnt;
	mount_o_msg_t *omnt;

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			TRACE("DEV mtOpen/Close");
			msg->o.err = EOK;
			break;

		case mtRead:
			TRACE("DEV read - id: %llu, size: %zu, off: %lld", msg->oid.id, msg->o.size, msg->i.io.offs);
			msg->o.err = (int)flashsrv_read(&msg->oid, msg->i.io.offs, msg->o.data, msg->o.size);
			break;

		case mtWrite:
			TRACE("DEV write - id: %llu, size: %zu, off: %lld", msg->oid.id, msg->i.size, msg->i.io.offs);
			msg->o.err = (int)flashsrv_write(&msg->oid, msg->i.io.offs, msg->i.data,
					msg->i.size ? msg->i.size : msg->i.io.len);
			break;

		case mtMount:
			imnt = (mount_i_msg_t *)msg->i.raw;
			omnt = (mount_o_msg_t *)msg->o.raw;
			TRACE("DEV mount - fs: %s", imnt->fstype);
			msg->o.err = storage_mountfs(storage_get(msg->oid.id), imnt->fstype,
					msg->i.data, imnt->mode, &imnt->mnt, &omnt->oid);
			break;

		case mtUmount:
			msg->o.err = storage_umountfs(storage_get(msg->oid.id));
			break;

		case mtMountPoint:
			omnt = (mount_o_msg_t *)msg->o.raw;
			msg->o.err = storage_mountpoint(storage_get(msg->oid.id), &omnt->oid);
			break;

		case mtGetAttr:
			TRACE("DEV mtGetAttr - id: %llu", msg->oid.id);
			msg->o.err = flashsrv_fileAttrGet(&msg->oid, msg->i.attr.type, &msg->o.attr.val);
			break;

		case mtSync:
			TRACE("DEV Sync");
			msg->o.err = EOK;
			break;

		case mtDevCtl:
			TRACE("DEV DevCtl");
			flashsrv_devCtrl(msg);
			break;

		default:
			TRACE("DEV invalid message type %d", msg->type);
			msg->o.err = -ENOSYS;
			break;
	}
}


/* ============================== Partition management ============================== */

/*
 * Add a partition to the die identified by rootPath.
 * start and size are in erase blocks.
 */
static int flashsrv_partAdd(blkcnt_t start, blkcnt_t size, const char *name, const char *rootPath)
{
	int err;
	oid_t oid, poid;
	char path[PATH_MAX_LEN];
	storage_t *strg, *part, *parent;
	unsigned int partID = 0;

	err = flash_oidResolve(rootPath, &poid);
	if (err < 0) {
		LOG_ERROR("cannot resolve %s", rootPath);
		return err;
	}

	parent = storage_get(poid.id);
	if (parent == NULL) {
		LOG_ERROR("failed to find parent %s", rootPath);
		return -EINVAL;
	}

	/* Find next free partition index  */
	part = parent->parts;
	if (part != NULL) {
		do {
			partID++;
			part = part->next;
		} while (part != parent->parts);
	}
	partID++;

	strg = calloc(1, sizeof(storage_t));
	if (strg == NULL) {
		return -ENOMEM;
	}

	strg->parent = parent;
	strg->start = start * parent->dev->mtd->erasesz;
	strg->size = size * parent->dev->mtd->erasesz;

	err = flashdev_init(strg, parent->dev->ctx->target);
	if (err < 0) {
		free(strg);
		LOG_ERROR("failed to initialize MTD interface, err: %d", err);
		return err;
	}

	err = storage_add(strg, &oid);
	if (err < 0) {
		(void)flashdev_done(strg);
		free(strg);
		LOG_ERROR("failed to create partition, err: %d", err);
		return err;
	}

	err = snprintf(path, sizeof(path), "%sp%u", rootPath, partID);
	if (err >= (int)sizeof(path)) {
		(void)flashdev_done(strg);
		free(strg);
		return -ENAMETOOLONG;
	}

	err = create_dev(&oid, path);
	if (err < 0) {
		(void)flashdev_done(strg);
		free(strg);
		LOG_ERROR("failed to create device file %s, err: %d", path, err);
		return err;
	}

	if (name != NULL) {
		err = flashsrv_devfsSymlink(name, path);
		if (err < 0) {
			LOG_ERROR("symlink %s -> %s failed: %s", path, name, strerror(err));
			/* not fatal */
		}
	}

	LOG("%-18s <%4llu, %4llu>: %s", path, start, start + size,
			(name != NULL) ? name : "(unnamed)");

	return EOK;
}


/* ============================== Initialisation ============================== */

static int flashsrv_mountRoot(int rootfs1, const char *fs)
{
	oid_t oid;
	int err;

	if ((fs == NULL) || (rootfs1 <= 0)) {
		LOG("no rootfs definition - not mounting '/'");
		return 0;
	}

	err = storage_mountfs(storage_get(rootfs1), fs, NULL, 0, NULL, &oid);
	if (err < 0) {
		LOG_ERROR("failed to mount filesystem %s: %d", fs, err);
		return err;
	}

	err = portRegister(oid.port, "/", &oid);
	if (err < 0) {
		LOG_ERROR("failed to register root: %d", err);
		return err;
	}

	LOG("mounted partition %d as '/' using %s", rootfs1, fs);

	err = flashsrv_devfsSymlink("root", PATH_ROOT_STRG "p1");
	if (err < 0) {
		LOG_ERROR("root symlink creation failed: %s", strerror(err));
		/* Not fatal if symlink fails */
	}

	return EOK;
}


/* Initialize the storage device for one NAND die and register /dev/mtdN */
static storage_t *flashsrv_nandInit(unsigned int target)
{
	int err;
	oid_t oid;
	storage_t *strg;
	char path[PATH_MAX_LEN];

	strg = calloc(1, sizeof(storage_t));
	if (strg == NULL) {
		LOG_ERROR("cannot allocate storage for die %u", target);
		return NULL;
	}

	err = flashdev_init(strg, target);
	if (err < 0) {
		LOG_ERROR("failed to init die %u: %d", target, err);
		free(strg);
		return NULL;
	}

	err = storage_add(strg, &oid);
	if (err < 0) {
		LOG_ERROR("storage_add failed for die %u: %d", target, err);
		(void)flashdev_done(strg);
		free(strg);
		return NULL;
	}

	err = snprintf(path, sizeof(path), PATH_ROOT_FMT, target);
	if (err >= (int)sizeof(path)) {
		(void)flashdev_done(strg);
		free(strg);
		return NULL;
	}

	err = create_dev(&oid, path);
	if (err < 0) {
		LOG_ERROR("create_dev %s failed: %d", path, err);
		(void)flashdev_done(strg);
		free(strg);
		return NULL;
	}

	LOG("%s: %s (%llu bytes)", path,
			(strg->dev->mtd->name != NULL) ? strg->dev->mtd->name : "?",
			(unsigned long long)strg->size);

	return strg;
}


static int flashsrv_ptablePartCompare(const void *p1, const void *p2)
{
	const ptable_part_t *a = p1, *b = p2;

	if (a->offset < b->offset)
		return -1;
	if (a->offset > b->offset)
		return 1;
	return 0;
}


static int flashsrv_parseOpts(int argc, char **argv, const char **fs, int *rootfs1)
{
	blkcnt_t partStart, partSize;
	char *p, *partName;
	int err, c;

	while ((c = getopt(argc, argv, "r:p:")) != -1) {
		switch (c) {
			case 'r': /* fs_name:rootfs_part_id */
				*fs = optarg;
				p = strchr(optarg, ':');
				if (p == NULL) {
					LOG_ERROR("missing rootfs partition id");
					return -EINVAL;
				}
				*p++ = '\0';

				errno = 0;
				*rootfs1 = (int)strtol(p, NULL, 10);
				break;

			case 'p': /* start:size[:name] */
				errno = 0;
				partStart = strtol(optarg, &p, 10);
				if (*p++ != ':') {
					LOG_ERROR("missing partition size");
					return -EINVAL;
				}

				partSize = strtol(p, &p, 10);
				if (errno == ERANGE) {
					LOG_ERROR("partition parameters out of range");
					return -ERANGE;
				}

				partName = (*p == ':') ? (p + 1) : NULL;

				/* Default: attach partition to die 0 */
				err = flashsrv_partAdd(partStart, partSize, partName, PATH_ROOT_STRG);
				if (err < 0) {
					LOG_ERROR("failed to add partition %lld:%lld", partStart, partSize);
					return err;
				}
				break;

			default:
				break;
		}
	}

	return EOK;
}


static void flashsrv_signalExit(int sig)
{
	(void)sig;
	exit(EXIT_SUCCESS);
}


int main(int argc, char **argv)
{
	int rootfs1 = -1, err;
	const char *fs = NULL;
	storage_t *strg[NAND_DIE_CNT];
	ptable_part_t *part;
	ptable_t *ptable;
	unsigned int die;
	char diePath[PATH_MAX_LEN];
	pid_t pid;
	int i;

	signal(SIGUSR1, flashsrv_signalExit);

	/* Daemonize */
	pid = fork();
	if (pid < 0) {
		LOG_ERROR("fork failed");
		exit(EXIT_FAILURE);
	}
	else if (pid > 0) {
		/* Parent waits to be killed by child after init */
		sleep(10);
		exit(EXIT_FAILURE);
	}

	signal(SIGUSR1, flashsrv_signalExit);

	if (setsid() < 0) {
		LOG_ERROR("setsid failed");
		exit(EXIT_FAILURE);
	}

	err = storage_init(flashsrv_msgHandler, 32);
	if (err < 0) {
		LOG_ERROR("storage_init failed: %d", err);
		return EXIT_FAILURE;
	}

	err = storage_registerfs("jffs2", libjffs2_mount, libjffs2_umount);
	if (err < 0) {
		LOG_ERROR("failed to register jffs2: %d", err);
		return EXIT_FAILURE;
	}

	/* Initialise hardware driver (once for all dies) */
	err = flashdrv_init();
	if (err < 0) {
		LOG_ERROR("flashdrv_init failed: %d", err);
		return EXIT_FAILURE;
	}

	/* Create one storage device per NAND die */
	for (die = 0u; die < (unsigned int)NAND_DIE_CNT; die++) {
		strg[die] = flashsrv_nandInit(die);
		if (strg[die] == NULL) {
			LOG_ERROR("failed to init NAND die %u", die);
			return EXIT_FAILURE;
		}
	}

	/* Command-line partitions / rootfs option */
	err = flashsrv_parseOpts(argc, argv, &fs, &rootfs1);
	if (err < 0) {
		return EXIT_FAILURE;
	}

	/*
	 * If no partitions were specified on the command line, try reading
	 * the partition table from each die.
	 */
	for (die = 0u; die < (unsigned int)NAND_DIE_CNT; die++) {
		if (strg[die]->parts != NULL) {
			continue; /* already has partitions from args */
		}

		ptable = malloc(strg[die]->dev->mtd->writesz);
		if (ptable == NULL) {
			LOG_ERROR("cannot allocate ptable buffer for die %u", die);
			return EXIT_FAILURE;
		}

		err = flashsrv_ptableRead(strg[die], ptable);
		if (err == EOK) {
			LOG("die %u: initialising partitions from partition table", die);

			qsort(ptable->parts, ptable->count,
					sizeof(ptable_part_t), flashsrv_ptablePartCompare);

			err = snprintf(diePath, sizeof(diePath), PATH_ROOT_FMT, die);
			if (err >= (int)sizeof(diePath)) {
				free(ptable);
				return EXIT_FAILURE;
			}

			for (i = 0; i < (int)ptable->count; i++) {
				part = &ptable->parts[i];
				err = flashsrv_partAdd(
						part->offset / strg[die]->dev->mtd->erasesz,
						part->size / strg[die]->dev->mtd->erasesz,
						(const char *)part->name,
						diePath);
				if (err < 0) {
					LOG_ERROR("failed to add partition %s on die %u",
							(const char *)part->name, die);
					free(ptable);
					return EXIT_FAILURE;
				}
			}
		}
		free(ptable);
	}

	err = flashsrv_mountRoot(rootfs1, fs);
	if (err < 0) {
		LOG_ERROR("failed to mount rootfs");
		return EXIT_FAILURE;
	}

	/* Signal parent that initialisation is complete */
	kill(getppid(), SIGUSR1);

	LOG("initialized");

	err = storage_run(31, 2 * _PAGE_SIZE);

	return (err < 0) ? EXIT_FAILURE : EXIT_SUCCESS;
}
