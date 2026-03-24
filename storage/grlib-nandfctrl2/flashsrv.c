/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash server.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <endian.h>
#include <errno.h>
#include <inttypes.h>
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
#define LOG(str_, ...)       do { printf("grlib-nandfctrl2: " str_ "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(str_, ...) LOG(__FILE__ ":%d error: " str_, __LINE__, ##__VA_ARGS__)
#define TRACE(str_, ...)     do { if (0) LOG(__FILE__ ":%d TRACE: " str_, __LINE__, ##__VA_ARGS__); } while (0)
/* clang-format on */

#define PATH_ROOT_FMT "/dev/mtd%u"

#define PATH_MAX_LEN 32

/* Number of partition table copies saved on flash */
#ifndef PTABLE_NCOPIES
#define PTABLE_NCOPIES 1
#endif


typedef struct {
	struct {
		unsigned int die;
		char *partname;
		char *fs;
	} root;
} flashsrv_opts_t;


/* ============================== Auxiliary ============================== */

static int flash_oidResolve(const char *devPath, oid_t *oid)
{
	oid_t dir;
	char temp[PATH_MAX_LEN];

	int res = lookup("devfs", NULL, &dir);
	if (res >= 0) {
		if (strncmp("/dev/", devPath, 5) != 0) {
			return -EINVAL;
		}

		res = snprintf(temp, sizeof(temp), "devfs/%s", devPath + 5);
		if (res >= sizeof(temp)) {
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
	int ret = lookup("devfs", NULL, &dir);
	if (ret < 0) {
		return ret;
	}

	msg_t msg = { 0 };

	msg.type = mtCreate;
	msg.oid = dir;
	msg.i.create.type = otSymlink;
	msg.i.create.mode = S_IFLNK | ACCESSPERMS;

	size_t len1 = strlen(name);
	size_t len2 = strlen(target);

	msg.i.size = len1 + len2 + 2;
	void *idata = calloc(1, msg.i.size);
	if (idata == NULL) {
		return -ENOMEM;
	}

	memcpy(idata, name, len1);
	memcpy((char *)idata + len1 + 1, target, len2);
	msg.i.data = idata;

	ret = msgSend(dir.port, &msg);
	free(idata);

	return (ret != 0) ? -EIO : msg.o.err;
}


/* ============================== Partition table ============================== */

/* ptable buffer must have at least mtd->writesz bytes */
static int flashsrv_ptableRead(storage_t *strg, ptable_t *ptable)
{
	storage_mtd_t *mtd = strg->dev->mtd;

	for (int i = 0; i < PTABLE_NCOPIES; i++) {
		off_t offs = (off_t)strg->size - (off_t)(i + 1) * mtd->erasesz;
		if (mtd->ops->block_isBad(strg, offs) != 0) {
			continue;
		}

		size_t retlen;
		int err = mtd->ops->read(strg, offs, ptable, mtd->writesz, &retlen);
		if (((err < 0) && (err != -EUCLEAN)) || (retlen != mtd->writesz)) {
			continue;
		}

		uint32_t count = le32toh(ptable->count);
		if (ptable_size(count) > mtd->writesz) {
			continue;
		}

		if (ptable_deserialize(ptable, strg->size, mtd->erasesz) < 0) {
			continue;
		}

		return 0;
	}

	return -ENOENT;
}


static int flashsrv_ptableWrite(storage_t *strg, const ptable_t *ptable)
{
	storage_mtd_t *mtd = strg->dev->mtd;

	size_t size = ptable_size(ptable->count);
	if (size > mtd->writesz) {
		return -EINVAL;
	}

	ptable_t *ptab = malloc(mtd->writesz);
	if (ptab == NULL) {
		return -ENOMEM;
	}
	memcpy(ptab, ptable, size - sizeof(ptable_magic));

	if (ptable_serialize(ptab, strg->size, mtd->erasesz) < 0) {
		free(ptab);
		return -EINVAL;
	}

	int n = 0;
	for (int i = 0; i < PTABLE_NCOPIES; i++) {
		off_t offs = (off_t)strg->size - (off_t)(i + 1) * mtd->erasesz;
		if (mtd->ops->block_isBad(strg, offs) != 0) {
			continue;
		}

		if (mtd->ops->erase(strg, offs, mtd->erasesz) != 1) {
			continue;
		}

		size_t retlen;
		int err = mtd->ops->write(strg, offs, ptab, mtd->writesz, &retlen);
		if ((err < 0) || (retlen != mtd->writesz)) {
			continue;
		}
		n++;
	}
	free(ptab);

	return (n > 0) ? 0 : -EIO;
}


/* ============================== Device control ============================== */

static int flashsrv_devInfo(id_t id, flash_o_devctl_t *odevctl)
{
	storage_t *strg = storage_get(id);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL)) {
		return -EINVAL;
	}

	const flashdrv_info_t *devInfo = &strg->dev->ctx->die->info;

	odevctl->info.size = devInfo->size;
	odevctl->info.writesz = devInfo->writesz;
	odevctl->info.metasz = devInfo->sparesz;
	odevctl->info.oobsz = devInfo->spareavail;
	odevctl->info.erasesz = devInfo->erasesz;

	return 0;
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
	size_t size = idevctl->read.size;

	TRACE("META read off: %jd, size: %zu, ptr: %p", (intmax_t)offs, size, data);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (strg->dev->mtd->ops == NULL) ||
			(strg->dev->mtd->ops->meta_read == NULL) ||
			((offs + (off_t)size) > (off_t)strg->size) || (data == NULL)) {
		return -EINVAL;
	}

	size_t retlen;
	int res = strg->dev->mtd->ops->meta_read(strg, strg->start + offs, data, size, &retlen);
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

	TRACE("META write off: %jd, size: %zu, ptr: %p", (intmax_t)offs, size, data);

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


static int flashsrv_devWriteRaw(id_t id, flash_i_devctl_t *idevctl, const char *data)
{
	size_t len = idevctl->write.size;
	size_t offs = idevctl->write.address;
	storage_t *strg = storage_get(id);

	TRACE("RAW write off: %zu, len: %zu, ptr: %p", offs, len, data);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) || (strg->dev->mtd == NULL) || (data == NULL)) {
		return -EINVAL;
	}

	const size_t rawEraseBlockSz = strg->dev->ctx->die->info.pagesPerBlock * (strg->dev->mtd->writesz + strg->dev->mtd->metaSize);
	const size_t rawPartsz = (strg->size / strg->dev->mtd->erasesz) * rawEraseBlockSz;

	if ((offs + len) > rawPartsz) {
		return -EINVAL;
	}

	offs += (strg->start / strg->dev->mtd->erasesz) * rawEraseBlockSz;

	size_t retlen;
	int err = flashdev_writeRaw(strg, offs, data, len, &retlen);

	return (err < 0) ? err : retlen;
}


static int flashsrv_devReadRaw(id_t id, flash_i_devctl_t *idevctl, char *data)
{
	size_t len = idevctl->read.size;
	size_t offs = idevctl->read.address;
	storage_t *strg = storage_get(id);

	TRACE("RAW read off: %zu, len: %zu", offs, len);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) || (strg->dev->mtd == NULL) || (data == NULL)) {
		return -EINVAL;
	}

	const size_t rawEraseBlockSz = strg->dev->ctx->die->info.pagesPerBlock * (strg->dev->mtd->writesz + strg->dev->mtd->metaSize);
	const size_t rawPartsz = (strg->size / strg->dev->mtd->erasesz) * rawEraseBlockSz;

	if ((offs + len) > rawPartsz) {
		return -EINVAL;
	}

	offs += (strg->start / strg->dev->mtd->erasesz) * rawEraseBlockSz;

	size_t retlen;
	int err = flashdev_readRaw(strg, offs, data, len, &retlen);

	return (err < 0) ? err : retlen;
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
	size_t retlen;
	storage_t *strg = storage_get(oid->id);

	TRACE("Read off: %jd, size: %zu", (intmax_t)offs, size);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (strg->dev->mtd->ops == NULL) ||
			(strg->dev->mtd->ops->read == NULL) ||
			((offs + (off_t)size) > (off_t)strg->size) || (data == NULL)) {
		return -EINVAL;
	}

	int res = strg->dev->mtd->ops->read(strg, strg->start + offs, data, size, &retlen);
	if ((res < 0) && (res != -EUCLEAN)) {
		return res;
	}

	return (ssize_t)retlen;
}


static ssize_t flashsrv_write(oid_t *oid, off_t offs, const char *data, size_t size)
{
	size_t retlen;
	storage_t *strg = storage_get(oid->id);

	TRACE("Write off: %jd, size: %zu, ptr: %p", (intmax_t)offs, size, data);

	if ((strg == NULL) || (strg->dev == NULL) || (strg->dev->ctx == NULL) ||
			(strg->dev->mtd == NULL) || (strg->dev->mtd->ops == NULL) ||
			(strg->dev->mtd->ops->write == NULL) ||
			((offs + (off_t)size) > (off_t)strg->size) || (data == NULL)) {
		return -EINVAL;
	}

	int res = strg->dev->mtd->ops->write(strg, strg->start + offs, data, size, &retlen);
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

	return 0;
}


static void flashsrv_devctl(msg_t *msg)
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
			msg->o.err = flashsrv_devWriteRaw(msg->oid.id, idevctl, msg->i.data);
			break;

		case flashsrv_devctl_writemeta:
			msg->o.err = flashsrv_devWriteMeta(msg->oid.id, idevctl, msg->i.data);
			break;

		case flashsrv_devctl_readraw:
			msg->o.err = flashsrv_devReadRaw(msg->oid.id, idevctl, msg->o.data);
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
			msg->o.err = 0;
			break;

		case mtRead:
			TRACE("DEV read - id: %ju, size: %zu, off: %jd", (uintmax_t)msg->oid.id, msg->o.size, (intmax_t)msg->i.io.offs);
			msg->o.err = (int)flashsrv_read(&msg->oid, msg->i.io.offs, msg->o.data, msg->o.size);
			break;

		case mtWrite:
			TRACE("DEV write - id: %ju, size: %zu, off: %jd", (uintmax_t)msg->oid.id, msg->i.size, (intmax_t)msg->i.io.offs);
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
			TRACE("DEV mtGetAttr - id: %ju", (uintmax_t)msg->oid.id);
			msg->o.err = flashsrv_fileAttrGet(&msg->oid, msg->i.attr.type, &msg->o.attr.val);
			break;

		case mtSync:
			TRACE("DEV Sync");
			msg->o.err = 0;
			break;

		case mtDevCtl:
			TRACE("DEV DevCtl");
			flashsrv_devctl(msg);
			break;

		default:
			TRACE("DEV invalid message type %d", msg->type);
			msg->o.err = -ENOSYS;
			break;
	}
}


/* ============================== Partition management ============================== */

/*
 * Add a partition to the die identified by die number.
 * start and size are in erase blocks.
 */
static int flashsrv_partAdd(unsigned int die, uint32_t startBlk, uint32_t blkCnt, const char *partName)
{
	char rootPath[PATH_MAX_LEN];
	int err = snprintf(rootPath, sizeof(rootPath), PATH_ROOT_FMT, die);
	if (err >= (int)sizeof(rootPath)) {
		LOG_ERROR("path too long for die %u", die);
		return -ENAMETOOLONG;
	}

	oid_t poid;
	err = flash_oidResolve(rootPath, &poid);
	if (err < 0) {
		LOG_ERROR("cannot resolve %s", rootPath);
		return err;
	}

	storage_t *parent = storage_get(poid.id);
	if (parent == NULL) {
		LOG_ERROR("failed to find parent %s", rootPath);
		return -EINVAL;
	}

	/* Find next free partition index  */
	storage_t *part = parent->parts;
	if (part != NULL) {
		do {
			part = part->next;
		} while (part != parent->parts);
	}

	part = calloc(1, sizeof(storage_t));
	if (part == NULL) {
		return -ENOMEM;
	}

	part->parent = parent;
	part->start = startBlk * parent->dev->mtd->erasesz;
	part->size = blkCnt * parent->dev->mtd->erasesz;

	err = flashdev_init(part, parent->dev->ctx->die->target);
	if (err < 0) {
		free(part);
		LOG_ERROR("failed to initialize MTD interface, err: %d", err);
		return err;
	}

	oid_t oid;
	err = storage_add(part, &oid);
	if (err < 0) {
		(void)flashdev_done(part);
		free(part);
		LOG_ERROR("failed to create partition, err: %d", err);
		return err;
	}

	char path[PATH_MAX_LEN];
	err = snprintf(path, sizeof(path), "mtd%u.%s", die, partName);
	if (err >= sizeof(path)) {
		(void)flashdev_done(part);
		free(part);
		return -ENAMETOOLONG;
	}

	err = create_dev(&oid, path);
	if (err < 0) {
		(void)flashdev_done(part);
		free(part);
		LOG_ERROR("failed to create device file %s, err: %d", path, err);
		return err;
	}

	if (partName != NULL) {
		err = flashsrv_devfsSymlink(partName, path);
		if (err < 0) {
			LOG_ERROR("symlink %s -> %s failed: %s", path, partName, strerror(err));
			/* not fatal */
		}
	}

	LOG("%-18s <%" PRIu32 ", %" PRIu32 ">: %s", path, startBlk, startBlk + blkCnt, (partName != NULL) ? partName : "(unnamed)");

	return 0;
}


/* ============================== Initialization ============================== */


static int flashsrv_mountRoot(unsigned int die, const char *name, const char *fstype)
{
	char path[38];
	if (snprintf(path, sizeof(path), "devfs/mtd%u.%s", die, name) >= sizeof(path)) {
		return -ENAMETOOLONG;
	}

	oid_t oid;
	int res = lookup(path, NULL, &oid);
	if (res < 0) {
		return res;
	}

	res = storage_mountfs(storage_get(oid.id), fstype, NULL, 0, NULL, &oid);
	if (res < 0) {
		LOG_ERROR("failed to mount filesystem %s: %d", fstype, res);
		return res;
	}

	res = portRegister(oid.port, "/", &oid);
	if (res < 0) {
		LOG_ERROR("failed to register root: %d", res);
		return res;
	}

	LOG("mounted partition %s as '/' using %s", name, fstype);

	return 0;
}


/* Initialize the storage device for one NAND die and register /dev/mtdN */
static storage_t *flashsrv_nandInit(unsigned int die)
{
	int err;
	oid_t oid;
	storage_t *strg;
	char path[PATH_MAX_LEN];

	strg = calloc(1, sizeof(storage_t));
	if (strg == NULL) {
		LOG_ERROR("cannot allocate storage for die %u", die);
		return NULL;
	}

	err = flashdev_init(strg, die);
	if (err < 0) {
		LOG_ERROR("failed to init die %u: %d", die, err);
		free(strg);
		return NULL;
	}

	err = storage_add(strg, &oid);
	if (err < 0) {
		LOG_ERROR("storage_add failed for die %u: %d", die, err);
		(void)flashdev_done(strg);
		free(strg);
		return NULL;
	}

	err = snprintf(path, sizeof(path), PATH_ROOT_FMT, die);
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

	LOG("%s: %s (%llu bytes)", path, (strg->dev->mtd->name != NULL) ? strg->dev->mtd->name : "?", (unsigned long long)strg->size);

	return strg;
}


static int flashsrv_ptablePartCompare(const void *p1, const void *p2)
{
	const ptable_part_t *a = p1, *b = p2;

	if (a->offset < b->offset) {
		return -1;
	}
	if (a->offset > b->offset) {
		return 1;
	}
	return 0;
}


static int flashsrv_handleOpts(int argc, char *argv[], flashsrv_opts_t *opts)
{
	uint32_t partStart, partSize;

	for (;;) {
		int c = getopt(argc, argv, "r:p:");
		if (c == -1) {
			break;
		}
		switch (c) {
			case 'r': { /* <die>:<rootfs name>:<fs> */
				errno = 0;
				char *p;
				opts->root.die = strtoul(optarg, &p, 0);
				if (errno != 0 || *p++ != ':') {
					LOG_ERROR("invalid die argument");
					return -EINVAL;
				}

				opts->root.partname = p;
				opts->root.fs = strchr(p, ':');
				if (opts->root.fs == NULL) {
					LOG_ERROR("missing rootfs partition id");
					return -EINVAL;
				}

				*opts->root.fs++ = '\0';
				break;
			}

			case 'p': { /* <die>:<start>:<size>[:<name>] */
				errno = 0;
				char *p;
				unsigned int die = strtoul(optarg, &p, 0);
				if (errno != 0 || *p++ != ':') {
					LOG_ERROR("invalid arguments");
					return -EINVAL;
				}

				partStart = strtol(p, &p, 0);
				if (errno != 0 || *p++ != ':') {
					LOG_ERROR("invalid arguments");
					return -EINVAL;
				}

				partSize = strtol(p, &p, 0);
				if (errno != 0) {
					LOG_ERROR("invalid arguments");
					return -ERANGE;
				}

				char *partName = (*p == ':') ? (p + 1) : NULL;

				int err = flashsrv_partAdd(die, partStart, partSize, partName);
				if (err < 0) {
					LOG_ERROR("failed to add partition %" PRIu32 ":%" PRIu32, partStart, partSize);
					return err;
				}
				break;
			}

			default:
				break;
		}
	}

	return 0;
}


static void flashsrv_signalExit(int sig)
{
	(void)sig;
	exit(EXIT_SUCCESS);
}


int main(int argc, char **argv)
{
	signal(SIGUSR1, flashsrv_signalExit);

	/* Daemonize */
	pid_t pid = fork();
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

	int err = storage_init(flashsrv_msgHandler, 32);
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

	storage_t *strg[NAND_DIE_CNT];

	/* Create one storage device per NAND die */
	for (unsigned int die = 0U; die < NAND_DIE_CNT; die++) {
		strg[die] = flashsrv_nandInit(die);
		if (strg[die] == NULL) {
			LOG_ERROR("failed to init NAND die %u", die);
			return EXIT_FAILURE;
		}
	}

	flashsrv_opts_t opts = { 0 };

	err = flashsrv_handleOpts(argc, argv, &opts);
	if (err < 0) {
		return EXIT_FAILURE;
	}

	/*
	 * If no partitions were specified on the command line, try reading
	 * the partition table from each die.
	 */
	for (unsigned int die = 0U; die < NAND_DIE_CNT; die++) {
		if (strg[die]->parts != NULL) {
			continue; /* already has partitions from args */
		}

		ptable_t *ptable = malloc(strg[die]->dev->mtd->writesz);
		if (ptable == NULL) {
			LOG_ERROR("cannot allocate ptable buffer for die %u", die);
			return EXIT_FAILURE;
		}

		err = flashsrv_ptableRead(strg[die], ptable);
		if (err == 0) {
			LOG("die %u: initializing partitions from partition table", die);

			qsort(ptable->parts, ptable->count, sizeof(ptable_part_t), flashsrv_ptablePartCompare);

			for (uint32_t i = 0; i < ptable->count; i++) {
				ptable_part_t *part = &ptable->parts[i];
				err = flashsrv_partAdd(die, part->offset / strg[die]->dev->mtd->erasesz, part->size / strg[die]->dev->mtd->erasesz, (const char *)part->name);
				if (err < 0) {
					LOG_ERROR("failed to add partition %s on die %u", (const char *)part->name, die);
					free(ptable);
					return EXIT_FAILURE;
				}
			}
		}
		free(ptable);
	}

	if ((opts.root.partname != NULL) && (opts.root.fs != NULL)) {
		err = flashsrv_mountRoot(opts.root.die, opts.root.partname, opts.root.fs);
		if (err < 0) {
			LOG_ERROR("failed to mount rootfs");
			return EXIT_FAILURE;
		}
	}

	/* Signal parent that initialisation is complete */
	kill(getppid(), SIGUSR1);

	LOG("initialized");

	err = storage_run(31, 2 * _PAGE_SIZE);

	return (err < 0) ? EXIT_FAILURE : EXIT_SUCCESS;
}
