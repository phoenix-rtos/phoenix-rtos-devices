/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND flash server.
 *
 * Copyright 2018 - 2019, 2022 Phoenix Systems
 * Author: Jan Sikorski, Hubert Buczyński
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/file.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/threads.h>
#include <sys/reboot.h>
#include <posix/utils.h>

#include <libjffs2.h>
#include <storage/storage.h>

#include "imx6ull-flashsrv.h"
#include "imx6ull-flashdrv.h"
#include "imx6ull-flashdev.h"


/* clang-format off */
#define LOG(str_, ...) do { printf("imx6ull-flash: " str_ "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(str_, ...) LOG(__FILE__ ":%d error: " str_, __LINE__, ##__VA_ARGS__)
#define TRACE(str_, ...) do { if (0) LOG(__FILE__ ":%d TRACE: " str_, __LINE__, ##__VA_ARGS__); } while (0)
/* clang-format on */


/* TODO: If server will handle many flash memory devices, the path to parent storage should be passed as an argument to create a partition
         and this define should be removed */
/* Path for the whole NAND flash memory device */
#define PATH_ROOT_STRG "/dev/mtd0"


/* Auxiliary functions */

static int flash_oidResolve(const char *devPath, oid_t *oid)
{
	int res;
	oid_t dir;
	char temp[32];

	res = lookup("devfs", NULL, &dir);
	if (res >= 0) {
		if (strncmp("/dev/", devPath, 5) != 0) {
			return -EINVAL;
		}

		res = snprintf(temp, sizeof(temp), "devfs/%s", devPath + 5);
		if (res >= sizeof(temp)) {
			res = -ENAMETOOLONG;
			LOG_ERROR("failed to build file path, err: %d", res);
			return res;
		}
	}
	else {
		strncpy(temp, devPath, sizeof(temp));
		temp[sizeof(temp) - 1] = '\0';
	}

	return lookup(temp, NULL, oid);
}


/* create symlink manually as at this point we might not have '/' yet, so resolve_path would fail */
static int flashsrv_devfsSymlink(const char *name, const char *target)
{
	oid_t dir;
	msg_t msg = { 0 };
	int len1, len2;
	int ret;

	ret = lookup("devfs", NULL, &dir);
	if (ret < 0) {
		return ret;
	}

	msg.type = mtCreate;

	memcpy(&msg.i.create.dir, &dir, sizeof(oid_t));
	msg.i.create.type = otSymlink;
	/* POSIX: symlink file permissions are undefined, use sane default */
	msg.i.create.mode = S_IFLNK | ACCESSPERMS;

	len1 = strlen(name);
	len2 = strlen(target);

	msg.i.size = len1 + len2 + 2;
	msg.i.data = calloc(1, msg.i.size);
	if (msg.i.data == NULL) {
		return -ENOMEM;
	}

	memcpy(msg.i.data, name, len1);
	memcpy(msg.i.data + len1 + 1, target, len2);

	ret = msgSend(dir.port, &msg);
	free(msg.i.data);

	return ret != EOK ? -EIO : msg.o.create.err;
}


/* Device control functions */

static int flashsrv_devInfo(flash_o_devctl_t *odevctl)
{
	flashsrv_info_t info;
	const flashdrv_info_t *devInfo = flashdrv_info();

	if (devInfo == NULL) {
		return -EINVAL;
	}

	info.size = devInfo->size;
	info.writesz = devInfo->writesz;
	info.metasz = devInfo->metasz;
	info.erasesz = devInfo->erasesz;

	memcpy(&odevctl->info, &info, sizeof(flashsrv_info_t));

	return EOK;
}


static int flashsrv_devErase(const flash_i_devctl_t *idevctl)
{
	size_t offs = idevctl->erase.address;
	size_t size = idevctl->erase.size;
	storage_t *strg = storage_get(idevctl->badblock.oid.id);

	TRACE("DevErase: off:%zu, size: %zu", offs, size);

	if (strg == NULL || strg->dev == NULL || strg->dev->ctx == NULL || strg->dev->mtd == NULL || strg->dev->mtd->ops == NULL ||
			strg->dev->mtd->ops->block_markBad == NULL || strg->dev->mtd->ops->block_isBad == NULL || strg->dev->mtd->ops->erase == NULL ||
			(offs + size) >= strg->size) {
		return -EINVAL;
	}

	/* Erase from offset till the end of the storage partition */
	if (size == 0) {
		size = strg->size - offs;
	}

	return strg->dev->mtd->ops->erase(strg, strg->start + offs, size);
}


static int flashsrv_devWriteMeta(flash_i_devctl_t *idevctl, char *data)
{
	int res;
	size_t retlen;
	off_t offs = idevctl->write.address;
	size_t size = idevctl->write.size;
	storage_t *strg = storage_get(idevctl->write.oid.id);

	TRACE("META write off: %lld, size: %d, ptr: %p", offs, size, data);

	if (strg == NULL || strg->dev == NULL || strg->dev->ctx == NULL || strg->dev->mtd == NULL || strg->dev->mtd->ops == NULL ||
			strg->dev->mtd->ops->meta_write == NULL || (offs + size) >= strg->size || data == NULL) {
		return -EINVAL;
	}

	res = strg->dev->mtd->ops->meta_write(strg, strg->start + offs, data, size, &retlen);
	if (res >= 0) {
		res = retlen;
	}

	return res;
}


static ssize_t flashsrv_devWriteRaw(flash_i_devctl_t *idevctl, char *data)
{
	int res = EOK;
	size_t rawPagesz, rawEraseBlockSz, rawPartsz, tempsz = 0;

	size_t rawsz = idevctl->write.size;
	size_t rawoffs = idevctl->write.address;
	storage_t *strg = storage_get(idevctl->badblock.oid.id);

	TRACE("RAW write off: %d, size: %d, ptr: %p", rawoffs, rawsz, data);

	if (strg == NULL || strg->dev == NULL || strg->dev->ctx == NULL || strg->dev->mtd == NULL || data == NULL) {
		return -EINVAL;
	}

	rawEraseBlockSz = (strg->dev->mtd->erasesz / strg->dev->mtd->writesz) * (strg->dev->mtd->writesz + strg->dev->mtd->metaSize);
	rawPartsz = (strg->size / strg->dev->mtd->erasesz) * rawEraseBlockSz;

	if ((rawoffs + rawsz) >= rawPartsz) {
		return -EINVAL;
	}

	rawPagesz = strg->dev->mtd->writesz + strg->dev->mtd->metaSize;
	if ((rawsz % rawPagesz) != 0 || (rawoffs % rawPagesz) != 0) {
		return -EINVAL;
	}
	rawoffs += (strg->start / strg->dev->mtd->erasesz) * rawEraseBlockSz;

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < rawsz) {
		memcpy(strg->dev->ctx->databuf, data + tempsz, rawPagesz);

		res = flashdrv_writeraw(strg->dev->ctx->dma, rawoffs / rawPagesz, strg->dev->ctx->databuf, rawPagesz);
		if (res < 0) {
			LOG_ERROR("raw write error %d", res);
			mutexUnlock(strg->dev->ctx->lock);
			break;
		}

		rawoffs += rawPagesz;
		tempsz += rawPagesz;
	}
	mutexUnlock(strg->dev->ctx->lock);

	return tempsz > 0 ? tempsz : res;
}


static int flashsrv_devReadRaw(flash_i_devctl_t *idevctl, char *data)
{
	int res = EOK;
	size_t rawPagesz, rawEraseBlockSz, rawPartsz, tempsz = 0;
	storage_t *strg = storage_get(idevctl->badblock.oid.id);

	size_t rawsz = idevctl->readraw.size;
	size_t rawoffs = idevctl->readraw.address;

	if (strg == NULL || strg->dev == NULL || strg->dev->ctx == NULL || strg->dev->mtd == NULL || data == NULL) {
		return -EINVAL;
	}

	rawEraseBlockSz = (strg->dev->mtd->erasesz / strg->dev->mtd->writesz) * (strg->dev->mtd->writesz + strg->dev->mtd->metaSize);
	rawPartsz = (strg->size / strg->dev->mtd->erasesz) * rawEraseBlockSz;

	if ((rawoffs + rawsz) >= rawPartsz) {
		return -EINVAL;
	}

	rawPagesz = strg->dev->mtd->writesz + strg->dev->mtd->metaSize;
	if ((rawsz % rawPagesz) != 0 || (rawoffs % rawPagesz) != 0) {
		return -EINVAL;
	}
	rawoffs += (strg->start / strg->dev->mtd->erasesz) * rawEraseBlockSz;

	mutexLock(strg->dev->ctx->lock);
	while (tempsz < rawsz) {
		res = flashdrv_readraw(strg->dev->ctx->dma, rawoffs / rawPagesz, strg->dev->ctx->databuf, rawPagesz);
		if (res < 0) {
			LOG_ERROR("error in readraw(): %d", res);
			mutexUnlock(strg->dev->ctx->lock);
			break;
		}

		memcpy(data, strg->dev->ctx->databuf, rawPagesz);
		rawoffs += rawPagesz;
		tempsz += rawPagesz;
	}
	mutexUnlock(strg->dev->ctx->lock);

	return tempsz > 0 ? tempsz : res;
}


static int flashsrv_devIsbad(const flash_i_devctl_t *idevctl)
{
	size_t addr = idevctl->badblock.address;
	storage_t *strg = storage_get(idevctl->badblock.oid.id);

	TRACE("DevIsbad: off:%zu", addr);

	if (strg == NULL || strg->dev == NULL || strg->dev->ctx == NULL || strg->dev->mtd == NULL || strg->dev->mtd->ops == NULL ||
			strg->dev->mtd->ops->block_isBad == NULL || addr >= strg->size) {
		return -EINVAL;
	}

	return strg->dev->mtd->ops->block_isBad(strg, strg->start + addr);
}


static int flashsrv_devMarkbad(const flash_i_devctl_t *idevctl)
{
	size_t addr = idevctl->badblock.address;
	storage_t *strg = storage_get(idevctl->badblock.oid.id);

	TRACE("DevMarkbad: off:%zu", addr);

	if (strg == NULL || strg->dev == NULL || strg->dev->ctx == NULL || strg->dev->mtd == NULL || strg->dev->mtd->ops == NULL ||
			strg->dev->mtd->ops->block_markBad == NULL || addr >= strg->size) {
		return -EINVAL;
	}

	return strg->dev->mtd->ops->block_markBad(strg, strg->start + addr);
}


static int flashsrv_devMaxBitflips(const flash_i_devctl_t *idevctl)
{
	size_t addr = idevctl->maxbitflips.address;
	storage_t *strg = storage_get(idevctl->maxbitflips.oid.id);

	if (strg == NULL || strg->dev == NULL || strg->dev->ctx == NULL || strg->dev->mtd == NULL || strg->dev->mtd->ops == NULL ||
			strg->dev->mtd->ops->block_maxBitflips == NULL || addr >= strg->size) {
		return -EINVAL;
	}

	return strg->dev->mtd->ops->block_maxBitflips(strg, strg->start + addr);
}


/* Handling flash functions */

static ssize_t flashsrv_read(oid_t *oid, size_t offs, char *data, size_t size)
{
	int res;
	size_t retlen;
	storage_t *strg = storage_get(oid->id);

	TRACE("Read off: %d, size: %d.", offs, size);

	if (strg == NULL || strg->dev == NULL || strg->dev->ctx == NULL || strg->dev->mtd == NULL || strg->dev->mtd->ops == NULL ||
			strg->dev->mtd->ops->read == NULL || (offs + size) > strg->size || data == NULL) {
		return -EINVAL;
	}

	res = strg->dev->mtd->ops->read(strg, strg->start + offs, data, size, &retlen);
	/* -EUCLEAN isn't a fatal error (indicates dangerous page degradation but all bitflips were successfully corrected) */
	if ((res < 0) && (res != -EUCLEAN)) {
		return res;
	}

	return retlen;
}


static ssize_t flashsrv_write(oid_t *oid, size_t offs, char *data, size_t size)
{
	int res;
	size_t retlen;
	storage_t *strg = storage_get(oid->id);

	TRACE("Write off: %d, size: %d, ptr: %p", offs, size, data);

	if (strg == NULL || strg->dev == NULL || strg->dev->ctx == NULL || strg->dev->mtd == NULL || strg->dev->mtd->ops == NULL ||
			strg->dev->mtd->ops->write == NULL || (offs + size) > strg->size || data == NULL) {
		return -EINVAL;
	}

	res = strg->dev->mtd->ops->write(strg, strg->start + offs, data, size, &retlen);
	if (res >= 0) {
		res = retlen;
	}

	return res;
}


static int flashsrv_fileAttrGet(oid_t *oid, int type, long long *attr)
{
	storage_t *strg = storage_get(oid->id);

	if (strg == NULL) {
		return -EINVAL;
	}

	switch (type) {
		case atSize:
			*attr = (off_t)strg->size;
			break;

		case atDev:
			*attr = (off_t)strg->start;
			break;

		default:
			return -EINVAL;
	}

	return EOK;
}


static void flashsrv_syncAll(void)
{
	/* TODO: */
}


/* TODO: This API should be replaced by mtd-user API */
static void flashsrv_devCtrl(msg_t *msg)
{
	flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg->i.raw;
	flash_o_devctl_t *odevctl = (flash_o_devctl_t *)msg->o.raw;

	switch (idevctl->type) {
		case flashsrv_devctl_info:
			odevctl->err = flashsrv_devInfo(odevctl);
			break;

		case flashsrv_devctl_erase:
			odevctl->err = flashsrv_devErase(idevctl);
			break;

		case flashsrv_devctl_writeraw:
			odevctl->err = flashsrv_devWriteRaw(idevctl, msg->i.data);
			break;

		case flashsrv_devctl_writemeta:
			odevctl->err = flashsrv_devWriteMeta(idevctl, msg->i.data);
			break;

		case flashsrv_devctl_readraw:
			odevctl->err = flashsrv_devReadRaw(idevctl, msg->o.data);
			break;

		case flashsrv_devctl_isbad:
			odevctl->err = flashsrv_devIsbad(idevctl);
			break;

		case flashsrv_devctl_markbad:
			odevctl->err = flashsrv_devMarkbad(idevctl);
			break;

		case flashsrv_devctl_maxbitflips:
			odevctl->err = flashsrv_devMaxBitflips(idevctl);
			break;

		default:
			odevctl->err = -EINVAL;
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
			TRACE("DEV mtOpen");
			msg->o.io.err = EOK;
			break;

		case mtRead:
			TRACE("DEV read - id: %llu, size: %d, off: %llu ", msg->i.io.oid.id, msg->o.size, msg->i.io.offs);
			msg->o.io.err = flashsrv_read(&msg->i.io.oid, msg->i.io.offs, msg->o.data, msg->o.size);

			break;

		case mtWrite:
			TRACE("DEV write - id: %llu, size: %d, off: %llu", msg->i.io.oid.id, msg->i.size, msg->i.io.offs);
			msg->o.io.err = flashsrv_write(&msg->i.io.oid, msg->i.io.offs, msg->i.data, msg->i.size ? msg->i.size : msg->i.io.len);
			break;

		case mtMount:
			imnt = (mount_i_msg_t *)msg->i.raw;
			omnt = (mount_o_msg_t *)msg->o.raw;
			TRACE("DEV mount - fs: %s", imnt->fstype);
			omnt->err = storage_mountfs(storage_get(imnt->dev.id), imnt->fstype, msg->i.data, imnt->mode, &imnt->mnt, &omnt->oid);
			break;

		case mtUmount:
			msg->o.io.err = storage_umountfs(storage_get(((oid_t *)msg->i.data)->id));
			break;

		case mtMountPoint:
			omnt = (mount_o_msg_t *)msg->o.raw;
			omnt->err = storage_mountpoint(storage_get(((oid_t *)msg->i.data)->id), &omnt->oid);
			break;

		case mtGetAttr:
			TRACE("DEV mtgetAttr - id: %llu", msg->i.attr.oid.id);
			msg->o.attr.err = flashsrv_fileAttrGet(&msg->i.attr.oid, msg->i.attr.type, &msg->o.attr.val);
			break;

		case mtSync:
			TRACE("DEV Sync All");
			flashsrv_syncAll();
			break;

		case mtDevCtl:
			TRACE("DEV control");
			flashsrv_devCtrl(msg);
			break;

		default:
			TRACE("DEV MSG Handler error");
			msg->o.io.err = -EINVAL;
			break;
	}
}


/* Initialization functions */

static int flashsrv_bootImage(void)
{
	uint32_t reason;
	int err;

	err = reboot_reason(&reason);
	if (err < 0) {
		return err;
	}

	/* Secondary boot image */
	if (reason & (1 << 30)) {
		return 1;
	}

	/* First boot image */
	return 0;
}


static int flashsrv_mountRoot(int rootFirst, int rootSecond, const char *fs)
{
	int err, rootfsID;
	oid_t oid;
	char path[32];

	if ((fs == NULL) || (rootFirst <= 0)) {
		/* code path for psu/psd */
		LOG("missing/invalid rootfs definition, not mounting '/'");
		return 0;
	}

	rootfsID = rootFirst;
	if (rootSecond > 0) {
		err = flashsrv_bootImage();
		if (err < 0) {
			LOG_ERROR("failed to check boot image, mounting first rootfs");
		}
		else if (err == 1) {
			LOG("using secondary boot image");
			rootfsID = rootSecond;
		}
	}

	/* mount / symlink rootfs partition */
	err = storage_mountfs(storage_get(rootfsID), fs, NULL, 0, NULL, &oid);
	if (err < 0) {
		LOG_ERROR("failed to mount a filesystem - %s: %d", fs, err);
		return err;
	}

	err = portRegister(oid.port, "/", &oid);
	if (err < 0) {
		LOG_ERROR("failed to register root %d", err);
		return err;
	}

	err = sprintf(path, "%sp%d", PATH_ROOT_STRG, rootfsID);
	if (err >= sizeof(path)) {
		err = -ENAMETOOLONG;
		LOG_ERROR("failed to build file path, err: %d", err);
		return err;
	}

	LOG("mounting %s as a rootfs (%s)", path, fs);
	err = flashsrv_devfsSymlink("root", path);
	/* Symlink error is not critical, does not return with error */
	if (err < 0) {
		LOG_ERROR("root symlink creation failed: %s", strerror(err));
	}

	return EOK;
}


/* Start and size of the partition are given in erase blocks (64 * 4096 (FLASH_PAGE_SIZE)) */
static int flashsrv_partAdd(blkcnt_t start, blkcnt_t size, const char *name)
{
	int err;
	oid_t oid, poid;
	char path[32];
	storage_t *strg, *part, *parent;
	unsigned int partID = 0;

	err = flash_oidResolve(PATH_ROOT_STRG, &poid);
	if (err < 0) {
		LOG_ERROR("cannot resolve %s", PATH_ROOT_STRG);
		return err;
	}

	parent = storage_get(poid.id);
	if (parent == NULL) {
		err = -EINVAL;
		LOG_ERROR("failed to find a parent %s, err: %d", PATH_ROOT_STRG, err);
		return err;
	}

	/* Find id for a new partition */
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
		err = -ENOMEM;
		LOG_ERROR("failed to allocate a device, err: %d", err);
		return err;
	}

	/* Start and size in libstorage are defined in bytes */
	strg->parent = parent;
	strg->start = start * parent->dev->mtd->erasesz;
	strg->size = size * parent->dev->mtd->erasesz;

	err = flashdev_init(strg);
	if (err < 0) {
		free(strg);
		LOG_ERROR("failed to initialize MTD interface, err: %d", err);
		return err;
	}

	err = storage_add(strg, &oid);
	if (err < 0) {
		flashdev_done(strg);
		free(strg);
		LOG_ERROR("failed to create a partition, err: %d", err);
		return err;
	}

	err = snprintf(path, sizeof(path), "%sp%u", PATH_ROOT_STRG, partID);
	if (err >= sizeof(path)) {
		flashdev_done(strg);
		free(strg);
		err = -ENAMETOOLONG;
		LOG_ERROR("failed to build file path, err: %d", err);
		return err;
	}

	err = create_dev(&oid, path);
	if (err < 0) {
		flashdev_done(strg);
		free(strg);
		LOG_ERROR("failed to create a device file, err: %d", err);
		return err;
	}

	if (name != NULL) {
		err = flashsrv_devfsSymlink(name, path);
		/* Symlink error is not critical, does not return with error */
		if (err < 0) {
			LOG_ERROR("symlink creation %s -> %s, failed: %s", path, name, strerror(err));
		}
	}

	LOG("%-13s <%4llu, %4llu>: name: %s", path, start, start + size, name != NULL ? name : "(nil)");

	return EOK;
}


static int flashsrv_parseOpts(int argc, char **argv)
{
	int err;
	blkcnt_t partStart, partSize; /* start and size of the partition in erase blocks */
	char *p, *partName, *fs = NULL;
	int c, rootfsFirst = -1, rootfsSecond = -1, magic = PHOENIX_REBOOT_MAGIC;

	while ((c = getopt(argc, argv, "r:p:")) != -1) {
		switch (c) {
			case 'r': /* fs_name:rootfs_part_id[:secondary_rootfs_part_id] */

				fs = optarg;
				p = strchr(optarg, ':');
				if (p == NULL) {
					LOG_ERROR("missing rootfs filesystem name");
					return -EINVAL;
				}

				*p++ = '\0';

				errno = 0;
				rootfsFirst = strtol(p, &p, 10);
				if (*p++ == ':') {
					rootfsSecond = strtol(p, &p, 10);
				}
				break;

			case 'p': /* start:size[:name] */
				/* TODO: switch to positional arguments to highlight partition order matter */

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
				err = flashsrv_partAdd(partStart, partSize, partName);
				if (err < 0) {
					LOG_ERROR("failed to add partition %lld:%lld", partStart, partSize);
				}
				break;

			default:
				break;
		}
	}

	/* TODO: add partition table support */
	err = flashsrv_mountRoot(rootfsFirst, rootfsSecond, fs);
	if (err < 0) {
		LOG_ERROR("failed to mount rootfs, rebooting to secondary image");

		c = flashsrv_bootImage();
		if (c != 1) {
			magic = ~magic;
		}
		reboot(magic);

		return err;
	}

	return EOK;
}


static int flashsrv_nandInit(void)
{
	int err;
	oid_t oid;
	storage_t *strg;

	/* Initialize nand flash driver */
	flashdrv_init();

	strg = calloc(1, sizeof(storage_t));
	if (strg == NULL) {
		LOG_ERROR("cannot allocate memory");
		return -ENOMEM;
	}

	err = flashdev_init(strg);
	if (err < 0) {
		LOG_ERROR("failed to initialize libstorage interface, err: %d", err);
		return err;
	}

	err = storage_add(strg, &oid);
	if (err < 0) {
		LOG_ERROR("failed to add new storage, err: %d", err);
		flashdev_done(strg);
		free(strg);
		return err;
	}

	err = create_dev(&oid, PATH_ROOT_STRG);
	if (err < 0) {
		LOG_ERROR("failed to create a device file, err: %d", err);
		flashdev_done(strg);
		free(strg);
		return err;
	}

	LOG("%s", strg->dev->mtd->name);

	return EOK;
}


static void flashsrv_signalExit(int sig)
{
	exit(EXIT_SUCCESS);
}


int main(int argc, char **argv)
{
	int err;
	pid_t pid;

	/* Set parent exit handler */
	signal(SIGUSR1, flashsrv_signalExit);

	/* Daemonize server */
	pid = fork();
	if (pid < 0) {
		LOG_ERROR("failed to daemonize server");
		exit(EXIT_FAILURE);
	}
	/* Parent waits to be killed by the child after finished server initialization */
	else if (pid > 0) {
		sleep(10);
		exit(EXIT_FAILURE);
	}

	/* Set child exit handler */
	signal(SIGUSR1, flashsrv_signalExit);

	if (setsid() < 0) {
		LOG_ERROR("failed to create new session");
		exit(EXIT_FAILURE);
	}

	/* Initialize storage library with the message handler for the NAND flash memory */
	err = storage_init(flashsrv_msgHandler, 32);
	if (err < 0) {
		LOG_ERROR("failed to initialize storage library, err: %d", err);
		return EXIT_FAILURE;
	}

	/* Register file system related to NAND flash memory */
	err = storage_registerfs("jffs2", libjffs2_mount, libjffs2_umount);
	if (err < 0) {
		LOG_ERROR("failed to register jffs2 filesystem, err: %d", err);
		return EXIT_FAILURE;
	}

	/* Flash driver and mtd interface initialization */
	err = flashsrv_nandInit();
	if (err < 0) {
		return EXIT_FAILURE;
	}

	/* Based on args, create new partitions and mount rootfs */
	if (flashsrv_parseOpts(argc, argv) < 0) {
		return EXIT_FAILURE;
	}

	/* Finished server initialization - kill parent */
	kill(getppid(), SIGUSR1);

	LOG("initialized");
	err = storage_run(4, 2 * _PAGE_SIZE);

	return err < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}
