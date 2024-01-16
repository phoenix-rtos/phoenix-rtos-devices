/*
 * Phoenix-RTOS
 *
 * GR712RC Flash server
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <endian.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/msg.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <libjffs2.h>
#include <ptable.h>
#include <storage/storage.h>

#include "flashdrv.h"

#define STRG_PATH "mtd0"


/* Flash server operations */


static ssize_t flashsrv_read(storage_t *strg, offs_t offs, void *buf, size_t size)
{
	if ((strg == NULL) || (strg->dev == NULL) || ((offs + size) > strg->size)) {
		return -EINVAL;
	}

	if (size == 0) {
		return 0;
	}

	storage_mtd_t *mtd = strg->dev->mtd;
	if ((mtd != NULL) && (mtd->ops != NULL) && (mtd->ops->read != NULL)) {
		size_t retlen;
		int res = mtd->ops->read(strg, offs, buf, size, &retlen);
		if (res < 0) {
			return res;
		}
		return retlen;
	}

	return -EINVAL;
}


static ssize_t flashsrv_write(storage_t *strg, offs_t offs, const void *buf, size_t size)
{
	if ((strg == NULL) || (strg->dev == NULL) || ((offs + size) > strg->size)) {
		return -EINVAL;
	}

	if (size == 0) {
		return 0;
	}

	storage_mtd_t *mtd = strg->dev->mtd;
	if ((mtd != NULL) && (mtd->ops != NULL) && (mtd->ops->write != NULL)) {
		size_t retlen;
		int res = mtd->ops->write(strg, offs, buf, size, &retlen);
		if (res < 0) {
			return res;
		}
		return retlen;
	}

	return -EINVAL;
}


static int flashsrv_sync(storage_t *strg)
{
	if ((strg == NULL) || (strg->dev == NULL)) {
		return -EINVAL;
	}

	storage_mtd_t *mtd = strg->dev->mtd;
	if ((mtd != NULL) && (mtd->ops != NULL) && (mtd->ops->sync != NULL)) {
		mtd->ops->sync(strg);
		return EOK;
	}

	return -EINVAL;
}


static int flashsrv_getAttr(storage_t *strg, int type, long long *attr)
{
	if ((strg == NULL) || (attr == NULL)) {
		return -EINVAL;
	}

	switch (type) {
		case atSize:
			*attr = strg->size;
			break;

		default:
			return -EINVAL;
	}

	return EOK;
}


static void flashsrv_msgHandler(void *arg, msg_t *msg)
{
	storage_t *strg;
	mount_i_msg_t *imnt = (mount_i_msg_t *)msg->i.raw;
	mount_o_msg_t *omnt = (mount_o_msg_t *)msg->o.raw;

	(void)arg;

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			strg = storage_get(msg->i.openclose.oid.id);
			msg->o.io.err = (strg == NULL) ? -EINVAL : EOK;
			TRACE("mtOpen/mtClose: %d", msg->o.io.err);
			break;

		case mtRead:
			strg = storage_get(msg->i.io.oid.id);
			TRACE("mtRead: id: %u, size: %d, off: %llu", msg->i.io.oid.id, msg->o.size, msg->i.io.offs);
			msg->o.io.err = flashsrv_read(strg, msg->i.io.offs, msg->o.data, msg->o.size);
			break;

		case mtWrite:
			strg = storage_get(msg->i.io.oid.id);
			TRACE("mtWrite: id: %u, size: %d, off: %llu", msg->i.io.oid.id, msg->o.size, msg->i.io.offs);
			msg->o.io.err = flashsrv_write(strg, msg->i.io.offs, msg->o.data, msg->o.size);
			break;

		case mtSync:
			strg = storage_get(msg->i.io.oid.id);
			TRACE("mtSync: id: %u", msg->i.io.oid.id);
			msg->o.io.err = flashsrv_sync(strg);
			break;

		case mtGetAttr:
			strg = storage_get(msg->i.attr.oid.id);
			TRACE("mtGetAttr: id: %u, type: %d", msg->i.attr.oid.id, msg->i.attr.type);
			msg->o.attr.err = flashsrv_getAttr(strg, msg->i.attr.type, &msg->o.attr.val);
			break;

		case mtMount:
			TRACE("mtMount: id: %u, fstype: %s, mode: %ld", imnt->dev.id, imnt->fstype, imnt->mode);
			omnt->err = storage_mountfs(storage_get(imnt->dev.id), imnt->fstype, msg->i.data, imnt->mode, &imnt->mnt, &omnt->oid);
			break;

		case mtUmount:
			TRACE("mtUmount: id: %u", ((oid_t *)msg->i.data)->id);
			omnt->err = storage_umountfs(storage_get(((oid_t *)msg->i.data)->id));
			break;

		case mtMountPoint:
			TRACE("mtMountPoint: id: %u", ((oid_t *)msg->i.data)->id);
			omnt->err = storage_mountpoint(storage_get(((oid_t *)msg->i.data)->id), &omnt->oid);
			break;

		default:
			TRACE("unknown: %d", msg->type);
			msg->o.io.err = -EINVAL;
			break;
	}
}


/* Auxiliary functions */


static int flashsrv_mountRoot(const char *name, const char *fstype)
{
	char path[38];
	if (snprintf(path, sizeof(path), "devfs/%s.%s", STRG_PATH, name) >= sizeof(path)) {
		return -ENAMETOOLONG;
	}

	oid_t oid;
	int res = lookup(path, NULL, &oid);
	if (res < 0) {
		return res;
	}

	LOG("Mounting %s as %s root filesystem", path, fstype);

	res = storage_mountfs(storage_get(oid.id), fstype, NULL, 0, NULL, &oid);
	if (res < 0) {
		LOG_ERROR("Failed to mount root filesystem");
		return res;
	}

	res = portRegister(oid.port, "/", &oid);
	if (res < 0) {
		LOG_ERROR("Failed to register root filesystem");
		storage_umountfs(storage_get(oid.id));
		return res;
	}

	return EOK;
}


static void flashsrv_help(const char *prog)
{
	printf("Usage: %s [options]\n", prog);
	printf("\t-r <name:fs> - mount partition <name> as root\n");
	printf("\t               use psdisk to create partitions\n");
	printf("\t-h           - print this message\n");
}


static int flashsrv_parseArgs(int argc, char **argv)
{
	for (;;) {
		int c = getopt(argc, argv, "r:h");
		if (c == -1) {
			return 0;
		}

		char *partName, *fs;
		switch (c) {
			case 'r':
				partName = optarg;
				fs = strchr(optarg, ':');
				if (fs == NULL) {
					LOG_ERROR("Invalid argument: %s", optarg);
					return -1;
				}
				*fs = '\0';
				fs++;
				if (flashsrv_mountRoot(partName, fs) < 0) {
					LOG_ERROR("Failed to mount root filesystem");
					return -1;
				}
				break;

			case 'h':
				flashsrv_help(argv[0]);
				return -1;

			default:
				LOG_ERROR("Unknown option: %c", c);
				return -1;
		}
	}
}


/* Initialization functions */


static int flashsrv_devInit(storage_t *strg, struct _storage_devCtx_t *ctx)
{
	if (strg->parent == NULL) {
		strg->start = 0;
		strg->size = CFI_SIZE(ctx->cfi.chipSz);
	}

	/* Initialize device structure */
	strg->dev = malloc(sizeof(storage_dev_t));
	if (strg->dev == NULL) {
		return -ENOMEM;
	}

	/* Initialize MTD interface */
	storage_mtd_t *mtd = malloc(sizeof(storage_mtd_t));
	if (mtd == NULL) {
		free(strg->dev);
		return -ENOMEM;
	}

	mtd->ops = flashdrv_getMtdOps();
	mtd->type = mtd_norFlash;
	mtd->name = "Intel";
	mtd->metaSize = 0;
	mtd->oobSize = 0;
	mtd->oobAvail = 0;
	mtd->writeBuffsz = CFI_SIZE(ctx->cfi.bufSz);
	mtd->writesz = 1;
	mtd->erasesz = ctx->blockSz;

	strg->dev->mtd = mtd;

	/* No block device interface */
	strg->dev->blk = NULL;

	/* Assign device context */
	strg->dev->ctx = ctx;

	return EOK;
}


static void flashsrv_devDestroy(storage_t *strg)
{
	if (strg->dev != NULL) {
		if (strg->dev->mtd != NULL) {
			free(strg->dev->mtd);
		}
		if (strg->dev->blk != NULL) {
			free(strg->dev->blk);
		}
		free(strg->dev);
		strg->dev = NULL;
	}
}


static ptable_t *flashsrv_ptableRead(storage_t *strg)
{
	uint32_t count, offs = CFI_SIZE(strg->dev->ctx->cfi.chipSz) - strg->dev->ctx->blockSz;
	/* Read number of partitions */
	if (flashsrv_read(strg, offs, &count, sizeof(count)) != sizeof(count)) {
		return NULL;
	}
	count = le32toh(count);

	/* Verify ptable size */
	uint32_t size = ptable_size(count);
	if (size > strg->dev->ctx->blockSz) {
		return NULL;
	}

	/* Verify magic signature */
	uint8_t magic[sizeof(ptable_magic)];
	if (flashsrv_read(strg, offs + size - sizeof(magic), magic, sizeof(magic)) != sizeof(magic)) {
		return NULL;
	}

	if (memcmp(magic, ptable_magic, sizeof(magic)) != 0) {
		return NULL;
	}

	ptable_t *ptable = malloc(size);
	if (ptable == NULL) {
		return NULL;
	}

	/* Read partition table */
	if (flashsrv_read(strg, offs, ptable, size) != size) {
		free(ptable);
		return NULL;
	}

	if (ptable_deserialize(ptable, CFI_SIZE(strg->dev->ctx->cfi.chipSz), strg->dev->ctx->blockSz) < 0) {
		free(ptable);
		return NULL;
	}

	return ptable;
}


static int flashsrv_partAdd(storage_t *parent, storage_t **newPart, uint32_t offset, uint32_t size, const char *name)
{
	unsigned int partId = 0;
	storage_t *part = parent->parts;
	if (part != NULL) {
		do {
			partId++;
			part = part->next;
		} while (part != parent->parts);
	}

	part = calloc(1, sizeof(storage_t));
	if (part == NULL) {
		LOG_ERROR("failed to allocate storage_t");
		return -ENOMEM;
	}

	part->parent = parent;
	part->start = offset;
	part->size = size;

	int res = flashsrv_devInit(part, parent->dev->ctx);
	if (res < 0) {
		LOG_ERROR("failed to initialize a partition");
		free(part);
		return res;
	}

	oid_t oid;
	res = storage_add(part, &oid);
	if (res < 0) {
		LOG_ERROR("failed to add a partition");
		flashsrv_devDestroy(part);
		free(part);
		return res;
	}

	char path[32];
	if (snprintf(path, sizeof(path), "%s.%s", STRG_PATH, name) >= sizeof(path)) {
		LOG_ERROR("failed to build partition path");
		storage_remove(part);
		flashsrv_devDestroy(part);
		free(part);
		return -ENAMETOOLONG;
	}

	res = create_dev(&oid, path);
	if (res < 0) {
		LOG_ERROR("failed to create partition device file");
		storage_remove(part);
		flashsrv_devDestroy(part);
		free(part);
		return res;
	}

	TRACE("initialized partition %s: offset=%u, size=%u", name, offset, size);

	if (newPart != NULL) {
		*newPart = part;
	}

	return EOK;
}


static int flashsrv_partsInit(storage_t *strg)
{
	ptable_t *ptable = flashsrv_ptableRead(strg);
	if (ptable == NULL) {
		LOG_ERROR("failed to read partition table");
		return -1;
	}

	for (size_t i = 0; i < ptable->count; i++) {
		storage_t *newPart;
		if (flashsrv_partAdd(strg, &newPart, ptable->parts[i].offset, ptable->parts[i].size, (const char *)ptable->parts[i].name) < 0) {
			LOG_ERROR("failed to add partition %s", (const char *)ptable->parts[i].name);
			free(ptable);
			return -1;
		}
	}

	free(ptable);

	return 0;
}


static storage_t *flashsrv_init(void)
{
	storage_t *strg = calloc(1, sizeof(storage_t));
	if (strg == NULL) {
		LOG_ERROR("failed to allocate storage_t");
		return NULL;
	}

	struct _storage_devCtx_t *ctx = flashdrv_contextInit();
	if (ctx == NULL) {
		LOG_ERROR("failed to initialize flash context");
		free(strg);
		return NULL;
	}

	int res = flashsrv_devInit(strg, ctx);
	if (res < 0) {
		LOG_ERROR("failed to initialize libstorage interface (%d)", res);
		flashdrv_contextDestroy(ctx);
		free(strg);
		return NULL;
	}

	oid_t oid;
	res = storage_add(strg, &oid);
	if (res < 0) {
		LOG_ERROR("failed to add storage device (%d)", res);
		flashdrv_contextDestroy(ctx);
		flashsrv_devDestroy(strg);
		free(strg);
		return NULL;
	}

	res = create_dev(&oid, STRG_PATH);
	if (res < 0) {
		LOG_ERROR("failed to create device file (%d)", res);
		storage_remove(strg);
		flashdrv_contextDestroy(ctx);
		flashsrv_devDestroy(strg);
		free(strg);
		return NULL;
	}

	return strg;
}


static void flashsrv_signalExit(int sig)
{
	(void)sig;
	exit(EXIT_SUCCESS);
}


int main(int argc, char **argv)
{
	/* Set parent exit handler */
	signal(SIGUSR1, flashsrv_signalExit);

	/* Daemonize server */
	pid_t pid = fork();
	if (pid < 0) {
		LOG_ERROR("failed to daemonize server");
		exit(EXIT_FAILURE);
	}
	/* Parent waits to be killed by the child after finished server initialization */
	else if (pid > 0) {
		(void)sleep(10);
		exit(EXIT_FAILURE);
	}

	/* Set child exit handler */
	signal(SIGUSR1, flashsrv_signalExit);

	if (setsid() < 0) {
		LOG_ERROR("failed to create new session");
		exit(EXIT_FAILURE);
	}

	/* Initialize storage library with the message handler for the flash memory */
	int err = storage_init(flashsrv_msgHandler, 16);
	if (err < 0) {
		LOG_ERROR("failed to initialize server (%d)\n", err);
		exit(EXIT_FAILURE);
	}

	/* Register JFFS2 filesystem */
	err = storage_registerfs("jffs2", libjffs2_mount, libjffs2_umount);
	if (err < 0) {
		LOG_ERROR("failed to register jffs2 (%d)\n", err);
		exit(EXIT_FAILURE);
	}

	/* Initialize flash driver and mtd interface */
	storage_t *strg = flashsrv_init();
	if (strg == NULL) {
		exit(EXIT_FAILURE);
	}

	/* Read partition table and initialize */
	if (flashsrv_partsInit(strg) < 0) {
		storage_remove(strg);
		flashdrv_contextDestroy(strg->dev->ctx);
		flashsrv_devDestroy(strg);
		free(strg);
		exit(EXIT_FAILURE);
	}

	if (flashsrv_parseArgs(argc, argv) < 0) {
		storage_remove(strg);
		flashdrv_contextDestroy(strg->dev->ctx);
		flashsrv_devDestroy(strg);
		free(strg);
		exit(EXIT_FAILURE);
	}

	/* Finished server initialization - kill parent */
	kill(getppid(), SIGUSR1);
	storage_run(1, 2 * _PAGE_SIZE);

	return EXIT_SUCCESS;
}
