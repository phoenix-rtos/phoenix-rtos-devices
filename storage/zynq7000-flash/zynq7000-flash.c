/*
 * Phoenix-RTOS
 *
 * Zynq-7000 NOR flash server
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/file.h>
#include <string.h>
#include <sys/stat.h>
#include <posix/utils.h>

#include <libjffs2.h>
#include <mtd/mtd.h>
#include <storage/storage.h>

#include "flashdrv.h"

#define MTD_POS   (29)
#define MTD_MASK  (3 << MTD_POS)
#define MTD_CHAR  (1 << MTD_POS)
#define MTD_BLOCK (2 << MTD_POS)

#define GET_STORAGE_ID(id) (id & ~MTD_MASK)
#define GET_MTD_TYPE(id)   (id & MTD_MASK)


/* Operations on flash memory device */

static ssize_t flash_read(oid_t *oid, offs_t offs, void *buff, size_t len)
{
	size_t retlen;
	ssize_t res = -ENOSYS;
	storage_t *strg = storage_get(GET_STORAGE_ID(oid->id));

	if (strg == NULL || strg->dev == NULL || (offs + len) > strg->size || buff == NULL) {
		res = -EINVAL;
	}
	else if (len == 0) {
		res = 0;
	}
	else if (strg->dev->mtd != NULL && strg->dev->mtd->ops != NULL && strg->dev->mtd->ops->read != NULL && GET_MTD_TYPE(oid->id) == MTD_CHAR) {
		res = strg->dev->mtd->ops->read(strg, strg->start + offs, buff, len, &retlen);

		if (retlen > 0) {
			res = retlen;
		}
	}
	else if (strg->dev->blk != NULL && strg->dev->blk->ops != NULL && strg->dev->blk->ops->read != NULL && GET_MTD_TYPE(oid->id) == MTD_BLOCK) {
		res = strg->dev->blk->ops->read(strg, strg->start + offs, buff, len);
	}
	else {
		res = -EINVAL;
	}

	return res;
}


static ssize_t flash_write(oid_t *oid, offs_t offs, const void *buff, size_t len)
{
	size_t retlen;
	ssize_t res = -ENOSYS;
	storage_t *strg = storage_get(GET_STORAGE_ID(oid->id));

	if (strg == NULL || strg->dev == NULL || (offs + len) > strg->size) {
		res = -EINVAL;
	}
	else if (len == 0) {
		res = 0;
	}
	else if (strg->dev->mtd != NULL && strg->dev->mtd->ops != NULL && strg->dev->mtd->ops->write != NULL && GET_MTD_TYPE(oid->id) == MTD_CHAR && buff != NULL) {
		res = strg->dev->mtd->ops->write(strg, strg->start + offs, buff, len, &retlen);

		if (retlen > 0) {
			res = retlen;
		}
	}
	else if (strg->dev->blk != NULL && strg->dev->blk->ops != NULL && strg->dev->blk->ops->write != NULL && GET_MTD_TYPE(oid->id) == MTD_BLOCK && buff != NULL) {
		res = strg->dev->blk->ops->write(strg, strg->start + offs, buff, len);
	}
	else {
		res = -EINVAL;
	}

	return res;
}


static int flash_sync(oid_t *oid)
{
	ssize_t res = -ENOSYS;
	storage_t *strg = storage_get(GET_STORAGE_ID(oid->id));

	if (strg == NULL || strg->dev == NULL) {
		res = -EINVAL;
	}
	else if (GET_MTD_TYPE(oid->id) == MTD_CHAR) {
		res = -ENOSYS;
	}
	else if (strg->dev->blk != NULL && strg->dev->blk->ops != NULL && strg->dev->blk->ops->sync != NULL && GET_MTD_TYPE(oid->id) == MTD_BLOCK) {
		res = strg->dev->blk->ops->sync(strg);
	}
	else {
		res = -EINVAL;
	}

	return res;
}


static int flash_getAttr(oid_t *oid, int type, long long *attr)
{
	storage_t *strg = storage_get(GET_STORAGE_ID(oid->id));

	if (strg == NULL || attr == NULL) {
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


static int flash_devCtl(oid_t *oid)
{
	ssize_t res;

	/* TODO: use libmtd to handle ioctl */
	if (GET_MTD_TYPE(oid->id) == MTD_CHAR) {
		res = -ENOSYS;
	}
	else if (GET_MTD_TYPE(oid->id) == MTD_BLOCK) {
		res = -ENOSYS;
	}
	else {
		res = -EINVAL;
	}

	return res;
}


static void flash_msgHandler(void *arg, msg_t *msg)
{
	storage_t *strg;
	mount_i_msg_t *imnt;
	mount_o_msg_t *omnt;

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			strg = storage_get(GET_STORAGE_ID(msg->i.openclose.oid.id));
			msg->o.io.err = (strg == NULL) ? -EINVAL : EOK;
			break;

		case mtRead:
			msg->o.io.err = flash_read(&msg->i.io.oid, msg->i.io.offs, msg->o.data, msg->o.size);
			break;

		case mtWrite:
			msg->o.io.err = flash_write(&msg->i.io.oid, msg->i.io.offs, msg->i.data, msg->i.size);
			break;

		case mtSync:
			msg->o.io.err = flash_sync(&msg->i.io.oid);
			break;

		case mtGetAttr:
			msg->o.attr.err = flash_getAttr(&msg->i.attr.oid, msg->i.attr.type, &msg->o.attr.val);
			break;

		case mtMount:
			imnt = (mount_i_msg_t *)msg->i.raw;
			omnt = (mount_o_msg_t *)msg->o.raw;
			omnt->err = storage_mountfs(storage_get(GET_STORAGE_ID(imnt->dev.id)), imnt->fstype, msg->i.data, imnt->mode, &imnt->mnt, &omnt->oid);
			break;

		case mtUmount:
			msg->o.io.err = storage_umountfs(storage_get(GET_STORAGE_ID(((oid_t *)msg->i.data)->id)));
			break;

		case mtMountPoint:
			omnt = (mount_o_msg_t *)msg->o.raw;
			omnt->err = storage_mountpoint(storage_get(GET_STORAGE_ID(((oid_t *)msg->i.data)->id)), &omnt->oid);
			break;

		case mtDevCtl:
			msg->o.io.err = flash_devCtl(&msg->i.io.oid);
			break;

		default:
			msg->o.io.err = -EINVAL;
			break;
	}
}


static void flash_help(const char *prog)
{
	printf("Usage: %s [options] or no args to automatically detect and initialize a NOR flash device\n", prog);
	printf("\t-p <dev:start:size>          - register partition\n");
	printf("\t\tdev:    device path\n");
	printf("\t\tstart:  partition start in bytes\n");
	printf("\t\tsize:   partition size in bytes\n");
	printf("\t-r <dev:start:size:fs>       - mount root filesystem\n");
	printf("\t\tdev:    device name\n");
	printf("\t\tstart:  partition start in bytes\n");
	printf("\t\tsize:   partition size in bytes\n");
	printf("\t\tfs:     filesystem name\n");
	printf("\t-h                           - print this help message\n");
}


static int flash_oidResolve(const char *devPath, oid_t *oid)
{
	int res;
	char temp[32];

	if (strncmp("/dev/", devPath, 5) != 0) {
		return -EINVAL;
	}

	res = snprintf(temp, sizeof(temp), "devfs/%s", devPath + 5);
	if (res >= sizeof(temp)) {
		res = -ENAMETOOLONG;
		fprintf(stderr, "zynq7000-flash: failed to build file path, err: %d\n", res);
		return res;
	}

	return lookup(temp, NULL, oid);
}


static int flash_createMtdDev(const storage_t *strg, oid_t *oid)
{
	int res;
	char path[32];
	storage_t *part;
	unsigned int partID = 0;

	/* Find id for a new partition */
	if (strg->parent != NULL) {
		part = strg->parent->parts;
		if (part != NULL) {
			do {
				partID++;
				part = part->next;
			} while (part != strg->parent->parts);
		}
	}

	/* Add mtdchar device */
	if (strg->dev->mtd != NULL) {
		oid->id &= ~MTD_MASK;
		oid->id |= MTD_CHAR;

		res = snprintf(path, sizeof(path), "/dev/mtd%u", strg->dev->ctx->id);
		if (res >= sizeof(path)) {
			res = -ENAMETOOLONG;
			fprintf(stderr, "zynq7000-flash: failed to build file path, err: %d\n", res);
			return res;
		}

		if (strg->parent != NULL) {
			res += snprintf(path + res, sizeof(path) - res, "p%u", partID);
			if (res >= sizeof(path)) {
				res = -ENAMETOOLONG;
				fprintf(stderr, "zynq7000-flash: failed to build file path, err: %d\n", res);
				return res;
			}
		}

		res = create_dev(oid, path);
		if (res < 0) {
			fprintf(stderr, "zynq7000-flash: failed to create a device file, err: %d\n", res);
			return res;
		}
	}

	/* Add mtdblock device */
	if (strg->dev->blk != NULL) {
		oid->id &= ~MTD_MASK;
		oid->id |= MTD_BLOCK;

		res = snprintf(path, sizeof(path), "/dev/mtdblock%u", strg->dev->ctx->id);
		if (res >= sizeof(path)) {
			res = -ENAMETOOLONG;
			fprintf(stderr, "zynq7000-flash: failed to build file path, err: %d\n", res);
			return res;
		}

		if (strg->parent != NULL) {
			res += snprintf(path + res, sizeof(path) - res, "p%u", partID);
			if (res >= sizeof(path)) {
				res = -ENAMETOOLONG;
				fprintf(stderr, "zynq7000-flash: failed to build file path, err: %d\n", res);
				return res;
			}
		}

		res = create_dev(oid, path);
		if (res < 0) {
			fprintf(stderr, "zynq7000-flash: failed to create a device file, err: %d\n", res);
			return res;
		}
	}

	return EOK;
}


static int flash_partAdd(const char *parentPath, off_t start, size_t size)
{
	int err;
	oid_t poid, oid;
	storage_t *strg, *parent;

	err = flash_oidResolve(parentPath, &poid);
	if (err < 0) {
		fprintf(stderr, "zynq7000-flash: cannot resolve %s\n", parentPath);
		return err;
	}

	parent = storage_get(GET_STORAGE_ID(poid.id));
	if (parent == NULL) {
		err = -EINVAL;
		fprintf(stderr, "zynq7000-flash: failed to find a parent %s, err: %d\n", parentPath, err);
		return err;
	}

	strg = malloc(sizeof(storage_t));
	if (strg == NULL) {
		err = -ENOMEM;
		fprintf(stderr, "zynq7000-flash: failed to allocate a device, err: %d\n", err);
		return err;
	}

	strg->start = parent->start + start;
	strg->size = size;
	strg->parent = parent;
	strg->dev = parent->dev;
	strg->parts = NULL;

	err = storage_add(strg, &oid);
	if (err < 0) {
		free(strg);
		fprintf(stderr, "zynq7000-flash: failed to create a partition, err: %d\n", err);
		return err;
	}

	err = flash_createMtdDev(strg, &oid);
	if (err < 0) {
		storage_remove(strg);
		free(strg);
		return err;
	}

	return GET_STORAGE_ID(oid.id);
}


static int flash_optsParse(int argc, char **argv)
{
	int err, c;
	unsigned int id;
	oid_t oid;
	offs_t start;
	size_t size;
	char *devPath, *arg, *fs;

	while ((c = getopt(argc, argv, "p:r:h")) != -1) {
		err = -EINVAL;
		switch (c) {
			case 'p': /* <dev:start:size> */
				devPath = optarg;
				arg = strchr(optarg, ':');
				if (arg == NULL) {
					fprintf(stderr, "zynq7000-flash: missing a partition offset, err: %d\n", err);
					return err;
				}

				*arg++ = '\0';
				start = strtol(arg, &arg, 0);
				if (*arg++ != ':') {
					fprintf(stderr, "zynq7000-flash: missing a partition size, err: %d\n", err);
					return err;
				}

				size = strtol(arg, &arg, 0);
				if (*arg != '\0') {
					fprintf(stderr, "zynq7000-flash: wrong partition size %s, err: %d\n", arg, err);
					return err;
				}

				err = flash_partAdd(devPath, start, size);
				if (err < 0) {
					fprintf(stderr, "zynq7000-flash: cannot add a partition %s: %d\n", arg, err);
					return err;
				}
				break;

			case 'r': /* <dev:start:size:fs> */
				devPath = optarg;
				arg = strchr(optarg, ':');
				if (arg == NULL) {
					fprintf(stderr, "zynq7000-flash: missing a partition offset, err: %d\n", err);
					return err;
				}

				*arg++ = '\0';
				start = strtol(arg, &arg, 0);
				if (*arg++ != ':') {
					fprintf(stderr, "zynq7000-flash: missing a partition size, err: %d\n", err);
					return err;
				}

				size = strtol(arg, &arg, 0);
				if (*arg != ':') {
					fprintf(stderr, "zynq7000-flash: missing a filesystem name, err: %d\n", err);
					return err;
				}

				*arg++ = '\0';
				fs = arg;

				err = flash_partAdd(devPath, start, size);
				if (err < 0) {
					fprintf(stderr, "zynq7000-flash: cannot add a partition %s: %d\n", arg, err);
					return err;
				}

				id = err;
				err = storage_mountfs(storage_get(id), fs, NULL, 0, NULL, &oid);
				if (err < 0) {
					fprintf(stderr, "zynq7000-flash: failed to mount a filesystem - %s: %d\n", fs, err);
					return err;
				}

				portRegister(oid.port, "/", &oid);
				break;

			case 'h':
				flash_help(argv[0]);
				return EOK;

			default:
				return -EINVAL;
		}
	}

	return EOK;
}


static int flash_drvInit(void)
{
	oid_t oid;
	storage_t *strg;
	int res, drvRes;

	do {
		strg = calloc(1, sizeof(storage_t));
		if (strg == NULL) {
			res = -ENOMEM;
			fprintf(stderr, "zynq7000-flash: failed to allocate storage, err: %d\n", res);
			return res;
		}

		drvRes = flashdrv_init(strg);
		if (drvRes < 0) {
			fprintf(stderr, "zynq7000-flash: failed to initialize flash memory driver, err: %d\n", drvRes);
			return drvRes;
		}

		res = storage_add(strg, &oid);
		if (res < 0) {
			fprintf(stderr, "zynq7000-flash: failed to add new storage, err: %d\n", res);
			return res;
		}

		res = flash_createMtdDev(strg, &oid);
		if (res < 0) {
			return res;
		}
	} while (drvRes > 0);

	return EOK;
}


static void flash_signalexit(int sig)
{
	exit(EXIT_SUCCESS);
}


int main(int argc, char **argv)
{
	int res;
	pid_t pid;

	/* Set parent exit handler */
	signal(SIGUSR1, flash_signalexit);

	/* Daemonize server */
	pid = fork();
	if (pid < 0) {
		fprintf(stderr, "zynq7000-flash: failed to daemonize server\n");
		exit(EXIT_FAILURE);
	}
	/* Parent waits to be killed by the child after finished server initialization */
	else if (pid > 0) {
		sleep(10);
		exit(EXIT_FAILURE);
	}

	/* Set child exit handler */
	signal(SIGUSR1, flash_signalexit);

	if (setsid() < 0) {
		fprintf(stderr, "zynq7000-flash: failed to create new session\n");
		exit(EXIT_FAILURE);
	}

	/* Initialize storage library */
	res = storage_init(flash_msgHandler, 16);
	if (res < 0) {
		fprintf(stderr, "zynq7000-flash: failed to initialize storage library, err: %d\n", res);
		return EXIT_FAILURE;
	}

	/* Register file system related to NOR flash */
	res = storage_registerfs("jffs2", libjffs2_mount, libjffs2_umount);
	if (res < 0) {
		fprintf(stderr, "zynq7000-flash: failed to register jffs2 filesystem, err: %d\n", res);
		return EXIT_FAILURE;
	}

	/* Initialize all flash devices and add them to the storage */
	res = flash_drvInit();
	if (res < 0) {
		fprintf(stderr, "zynq7000-flash: failed to initialize NOR flash memory driver, err: %d\n", res);
		return EXIT_FAILURE;
	}

	/* Based on args, create new partitions and mount rootfs */
	res = flash_optsParse(argc, argv);
	if (res < 0) {
		fprintf(stderr, "zynq7000-flash: failed to parse arguments, err: %d\n", res);
		return EXIT_FAILURE;
	}

	/* Finished server initialization - kill parent */
	kill(getppid(), SIGUSR1);
	storage_run(2, 2 * _PAGE_SIZE);

	return EXIT_SUCCESS;
}
