/*
 * Phoenix-RTOS
 *
 * SD Card libstorage-based server
 *
 * Copyright 2023 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <sys/types.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <posix/utils.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <cache.h>
#include <storage/storage.h>
#include <libjffs2.h>
#include <libext2.h>

#include "sdstorage_dev.h"

#define LOG_TAG "sdstorage_srv"
/* clang-format off */
#define LOG_ERROR(str, ...) do { fprintf(stderr, LOG_TAG " error: " str "\n", ##__VA_ARGS__); } while (0)
#define TRACE(str, ...)     do { if (0) fprintf(stderr, LOG_TAG " trace: " str "\n", ##__VA_ARGS__); } while (0)
/* clang-format on */


static ssize_t storage_read(id_t id, offs_t offs, void *buff, size_t len)
{
	ssize_t res = -ENOSYS;
	storage_t *strg = storage_get(GET_STORAGE_ID(id));

	if ((strg == NULL) ||
		(strg->dev == NULL) ||
		((offs + len) > strg->size) ||
		(buff == NULL)) {
		res = -EINVAL;
	}
	else if (len == 0) {
		res = 0;
	}
	else if ((strg->dev->mtd != NULL) &&
		(strg->dev->mtd->ops != NULL) &&
		(strg->dev->mtd->ops->read != NULL) &&
		IS_MTD_DEVICE_ID(id)) {
		size_t retlen;
		res = strg->dev->mtd->ops->read(strg, strg->start + offs, buff, len, &retlen);

		if (retlen > 0) {
			res = retlen;
		}
	}
	else if ((strg->dev->blk != NULL) &&
		(strg->dev->blk->ops != NULL) &&
		(strg->dev->blk->ops->read != NULL) &&
		IS_BLOCK_DEVICE_ID(id)) {
		res = strg->dev->blk->ops->read(strg, strg->start + offs, buff, len);
	}
	else {
		res = -EINVAL;
	}

	return res;
}


static ssize_t storage_write(id_t id, offs_t offs, const void *buff, size_t len)
{
	ssize_t res = -ENOSYS;
	storage_t *strg = storage_get(GET_STORAGE_ID(id));

	if ((strg == NULL) ||
		(strg->dev == NULL) ||
		((offs + len) > strg->size)) {
		res = -EINVAL;
	}
	else if (len == 0) {
		res = 0;
	}
	else if ((strg->dev->mtd != NULL) &&
		(strg->dev->mtd->ops != NULL) &&
		(strg->dev->mtd->ops->write != NULL) &&
		IS_MTD_DEVICE_ID(id) &&
		(buff != NULL)) {
		size_t retlen;
		res = strg->dev->mtd->ops->write(strg, strg->start + offs, buff, len, &retlen);

		if (retlen > 0) {
			res = retlen;
		}
	}
	else if ((strg->dev->blk != NULL) &&
		(strg->dev->blk->ops != NULL) &&
		(strg->dev->blk->ops->write != NULL) &&
		(IS_BLOCK_DEVICE_ID(id)) &&
		(buff != NULL)) {
		res = strg->dev->blk->ops->write(strg, strg->start + offs, buff, len);
	}
	else {
		res = -EINVAL;
	}

	return res;
}


static int storage_sync(id_t id)
{
	ssize_t res = -ENOSYS;
	storage_t *strg = storage_get(GET_STORAGE_ID(id));

	if ((strg == NULL) || (strg->dev == NULL)) {
		res = -EINVAL;
	}
	else if (IS_MTD_DEVICE_ID(id)) {
		res = -ENOSYS;
	}
	else if ((strg->dev->blk != NULL) &&
		(strg->dev->blk->ops != NULL) &&
		(strg->dev->blk->ops->sync != NULL) &&
		IS_BLOCK_DEVICE_ID(id)) {
		res = strg->dev->blk->ops->sync(strg);
	}
	else {
		res = -EINVAL;
	}

	return res;
}


static int storage_getAttr(id_t id, int type, long long *attr)
{
	storage_t *strg = storage_get(GET_STORAGE_ID(id));

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


static void sdcard_msgHandler(void *arg, msg_t *msg)
{
	storage_t *strg;
	mount_i_msg_t *imnt = (mount_i_msg_t *)msg->i.raw;
	mount_o_msg_t *omnt = (mount_o_msg_t *)msg->o.raw;

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			strg = storage_get(GET_STORAGE_ID(msg->i.openclose.oid.id));
			msg->o.io.err = (strg == NULL) ? -EINVAL : EOK;
			break;

		case mtRead:
			msg->o.io.err = storage_read(msg->i.io.oid.id, msg->i.io.offs, msg->o.data, msg->o.size);
			break;

		case mtWrite:
			msg->o.io.err = storage_write(msg->i.io.oid.id, msg->i.io.offs, msg->i.data, msg->i.size);
			break;

		case mtSync:
			msg->o.io.err = storage_sync(msg->i.io.oid.id);
			break;

		case mtGetAttr:
			msg->o.attr.err = storage_getAttr(msg->i.attr.oid.id, msg->i.attr.type, &msg->o.attr.val);
			break;

		case mtMount:
			omnt->err = storage_mountfs(storage_get(GET_STORAGE_ID(imnt->dev.id)), imnt->fstype, msg->i.data, imnt->mode, &imnt->mnt, &omnt->oid);
			break;

		case mtUmount:
			msg->o.io.err = storage_umountfs(storage_get(GET_STORAGE_ID(((oid_t *)msg->i.data)->id)));
			break;

		case mtMountPoint:
			omnt->err = storage_mountpoint(storage_get(GET_STORAGE_ID(((oid_t *)msg->i.data)->id)), &omnt->oid);
			break;

		default:
			msg->o.io.err = -EINVAL;
			break;
	}
}


static int storage_oidResolve(const char *devPath, oid_t *oid)
{
	int ret;
	char temp[32];

	if (strncmp("/dev/", devPath, 5) != 0) {
		return -EINVAL;
	}

	ret = snprintf(temp, sizeof(temp), "devfs/%s", devPath + 5);
	if (ret >= sizeof(temp)) {
		ret = -ENAMETOOLONG;
		LOG_ERROR("failed to build file path, err: %d", ret);
		return ret;
	}

	return lookup(temp, NULL, oid);
}


static void flash_help(const char *prog)
{
	printf("Usage: %s [options] or no args to automatically detect and initialize SD cards\n", prog);
	printf("\t-c {0,1}    - Cache setting: 0 - write back, 1 - write through (default)\n");
	printf("\t-r <dev:fs> - mount root filesystem\n");
	printf("\t\tdev:    device name\n");
	printf("\t\tfs:     filesystem name\n");
	printf("\t-h          - print this help message\n");
}


typedef struct {
	int cachePolicy;
	char rootDev[32];
	char rootFsName[32];
} options_parsed_t;


static int sdstorage_optsParse(int argc, char **argv, options_parsed_t *opts)
{
	int c;
	char *arg, *devPath;
	opts->rootDev[0] = '\0';
	opts->rootFsName[0] = '\0';
	opts->cachePolicy = LIBCACHE_WRITE_THROUGH;

	do {
		c = getopt(argc, argv, "c:r:h");
		switch (c) {
			case 'c':
				if ((optarg[0] != '\0') && (optarg[1] == '\0')) {
					if (optarg[0] == '0') {
						opts->cachePolicy = LIBCACHE_WRITE_BACK;
						break;
					}
					else if (optarg[0] == '1') {
						opts->cachePolicy = LIBCACHE_WRITE_THROUGH;
						break;
					}
				}

				LOG_ERROR("unrecognized cache option: %s", optarg);
				return -EINVAL;

			case 'r': { /* <dev:fs> */
				devPath = optarg;
				arg = strchr(optarg, ':');
				if ((arg == NULL) || (*arg != ':')) {
					LOG_ERROR("missing a filesystem name");
					return -EINVAL;
				}

				*arg = '\0';
				arg++;
				strncpy(opts->rootDev, devPath, sizeof(opts->rootDev));
				opts->rootDev[sizeof(opts->rootDev) - 1] = '\0';
				strncpy(opts->rootFsName, arg, sizeof(opts->rootFsName));
				opts->rootFsName[sizeof(opts->rootFsName) - 1] = '\0';
			} break;

			case 'h':
				flash_help(argv[0]);
				break;

			case EOF:
				break;

			default:
				return -EINVAL;
		}
	} while (c != EOF);

	return EOK;
}


static int sdstorage_mountRootFs(options_parsed_t *opts)
{
	if (opts->rootDev[0] == '\0') {
		return EOK;
	}

	oid_t oid, devOid;
	int err = storage_oidResolve(opts->rootDev, &devOid);
	if (err < 0) {
		LOG_ERROR("cannot resolve %s: %d", opts->rootDev, err);
		return err;
	}

	err = storage_mountfs(storage_get(GET_STORAGE_ID(devOid.id)), opts->rootFsName, NULL, 0, NULL, &oid);
	if (err < 0) {
		LOG_ERROR("failed to mount %s: %d", opts->rootFsName, err);
		return err;
	}

	return portRegister(oid.port, "/", &oid);
}


static void sdcard_signalexit(int sig)
{
	exit(EXIT_SUCCESS);
}


int main(int argc, char *argv[])
{
	/* Set parent exit handler */
	signal(SIGUSR1, sdcard_signalexit);

	/* Daemonize server */
	pid_t pid = fork();
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
	signal(SIGUSR1, sdcard_signalexit);

	if (setsid() < 0) {
		LOG_ERROR("failed to create new session");
		exit(EXIT_FAILURE);
	}

	options_parsed_t opts;
	int ret = sdstorage_optsParse(argc, argv, &opts);
	if (ret < 0) {
		LOG_ERROR("failed to parse arguments, err: %d", ret);
		exit(EXIT_FAILURE);
	}

	int nSlots = 0;
	do {
		ret = sdstorage_initHost(nSlots);
		if (ret < 0) {
			LOG_ERROR("failed to init host, err: %d", ret);
			exit(EXIT_FAILURE);
		}

		nSlots++;
	} while (ret > 0);

	sdstorage_setDefaultCachePolicy(opts.cachePolicy);

	ret = storage_init(sdcard_msgHandler, 16);
	if (ret < 0) {
		LOG_ERROR("failed to initialize storage library, err: %d", ret);
		exit(EXIT_FAILURE);
	}

	ret = storage_registerfs("jffs2", libjffs2_mount, libjffs2_umount);
	if (ret < 0) {
		LOG_ERROR("failed to register jffs2 filesystem, err: %d", ret);
		exit(EXIT_FAILURE);
	}

	ret = storage_registerfs("ext2", libext2_storage_mount, libext2_storage_umount);
	if (ret < 0) {
		LOG_ERROR("failed to register ext2 filesystem, err: %d", ret);
		exit(EXIT_FAILURE);
	}

	ret = sdstorage_runPresenceDetection();
	if (ret < 0) {
		LOG_ERROR("failed to start presence detection thread");
		exit(EXIT_FAILURE);
	}

	ret = sdstorage_mountRootFs(&opts);
	if (ret < 0) {
		LOG_ERROR("failed to mount rootfs, err: %d", ret);
		exit(EXIT_FAILURE);
	}

	kill(getppid(), SIGUSR1);

	storage_run(2, 2 * _PAGE_SIZE);

	return 0;
}
