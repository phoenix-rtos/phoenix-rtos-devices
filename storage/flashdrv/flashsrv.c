/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * Flash server
 *
 * Copyright 2023-2025 Phoenix Systems
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

#include <flashdrv/flashsrv.h>

#define STRG_PATH "mtd0"

#define MAX_DRIVERS 4


struct flashsrv_opts {
	const struct flash_driver *driver;
	addr_t mctrlBase;
	addr_t flashBase;
	struct {
		char *partname;
		char *fs;
	} root;
};


static struct {
	const struct flash_driver *registry[MAX_DRIVERS];
	size_t ndrivers;
} common;


/* Flash server operations */


static ssize_t flashsrv_read(storage_t *strg, off_t offs, void *buf, size_t size)
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


static ssize_t flashsrv_write(storage_t *strg, off_t offs, const void *buf, size_t size)
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
		return 0;
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

	return 0;
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
			strg = storage_get(msg->oid.id);
			msg->o.err = (strg == NULL) ? -EINVAL : 0;
			TRACE("mtOpen/mtClose: %d", msg->o.err);
			break;

		case mtRead:
			strg = storage_get(msg->oid.id);
			TRACE("mtRead: id: %ju, size: %zu, off: %ju", (uintmax_t)msg->oid.id, msg->o.size, (uintmax_t)msg->i.io.offs);
			msg->o.err = flashsrv_read(strg, msg->i.io.offs, msg->o.data, msg->o.size);
			break;

		case mtWrite:
			strg = storage_get(msg->oid.id);
			TRACE("mtWrite: id: %ju, size: %zu, off: %ju", (uintmax_t)msg->oid.id, msg->o.size, (uintmax_t)msg->i.io.offs);
			msg->o.err = flashsrv_write(strg, msg->i.io.offs, msg->o.data, msg->o.size);
			break;

		case mtSync:
			strg = storage_get(msg->oid.id);
			TRACE("mtSync: id: %ju", (uintmax_t)msg->oid.id);
			msg->o.err = flashsrv_sync(strg);
			break;

		case mtGetAttr:
			strg = storage_get(msg->oid.id);
			TRACE("mtGetAttr: id: %ju, type: %d", (uintmax_t)msg->oid.id, msg->i.attr.type);
			msg->o.err = flashsrv_getAttr(strg, msg->i.attr.type, &msg->o.attr.val);
			break;

		case mtMount:
			TRACE("mtMount: id: %ju, fstype: %s, mode: %ld", (uintmax_t)msg->oid.id, imnt->fstype, imnt->mode);
			msg->o.err = storage_mountfs(storage_get(msg->oid.id), imnt->fstype, msg->i.data, imnt->mode, &imnt->mnt, &omnt->oid);
			break;

		case mtUmount:
			TRACE("mtUmount: id: %ju", (uintmax_t)msg->oid.id);
			msg->o.err = storage_umountfs(storage_get(msg->oid.id));
			break;

		case mtMountPoint:
			TRACE("mtMountPoint: id: %ju", (uintmax_t)msg->oid.id);
			msg->o.err = storage_mountpoint(storage_get(msg->oid.id), &omnt->oid);
			break;

		default:
			TRACE("unknown: %d", msg->type);
			msg->o.err = -ENOSYS;
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

	return 0;
}


static void flashsrv_help(const char *prog)
{
	printf("Usage: %s [options]\n", prog);
	printf("\t-r <name:fs> - mount partition <name> as root\n");
	printf("\t               use psdisk to create partitions\n");
	printf("\t-h           - print this message\n");
	printf("\t-c <addr>    - use <addr> as memory controller base address (required)\n");
	printf("\t-m <addr>    - use <addr> as flash memory base address (required)\n");
	printf("\t-d <driver>  - use <driver> for flash memory (required)\n");
	printf("\t               available drivers: \n");
	printf("\t               ");

	for (size_t i = 0; i < common.ndrivers; i++) {
		printf("%s%s", common.registry[i]->name, (i < common.ndrivers - 1) ? ", " : "\n");
	}
}


static int flashsrv_parseInitArgs(int argc, char **argv, struct flashsrv_opts *opts)
{
	for (;;) {
		int c = getopt(argc, argv, "r:d:c:m:h");
		if (c == -1) {
			return 0;
		}

		switch (c) {
			case 'h':
				flashsrv_help(argv[0]);
				return -1;

			case 'd': {
				const char *name = optarg;
				for (size_t i = 0; i < common.ndrivers; i++) {
					if (strcmp(common.registry[i]->name, name) == 0) {
						opts->driver = common.registry[i];
						break;
					}
				}
				if (opts->driver == NULL) {
					LOG_ERROR("Unknown driver: %s", name);
					return -1;
				}
				break;
			}

			case 'r': {
				opts->root.partname = optarg;
				opts->root.fs = strchr(optarg, ':');
				if (opts->root.fs == NULL) {
					LOG_ERROR("Invalid argument: %s", optarg);
					return -1;
				}
				*opts->root.fs = '\0';
				opts->root.fs++;
				break;
			}

			case 'c':
				errno = 0;
				opts->mctrlBase = strtoul(optarg, NULL, 0);
				if ((opts->mctrlBase == 0) && (errno != 0)) {
					LOG_ERROR("Invalid argument: %s", optarg);
					return -1;
				}
				break;

			case 'm':
				errno = 0;
				opts->flashBase = strtoul(optarg, NULL, 0);
				if ((opts->flashBase == 0) && (errno != 0)) {
					LOG_ERROR("Invalid argument: %s", optarg);
					return -1;
				}
				break;

			default:
				LOG_ERROR("Unknown option: %c", c);
				return -1;
		}
	}

	return 0;
}


/* Initialization functions */


void flashsrv_register(const struct flash_driver *driver)
{
	if (common.ndrivers < MAX_DRIVERS) {
		common.registry[common.ndrivers++] = driver;
	}
	else {
		LOG("Too many drivers: %s not registered. Please increase MAX_DRIVERS", driver->name);
	}
}


static ptable_t *flashsrv_ptableRead(storage_t *strg)
{
	uint32_t count;
	off_t offs = strg->size - strg->dev->mtd->erasesz;
	/* Read number of partitions */
	if (flashsrv_read(strg, offs, &count, sizeof(count)) != sizeof(count)) {
		return NULL;
	}
	count = le32toh(count);

	/* Verify ptable size */
	uint32_t size = ptable_size(count);
	if (size > strg->dev->mtd->erasesz) {
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

	if (ptable_deserialize(ptable, strg->size, strg->dev->mtd->erasesz) < 0) {
		free(ptable);
		return NULL;
	}

	return ptable;
}


static int flashsrv_partAdd(storage_t *parent, uint32_t offset, uint32_t size, const char *name)
{
	storage_t *part = parent->parts;
	if (part != NULL) {
		do {
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
	part->dev = parent->dev;

	oid_t oid;
	int res = storage_add(part, &oid);
	if (res < 0) {
		LOG_ERROR("failed to add a partition");
		free(part);
		return res;
	}

	char path[32];
	if (snprintf(path, sizeof(path), "%s.%s", STRG_PATH, name) >= sizeof(path)) {
		LOG_ERROR("failed to build partition path");
		storage_remove(part);
		free(part);
		return -ENAMETOOLONG;
	}

	res = create_dev(&oid, path);
	if (res < 0) {
		LOG_ERROR("failed to create partition device file");
		storage_remove(part);
		free(part);
		return res;
	}

	TRACE("initialized partition %s: offset=%u, size=%u", name, offset, size);

	return 0;
}


static int flashsrv_partsInit(storage_t *strg)
{
	ptable_t *ptable = flashsrv_ptableRead(strg);
	if (ptable == NULL) {
		LOG_ERROR("failed to read partition table");
		return -1;
	}

	for (size_t i = 0; i < ptable->count; i++) {
		if (flashsrv_partAdd(strg, ptable->parts[i].offset, ptable->parts[i].size, (const char *)ptable->parts[i].name) < 0) {
			LOG_ERROR("failed to add partition %s", (const char *)ptable->parts[i].name);
			free(ptable);
			return -1;
		}
	}

	free(ptable);

	return 0;
}


static storage_t *flashsrv_init(struct flashsrv_opts *opts)
{
	storage_t *strg = opts->driver->init(opts->mctrlBase, opts->flashBase);
	if (strg == NULL) {
		LOG_ERROR("failed initialize storage interface");
		return NULL;
	}

	oid_t oid;
	int res = storage_add(strg, &oid);
	if (res < 0) {
		LOG_ERROR("failed to add storage device (%d)", res);
		opts->driver->destroy(strg);
		return NULL;
	}

	res = create_dev(&oid, STRG_PATH);
	if (res < 0) {
		LOG_ERROR("failed to create device file (%d)", res);
		storage_remove(strg);
		opts->driver->destroy(strg);
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

	struct flashsrv_opts opts = {
		.driver = NULL,
		.mctrlBase = (addr_t)-1,
		.flashBase = (addr_t)-1,
		.root = { 0 }
	};

	int err = flashsrv_parseInitArgs(argc, argv, &opts);
	if (err < 0) {
		exit(EXIT_FAILURE);
	}

	if ((opts.driver == NULL) || (opts.mctrlBase == (addr_t)-1) || (opts.flashBase == (addr_t)-1)) {
		LOG("Required arguments not present");
		flashsrv_help(argv[0]);
		exit(EXIT_FAILURE);
	}

	/* Initialize storage library with the message handler for the flash memory */
	err = storage_init(flashsrv_msgHandler, 16);
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
	storage_t *strg = flashsrv_init(&opts);
	if (strg == NULL) {
		exit(EXIT_FAILURE);
	}

	/* Read partition table and initialize */
	if (flashsrv_partsInit(strg) < 0) {
		storage_remove(strg);
		opts.driver->destroy(strg);
		exit(EXIT_FAILURE);
	}

	if ((opts.root.partname != NULL) && (opts.root.fs != NULL)) {
		if (flashsrv_mountRoot(opts.root.partname, opts.root.fs) < 0) {
			storage_remove(strg);
			opts.driver->destroy(strg);
			exit(EXIT_FAILURE);
		}
	}

	/* Finished server initialization - kill parent */
	kill(getppid(), SIGUSR1);
	err = storage_run(1, 2 * _PAGE_SIZE);

	storage_remove(strg);
	opts.driver->destroy(strg);

	if (err < 0) {
		exit(EXIT_FAILURE);
	}

	return EXIT_SUCCESS;
}
