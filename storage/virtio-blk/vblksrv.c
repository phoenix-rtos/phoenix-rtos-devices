/*
 * Phoenix-RTOS
 *
 * VirtIO block device server
 *
 * Copyright 2020, 2024 Phoenix Systems
 * Author: Lukasz Kosinski, Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <posix/utils.h>
#include <sys/file.h>
#include <sys/threads.h>

#include <mbr.h>
#include <libext2.h>

#include "vblk.h"


typedef struct {
	bool root;
	char *diskId;
	char *partId;
} vblksrv_args_t;


/* VirtIO block devices descriptors */
static const virtio_devinfo_t info[] = {
	{ .type = vdevPCI, .id = 0x1001 },
	{ .type = vdevPCI, .id = 0x1042 },
#ifdef __TARGET_RISCV64
	/* Direct VirtIO MMIO QEMU block device descriptors */
	{ .type = vdevMMIO, .id = 0x02, .irq = 8, .base = { (void *)0x10008000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 7, .base = { (void *)0x10007000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 6, .base = { (void *)0x10006000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 5, .base = { (void *)0x10005000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 4, .base = { (void *)0x10004000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 3, .base = { (void *)0x10003000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 2, .base = { (void *)0x10002000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 1, .base = { (void *)0x10001000, 0x1000 } },
#endif
	{ .type = vdevNONE }
};


/* Server operations */


static ssize_t vblksrv_read(storage_t *strg, off_t off, void *buf, size_t size)
{
	if ((strg == NULL) || (strg->dev == NULL) || ((off + size) > strg->size)) {
		return -EINVAL;
	}

	if (size == 0) {
		return 0;
	}

	storage_blk_t *blk = strg->dev->blk;
	if ((blk != NULL) && (blk->ops != NULL) && (blk->ops->read != NULL)) {
		return blk->ops->read(strg, off, buf, size);
	}

	return -EINVAL;
}


static ssize_t vblksrv_write(storage_t *strg, off_t off, const void *buf, size_t size)
{
	if ((strg == NULL) || (strg->dev == NULL) || ((off + size) > strg->size)) {
		return -EINVAL;
	}

	if (size == 0) {
		return 0;
	}

	storage_blk_t *blk = strg->dev->blk;
	if ((blk != NULL) && (blk->ops != NULL) && (blk->ops->write != NULL)) {
		return blk->ops->write(strg, off, buf, size);
	}

	return -EINVAL;
}


static int vblksrv_getAttr(storage_t *strg, int type, long long *attr)
{
	if ((strg == NULL) || (attr == NULL)) {
		return -EINVAL;
	}

	switch (type) {
		case atSize:
			*attr = (off_t)strg->size;
			break;

		default:
			return -EINVAL;
	}

	return EOK;
}


/* Message handler */


static void vblk_msgHandler(void *arg, msg_t *msg)
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
			break;

		case mtRead:
			strg = storage_get(msg->i.io.oid.id);
			msg->o.io.err = vblksrv_read(strg, msg->i.io.offs, msg->o.data, msg->o.size);
			break;

		case mtWrite:
			strg = storage_get(msg->i.io.oid.id);
			msg->o.io.err = vblksrv_write(strg, msg->i.io.offs, msg->i.data, msg->i.size);
			break;

		case mtSync:
			msg->o.io.err = -ENOSYS;
			break;

		case mtGetAttr:
			strg = storage_get(msg->i.attr.oid.id);
			msg->o.attr.err = vblksrv_getAttr(strg, msg->i.attr.type, &msg->o.attr.val);
			break;

		case mtMount:
			strg = storage_get(imnt->dev.id);
			omnt->err = storage_mountfs(strg, imnt->fstype, msg->i.data, imnt->mode, &imnt->mnt, &omnt->oid);

		case mtUmount:
			strg = storage_get(((oid_t *)msg->i.data)->id);
			omnt->err = storage_umountfs(strg);
			break;

		case mtMountPoint:
			strg = storage_get(((oid_t *)msg->i.data)->id);
			omnt->err = storage_mountpoint(strg, &omnt->oid);
			break;

		default:
			msg->o.io.err = -EINVAL;
			break;
	}
}


/* Device initialization */


static int vblksrv_devInit(storage_t *strg, struct _storage_devCtx_t *ctx)
{
	if (strg->parent == NULL) {
		strg->start = 0;
		strg->size = ctx->size;
	}

	/* Initialize dev structure */
	strg->dev = malloc(sizeof(storage_dev_t));
	if (strg->dev == NULL) {
		return -ENOMEM;
	}

	/* Initialize BLK interface */
	strg->dev->blk = malloc(sizeof(storage_blk_t));
	if (strg->dev->blk == NULL) {
		free(strg->dev);
		strg->dev = NULL;
		return -ENOMEM;
	}

	strg->dev->blk->ops = vblk_getBlkOps();

	/* No MTD interface */
	strg->dev->mtd = NULL;

	ctx->req = vblk_open();
	strg->dev->ctx = ctx;

	return EOK;
}


static void vblksrv_devDestroy(storage_t *strg)
{
	if (strg->dev != NULL) {
		free(strg->dev->blk);
		free(strg->dev);
		strg->dev = NULL;
	}
}


/* Partition initialization */


static int vblksrv_mbrRead(storage_t *strg, mbr_t *mbr)
{
	if (vblksrv_read(strg, 0, (void *)mbr, sizeof(mbr_t)) != sizeof(mbr_t)) {
		return -EIO;
	}

	if (mbr_deserialize(mbr) < 0) {
		return -ENOENT;
	}

	return EOK;
}


static int vblksrv_partAdd(storage_t *parent, char *parentPath, storage_t **newPart, size_t partId, off_t sectOffs, size_t sectors)
{
	storage_t *part = calloc(1, sizeof(storage_t));
	if (part == NULL) {
		return -ENOMEM;
	}

	part->parent = parent;
	part->start = sectOffs * parent->dev->ctx->sectorsz;
	part->size = sectors * parent->dev->ctx->sectorsz;

	int res = vblksrv_devInit(part, parent->dev->ctx);
	if (res < 0) {
		LOG_ERROR("failed to initialize partition");
		free(part);
		return res;
	}

	oid_t oid;
	res = storage_add(part, &oid);
	if (res < 0) {
		LOG_ERROR("failed to add partition");
		vblksrv_devDestroy(part);
		free(part);
		return res;
	}

	char path[16];
	if (snprintf(path, sizeof(path), "%s.%zu", parentPath, partId) >= sizeof(path)) {
		LOG_ERROR("failed to create partition path");
		storage_remove(part);
		vblksrv_devDestroy(part);
		free(part);
		return -ENAMETOOLONG;
	}

	res = create_dev(&oid, path);
	if (res < 0) {
		LOG_ERROR("failed to create partition device");
		storage_remove(part);
		vblksrv_devDestroy(part);
		free(part);
		return res;
	}

	TRACE("initialized partition %s", path);

	*newPart = part;

	return EOK;
}


static int vblksrv_partsInit(storage_t *parent, char *parentPath)
{
	mbr_t mbr;
	int ret = vblksrv_mbrRead(parent, &mbr);
	if (ret < 0) {
		return ret;
	}

	size_t partId = 0;
	for (size_t i = 0; i < MBR_PARTITIONS; i++) {
		storage_t *part;
		if (mbr.pent[i].type == PENTRY_EMPTY) {
			continue;
		}
		ret = vblksrv_partAdd(parent, parentPath, &part, partId, mbr.pent[i].start, mbr.pent[i].sectors);
		if (ret < 0) {
			LOG_ERROR("failed to initialize partition %zu on %s", i, parentPath);
			return ret;
		}
		partId++;
	}

	return EOK;
}


static int vblksrv_mountRoot(const char *disk, const char *part)
{
	char path[20];
	if (snprintf(path, sizeof(path), "devfs/vblk%s.%s", disk, part) >= sizeof(path)) {
		LOG_ERROR("failed to create root path");
		return -ENAMETOOLONG;
	}

	oid_t oid;
	int ret = lookup(path, NULL, &oid);
	if (ret < 0) {
		LOG_ERROR("failed to lookup %s", path);
		return ret;
	}

	TRACE("mounting %s as root", path);

	ret = storage_mountfs(storage_get(oid.id), "ext2", NULL, 0, NULL, &oid);
	if (ret < 0) {
		LOG_ERROR("failed to mount %s as rootfs (%d)", path, ret);
		return ret;
	}

	ret = portRegister(oid.port, "/", &oid);
	if (ret < 0) {
		LOG_ERROR("failed to register root filesystem");
		storage_umountfs(storage_get(oid.id));
		return ret;
	}

	return EOK;
}


/* Server initialization */


static int vblksrv_init(virtio_dev_t *vdev, unsigned int idx)
{
	struct _storage_devCtx_t *ctx;
	int ret = vblk_ctxInit(&ctx, vdev);
	if (ret < 0) {
		if (ret != -ENODEV) {
			LOG_ERROR("failed to initialize device context");
		}
		return ret;
	}

	ctx->vdev = *vdev;

	storage_t *strg = calloc(1, sizeof(storage_t));
	if (strg == NULL) {
		LOG_ERROR("failed to allocate storage_t");
		vblk_ctxDestroy(ctx);
		return -ENOMEM;
	}

	ret = vblksrv_devInit(strg, ctx);
	if (ret < 0) {
		LOG_ERROR("failed to initialize device");
		vblk_ctxDestroy(ctx);
		free(strg);
		return ret;
	}

	oid_t oid;
	ret = storage_add(strg, &oid);
	if (ret < 0) {
		LOG_ERROR("failed to add storage device");
		vblk_ctxDestroy(ctx);
		vblksrv_devDestroy(strg);
		free(strg);
		return ret;
	}

	char path[8];
	(void)snprintf(path, sizeof(path), "vblk%u", idx);
	ret = create_dev(&oid, path);
	if (ret < 0) {
		LOG_ERROR("failed to create device");
		storage_remove(strg);
		vblk_ctxDestroy(ctx);
		vblksrv_devDestroy(strg);
		free(strg);
		return ret;
	}

	TRACE("initialized device %s", path);

	/* Read MBR and initialize partitions */
	ret = vblksrv_partsInit(strg, path);
	if ((ret < 0) && (ret != -ENOENT)) {
		LOG_ERROR("failed to initialize partitions");
		storage_remove(strg);
		vblk_ctxDestroy(ctx);
		vblksrv_devDestroy(strg);
		free(strg);
		return ret;
	}

	return EOK;
}


static void vblksrv_signalExit(int sig)
{
	(void)sig;
	exit(EXIT_SUCCESS);
}


static void vblksrv_help(const char *prog)
{
	printf("Usage: %s [options]\n", prog);
	printf("\t-r <diskId:partId> - mount partition <partId> on disk <diskId> as root\n");
	printf("\t                     partitions are read as MBR\n");
	printf("\t-h                 - print this message\n");
}


static int vblksrv_parseArgs(int argc, char **argv, vblksrv_args_t *args)
{
	for (;;) {
		int c = getopt(argc, argv, "r:h");
		if (c == -1) {
			return 0;
		}

		switch (c) {
			case 'r':
				args->diskId = optarg;
				args->partId = strchr(optarg, ':');
				if (args->partId == NULL) {
					LOG_ERROR("Invalid argument: %s", optarg);
					return -1;
				}
				*args->partId = '\0';
				args->partId++;
				args->root = true;
				break;

			case 'h':
				vblksrv_help(argv[0]);
				return -1;

			default:
				LOG_ERROR("Unknown option: %c", c);
				return -1;
		}
	}
}


int main(int argc, char **argv)
{
	/* Set parent exit handler */
	signal(SIGUSR1, vblksrv_signalExit);

	/* Daemonize */
	pid_t pid = fork();
	if (pid < 0) {
		LOG_ERROR("failed to daemonize server");
		exit(EXIT_FAILURE);
	}
	/* Parent waits to be killed by the child after finished server initialization */
	else if (pid > 0) {
		(void)sleep(10);
		exit(EXIT_SUCCESS);
	}

	/* Set child exit handler */
	signal(SIGUSR1, vblksrv_signalExit);

	if (setsid() < 0) {
		LOG_ERROR("failed to create new session");
		exit(EXIT_FAILURE);
	}

	/* Parse arguments */
	vblksrv_args_t args = { 0 };
	int err = vblksrv_parseArgs(argc, argv, &args);
	if (err < 0) {
		exit(EXIT_FAILURE);
	}

	/* Initialize storage library with the message handler for the drives */
	err = storage_init(vblk_msgHandler, 16);
	if (err < 0) {
		LOG_ERROR("failed to initialize server (%d)", err);
		exit(EXIT_FAILURE);
	}

	/* Register EXT2 filesystem */
	err = storage_registerfs("ext2", libext2_storage_mount, libext2_storage_umount);
	if (err < 0) {
		LOG_ERROR("failed to register ext2 (%d)", err);
		exit(EXIT_FAILURE);
	}

	virtio_init();

	unsigned int devs = 0;

	/* Detect and initialize VirtIO block devices */
	for (size_t i = 0; info[i].type != vdevNONE; i++) {
		virtio_dev_t vdev;
		virtio_ctx_t vctx = { .reset = 1 };

		err = virtio_find(&info[i], &vdev, &vctx);
		if (err == 0) {
			err = vblksrv_init(&vdev, devs);
		}

		if (err < 0) {
			if (err == -ENODEV) {
				continue;
			}
			LOG_ERROR("failed to initialize VirtIO block device (%d)", err);
			exit(EXIT_FAILURE);
		}

		devs++;
	}

	if (args.root) {
		if (vblksrv_mountRoot(args.diskId, args.partId) < 0) {
			LOG_ERROR("Failed to mount root filesystem");
			return -1;
		}
	}

	LOG("initialized");

	kill(getppid(), SIGUSR1);
	storage_run(1, 2 * _PAGE_SIZE);

	return EXIT_SUCCESS;
}
