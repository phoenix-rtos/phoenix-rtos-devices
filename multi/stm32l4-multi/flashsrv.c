/*
 * Phoenix-RTOS
 *
 * STM32L4x6 Flash storage server
 *
 * Copyright 2019-2024 Phoenix Systems
 * Author: Hubert Buczynski, Gerard Swiderski, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "fs.h"

#if BUILTIN_FLASH_SERVER

#define LFS_SUPPORT 0 /* LittleFS support disabled for now */

#include <endian.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <posix/utils.h>
#include <signal.h>

#if LFS_SUPPORT
#include <liblfs.h>
#endif
#include <meterfs.h>

#include "flash.h"


#define LOG_TAG "storage"
/* clang-format off */
#define LOG_ERROR(str, ...) do { fprintf(stderr, "%s %d error: " str "\n", LOG_TAG, __LINE__, ##__VA_ARGS__); } while (0)
#define TRACE(str, ...)     do { if (0) fprintf(stderr, "%s trace: " str "\n", LOG_TAG, ##__VA_ARGS__); } while (0)
/* clang-format on */

#define FLASH_PROG_SIZE          8
#define FLASH_ERASE_SIZE         2048
#define FS_THREAD_STACK_SIZE     2048
#define BLKDEV_THREAD_STACK_SIZE 1024
#define FS_THREAD_PRIORITY       4

#define FLASH_BLOCK_PATH "/dev/flashblk%d"

struct flash_part {
	uint32_t start;
	uint32_t size;
	struct {
		void *info;
		oid_t mntpoint;
		unsigned int msgport;
		int (*msghandler)(struct flash_part *, msg_t *);
		int (*unmount)(struct flash_part *);
		void *threadStack;
		int threadID;
		handle_t handlerMutex; /* Optional, may not be initialized depending on filesystem */
	} fs;
};

static struct {
	unsigned int msgport;
	struct flash_part *parts;
	size_t nParts; /* Number of initialized partitions */
	char blkDevStack[FS_THREAD_STACK_SIZE] __attribute__((aligned(8)));
} g_common;


static inline ssize_t flash_partRead(const struct flash_part *part, uint32_t offs, void *buf, size_t size)
{
	return flash_readData(part->start + offs, buf, size);
}


static inline ssize_t flash_partProg(const struct flash_part *part, uint32_t offs, const void *buf, size_t size)
{
	if ((offs % FLASH_PROG_SIZE != 0) || (size % FLASH_PROG_SIZE != 0)) {
		return -EINVAL;
	}

	return flash_writeRaw(part->start + offs, buf, size);
}


static inline ssize_t flash_partErase(const struct flash_part *part, uint32_t offs)
{
	if (offs % FLASH_ERASE_SIZE != 0) {
		return -EINVAL;
	}

	return flash_erasePage(part->start + offs);
}


#if LFS_SUPPORT
static int flash_LFS_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
	struct flash_part *part = c->context;
	ssize_t ret = flash_partRead(part, block * c->block_size + off, buffer, size);
	return (ret == size) ? 0 : -1;
}


static int flash_LFS_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
	struct flash_part *part = c->context;
	ssize_t ret = flash_partProg(part, block * c->block_size + off, buffer, size);
	return (ret == size) ? 0 : -1;
}


static int flash_LFS_erase(const struct lfs_config *c, lfs_block_t block)
{
	struct flash_part *part = c->context;
	const uint32_t blockMultiplier = c->block_size / FLASH_ERASE_SIZE;
	int ret = 0;
	for (int i = 0; i < blockMultiplier; i++) {
		ret = flash_partErase(part, (block * blockMultiplier + i) * FLASH_ERASE_SIZE);
		if (ret < 0) {
			return ret;
		}
	}

	return ret;
}


static int flash_LFS_sync(const struct lfs_config *c)
{
	return 0;
}


static int flash_unmountLFS(struct flash_part *part)
{
	return liblfs_rawcfg_unmount(part->fs.info, 1);
}


static int flash_handleLFS(struct flash_part *part, msg_t *msg)
{
	return liblfs_handler(part->fs.info, msg);
}


static int flash_mountLFS(struct flash_part *part, oid_t *root, unsigned long mode)
{
	struct lfs_config *cfg = calloc(1, sizeof(struct lfs_config));
	cfg->context = part;
	cfg->read = flash_LFS_read;
	cfg->prog = flash_LFS_prog;
	cfg->erase = flash_LFS_erase;
	cfg->sync = flash_LFS_sync;

	int ret = liblfs_setConfig(cfg, part->size, mode);
	if ((ret < 0) || (cfg->block_size % FLASH_ERASE_SIZE != 0)) {
		TRACE("lfs config set failed failed");
		free(cfg);
		return (cfg->block_size % FLASH_ERASE_SIZE != 0) ? -EINVAL : ret;
	}

	void *ctx;
	ret = liblfs_rawcfg_mount(&ctx, root, cfg);
	if (ret < 0) {
		TRACE("lfs mount failed %d", ret);
		free(cfg);
		return ret;
	}

	part->fs.info = ctx;
	part->fs.msghandler = flash_handleLFS;
	part->fs.unmount = flash_unmountLFS;
	return EOK;
}
#endif /* LFS_SUPPORT */


static ssize_t flash_meterFS_read(struct _meterfs_devCtx_t *devCtx, off_t offs, void *buff, size_t bufflen)
{
	return flash_partRead((struct flash_part *)devCtx, offs, buff, bufflen);
}


static ssize_t flash_meterFS_write(struct _meterfs_devCtx_t *devCtx, off_t offs, const void *buff, size_t bufflen)
{
	return flash_partProg((struct flash_part *)devCtx, offs, buff, bufflen);
}


static int flash_meterFS_eraseSector(struct _meterfs_devCtx_t *devCtx, off_t offs)
{
	return flash_partErase((struct flash_part *)devCtx, offs);
}


static int flash_unmountMeterFS(struct flash_part *part)
{
	resourceDestroy(part->fs.handlerMutex);
	part->fs.handlerMutex = 0;
	free(part->fs.info);
	return EOK;
}


static int flash_handleMeterFS(struct flash_part *part, msg_t *msg)
{
	mutexLock(part->fs.handlerMutex);
	meterfs_i_devctl_t *idevctl = (meterfs_i_devctl_t *)msg->i.raw;
	meterfs_o_devctl_t *odevctl = (meterfs_o_devctl_t *)msg->o.raw;
	meterfs_ctx_t *ctx = (meterfs_ctx_t *)part->fs.info;

	switch (msg->type) {
		case mtRead:
			msg->o.err = meterfs_readFile(msg->oid.id, msg->i.io.offs, msg->o.data, msg->o.size, ctx);
			break;

		case mtWrite:
			msg->o.err = meterfs_writeFile(msg->oid.id, msg->i.data, msg->i.size, ctx);
			break;

		case mtLookup:
			msg->o.err = meterfs_lookup(msg->i.data, &msg->o.lookup.fil.id, ctx);
			msg->o.lookup.fil.port = part->fs.msgport;
			msg->o.lookup.dev = msg->o.lookup.fil;
			break;

		case mtOpen:
			msg->o.err = meterfs_open(msg->oid.id, ctx);
			break;

		case mtClose:
			msg->o.err = meterfs_close(msg->oid.id, ctx);
			break;

		case mtDevCtl:
			msg->o.err = meterfs_devctl(idevctl, odevctl, ctx);
			break;

		default:
			msg->o.err = -EINVAL;
			break;
	}

	mutexUnlock(part->fs.handlerMutex);
	return EOK;
}


static int flash_mountMeterFS(struct flash_part *part, oid_t *root, unsigned long mode)
{
	meterfs_ctx_t *ctx = calloc(1, sizeof(meterfs_ctx_t));
	ctx->offset = 0; /* Offset is handled by the read/write functions */
	ctx->sz = part->size;
	ctx->sectorsz = FLASH_ERASE_SIZE;
	ctx->devCtx = (struct _meterfs_devCtx_t *)part;
	ctx->read = flash_meterFS_read;
	ctx->write = flash_meterFS_write;
	ctx->eraseSector = flash_meterFS_eraseSector;
	ctx->powerCtrl = NULL;

	int ret = mutexCreate(&part->fs.handlerMutex);
	if (ret < 0) {
		free(ctx);
		return ret;
	}

	ret = meterfs_init(ctx);
	if (ret < 0) {
		resourceDestroy(part->fs.handlerMutex);
		free(ctx);
		return ret;
	}

	part->fs.msghandler = flash_handleMeterFS;
	part->fs.unmount = flash_unmountMeterFS;
	part->fs.info = ctx;
	return ret;
}


static int flash_sendUnmountMessage(unsigned int port)
{
	msg_t msg;
	msg.type = mtUmount;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	int ret = msgSend(port, &msg);
	if (ret < 0) {
		return ret;
	}

	return EOK;
}


static int flash_unmountFS(struct flash_part *part)
{
	if (part->fs.threadID >= 0) {
		flash_sendUnmountMessage(part->fs.msgport);
		threadJoin(part->fs.threadID, 0);
		part->fs.threadID = -1;
	}

	if ((part->fs.info != NULL) && (part->fs.unmount != NULL)) {
		part->fs.unmount(part);
		part->fs.info = NULL;
	}

	free(part->fs.threadStack);
	part->fs.threadStack = NULL;

	if (part->fs.msgport != 0) {
		portDestroy(part->fs.msgport);
		part->fs.msgport = 0;
	}

	return 0;
}


static void flash_fsMsgThread(void *arg)
{
	struct flash_part *part = (struct flash_part *)arg;
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		if (msgRecv(part->fs.msgport, &msg, &rid) < 0) {
			break;
		}

		if (msg.type == mtUmount) {
			/* This is a bit of a hack - mtUmount messages aren't supposed to be sent to filesystems
			 * but we need some way to break out of waiting for message */
			TRACE("unmount message received");
			msgRespond(part->fs.msgport, &msg, rid);
			break;
		}

		part->fs.msghandler(part, &msg);
		msgRespond(part->fs.msgport, &msg, rid);
	}

	endthread();
}


static int flash_mountFS(struct flash_part *part, const char fsType[16], oid_t *root, unsigned long mode, oid_t *mntpoint)
{
	if (part->fs.info != NULL) {
		return -EBUSY;
	}

	int ret = portCreate(&part->fs.msgport);
	if (ret != 0) {
		return ret;
	}

	root->port = part->fs.msgport;
	if (strcmp(fsType, "lfs") == 0) {
#if LFS_SUPPORT
		ret = flash_mountLFS(part, root, mode);
#else
		ret = -ENOTSUP;
#endif
	}
	else if (strcmp(fsType, "meterfs") == 0) {
		ret = flash_mountMeterFS(part, root, mode);
	}
	else {
		ret = -EINVAL;
	}

	if (ret < 0) {
		TRACE("mount fs failed");
		flash_unmountFS(part);
		return ret;
	}

	if (mntpoint == NULL) {
		part->fs.mntpoint.port = part->fs.msgport;
	}
	else {
		part->fs.mntpoint = *mntpoint;
	}

	part->fs.threadStack = malloc(FS_THREAD_STACK_SIZE);
	if (part->fs.threadStack == NULL) {
		flash_unmountFS(part);
		return -ENOMEM;
	}

	ret = beginthreadex(flash_fsMsgThread, FS_THREAD_PRIORITY, part->fs.threadStack, FS_THREAD_STACK_SIZE, part, &part->fs.threadID);
	if (ret < 0) {
		part->fs.threadID = -1;
		TRACE("begin thread failed");
		flash_unmountFS(part);
	}

	return ret;
}


static int flash_getAttr(struct flash_part *part, int type, long long *attr)
{
	switch (type) {
		case atSize:
			*attr = part->size;
			break;

		default:
			return -EINVAL;
	}

	return EOK;
}


static void flash_blockdevMsgHandler(msg_t *msg, struct flash_part *part)
{
	mount_i_msg_t *imnt = (mount_i_msg_t *)msg->i.raw;
	mount_o_msg_t *omnt = (mount_o_msg_t *)msg->o.raw;

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.err = EOK;
			break;

		case mtRead:
			msg->o.err = flash_partRead(part, msg->i.io.offs, msg->o.data, msg->o.size);
			break;

		case mtWrite:
			msg->o.err = flash_writeData(part->start + msg->i.io.offs, msg->i.data, msg->i.size);
			break;

		case mtSync:
			msg->o.err = 0;
			break;

		case mtGetAttr:
			msg->o.err = flash_getAttr(part, msg->i.attr.type, &msg->o.attr.val);
			break;

		case mtMount:
			msg->o.err = flash_mountFS(part, imnt->fstype, &omnt->oid, imnt->mode, &imnt->mnt);
			break;

		case mtUmount:
			msg->o.err = flash_unmountFS(part);
			break;


		case mtMountPoint:
			if (part->fs.info == NULL) {
				msg->o.err = -EINVAL;
			}
			else if (part->fs.mntpoint.port == part->fs.msgport) {
				msg->o.err = -ENOENT;
			}
			else {
				omnt->oid = part->fs.mntpoint;
				msg->o.err = EOK;
			}

			break;
		default:
			msg->o.err = -EINVAL;
	}
}


static void flash_blockdevMsgThread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		if (msgRecv(g_common.msgport, &msg, &rid) < 0) {
			break;
		}

		/* Ignore this msg, as it might have been sent by us after deletion event */
		if (msg.type == mtUnlink) {
			msg.o.err = EOK;
			msgRespond(g_common.msgport, &msg, rid);
			continue;
		}

		mount_i_msg_t *imnt = (mount_i_msg_t *)msg.i.raw;
		size_t id;
		switch (msg.type) {
			case mtUmount:
				id = ((oid_t *)msg.i.data)->id;
				break;

			default:
				id = msg.oid.id;
				break;
		}

		if (id >= g_common.nParts) {
			msg.o.err = -ENOENT;
			msgRespond(g_common.msgport, &msg, rid);
			continue;
		}

		flash_blockdevMsgHandler(&msg, &g_common.parts[id]);
		msgRespond(g_common.msgport, &msg, rid);
	}

	endthread();
}


int flash_partAdd(int flashBank, uint32_t start, size_t size)
{
	if ((start % FLASH_ERASE_SIZE != 0) || (size % FLASH_ERASE_SIZE != 0)) {
		return -EINVAL;
	}

	if ((start >= FLASH_PROGRAM_BANK_SIZE) || ((start + size) > FLASH_PROGRAM_BANK_SIZE)) {
		return -EINVAL;
	}

	uint32_t bankOffset;
	switch (flashBank) {
		case 1:
			bankOffset = FLASH_PROGRAM_1_ADDR;
			break;

		case 2:
			bankOffset = FLASH_PROGRAM_2_ADDR;
			break;

		default:
			return -EINVAL;
	}

	const size_t i = g_common.nParts;
	g_common.parts[i].start = bankOffset + start;
	g_common.parts[i].size = size;
	memset(&g_common.parts[i].fs, 0, sizeof(g_common.parts[i].fs));
	g_common.parts[i].fs.threadID = -1;

	oid_t oid;
	oid.port = g_common.msgport;
	oid.id = i;
	char path[32];
	snprintf(path, sizeof(path), FLASH_BLOCK_PATH, i);
	int ret = create_dev(&oid, path);
	if (ret < 0) {
		return ret;
	}

	g_common.nParts++;
	return i;
}


static char *flash_parsePart(char *arg, int *flashBank, uint32_t *start, size_t *size)
{
	*flashBank = strtol(arg, &arg, 0);
	if (*arg != ':') {
		LOG_ERROR("missing partition offset");
		return NULL;
	}

	arg++;
	*start = strtol(arg, &arg, 0);
	if (*arg != ':') {
		LOG_ERROR("missing partition size");
		return NULL;
	}

	arg++;
	*size = strtol(arg, &arg, 0);
	if ((*arg != '\0') && (*arg != ':')) {
		LOG_ERROR("wrong partition size %s", arg);
		return NULL;
	}

	return arg;
}


static char *flash_parseFS(char *arg, char **fsName, unsigned long *mode)
{
	*fsName = arg;
	arg = strchr(arg, ':');
	if (arg == NULL) {
		LOG_ERROR("missing filesystem mode");
		return NULL;
	}

	*arg = '\0';
	arg++;
	*mode = strtol(arg, &arg, 0);
	if ((*arg != '\0') && (*arg != ':')) {
		LOG_ERROR("wrong filesystem mode %s", arg);
		return NULL;
	}

	return arg;
}


static int flash_optsCount(int argc, char **argv, size_t *nParts, bool *haveRoot)
{
	int c;
	while ((c = getopt(argc, argv, "p:r:")) != -1) {
		switch (c) {
			case 'r':
				*haveRoot = true;
				/* Fall-through */
			case 'p':
				*nParts += 1;
				break;

			default:
				return -EINVAL;
		}
	}

	optind = 0; /* Reset getopt parsing */
	return EOK;
}


static int flash_optsParse(int argc, char **argv)
{
	int err, c, flashBank;
	unsigned int id;
	oid_t oid;
	uint32_t start;
	size_t size;
	unsigned long mode;
	char *arg, *fs;

	while ((c = getopt(argc, argv, "p:r:")) != -1) {
		err = -EINVAL;
		switch (c) {
			case 'p': /* <bank:start:size> */
				arg = flash_parsePart(optarg, &flashBank, &start, &size);
				if (arg == NULL) {
					return -EINVAL;
				}

				if (*arg != '\0') {
					LOG_ERROR("unrecognized arguments at end of list: %s", arg);
					return -EINVAL;
				}

				err = flash_partAdd(flashBank, start, size);
				if (err < 0) {
					LOG_ERROR("cannot add a partition %s: %d", arg, err);
					return err;
				}

				break;

			case 'r': /* <bank:start:size:fs:mode> */
				arg = flash_parsePart(optarg, &flashBank, &start, &size);
				if (arg == NULL) {
					return -EINVAL;
				}

				if (*arg != ':') {
					LOG_ERROR("missing a filesystem name, err: %d", err);
					return err;
				}

				arg++;
				arg = flash_parseFS(arg, &fs, &mode);
				if (arg == NULL) {
					return -EINVAL;
				}

				if (*arg != '\0') {
					LOG_ERROR("unrecognized arguments at end of list: %s", arg);
					return -EINVAL;
				}


				err = flash_partAdd(flashBank, start, size);
				if (err < 0) {
					LOG_ERROR("cannot add a partition %s: %d", arg, err);
					return err;
				}

				id = err;
				err = flash_mountFS(&g_common.parts[id], fs, &oid, mode, NULL);
				if (err < 0) {
					LOG_ERROR("failed to mount a filesystem - %s: %d", fs, err);
					return err;
				}

				portRegister(oid.port, "/", &oid);
				break;

			default:
				/* Assume we caught unrecognized arguments when counting partitions */
				break;
		}
	}

	return EOK;
}


int flashsrv_main(int argc, char *argv[], int allowRoot)
{
	int ret;
	size_t nParts = 0;
	bool haveRoot = false;
	ret = flash_optsCount(argc, argv, &nParts, &haveRoot);
	if (ret != 0) {
		LOG_ERROR("arg parse error, %d", ret);
		return ret;
	}

	if (nParts == 0) {
		/* Nothing to do... */
		return EOK;
	}

	if ((allowRoot == 0) && haveRoot) {
		LOG_ERROR("mounting root partition not allowed");
		return -EINVAL;
	}

	g_common.parts = malloc(sizeof(g_common.parts[0]) * nParts);
	if (g_common.parts == NULL) {
		return -ENOMEM;
	}

	g_common.nParts = 0;
	ret = portCreate(&g_common.msgport);
	if (ret != 0) {
		free(g_common.parts);
		g_common.parts = NULL;
		LOG_ERROR("cannot create port, %d", ret);
		return ret;
	}

	ret = flash_optsParse(argc, argv);
	if (ret < 0) {
		free(g_common.parts);
		LOG_ERROR("error parsing args, %d", ret);
		return ret;
	}

	return beginthread(flash_blockdevMsgThread, FS_THREAD_PRIORITY, g_common.blkDevStack, BLKDEV_THREAD_STACK_SIZE, NULL);
}

#endif /* BUILTIN_FLASH_SERVER */
