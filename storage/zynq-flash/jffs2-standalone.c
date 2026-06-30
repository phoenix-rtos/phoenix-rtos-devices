/*
 * Phoenix-RTOS
 *
 * Standalone JFFS2 server
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jakub Klimek
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/file.h>
#include <sys/msg.h>
#include <sys/types.h>

#include <posix/utils.h>

#include <storage/storage.h>
#include <libjffs2.h>

#include "zynq-flash.h"


static struct {
	oid_t flashOid;       /* Remote flash device (port + id) */
	storage_t strg;       /* Logical storage backing the filesystem */
	storage_dev_t dev;    /* Storage device description */
	storage_mtd_t mtd;    /* MTD geometry + operations */
	storage_mtdops_t ops; /* Forwarding MTD operations */
	int strgId;           /* Storage id within libstorage */
} jffs_common;


/* Forward an MTD read to the flash server. */
static int jffs_flashRead(storage_t *strg, off_t offs, void *data, size_t len, size_t *retlen)
{
	msg_t msg = { 0 };
	int err;

	(void)strg;

	msg.type = mtRead;
	msg.oid = jffs_common.flashOid;
	msg.i.io.offs = offs;
	msg.o.data = data;
	msg.o.size = len;

	err = msgSend(jffs_common.flashOid.port, &msg);
	if (err < 0) {
		*retlen = 0;
		return err;
	}

	if (msg.o.err < 0) {
		*retlen = 0;
		printf("jffs: flash read error %d (%s) offs=%zu, len=%zu\n", msg.o.err, strerror(abs(msg.o.err)), offs, len);
		return msg.o.err;
	}


	*retlen = (size_t)msg.o.err;

	return EOK;
}


/* Forward an MTD write to the flash server. */
static int jffs_flashWrite(storage_t *strg, off_t offs, const void *data, size_t len, size_t *retlen)
{
	msg_t msg = { 0 };
	int err;

	(void)strg;

	msg.type = mtWrite;
	msg.oid = jffs_common.flashOid;
	msg.i.io.offs = offs;
	msg.i.data = data;
	msg.i.size = len;

	err = msgSend(jffs_common.flashOid.port, &msg);
	if (err < 0) {
		*retlen = 0;
		return err;
	}

	if (msg.o.err < 0) {
		*retlen = 0;
		return msg.o.err;
	}

	*retlen = (size_t)msg.o.err;

	return EOK;
}


/* Forward an MTD erase to the flash server. */
static int jffs_flashErase(storage_t *strg, off_t offs, size_t size)
{
	msg_t msg = { 0 };
	flash_i_devctl_t *devctl = (flash_i_devctl_t *)msg.i.raw;
	int err;

	(void)strg;

	msg.type = mtDevCtl;
	msg.oid = jffs_common.flashOid;

	devctl->type = flashsrv_devctl_erase;
	devctl->erase.address = (size_t)offs;
	devctl->erase.size = size;

	err = msgSend(jffs_common.flashOid.port, &msg);
	if (err < 0) {
		return err;
	}

	return (msg.o.err < 0) ? msg.o.err : EOK;
}


/* Forward an MTD sync to the flash server (best effort). */
static void jffs_flashSync(storage_t *strg)
{
	msg_t msg = { 0 };

	(void)strg;

	msg.type = mtSync;
	msg.oid = jffs_common.flashOid;

	(void)msgSend(jffs_common.flashOid.port, &msg);
}


/* Query the size of the remote flash device. */
static ssize_t jffs_flashInfo(flashsrv_info_t *info)
{
	msg_t msg = { 0 };
	flash_i_devctl_t *devctl = (flash_i_devctl_t *)msg.i.raw;
	int err;

	msg.type = mtDevCtl;
	msg.oid = jffs_common.flashOid;

	devctl->type = flashsrv_devctl_info;

	err = msgSend(jffs_common.flashOid.port, &msg);
	if (err < 0) {
		return err;
	}
	if (msg.o.err < 0) {
		return msg.o.err;
	}

	*info = ((flash_o_devctl_t *)msg.o.raw)->info;
	return EOK;
}


/* Minimal device-port handler: lets external tools (re)mount the storage. */
static void jffs_devHandler(void *arg, msg_t *msg)
{
	mount_i_msg_t *imnt;
	mount_o_msg_t *omnt;

	(void)arg;

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.err = EOK;
			break;

		case mtGetAttr:
			if (msg->i.attr.type == atSize) {
				msg->o.attr.val = jffs_common.strg.size;
				msg->o.err = EOK;
			}
			else {
				msg->o.err = -EINVAL;
			}
			break;

		case mtMount:
			imnt = (mount_i_msg_t *)msg->i.raw;
			omnt = (mount_o_msg_t *)msg->o.raw;
			msg->o.err = storage_mountfs(storage_get(jffs_common.strgId), imnt->fstype, msg->i.data, imnt->mode, &imnt->mnt, &omnt->oid);
			break;

		case mtMountPoint:
			omnt = (mount_o_msg_t *)msg->o.raw;
			msg->o.err = storage_mountpoint(storage_get(jffs_common.strgId), &omnt->oid);
			break;

		case mtUmount:
			msg->o.err = storage_umountfs(storage_get(jffs_common.strgId));
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}
}


static void jffs_help(const char *prog)
{
	printf("Usage: %s -e <erasesz> [options]\n", prog);
	printf("\t-p <name>      preconfigured flash port resolved by name (partitioning)\n");
	printf("\t-d <path>      flash device path to resolve instead of -p (e.g. /dev/mtd0)\n");
	printf("\t-m <path>      mountpoint for the jffs2 root (default /)\n");
	printf("\t-h             print this help message\n");
}


static void jffs_signalExit(int sig)
{
	(void)sig;
	exit(EXIT_SUCCESS);
}


static int jffs_resolveFlash(const char *portName, const char *devPath)
{
	int err;

	if (portName != NULL) {
		err = sys_namedResource(portName, strlen(portName), &jffs_common.flashOid.port);
		if (err < 0) {
			fprintf(stderr, "jffs: cannot resolve named port %s, err: %d\n", portName, err);
			return err;
		}
		jffs_common.flashOid.id = 0;
		return EOK;
	}

	if (devPath != NULL) {
		err = lookup(devPath, NULL, &jffs_common.flashOid);
		if (err < 0) {
			fprintf(stderr, "jffs: cannot resolve device %s, err: %d\n", devPath, err);
			return err;
		}
		return EOK;
	}

	fprintf(stderr, "jffs: no flash port or device specified\n");
	return -EINVAL;
}


int main(int argc, char **argv)
{
	int c, err;
	const char *portName = NULL;
	const char *devPath = NULL;
	const char *mountpoint = "/";
	flashsrv_info_t info;
	oid_t root;
	pid_t pid;

	while ((c = getopt(argc, argv, "p:d:m:h")) != -1) {
		switch (c) {
			case 'p': portName = optarg; break;
			case 'd': devPath = optarg; break;
			case 'm': mountpoint = optarg; break;
			case 'h':
				jffs_help(argv[0]);
				return EXIT_SUCCESS;
			default:
				jffs_help(argv[0]);
				return EXIT_FAILURE;
		}
	}

	/* Parent exits cleanly once the child confirms a successful mount. */
	signal(SIGUSR1, jffs_signalExit);

	/* Daemonize so the launcher proceeds only once the filesystem is ready. */
	pid = fork();
	if (pid < 0) {
		fprintf(stderr, "jffs: failed to daemonize\n");
		return EXIT_FAILURE;
	}
	else if (pid > 0) {
		/* Parent is released by the child after a successful mount. */
		sleep(100);
		return EXIT_FAILURE;
	}

	if (setsid() < 0) {
		fprintf(stderr, "jffs: failed to create a new session\n");
		return EXIT_FAILURE;
	}

	err = jffs_resolveFlash(portName, devPath);
	if (err < 0) {
		return EXIT_FAILURE;
	}

	err = jffs_flashInfo(&info);
	if (err < 0) {
		fprintf(stderr, "jffs: failed to query flash info, err: %d\n", (int)err);
		return EXIT_FAILURE;
	}

	err = storage_init(jffs_devHandler, 16);
	if (err < 0) {
		fprintf(stderr, "jffs: failed to initialize storage library, err: %d\n", err);
		return EXIT_FAILURE;
	}

	err = storage_registerfs("jffs2", libjffs2_mount, libjffs2_umount);
	if (err < 0) {
		fprintf(stderr, "jffs: failed to register jffs2 filesystem, err: %d\n", err);
		return EXIT_FAILURE;
	}

	/* MTD operations forwarded to the remote flash server. */
	jffs_common.ops.read = jffs_flashRead;
	jffs_common.ops.write = jffs_flashWrite;
	jffs_common.ops.erase = jffs_flashErase;
	jffs_common.ops.sync = jffs_flashSync;

	jffs_common.mtd.type = info.type;
	jffs_common.mtd.name = "jffs-flash";
	jffs_common.mtd.erasesz = info.erasesz;
	jffs_common.mtd.writesz = info.writesz;
	jffs_common.mtd.writeBuffsz = info.writesz;
	jffs_common.mtd.metaSize = info.metaSize;
	jffs_common.mtd.oobSize = info.oobSize;
	jffs_common.mtd.oobAvail = info.oobAvail;

	jffs_common.mtd.ops = &jffs_common.ops;

	jffs_common.dev.mtd = &jffs_common.mtd;

	jffs_common.strg.start = 0;
	jffs_common.strg.size = info.size;
	jffs_common.strg.dev = &jffs_common.dev;
	jffs_common.strg.parent = NULL;

	err = storage_add(&jffs_common.strg, &root);
	if (err < 0) {
		fprintf(stderr, "jffs: failed to add storage, err: %d\n", err);
		return EXIT_FAILURE;
	}
	jffs_common.strgId = root.id;

	err = storage_mountfs(&jffs_common.strg, "jffs2", NULL, 0, NULL, &root);
	if (err < 0) {
		fprintf(stderr, "jffs: failed to mount jffs2, err: %d\n", err);
		return EXIT_FAILURE;
	}

	err = portRegister(root.port, mountpoint, &root);
	if (err < 0) {
		fprintf(stderr, "jffs: failed to register root at %s, err: %d\n", mountpoint, err);
		return EXIT_FAILURE;
	}

	/* Mount finished - release the parent. */
	kill(getppid(), SIGUSR1);

	storage_run(2, 2 * _PAGE_SIZE);

	return EXIT_SUCCESS;
}
