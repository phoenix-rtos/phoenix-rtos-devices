/*
 * Phoenix-RTOS
 *
 * STM32L4 Filesystem driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "config.h"

#if BUILTIN_DUMMYFS

#include <errno.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <stdlib.h>

#include <phoenix/sysinfo.h>

#include <dummyfs.h>

#define MSGTHR_STACKSZ 4096

struct {
	char stack[MSGTHR_STACKSZ] __attribute__((aligned(8)));
	unsigned port;
} fs_common;


static int syspage_create(void *ctx, oid_t *root)
{
	oid_t sysoid = { 0 };
	oid_t toid = { 0 };
	syspageprog_t prog;
	int i, progsz;

	if ((progsz = syspageprog(NULL, -1)) < 0)
		return -1;

	if (dummyfs_create(ctx, root, "syspage", &sysoid, 0666, otDir, NULL) != 0)
		return -ENOMEM;

	for (i = 0; i < progsz; i++) {
		if (syspageprog(&prog, i) != 0)
			continue;

		dummyfs_createMapped(ctx, &sysoid, prog.name, (void *)prog.addr, prog.size, &toid);
	}

	return EOK;
}


static void msgthr(void *ctx)
{
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		if (msgRecv(fs_common.port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {

			case mtOpen:
				msg.o.io.err = dummyfs_open(ctx, &msg.i.openclose.oid);
				break;

			case mtClose:
				msg.o.io.err = dummyfs_close(ctx, &msg.i.openclose.oid);
				break;

			case mtRead:
				msg.o.io.err = dummyfs_read(ctx, &msg.i.io.oid, msg.i.io.offs, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.io.err = dummyfs_write(ctx, &msg.i.io.oid, msg.i.io.offs, msg.i.data, msg.i.size);
				break;

			case mtTruncate:
				msg.o.io.err = dummyfs_truncate(ctx, &msg.i.io.oid, msg.i.io.len);
				break;

			case mtDevCtl:
				msg.o.io.err = -EINVAL;
				break;

			case mtCreate:
				msg.o.create.err = dummyfs_create(ctx, &msg.i.create.dir, msg.i.data, &msg.o.create.oid, msg.i.create.mode, msg.i.create.type, &msg.i.create.dev);
				break;

			case mtDestroy:
				msg.o.io.err = dummyfs_destroy(ctx, &msg.i.destroy.oid);
				break;

			case mtSetAttr:
				msg.o.attr.err = dummyfs_setattr(ctx, &msg.i.attr.oid, msg.i.attr.type, msg.i.attr.val, msg.i.data, msg.i.size);
				break;

			case mtGetAttr:
				msg.o.attr.err = dummyfs_getattr(ctx, &msg.i.attr.oid, msg.i.attr.type, &msg.o.attr.val);
				break;

			case mtLookup:
				msg.o.lookup.err = dummyfs_lookup(ctx, &msg.i.lookup.dir, msg.i.data, &msg.o.lookup.fil, &msg.o.lookup.dev);
				break;

			case mtLink:
				msg.o.io.err = dummyfs_link(ctx, &msg.i.ln.dir, msg.i.data, &msg.i.ln.oid);
				break;

			case mtUnlink:
				msg.o.io.err = dummyfs_unlink(ctx, &msg.i.ln.dir, msg.i.data);
				break;

			case mtReaddir:
				msg.o.io.err = dummyfs_readdir(ctx, &msg.i.readdir.dir, msg.i.readdir.offs,
					msg.o.data, msg.o.size);
				break;
		}
		msgRespond(fs_common.port, &msg, rid);
	}
}


int fs_init(void)
{
	void *ctx;
	oid_t root = { 0 };

	if (portCreate(&fs_common.port) != 0)
		return -1;

	if (portRegister(fs_common.port, "/", &root)) {
		portDestroy(fs_common.port);
		return -1;
	}

	root.port = fs_common.port;
	if (dummyfs_mount(&ctx, NULL, 0, &root) != EOK) {
		printf("dummyfs mount failed\n");
		portDestroy(fs_common.port);
		return -1;
	}

	if (syspage_create(ctx, &root) != EOK) {
		dummyfs_unmount(ctx);
		portDestroy(fs_common.port);
		return -1;
	}

	if (beginthread(msgthr, 4, fs_common.stack, MSGTHR_STACKSZ, ctx) != EOK) {
		dummyfs_unmount(ctx);
		portDestroy(fs_common.port);
		return -1;
	}

	return EOK;
}

#endif /* BUILTIN_DUMMYFS */
