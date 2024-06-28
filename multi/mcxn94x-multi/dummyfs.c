/*
 * Phoenix-RTOS
 *
 * MCX N94x multi driver buit-in dummyds
 *
 * Copyright 2021, 2024 Phoenix Systems
 * Author: Maciej Purski, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <board_config.h>

#if defined(BUILTIN_DUMMYFS) && BUILTIN_DUMMYFS != 0

#include <errno.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <stdlib.h>

#include <phoenix/sysinfo.h>

#include <dummyfs.h>

static struct {
	char stack[2048] __attribute__((aligned(8)));
	unsigned port;
} dummy_common;


static int syspage_create(void *ctx, oid_t *root)
{
	oid_t sysoid = { 0 };

	int progsz = syspageprog(NULL, -1);
	if (progsz < 0) {
		return progsz;
	}

	if (dummyfs_create(ctx, root, "syspage", &sysoid, 0666, otDir, NULL) != 0) {
		return -ENOMEM;
	}

	for (int i = 0; i < progsz; i++) {
		syspageprog_t prog;
		oid_t toid;

		if (syspageprog(&prog, i) != 0) {
			continue;
		}

		dummyfs_createMapped(ctx, &sysoid, prog.name, (void *)prog.addr, prog.size, &toid);
	}

	return 0;
}


static void msgthr(void *ctx)
{
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		if (msgRecv(dummy_common.port, &msg, &rid) < 0) {
			continue;
		}

		switch (msg.type) {

			case mtOpen:
				msg.o.err = dummyfs_open(ctx, &msg.oid);
				break;

			case mtClose:
				msg.o.err = dummyfs_close(ctx, &msg.oid);
				break;

			case mtRead:
				msg.o.err = dummyfs_read(ctx, &msg.oid, msg.i.io.offs, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.err = dummyfs_write(ctx, &msg.oid, msg.i.io.offs, msg.i.data, msg.i.size);
				break;

			case mtTruncate:
				msg.o.err = dummyfs_truncate(ctx, &msg.oid, msg.i.io.len);
				break;

			case mtDevCtl:
				msg.o.err = -ENOSYS;
				break;

			case mtCreate:
				msg.o.err = dummyfs_create(ctx, &msg.oid, msg.i.data, &msg.o.create.oid, msg.i.create.mode, msg.i.create.type, &msg.i.create.dev);
				break;

			case mtDestroy:
				msg.o.err = dummyfs_destroy(ctx, &msg.oid);
				break;

			case mtSetAttr:
				msg.o.err = dummyfs_setattr(ctx, &msg.oid, msg.i.attr.type, msg.i.attr.val, msg.i.data, msg.i.size);
				break;

			case mtGetAttr:
				msg.o.err = dummyfs_getattr(ctx, &msg.oid, msg.i.attr.type, &msg.o.attr.val);
				break;

			case mtGetAttrAll: {
				struct _attrAll *attrs = msg.o.data;
				if ((attrs == NULL) || (msg.o.size < sizeof(struct _attrAll))) {
					msg.o.err = -EINVAL;
					break;
				}
				msg.o.err = dummyfs_getattrAll(ctx, &msg.oid, attrs);
				break;
			}

			case mtLookup:
				msg.o.err = dummyfs_lookup(ctx, &msg.oid, msg.i.data, &msg.o.lookup.fil, &msg.o.lookup.dev);
				break;

			case mtLink:
				msg.o.err = dummyfs_link(ctx, &msg.oid, msg.i.data, &msg.i.ln.oid);
				break;

			case mtUnlink:
				msg.o.err = dummyfs_unlink(ctx, &msg.oid, msg.i.data);
				break;

			case mtReaddir:
				msg.o.err = dummyfs_readdir(ctx, &msg.oid, msg.i.readdir.offs, msg.o.data, msg.o.size);
				break;

			default:
				msg.o.err = -EINVAL;
				break;
		}
		msgRespond(dummy_common.port, &msg, rid);
	}
}


void __attribute__((constructor(102))) dummyfs_init(void)
{
	void *ctx;
	oid_t root = { 0 };

	if (portCreate(&dummy_common.port) != 0) {
		return;
	}

	if (portRegister(dummy_common.port, "/", &root) != 0) {
		portDestroy(dummy_common.port);
		return;
	}

	root.port = dummy_common.port;
	if (dummyfs_mount(&ctx, NULL, 0, &root) != 0) {
		printf("dummyfs mount failed\n");
		portDestroy(dummy_common.port);
		return;
	}

	if (syspage_create(ctx, &root) != 0) {
		dummyfs_unmount(ctx);
		portDestroy(dummy_common.port);
		return;
	}

	if (beginthread(msgthr, 4, dummy_common.stack, sizeof(dummy_common.stack), ctx) != 0) {
		dummyfs_unmount(ctx);
		portDestroy(dummy_common.port);
		return;
	}
}

#endif /* BUILTIN_DUMMYFS */
