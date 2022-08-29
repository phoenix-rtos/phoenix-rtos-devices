/*
 * Phoenix-RTOS
 *
 * Storage devices
 *
 * Copyright 2021 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <sys/list.h>
#include <sys/rb.h>
#include <sys/threads.h>

#include <posix/idtree.h>

#include "storage.h"


#define REQTHR_PRIORITY  1
#define POOLTHR_PRIORITY 1


enum { state_exit = -1, state_stop, state_run };


typedef struct {
	char name[16];     /* Filesystem name */
	mount_t mount;     /* Filesystem mount */
	umount_t umount;   /* Filesystem umount */
	handler_t handler; /* Filesystem message handler */
	rbnode_t node;     /* RB tree node */
} filesystem_t;


typedef struct _request_t request_t;


typedef struct {
	int state;          /* Context state */
	unsigned int port;  /* Context port */
	unsigned int nreqs; /* Number of actively processed requests */
	handler_t handler;  /* Message handler */
	void *ctx;          /* Message handling context */
	request_t *stopped; /* Stopped requests */
	handle_t scond;     /* Stopped requests condition variable */
	handle_t lock;      /* Context mutex */
	char stack[512] __attribute__((aligned(8)));
} request_ctx_t;


typedef struct {
	request_ctx_t ctx; /* Filesystem requests context */
	filesystem_t *fs;  /* Filesystem data */
} filesystem_ctx_t;


struct _request_t {
	msg_t msg;              /* Request message */
	unsigned long rid;      /* Request message receiving context */
	request_ctx_t *ctx;     /* Request handling context */
	request_t *prev, *next; /* Doubly linked list */
};


typedef struct {
	request_t *reqs; /* Requests queue */
	handle_t lock;   /* Queue mutex */
} queue_t;


static struct {
	int state;         /* Storage handling state */
	idtree_t devs;     /* Storage devices */
	rbtree_t fss;      /* Registered filesystems */
	queue_t free;      /* Free requests queue */
	queue_t ready;     /* Ready requests queue */
	handle_t fcond;    /* Free requests condition variable */
	handle_t rcond;    /* Ready requests condition variable */
	handle_t lock;     /* Storage handling mutex */
	request_ctx_t ctx; /* Storage devices requests context */
} storage_common;


static request_t *queue_pop(queue_t *q)
{
	request_t *req;

	mutexLock(q->lock);

	if (q->reqs == NULL) {
		mutexUnlock(q->lock);
		return NULL;
	}

	req = q->reqs->prev;
	LIST_REMOVE(&q->reqs, req);

	mutexUnlock(q->lock);

	return req;
}


static void queue_push(queue_t *q, request_t *req)
{
	mutexLock(q->lock);

	LIST_ADD(&q->reqs, req);

	mutexUnlock(q->lock);
}


static void queue_done(queue_t *q)
{
	resourceDestroy(q->lock);
}


static int queue_init(queue_t *q)
{
	int err;

	if ((err = mutexCreate(&q->lock)) < 0)
		return err;

	q->reqs = NULL;

	return EOK;
}


static void storage_reqthr(void *arg)
{
	request_ctx_t *ctx = (request_ctx_t *)arg;
	request_t *req;
	int err;

	for (;;) {
		mutexLock(ctx->lock);

		while ((ctx->state != state_exit) && ((ctx->state == state_stop) || ((req = queue_pop(&storage_common.free)) == NULL)))
			condWait(storage_common.fcond, ctx->lock, 0);

		if (ctx->state == state_exit) {
			mutexUnlock(ctx->lock);

			endthread();
		}

		mutexUnlock(ctx->lock);

		while ((err = msgRecv(ctx->port, &req->msg, &req->rid)) < 0) {
			/* Closed port */
			if (err == -EINVAL)
				break;
		}

		req->ctx = ctx;
		mutexLock(ctx->lock);

		if ((err < 0) || (ctx->state == state_exit)) {
			mutexUnlock(ctx->lock);

			queue_push(&storage_common.free, req);
			condSignal(storage_common.fcond);
			endthread();
		}
		else if (ctx->state == state_stop) {
			LIST_ADD(&ctx->stopped, req);

			mutexUnlock(ctx->lock);
		}
		else if (ctx->state == state_run) {
			mutexUnlock(ctx->lock);

			queue_push(&storage_common.ready, req);
			condSignal(storage_common.rcond);
		}
	}
}


static void storage_poolthr(void *arg)
{
	request_ctx_t *ctx;
	request_t *req;

	for (;;) {
		mutexLock(storage_common.lock);

		while ((storage_common.state != state_exit) && ((storage_common.state == state_stop) || ((req = queue_pop(&storage_common.ready)) == NULL)))
			condWait(storage_common.rcond, storage_common.lock, 0);

		if (storage_common.state == state_exit) {
			mutexUnlock(storage_common.lock);

			endthread();
		}

		mutexUnlock(storage_common.lock);

		ctx = req->ctx;
		mutexLock(ctx->lock);

		if (ctx->state == state_stop) {
			LIST_ADD(&ctx->stopped, req);

			mutexUnlock(ctx->lock);
		}
		else {
			ctx->nreqs++;

			mutexUnlock(ctx->lock);

			priority(req->msg.priority);
			ctx->handler(ctx->ctx, &req->msg);
			priority(POOLTHR_PRIORITY);

			msgRespond(ctx->port, &req->msg, req->rid);
			queue_push(&storage_common.free, req);
			condSignal(storage_common.fcond);

			mutexLock(ctx->lock);

			if ((--ctx->nreqs == 0) && (ctx->state == state_stop))
				condSignal(ctx->scond);

			mutexUnlock(ctx->lock);
		}
	}
}


static void requestctx_run(request_ctx_t *ctx)
{
	request_t *req;

	mutexLock(ctx->lock);

	ctx->state = state_run;
	while (ctx->stopped != NULL) {
		req = ctx->stopped->prev;
		LIST_REMOVE(&ctx->stopped, req);
		queue_push(&storage_common.ready, req);
		condSignal(storage_common.rcond);
	}

	mutexUnlock(ctx->lock);
	condBroadcast(storage_common.fcond);
}


static void requestctx_stop(request_ctx_t *ctx)
{
	mutexLock(ctx->lock);

	ctx->state = state_stop;
	while (ctx->nreqs)
		condWait(ctx->scond, ctx->lock, 0);

	mutexUnlock(ctx->lock);
}


static void requestctx_done(request_ctx_t *ctx)
{
	request_t *req;

	requestctx_stop(ctx);
	mutexLock(ctx->lock);

	portDestroy(ctx->port);
	ctx->state = state_exit;
	while ((req = ctx->stopped) != NULL) {
		LIST_REMOVE(&ctx->stopped, req);
		queue_push(&storage_common.free, req);
	}

	mutexUnlock(ctx->lock);

	do {
		condBroadcast(storage_common.fcond);
	} while (threadJoin(-1, 10000) < 0);

	resourceDestroy(ctx->scond);
	resourceDestroy(ctx->lock);
}


static int storagectx_init(request_ctx_t *ctx, handler_t handler, void *fs)
{
	int err;

	if ((err = mutexCreate(&ctx->lock)) < 0)
		return err;

	if ((err = condCreate(&ctx->scond)) < 0) {
		resourceDestroy(ctx->lock);
		return err;
	}

	if ((err = portCreate(&ctx->port)) < 0) {
		resourceDestroy(ctx->scond);
		resourceDestroy(ctx->lock);
		return err;
	}

	ctx->handler = handler;
	ctx->ctx = fs;
	ctx->stopped = NULL;
	ctx->nreqs = 0;
	ctx->state = state_stop;

	if ((err = beginthread(storage_reqthr, REQTHR_PRIORITY, ctx->stack, sizeof(ctx->stack), ctx)) < 0) {
		portDestroy(ctx->port);
		resourceDestroy(ctx->scond);
		resourceDestroy(ctx->lock);
		return err;
	}

	return EOK;
}


storage_t *storage_get(int id)
{
	return lib_treeof(storage_t, node, idtree_find(&storage_common.devs, id));
}


static filesystem_t *storage_getfs(const char *name)
{
	filesystem_t fs;

	strncpy(fs.name, name, sizeof(fs.name));
	fs.name[sizeof(fs.name) - 1] = '\0';

	return lib_treeof(filesystem_t, node, lib_rbFind(&storage_common.fss, &fs.node));
}


int storage_registerfs(const char *name, mount_t mount, umount_t umount, handler_t handler)
{
	filesystem_t *fs;

	if ((name == NULL) || (mount == NULL) || (umount == NULL) || (handler == NULL))
		return -EINVAL;

	if ((fs = malloc(sizeof(filesystem_t))) == NULL)
		return -ENOMEM;

	strncpy(fs->name, name, sizeof(fs->name));
	fs->name[sizeof(fs->name) - 1] = '\0';
	fs->mount = mount;
	fs->umount = umount;
	fs->handler = handler;

	if (lib_rbInsert(&storage_common.fss, &fs->node) != NULL) {
		free(fs);
		return -EEXIST;
	}

	return EOK;
}


int storage_unregisterfs(const char *name)
{
	filesystem_t *fs = storage_getfs(name);

	if (fs == NULL)
		return -EINVAL;

	lib_rbRemove(&storage_common.fss, &fs->node);
	free(fs);

	return EOK;
}


int storage_mountfs(storage_t *dev, const char *name, const char *data, unsigned long mode, oid_t *root)
{
	filesystem_t *fs = storage_getfs(name);
	filesystem_ctx_t *ctx;
	int err;

	if ((dev == NULL) || (dev->ctx == NULL) || (dev->parts != NULL) || (fs == NULL) || (root == NULL))
		return -EINVAL;

	if (dev->fs != NULL)
		return -EBUSY;

	if ((ctx = malloc(sizeof(filesystem_ctx_t))) == NULL)
		return -ENOMEM;

	if ((err = storagectx_init(&ctx->ctx, fs->handler, NULL)) < 0) {
		free(ctx);
		return err;
	}
	root->port = ctx->ctx.port;

	if ((err = fs->mount(dev->ctx, &ctx->ctx.ctx, data, mode, root)) < 0) {
		requestctx_done(&ctx->ctx);
		free(ctx);
		return err;
	}

	requestctx_run(&ctx->ctx);
	dev->fs = ctx;

	return EOK;
}


int storage_umountfs(storage_t *dev)
{
	filesystem_ctx_t *ctx;
	int err;

	if ((dev == NULL) || (dev->fs == NULL))
		return -EINVAL;

	ctx = (filesystem_ctx_t *)dev->fs;
	requestctx_stop(&ctx->ctx);

	if ((err = ctx->fs->umount(ctx->ctx.ctx)) < 0) {
		requestctx_run(&ctx->ctx);
		return err;
	}

	requestctx_done(&ctx->ctx);
	free(ctx);
	dev->fs = NULL;

	return EOK;
}


int storage_add(storage_t *dev)
{
	storage_t *pdev, *part;

	if ((dev == NULL) || (dev->ctx == NULL) || (dev->size == 0))
		return -EINVAL;

	if ((pdev = dev->parent) != NULL) {
		if ((dev->start < pdev->start) || (dev->start + dev->size > pdev->start + pdev->size))
			return -EINVAL;

		if ((part = pdev->parts) != NULL) {
			do {
				if (dev->start + dev->size <= part->start)
					break;
				else if (dev->start >= part->start + part->size)
					part = part->next;
				else
					return -EINVAL;
			} while (part != pdev->parts);
		}

		if ((part == NULL) || ((part == pdev->parts) && (dev->start + dev->size <= part->start)))
			pdev->parts = dev;
		LIST_ADD(&part, dev);
	}

	dev->parts = NULL;
	dev->fs = NULL;

	return idtree_alloc(&storage_common.devs, &dev->node);
}


int storage_remove(storage_t *dev)
{
	if ((dev == NULL) || (dev->parts != NULL))
		return -EINVAL;

	if (dev->fs != NULL)
		return -EBUSY;

	if (dev->parent != NULL)
		LIST_REMOVE(&dev->parent->parts, dev);

	idtree_remove(&storage_common.devs, &dev->node);

	return EOK;
}


int storage_run(unsigned int nthreads, unsigned int stacksz)
{
	unsigned int i, j;
	char *stacks;
	int err;

	if ((stacks = malloc(nthreads * stacksz)) == NULL)
		return -ENOMEM;

	storage_common.state = state_run;

	for (i = 0; i < nthreads; i++) {
		if ((err = beginthread(storage_poolthr, POOLTHR_PRIORITY, stacks + i * stacksz, stacksz, NULL)) < 0) {
			mutexLock(storage_common.lock);

			storage_common.state = state_exit;

			mutexUnlock(storage_common.lock);
			condBroadcast(storage_common.rcond);

			for (j = 0; j < i; j++) {
				while (threadJoin(-1, 10000) < 0)
					condSignal(storage_common.rcond);
			}
			free(stacks);
			return err;
		}
	}

	priority(POOLTHR_PRIORITY);
	storage_poolthr(NULL);

	return EOK;
}


static int storage_cmpfs(rbnode_t *n1, rbnode_t *n2)
{
	filesystem_t *fs1 = lib_treeof(filesystem_t, node, n1);
	filesystem_t *fs2 = lib_treeof(filesystem_t, node, n2);

	return strncmp(fs1->name, fs2->name, sizeof(fs1->name));
}


int storage_init(handler_t handler, unsigned int queuesz)
{
	request_t *reqs;
	unsigned int i;
	int err;

	if ((err = mutexCreate(&storage_common.lock)) < 0)
		goto lock_fail;

	if ((err = condCreate(&storage_common.rcond)) < 0)
		goto rcond_fail;

	if ((err = condCreate(&storage_common.fcond)) < 0)
		goto fcond_fail;

	if ((err = queue_init(&storage_common.ready)) < 0)
		goto ready_fail;

	if ((err = queue_init(&storage_common.free)) < 0)
		goto free_fail;

	if ((reqs = malloc(queuesz * sizeof(request_t))) == NULL) {
		err = -ENOMEM;
		goto reqs_fail;
	}

	for (i = 0; i < queuesz; i++)
		LIST_ADD(&storage_common.free.reqs, reqs + i);

	if ((err = storagectx_init(&storage_common.ctx, handler, NULL)) < 0)
		goto ctx_fail;

	storage_common.state = state_stop;
	lib_rbInit(&storage_common.fss, storage_cmpfs, NULL);
	idtree_init(&storage_common.devs);
	requestctx_run(&storage_common.ctx);

	return storage_common.ctx.port;

ctx_fail:
	free(reqs);
reqs_fail:
	queue_done(&storage_common.free);
free_fail:
	queue_done(&storage_common.ready);
ready_fail:
	resourceDestroy(storage_common.fcond);
fcond_fail:
	resourceDestroy(storage_common.rcond);
rcond_fail:
	resourceDestroy(storage_common.lock);
lock_fail:
	return err;
}
