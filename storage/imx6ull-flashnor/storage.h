/*
 * Phoenix-RTOS
 *
 * Storage devices interface
 *
 * Copyright 2021 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _STORAGE_H_
#define _STORAGE_H_

#include <sys/msg.h>
#include <sys/types.h>

#include <posix/idtree.h>


/* Filesystem callbacks */
typedef int (*mount_t)(void *ctx, void **fs, const char *data, unsigned long mode, oid_t *root);
typedef int (*umount_t)(void *fs);
typedef void (*handler_t)(void *fs, msg_t *msg);


typedef struct _storage_t {
	blkcnt_t start;                 /* Storage start */
	blkcnt_t size;                  /* Storage size */
	void *ctx;                      /* Storage device context */
	void *fs;                       /* Mounted filesystem context */
	struct _storage_t *parts;       /* Storage partitions */
	struct _storage_t *parent;      /* Storage parent */
	struct _storage_t *prev, *next; /* Doubly linked list */
	idnode_t node;                  /* ID tree node */
} storage_t;


/* Returns registered storage device instance */
extern storage_t *storage_get(int id);


/* Registers new supported filesystem */
extern int storage_registerfs(const char *name, mount_t mount, umount_t umount, handler_t handler);


/* Unregisters supported filesystem */
extern int storage_unregisterfs(const char *name);


/* Mounts filesystem */
extern int storage_mountfs(storage_t *dev, const char *name, const char *data, unsigned long mode, oid_t *root);


/* Unmounts filesystem */
extern int storage_umountfs(storage_t *dev);


/* Registers new storage device */
extern int storage_add(storage_t *dev);


/* Removes registered storage device */
extern int storage_remove(storage_t *dev);


/* Starts storage requests handling */
extern int storage_run(unsigned int nthreads, unsigned int stacksz);


/* Initializes storage handling */
extern int storage_init(handler_t handler, unsigned int queuesz);


#endif
