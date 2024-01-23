/*
 * Phoenix-RTOS
 *
 * VirtIO block device driver
 *
 * Copyright 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _VBLK_H_
#define _VBLK_H_


#include <stdio.h>
#include <virtio.h>
#include <storage/storage.h>


struct _storage_devCtx_t {
	/* Device data */
	virtio_dev_t vdev;       /* VirtIO device */
	virtqueue_t vq;          /* Device virtqueue */
	unsigned int sectorsz;   /* Device sector size */
	unsigned long long size; /* Device storage size */

	struct _virtioblk_req_t *req; /* Request buffers */
	handle_t lock;
};


/* clang-format off */
#define LOG(fmt, ...) do { (void)fprintf(stdout, "virtio-blk: " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(fmt, ...) do { (void)fprintf(stderr, "virtio-blk:%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } while (0)
#define TRACE(fmt, ...) do { if (0) { (void)fprintf(stdout, "virtio-blk:%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } } while (0)
/* clang-format on */


/* Opens request context */
struct _virtioblk_req_t *vblk_open(void);


/* Closes request context */
void vblk_close(struct _virtioblk_req_t *rctx);


const storage_blkops_t *vblk_getBlkOps(void);


/* Initializes device context */
int vblk_ctxInit(struct _storage_devCtx_t **ctx, virtio_dev_t *vdev);


/* Destroys device context */
void vblk_ctxDestroy(struct _storage_devCtx_t *ctx);


#endif
