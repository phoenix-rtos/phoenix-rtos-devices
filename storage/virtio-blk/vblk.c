/*
 * Phoenix-RTOS
 *
 * VirtIO block device driver
 *
 * Copyright 2020, 2024 Phoenix Systems
 * Author: Lukasz Kosinski, Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/threads.h>

#include "vblk.h"


typedef struct _virtioblk_req_t {
	/* Request buffers (accessible by device) */
	struct {
		/* Request header (device read-only) */
		uint32_t type;     /* Request type */
		uint32_t reserved; /* Reserved field (previously request priority) */
		uint64_t sector;   /* Starting sector (512-byte offset) */

		/* Footer (device write-only) */
		volatile uint8_t status; /* Returned status */
	} __attribute__((packed)) descr;

	/* VirtIO request segments */
	virtio_seg_t hdr;  /* Header segment */
	virtio_seg_t data; /* Data segment */
	virtio_seg_t ftr;  /* Footer segment */
	virtio_req_t vreq; /* VirtIO request */

	/* Custom helper fields */
	volatile unsigned int len; /* Number of bytes written to request buffers */
	size_t buffsz;             /* Size of physicallly contiguous data buffer */
	void *buff;                /* Physically contiguous data buffer */
} virtioblk_req_t;


/* Resizes physically contiguous request data buffer */
static int _vblk_resizeBuff(virtioblk_req_t *req, size_t len)
{
	size_t buffsz;
	void *buff;

	if (len > req->buffsz) {
		buffsz = (len + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1);
		buff = mmap(NULL, buffsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS | MAP_CONTIGUOUS, -1, 0);
		if (buff == MAP_FAILED) {
			return -ENOMEM;
		}

		munmap(req->buff, req->buffsz);
		req->buffsz = buffsz;
		req->buff = buff;
	}

	return EOK;
}


/* Opens request context */
virtioblk_req_t *vblk_open(void)
{
	virtioblk_req_t *req;
	req = mmap(NULL, (sizeof(virtioblk_req_t) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1), PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS | MAP_CONTIGUOUS, -1, 0);
	if (req == MAP_FAILED) {
		return NULL;
	}

	req->buffsz = _PAGE_SIZE;
	req->buff = mmap(NULL, req->buffsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS | MAP_CONTIGUOUS, -1, 0);
	if (req->buff == MAP_FAILED) {
		munmap(req, (sizeof(virtioblk_req_t) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1));
		return NULL;
	}

	req->hdr.buff = &req->descr.type;
	req->hdr.len = 0x10;
	req->hdr.prev = &req->ftr;
	req->hdr.next = &req->data;
	req->data.prev = &req->hdr;
	req->data.next = &req->ftr;
	req->ftr.buff = (void *)&req->descr.status;
	req->ftr.len = 0x01;
	req->ftr.prev = &req->data;
	req->ftr.next = &req->hdr;
	req->vreq.segs = &req->hdr;

	return req;
}


/* Closes request context */
void vblk_close(virtioblk_req_t *req)
{
	munmap(req->buff, req->buffsz);
	munmap(req, (sizeof(virtioblk_req_t) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1));
}


/* Sends request to device */
static int _vblk_send(struct _storage_devCtx_t *vblk, virtioblk_req_t *req)
{
	virtio_dev_t *vdev = &vblk->vdev;
	int err;

	req->len = 0;
	err = virtqueue_enqueue(vdev, &vblk->vq, &req->vreq);
	if (err < 0) {
		return err;
	}

	virtqueue_notify(vdev, &vblk->vq);

	unsigned int len;
	do {
		req = virtqueue_dequeue(vdev, &vblk->vq, &len);
	} while (req == NULL);

	req->len = len;

	return (req->descr.status != 0) ? -EFAULT : EOK;
}


/* Reads data from device */
static ssize_t vblk_read(storage_t *strg, off_t offs, void *buff, size_t len)
{
	if (len == 0) {
		return 0;
	}

	struct _storage_devCtx_t *vblk = strg->dev->ctx;
	if (offs + len > vblk->size) {
		if (offs > vblk->size) {
			return -EINVAL;
		}
		len = vblk->size - offs;
	}

	if (((offs % 512) != 0) || ((len % 512) != 0)) {
		return -EINVAL;
	}

	mutexLock(vblk->lock);

	virtioblk_req_t *req = vblk->req;
	ssize_t ret = _vblk_resizeBuff(req, len);

	if (ret < 0) {
		mutexUnlock(vblk->lock);
		return ret;
	}

	req->descr.type = virtio_gtov32(&vblk->vdev, 0);
	req->descr.sector = virtio_gtov64(&vblk->vdev, offs / 512);
	req->data.buff = req->buff;
	req->data.len = len;
	req->vreq.rsegs = 1;
	req->vreq.wsegs = 2;

	if (_vblk_send(vblk, req) < 0) {
		ret = -EIO;
	}
	else {
		ret = req->len - 1;
		memcpy(buff, req->buff, ret);
	}

	mutexUnlock(vblk->lock);

	return ret;
}


/* Writes data to device */
static ssize_t vblk_write(storage_t *strg, off_t offs, const void *buff, size_t len)
{
	struct _storage_devCtx_t *vblk = strg->dev->ctx;
	if (offs + len > vblk->size) {
		if (offs > vblk->size) {
			return -EINVAL;
		}
		len = vblk->size - offs;
	}

	if (((offs % 512) != 0) || ((len % 512) != 0)) {
		return -EINVAL;
	}

	mutexLock(vblk->lock);

	virtioblk_req_t *req = vblk->req;
	ssize_t ret = _vblk_resizeBuff(req, len);
	if (ret < 0) {
		mutexUnlock(vblk->lock);
		return ret;
	}

	req->descr.type = virtio_gtov32(&vblk->vdev, 1);
	req->descr.sector = virtio_gtov64(&vblk->vdev, offs / 512);
	req->data.buff = req->buff;
	req->data.len = len;
	req->vreq.rsegs = 2;
	req->vreq.wsegs = 1;
	memcpy(req->buff, buff, len);

	ret = (_vblk_send(vblk, req) < 0) ? -EIO : len;

	mutexUnlock(vblk->lock);

	return ret;
}


static const storage_blkops_t blkOps = {
	.read = vblk_read,
	.write = vblk_write,
	.sync = NULL
};


const storage_blkops_t *vblk_getBlkOps(void)
{
	return &blkOps;
}


/* Destroys device */
void vblk_ctxDestroy(struct _storage_devCtx_t *ctx)
{
	virtio_dev_t *vdev = &ctx->vdev;

	resourceDestroy(ctx->lock);
	virtqueue_destroy(vdev, &ctx->vq);
	virtio_destroyDev(vdev);
	free(ctx);
}


int vblk_ctxInit(struct _storage_devCtx_t **ctx, virtio_dev_t *vdev)
{
	int ret = virtio_initDev(vdev);
	if (ret < 0) {
		return ret;
	}

	ret = virtio_writeFeatures(vdev, 0);
	if (ret < 0) {
		virtio_writeStatus(vdev, virtio_readStatus(vdev) | (1 << 7));
		virtio_destroyDev(vdev);
		return ret;
	}

	*ctx = malloc(sizeof(struct _storage_devCtx_t));
	if (*ctx == NULL) {
		return -ENOMEM;
	}

	ret = virtqueue_init(vdev, &(*ctx)->vq, 0, 128);
	if (ret < 0) {
		virtio_writeStatus(vdev, virtio_readStatus(vdev) | (1 << 7));
		virtio_destroyDev(vdev);
		free(*ctx);
		return ret;
	}

	(*ctx)->sectorsz = 512;
	(*ctx)->size = 512 * virtio_readConfig64(vdev, 0x00);

	ret = mutexCreate(&(*ctx)->lock);
	if (ret < 0) {
		virtqueue_destroy(vdev, &(*ctx)->vq);
		virtio_writeStatus(vdev, virtio_readStatus(vdev) | (1 << 7));
		virtio_destroyDev(vdev);
		free(*ctx);
		return ret;
	}

	virtqueue_disableIRQ(&(*ctx)->vdev, &(*ctx)->vq);
	virtio_writeStatus(vdev, virtio_readStatus(vdev) | (1 << 2));

	return EOK;
}
