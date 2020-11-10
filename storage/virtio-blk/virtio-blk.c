/*
 * Phoenix-RTOS
 *
 * VirtIO block device driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>

#include <sys/interrupt.h>
#include <sys/list.h>
#include <sys/threads.h>

#include "virtio-blk.h"


/* VirtIO block device features mask */
#define FEATURE_BLK_MSK \
	(FEATURE_MSK(FEAT_BARRIER)  | FEATURE_MSK(FEAT_MAX_SIZE) | FEATURE_MSK(FEAT_MAX_SEG) | FEATURE_MSK(FEAT_GEOMETRY) | \
	 FEATURE_MSK(FEAT_RO)       | FEATURE_MSK(FEAT_BLK_SIZE) | FEATURE_MSK(FEAT_SCSI)    | FEATURE_MSK(FEAT_FLUSH)    | \
	 FEATURE_MSK(FEAT_TOPOLOGY) | FEATURE_MSK(FEAT_WCE)      | FEATURE_MSK(FEAT_DISCARD) | FEATURE_MSK(FEAT_ZEROES))

/* Driver supported VirtIO block device features mask */
#define FEATURE_DRV_MSK (0)

/* VirtIO block device and driver common features mask */
#define FEATURE_DEV_MSK (((-1ULL) & ~(FEATURE_BLK_MSK)) | FEATURE_DRV_MSK)


virtio_blk_common_t virtio_blk_common;


/* Interrupt handler */
static int virtio_blk_int(unsigned int n, void *arg)
{
	virtio_blk_t *bdev = (virtio_blk_t *)arg;

	return bdev->cond;
}


/* Interrupt thread */
static void virtio_blk_intthr(void *arg)
{
	virtio_blk_t *bdev = (virtio_blk_t *)arg;
	virtq_t *vq = virtio_getVirtq(&bdev->mdev.vdev, 0);
	uint16_t i, used, d1, d2, d3;
	virtio_blkreq_t *req;
	uint32_t status;

	mutexLock(bdev->lock);

	for (;;) {
		while (!((status = (uint32_t)virtio_mmio_readReg(&bdev->mdev, REG_IRQ_STATUS, 4)) & 0x1))
			condWait(bdev->cond, bdev->lock, 0);

		/* Acknowledge the interrupt */
		virtio_mmio_writeReg(&bdev->mdev, REG_IRQ_ACK, 4, status);

		/* Handle processed requests */
		used = (uint16_t)virtio_toGuest(&bdev->mdev.vdev, vq->used->idx, 2) % vq->size;
		for (i = vq->last; i != used; i = (i + 1) % vq->size) {
			d1 = (uint16_t)virtio_toGuest(&bdev->mdev.vdev, vq->used->ring[i].id, 4);
			d2 = (uint16_t)virtio_toGuest(&bdev->mdev.vdev, vq->desc[d1].next, 2);
			d3 = (uint16_t)virtio_toGuest(&bdev->mdev.vdev, vq->desc[d2].next, 2);
			req = vq->vdesc[d1];

			/* Free request descriptors */
			virtio_freeDesc(&bdev->mdev.vdev, vq, d1);
			virtio_freeDesc(&bdev->mdev.vdev, vq, d2);
			virtio_freeDesc(&bdev->mdev.vdev, vq, d3);

			/* Save request buffer processed length */
			/* Legacy devices report this value unreliably. Assume full request completion */
			req->msg->o.io.err = (req->status == RSTATUS_OK) ? (uint32_t)virtio_toGuest(&bdev->mdev.vdev, (virtio_legacy(&bdev->mdev.vdev)) ? vq->desc[d2].len : vq->used->ring[i].len, 4) : -EIO;

			msgRespond(req->port, req->msg, req->rid);
			free(req);
		}
		vq->last = used;
	}
}


/* Sends request to device */
static int virtio_blk_sendRequest(virtio_blk_t *bdev, virtio_blkreq_t *req)
{
	virtq_t *vq = virtio_getVirtq(&bdev->mdev.vdev, 0);
	int d1, d2, d3;
	uint16_t dmode;
	uint32_t len;
	void *buff;

	switch (req->type) {
	case CMD_READ:
		buff = req->msg->o.data;
		len = req->msg->o.size;
		dmode = DESC_WRITE;
		break;

	case CMD_WRITE:
		buff = req->msg->i.data;
		len = req->msg->i.size;
		dmode = 0;
		break;

	default:
		return -EINVAL;
	}

	/* Allocate request descriptors */
	if ((d1 = virtio_allocDesc(&bdev->mdev.vdev, vq, req)) < 0)
		return d1;

	if ((d2 = virtio_allocDesc(&bdev->mdev.vdev, vq, buff)) < 0) {
		virtio_freeDesc(&bdev->mdev.vdev, vq, d1);
		return d2;
	}

	if ((d3 = virtio_allocDesc(&bdev->mdev.vdev, vq, &req->status)) < 0) {
		virtio_freeDesc(&bdev->mdev.vdev, vq, d1);
		virtio_freeDesc(&bdev->mdev.vdev, vq, d2);
		return d3;
	}

	/* Fill out request descriptors */
	vq->desc[d1].len   = (uint32_t)virtio_toHost(&bdev->mdev.vdev, REQ_HEADER_SIZE, 4);
	vq->desc[d1].flags = (uint16_t)virtio_toHost(&bdev->mdev.vdev, DESC_NEXT, 2);
	vq->desc[d1].next  = (uint16_t)virtio_toHost(&bdev->mdev.vdev, d2, 2);
	vq->desc[d2].len   = (uint32_t)virtio_toHost(&bdev->mdev.vdev, len, 4);
	vq->desc[d2].flags = (uint16_t)virtio_toHost(&bdev->mdev.vdev, dmode | DESC_NEXT, 2);
	vq->desc[d2].next  = (uint16_t)virtio_toHost(&bdev->mdev.vdev, d3, 2);
	vq->desc[d3].len   = (uint32_t)virtio_toHost(&bdev->mdev.vdev, REQ_FOOTER_SIZE, 4);
	vq->desc[d3].flags = (uint16_t)virtio_toHost(&bdev->mdev.vdev, DESC_WRITE, 2);

	/* Insert request to virtqueue */
	vq->avail->ring[vq->avail->idx % vq->size] = (uint16_t)virtio_toHost(&bdev->mdev.vdev, d1, 2);
	virtio_memoryBarrier();

	/* Update avail index */
	vq->avail->idx = (uint16_t)virtio_toHost(&bdev->mdev.vdev, (uint16_t)virtio_toGuest(&bdev->mdev.vdev, vq->avail->idx, 2) + 1, 2);
	virtio_memoryBarrier();

	/* Notify the device - write virtqueue index */
	virtio_mmio_writeReg(&bdev->mdev, REG_QUEUE_NOTIFY, 4, 0);

	return EOK;
}


// static ssize_t virtio_blk_access(virtio_blk_t *bdev, msg_t *msg)
// {
// 	virtio_blkreq_t *req;
// 	ssize_t ret;

// 	if (!len)
// 		return 0;

// 	if ((req = _virtio_blk_sendRequest(bdev, type, offs / dev->sectorsz, buff, len)) == NULL)
// 		return -EIO;

// 	return ret;
// }


// ssize_t virtio_blk_read(virtio_blk_t *bdev, unsigned int port, unsigned long rid, msg_t *msg)
// {
// 	return virtio_blk_access(bdev, CMD_READ, port, msg, offs, (uint8_t *)buff, len);
// }


// ssize_t virtio_blk_write(virtio_blk_t *bdev, unsigned int port, unsigned long rid, msg_t *msg)
// {
// 	return virtio_blk_access(bdev, CMD_WRITE, offs, (uint8_t *)buff, len);
// }


static void virtio_blk_destroyDev(virtio_blk_t *bdev)
{
	/* Destory resources */
	resourceDestroy(bdev->lock);
	resourceDestroy(bdev->cond);
	resourceDestroy(bdev->inth);

	/* Reset and destroy device */
	virtio_mmio_writeReg(&bdev->mdev, REG_STATUS, 4, 0);
	virtio_mmio_destroyDev(&bdev->mdev);
}


static int virtio_blk_initDev(virtio_blk_t *bdev, addr_t addr, unsigned int irq)
{
	int err;

	if ((err = virtio_mmio_initDev(&bdev->mdev, addr)) < 0) {
		/* Set initialization failed status bit */
		virtio_mmio_writeReg(&bdev->mdev, REG_STATUS, 4, virtio_mmio_readReg(&bdev->mdev, REG_STATUS, 4) | CSTATUS_FAILED);
		return err;
	}

	do {
		/* Check if it is VirtIO block device */
		if (bdev->mdev.vdev.device != DEV_BLK) {
			err = -ENXIO;
			break;
		}

		/* Select driver supported features and finalize feature negotiation */
		if ((err = virtio_mmio_writeFeatures(&bdev->mdev, bdev->mdev.vdev.features & FEATURE_DRV_MSK)) < 0)
			break;
		bdev->mdev.vdev.features &= FEATURE_DEV_MSK;

		/* Initialize device */
		if ((err = virtio_mmio_addVirtq(&bdev->mdev, 128)) < 0)
			break;

		bdev->sectorsz = SECTOR_SIZE;
		bdev->size = bdev->sectorsz * virtio_mmio_readConfig(&bdev->mdev, CONFIG_CAPACITY, 8);
		bdev->irq = irq;
		bdev->prev = NULL;
		bdev->next = NULL;

		if ((err = mutexCreate(&bdev->lock)) < 0)
			break;

		if ((err = condCreate(&bdev->cond)) < 0) {
			resourceDestroy(bdev->lock);
			break;
		}

		/* Run interrupt thread */
		if ((err = beginthread(virtio_blk_intthr, 4, bdev->stack, sizeof(bdev->stack), bdev)) < 0) {
			resourceDestroy(bdev->cond);
			resourceDestroy(bdev->lock);
			break;
		}

		/* Attach interrupt */
		if ((err = interrupt(bdev->irq, virtio_blk_int, bdev, bdev->cond, &bdev->inth)) < 0) {
			resourceDestroy(bdev->cond);
			resourceDestroy(bdev->lock);
			break;
		}

		/* Write finished initialization status */
		virtio_mmio_writeReg(&bdev->mdev, REG_STATUS, 4, virtio_mmio_readReg(&bdev->mdev, REG_STATUS, 4) | CSTATUS_DRV_OK);
		/* Add device to VirtIO block devices list */
		LIST_ADD(&virtio_blk_common.devs, bdev);

		return virtio_blk_common.ndevs++;
	} while (0);

	/* Set initialization failed status bit and destroy device */
	virtio_mmio_writeReg(&bdev->mdev, REG_STATUS, 4, virtio_mmio_readReg(&bdev->mdev, REG_STATUS, 4) | CSTATUS_FAILED);
	virtio_mmio_destroyDev(&bdev->mdev);

	return err;
}


void virtio_blk_destroy(void)
{
	virtio_blk_t *bdev;

	while ((bdev = virtio_blk_common.devs) != NULL) {
		LIST_REMOVE(&virtio_blk_common.devs, bdev);
		virtio_blk_destroyDev(bdev);
	}

	virtio_blk_common.ndevs = 0;
}


int virtio_blk_init(void)
{
	virtio_blk_t *bdev;
	int err;

	virtio_blk_common.ndevs = 0;
	virtio_blk_common.devs = NULL;

	if ((bdev = malloc(sizeof(virtio_blk_t))) == NULL)
		return -ENOMEM;

	/* Detect and initialize VirtIO MMIO block devices */
	if ((err = virtio_blk_initDev(bdev, 0x10008000, 8)) < 0) {
		free(bdev);
		if (err != -ENXIO)
			return err;
	}

	return EOK;
}
