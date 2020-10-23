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
#define FEAT_BLK_MASK \
	(FEAT_BARRIER  | FEAT_MAX_SIZE | FEAT_MAX_SEG | FEAT_GEOMETRY | \
	 FEAT_RO       | FEAT_BLK_SIZE | FEAT_SCSI    | FEAT_FLUSH    | \
	 FEAT_TOPOLOGY | FEAT_WCE      | FEAT_DISCARD | FEAT_ZEROES)

/* Driver supported VirtIO block device features mask */
#define FEAT_DRV_MASK (0)

/* VirtIO block device and driver common features mask */
#define FEAT_DEV_MASK (((-1ULL) & ~(FEAT_BLK_MASK)) | FEAT_DRV_MASK)


virtio_blk_common_t virtio_blk_common;


/* Interrupt handler */
static int virtio_blk_interrupt(unsigned int n, void *arg)
{
	virtio_blk_t *bdev = (virtio_blk_t *)arg;

	return bdev->cond;
}


/* Interrupt thread */
static void virtio_blk_interruptThread(void *arg)
{
	virtio_blk_t *bdev = (virtio_blk_t *)arg;
	virtq_t *vq = virtio_getVirtq(&bdev->mdev.vdev, 0);
	uint16_t i, used, d1, d2, d3;
	uint32_t status;
	virtio_blkreq_t *req;

	mutexLock(bdev->lock);
	for (;;) {
		while (!((status = (uint32_t)virtio_mmio_readReg(&bdev->mdev, REG_IRQ_STATUS, 4)) & 0x1))
			condWait(bdev->cond, bdev->lock, 0);

		/* Acknowledge the interrupt */
		virtio_mmio_writeReg(&bdev->mdev, REG_IRQ_ACK, 4, status);

		/* Handle processed requests */
		used = vq->used->idx % vq->size;
		for (i = vq->last; i != used; i = (i + 1) % vq->size) {
			d1 = (uint16_t)vq->used->ring[i].id;
			d2 = vq->desc[d1].next;
			d3 = vq->desc[d2].next;

			req = vq->vdesc[d1];
			virtio_freeDesc(vq, d1);
			virtio_freeDesc(vq, d2);
			virtio_freeDesc(vq, d3);

			if (req->status == RSTATUS_OK) {
				/* Notify thread waiting for the request! */
			}

			free(req);
		}
		vq->last = used;
	}
}


/* Sends request to device */
int virtio_blk_sendRequest(virtio_blk_t *bdev, uint32_t type, uint64_t sector, char *buff, uint32_t len)
{
	virtq_t *vq = virtio_getVirtq(&bdev->mdev.vdev, 0);
	virtio_blkreq_t *req;
	uint16_t dmode;
	int d1, d2, d3;

	switch (type) {
	case CMD_READ:
		dmode = 0;
		break;

	case CMD_WRITE:
		dmode = DESC_WRITE;
		break;

	default:
		return -EINVAL;
	}

	/* Allocate the request */
	if ((req = malloc(sizeof(virtio_blkreq_t))) == NULL)
		return -ENOMEM;

	if ((d1 = virtio_allocDesc(vq, req)) < 0) {
		free(req);
		return d1;
	}

	if ((d2 = virtio_allocDesc(vq, buff)) < 0) {
		virtio_freeDesc(vq, d1);
		free(req);
		return d2;
	}

	if ((d3 = virtio_allocDesc(vq, &req->status)) < 0) {
		virtio_freeDesc(vq, d1);
		virtio_freeDesc(vq, d2);
		free(req);
		return d3;
	}

	/* Fill out the request */
	req->type = type;
	req->sector = sector;

	vq->desc[d1].len = REQ_HEADER;
	vq->desc[d1].flags = DESC_NEXT;
	vq->desc[d1].next = (uint16_t)d2;
	vq->desc[d2].len = len;
	vq->desc[d2].flags = dmode | DESC_NEXT;
	vq->desc[d2].next = (uint16_t)d3;
	vq->desc[d3].len = REQ_FOOTER;
	vq->desc[d3].flags = DESC_WRITE;

	/* Insert request to virtqueue */
	vq->avail->ring[vq->avail->idx % vq->size] = (uint16_t)d1;
	virtio_memoryBarrier();

	/* Update avail index */
	vq->avail->idx++;
	virtio_memoryBarrier();

	/* Notify the device - write virtqueue index */
	virtio_mmio_writeReg(&bdev->mdev, REG_QUEUE_NOTIFY, 4, 0);

	return EOK;
}


ssize_t virtio_blk_read(virtio_blk_t *dev, offs_t offs, char *buff, size_t len)
{
	ssize_t ret;

	if (offs > dev->size)
		return -EINVAL;

	if (offs + len > dev->size)
		len = dev->size - offs;

	if (!len)
		return 0;

	//ret = _virtio_blk_read();

	return ret;
}


ssize_t virtio_blk_write(virtio_blk_t *dev, offs_t offs, const char *buff, size_t len)
{
	ssize_t ret;

	if (!len)
		return 0;

	if (offs > dev->size)
		return -EINVAL;

	if (offs + len > dev->size)
		len = dev->size - offs;

	if (!len)
		return 0;

	//ret = _virtio_blk_write();

	return ret;
}


static void virtio_blk_destroyDev(virtio_blk_t *bdev)
{
	resourceDestroy(bdev->lock);
	resourceDestroy(bdev->cond);
	resourceDestroy(bdev->inth);
	virtio_mmio_destroyDev(&bdev->mdev);
}


static int virtio_blk_initDev(virtio_blk_t *bdev, addr_t addr, unsigned int irq)
{
	int err;

	if ((err = virtio_mmio_initDev(&bdev->mdev, addr)) < 0)
		return err;

	do {
		/* Check if it is VirtIO block device */
		if (bdev->mdev.vdev.id.device != DEV_BLK) {
			err = -ENXIO;
			break;
		}

		/* Select driver supported features and finalize feature negotiation */
		if ((err = virtio_mmio_writeFeatures(&bdev->mdev, bdev->mdev.vdev.features & FEAT_DRV_MASK)) < 0)
			break;
		bdev->mdev.vdev.features &= FEAT_DEV_MASK;

		/* Initialize virtqueues */
		if ((err = virtio_mmio_addVirtq(&bdev->mdev, 128)) < 0)
			break;

		bdev->size = 512 * virtio_mmio_readConfig(&bdev->mdev, CONFIG_CAPACITY, 8);
		bdev->irq = irq;
		bdev->prev = NULL;
		bdev->next = NULL;

		if ((err = mutexCreate(&bdev->lock)) < 0)
			break;

		if ((err = condCreate(&bdev->cond)) < 0) {
			resourceDestroy(bdev->lock);
			break;
		}

		/* Attach interrupt */
		if ((err = interrupt(bdev->irq, virtio_blk_interrupt, bdev, bdev->cond, &bdev->inth)) < 0) {
			resourceDestroy(bdev->lock);
			resourceDestroy(bdev->cond);
			break;
		}

		/* Write finished initialization status */
		virtio_mmio_writeReg(&bdev->mdev, REG_STATUS, 4, virtio_mmio_readReg(&bdev->mdev, REG_STATUS, 4) | CSTATUS_DRV_OK);

		/* Add device to VirtIO block devices list */
		LIST_ADD(&virtio_blk_common.devs, bdev);

		return virtio_blk_common.ndevs++;
	} while (0);

	virtio_mmio_destroyDev(&bdev->mdev);

	return err;
}


void virtio_blk_destroy(void)
{
	virtio_blk_t *nbdev, *bdev = virtio_blk_common.devs;
	unsigned int i;

	for (i = 0; i < virtio_blk_common.ndevs; i++) {
		nbdev = bdev->next;
		virtio_blk_destroyDev(bdev);
		bdev = nbdev;
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

	/* Detect VirtIO MMIO block devices */
	if ((err = virtio_blk_initDev(bdev, 0x10008000, 8)) < 0) {
		free(bdev);
		if (err != -ENXIO)
			return err;
	}

	return EOK;
}
