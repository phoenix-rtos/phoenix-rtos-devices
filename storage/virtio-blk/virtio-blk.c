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
	virtq_used_item_t *item;
	virtio_blkreq_t *req;
	uint16_t i, used, d1, d2, d3;
	uint32_t status;

	mutexLock(bdev->lock);
	for (;;) {
		while (!((status = (uint32_t)virtio_mmio_readReg(&bdev->mdev, REG_IRQ_STATUS, 4)) & 0x1))
			condWait(bdev->cond, bdev->lock, 0);

		/* Acknowledge the interrupt */
		virtio_mmio_writeReg(&bdev->mdev, REG_IRQ_ACK, 4, status);

		/* Handle processed requests */
		used = vq->used->idx % vq->size;
		for (i = vq->last; i != used; i = (i + 1) % vq->size) {
			item = (virtq_used_item_t *)vq->used->ring + i;
			d1 = (uint16_t)item->id;
			d2 = vq->desc[d1].next;
			d3 = vq->desc[d2].next;
			req = vq->vdesc[d1];

			/* Save request buffer processed length */
			/* Legacy devices report this value unreliably - assume full request completion */
			req->len = (virtio_legacy(&bdev->mdev.vdev)) ? vq->desc[d2].len : item->len;
			/* Change request state */
			req->state = RSTATE_USED;

			mutexLock(bdev->rlock);

			/* Free request descriptors */
			_virtio_freeDesc(vq, d1);
			_virtio_freeDesc(vq, d2);
			_virtio_freeDesc(vq, d3);

			mutexUnlock(bdev->rlock);

			/* Notify threads waiting for request */
			condBroadcast(bdev->rcond);
		}
		vq->last = used;
	}
}


/* Sends request to device (should be protected with mutex) */
static virtio_blkreq_t *_virtio_blk_sendRequest(virtio_blk_t *bdev, uint32_t type, uint64_t sector, uint8_t *buff, uint32_t len)
{
	virtq_t *vq = virtio_getVirtq(&bdev->mdev.vdev, 0);
	virtio_blkreq_t *req;
	uint16_t dmode;
	int d1, d2, d3;

	switch (type) {
	case CMD_READ:
		dmode = DESC_WRITE;
		break;

	case CMD_WRITE:
		dmode = 0;
		break;

	default:
		return NULL;
	}

	/* Allocate request */
	if ((req = malloc(sizeof(virtio_blkreq_t))) == NULL)
		return NULL;

	do {
		/* Allocate request descriptors */
		if ((d1 = _virtio_allocDesc(vq, req)) < 0)
			break;

		if ((d2 = _virtio_allocDesc(vq, buff)) < 0) {
			_virtio_freeDesc(vq, d1);
			break;
		}

		if ((d3 = _virtio_allocDesc(vq, &req->status)) < 0) {
			_virtio_freeDesc(vq, d1);
			_virtio_freeDesc(vq, d2);
			break;
		}

		/* Fill out request */
		req->type = type;
		req->sector = sector;
		req->state = RSTATE_AVAIL;

		/* Fill out request descriptors */
		vq->desc[d1].len = REQ_HEADER_SIZE;
		vq->desc[d1].flags = DESC_NEXT;
		vq->desc[d1].next = (uint16_t)d2;
		vq->desc[d2].len = len;
		vq->desc[d2].flags = dmode | DESC_NEXT;
		vq->desc[d2].next = (uint16_t)d3;
		vq->desc[d3].len = REQ_FOOTER_SIZE;
		vq->desc[d3].flags = DESC_WRITE;

		/* Insert request to virtqueue */
		vq->avail->ring[vq->avail->idx % vq->size] = (uint16_t)d1;
		virtio_memoryBarrier();

		/* Update avail index */
		vq->avail->idx++;
		virtio_memoryBarrier();

		/* Notify the device - write virtqueue index */
		virtio_mmio_writeReg(&bdev->mdev, REG_QUEUE_NOTIFY, 4, 0);

		return req;
	} while(0);

	free(req);

	return NULL;
}


static ssize_t virtio_blk_access(virtio_blk_t *bdev, uint32_t type, uint64_t offs, uint8_t *buff, uint32_t len)
{
	virtio_blkreq_t *req;
	ssize_t ret;

	if (offs + len > bdev->size)
		len = bdev->size - offs;

	if ((offs > bdev->size) || (offs % SECTOR_SIZE) || (len % SECTOR_SIZE))
		return -EINVAL;

	if (!len)
		return 0;

	mutexLock(bdev->rlock);

	if ((req = _virtio_blk_sendRequest(bdev, type, offs / SECTOR_SIZE, buff, len)) == NULL)
		return -EIO;

	mutexUnlock(bdev->rlock);

	while (req->state != RSTATE_USED)
		condWait(bdev->rcond, bdev->rlock, 0);

	ret = (req->status != RSTATUS_OK) ? -EIO : req->len;
	free(req);

	return ret;
}


ssize_t virtio_blk_read(virtio_blk_t *bdev, offs_t offs, char *buff, size_t len)
{
	return virtio_blk_access(bdev, CMD_READ, offs, (uint8_t *)buff, len);
}


ssize_t virtio_blk_write(virtio_blk_t *bdev, offs_t offs, const char *buff, size_t len)
{
	return virtio_blk_access(bdev, CMD_WRITE, offs, (uint8_t *)buff, len);
}


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

		bdev->size = SECTOR_SIZE * virtio_mmio_readConfig(&bdev->mdev, CONFIG_CAPACITY, 8);
		bdev->irq = irq;
		bdev->prev = NULL;
		bdev->next = NULL;

		if ((err = mutexCreate(&bdev->rlock)) < 0)
			break;

		if ((err = condCreate(&bdev->rcond)) < 0) {
			resourceDestroy(bdev->rlock);
			break;
		}

		if ((err = mutexCreate(&bdev->lock)) < 0) {
			resourceDestroy(bdev->rlock);
			resourceDestroy(bdev->rcond);
			break;
		}

		if ((err = condCreate(&bdev->cond)) < 0) {
			resourceDestroy(bdev->rlock);
			resourceDestroy(bdev->rcond);
			resourceDestroy(bdev->lock);
			break;
		}

		/* Run interrupt thread */
		if ((err = beginthread(virtio_blk_interruptThread, 4, bdev->stack, sizeof(bdev->stack), bdev)) < 0) {
			resourceDestroy(bdev->rlock);
			resourceDestroy(bdev->rcond);
			resourceDestroy(bdev->cond);
			resourceDestroy(bdev->lock);
			break;
		}

		/* Attach interrupt */
		if ((err = interrupt(bdev->irq, virtio_blk_interrupt, bdev, bdev->cond, &bdev->inth)) < 0) {
			resourceDestroy(bdev->rlock);
			resourceDestroy(bdev->rcond);
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

	/* Set initialization failed status bit */
	virtio_mmio_writeReg(&bdev->mdev, REG_STATUS, 4, virtio_mmio_readReg(&bdev->mdev, REG_STATUS, 4) | CSTATUS_FAILED);
	/* Destroy device */
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


// int virtio_blk_status(virtio_blk_t *bdev)
// {
// 	printf("Status: %u\n", virtio_mmio_readReg(&bdev->mdev, REG_STATUS, 4));
// 	printf("DeviceID: %u\n", virtio_mmio_readReg(&bdev->mdev, REG_DEV_ID, 4));
// 	printf("VendorID: %u\n", virtio_mmio_readReg(&bdev->mdev, REG_VENDOR_ID, 4));
// 	printf("IRQstatus: %u\n", virtio_mmio_readReg(&bdev->mdev, REG_IRQ_STATUS, 4));
// 	printf("Avail idx: %u\n", bdev->mdev.vdev.vqs->avail->idx);
// 	printf("Used idx: %u\n", bdev->mdev.vdev.vqs->used->idx);
// }


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

	char buff[1024] = { 0 };
	offs_t offs = 12 * SECTOR_SIZE;
	ssize_t ret = virtio_blk_read(bdev, offs, buff, sizeof(buff));

	printf("\n");
	printf("Successfully read %d bytes at offset %d\n", ret, offs);
	for (int i = 256; i < sizeof(buff); i += 16) {
		for (int j = 0; j < 8; j++)
			printf("%02x%02x ", buff[i + 2 * j + 1],  buff[i + 2 * j]);
		printf("\n");
	}

	return EOK;
}
