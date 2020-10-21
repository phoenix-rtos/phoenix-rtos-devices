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

#include "virtio-blk.h"


/* VirtIO block device features */
#define FEATURES_BLK \
	(FEAT_BARRIER  | FEAT_MAX_SIZE | FEAT_MAX_SEG | FEAT_GEOMETRY | \
	 FEAT_RO       | FEAT_BLK_SIZE | FEAT_SCSI    | FEAT_FLUSH    | \
	 FEAT_TOPOLOGY | FEAT_WCE      | FEAT_DISCARD | FEAT_ZEROES)

/* VirtIO block device features supported by the driver */
#define FEATURES_DRV (0)

/* VirtIO block device and driver common features mask */
#define FEATURES_DEV (((-1ULL) & ~(FEATURES_BLK)) | FEATURES_DRV)


virtio_blk_common_t virtio_blk_common;


/* Interrupt handler */
static int virtio_blk_interrupt(unsigned int n, void *arg)
{
	virtio_blk_t *bdev = (virtio_blk_t *)arg;
	virtq_t *vq = virtio_getVirtq(&bdev->mdev.vdev, 0);
	uint16_t i;

	/* Acknowledge interrupt */
	virtio_mmio_writeReg(&bdev->mdev, REG_IRQ_ACK, 4, virtio_mmio_readReg(&bdev->mdev, REG_IRQ_STATUS, 4));

	for (i = vq->last; i != vq->used->idx % vq->size; i = (i + 1) % vq->size) {
		/* Handle used requests */
	}
}


ssize_t virtio_blk_read(virtio_blk_t *dev, offs_t offs, char *buff, size_t len)
{
	// ssize_t ret;

	// if (offs > dev->size)
	// 	return -EINVAL;

	// if (offs + len > dev->size)
	// 	len = dev->size - offs;

	// if (!len)
	// 	return 0;
	return 0;
	// mutexLock(dev->lock);

	// ret = _virtio_blk_read();

	// mutexUnlock(dev->lock);

	// return ret;
}


ssize_t virtio_blk_write(virtio_blk_t *dev, offs_t offs, const char *buff, size_t len)
{
	// ssize_t ret;

	// if (!len)
	// 	return 0;

	// if (offs > dev->size)
	// 	return -EINVAL;

	// if (offs + len > dev->size)
	// 	len = dev->size - offs;

	// if (!len)
	// 	return 0;
	return 0;
	// mutexLock(dev->lock);

	// ret = _virtio_blk_write();

	// mutexUnlock(dev->lock);

	// return ret;
}


int virtio_blk_initDev(virtio_blk_t *bdev, addr_t addr, unsigned int irq)
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
		if ((err = virtio_mmio_writeFeatures(&bdev->mdev, bdev->mdev.vdev.features & FEATURES_DRV)) < 0)
			break;
		bdev->mdev.vdev.features &= FEATURES_DEV;

		/* Initialize virtqueues */
		if ((err = virtio_mmio_addVirtq(&bdev->mdev, 256)) < 0)
			break;

		bdev->size = 512 * virtio_mmio_readConfig(&bdev->mdev, CONFIG_CAPACITY, 8);
		bdev->irq = irq;
		bdev->prev = NULL;
		bdev->next = NULL;

		/* Attach interrupt */
		if ((err = interrupt(bdev->irq, virtio_blk_interrupt, bdev, bdev->cond, &bdev->inth)) < 0)
			break;

		/* Write finished initialization status */
		virtio_mmio_writeReg(&bdev->mdev, REG_STATUS, 4, virtio_mmio_readReg(&bdev->mdev, REG_STATUS, 4) | CSTATUS_DRV_OK);

		/* Add device to VirtIO block devices list */
		LIST_ADD(&virtio_blk_common.devs, bdev);

		return virtio_blk_common.ndevs++;
	} while (0);

	virtio_mmio_destroyDev(&bdev->mdev);

	return err;
}


//int virtio_blk_init(void)
int main()
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
