/*
 * Phoenix-RTOS
 *
 * VirtIO MMIO device
 *
 * Copyright 2020 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <endian.h>
#include <errno.h>
#include <stdlib.h>

#include <sys/list.h>
#include <sys/mman.h>

#include "virtio-mmio.h"


uint64_t virtio_mmio_readReg(virtio_mmio_t *mdev, uint32_t reg, uint8_t size)
{
	uintptr_t addr = (uintptr_t)mdev->base + reg;
	uint64_t ret;

	switch (size) {
	case 1:
		ret = *(volatile uint8_t *)addr;
		break;

	case 2:
		ret = le16toh(*(volatile uint16_t *)addr);
		break;

	case 4:
		ret = le32toh(*(volatile uint32_t *)addr);
		break;

	case 8:
		ret = le64toh(*(volatile uint64_t *)addr);
		break;

	default:
		return -EINVAL;
	}

	virtio_memoryBarrier();

	return ret;
}


void virtio_mmio_writeReg(virtio_mmio_t *mdev, uint32_t reg, uint8_t size, uint64_t val)
{
	uintptr_t addr = (uintptr_t)mdev->base + reg;

	switch (size) {
	case 1:
		*(volatile uint8_t *)addr = (uint8_t)val;
		break;

	case 2:
		*(volatile uint16_t *)addr = htole16((uint16_t)val);
		break;

	case 4:
		*(volatile uint32_t *)addr = htole32((uint32_t)val);
		break;

	case 8:
		*(volatile uint64_t *)addr = htole64(val);
		break;

	default:
		return;
	}

	virtio_memoryBarrier();
}


uint64_t virtio_mmio_readFeatures(virtio_mmio_t *mdev)
{
	uint64_t features;

	virtio_mmio_writeReg(mdev, REG_DEV_FEAT_SEL, 4, 1);
	features = virtio_mmio_readReg(mdev, REG_DEV_FEAT, 4);
	features <<= 32;
	virtio_mmio_writeReg(mdev, REG_DEV_FEAT_SEL, 4, 0);
	features |= virtio_mmio_readReg(mdev, REG_DEV_FEAT, 4);

	return features;
}


int virtio_mmio_writeFeatures(virtio_mmio_t *mdev, uint64_t features)
{
	virtio_mmio_writeReg(mdev, REG_DRV_FEAT_SEL, 4, 1);
	virtio_mmio_writeReg(mdev, REG_DRV_FEAT, 4, features >> 32);
	virtio_mmio_writeReg(mdev, REG_DEV_FEAT_SEL, 4, 0);
	virtio_mmio_writeReg(mdev, REG_DRV_FEAT, 4, features & 0xffffffff);

	if (virtio_legacy(&mdev->vdev))
		return EOK;

	virtio_mmio_writeReg(mdev, REG_STATUS, 4, virtio_mmio_readReg(mdev, REG_STATUS, 4) | CSTATUS_FEAT_OK);
	/* Check if VirtIO device accepted our features */
	if (!(virtio_mmio_readReg(mdev, REG_STATUS, 4) & CSTATUS_FEAT_OK))
		return -EFAULT;

	return EOK;
}


uint64_t virtio_mmio_readConfig(virtio_mmio_t *mdev, uint32_t offs, uint8_t size)
{
	uint32_t gen1 = 0, gen2 = 0;
	uint64_t ret;

	if (virtio_legacy(&mdev->vdev)) {
		do {
			ret = virtio_mmio_readReg(mdev, REG_CONFIG + offs, size);
		} while (ret != virtio_mmio_readReg(mdev, REG_CONFIG + offs, size));
	}
	else {
		do {
			if (size > 4)
				gen1 = virtio_mmio_readReg(mdev, REG_CONFIG_GEN, 4);

			ret = virtio_mmio_readReg(mdev, REG_CONFIG + offs, size);

			if (size > 4)
				gen2 = virtio_mmio_readReg(mdev, REG_CONFIG_GEN, 4);
		} while (gen1 != gen2);
	}

	return ret;
}


void virtio_mmio_writeConfig(virtio_mmio_t *mdev, uint32_t offs, uint8_t size, uint64_t val)
{
	virtio_mmio_writeReg(mdev, REG_CONFIG + offs, size, val);
}


int virtio_mmio_addVirtq(virtio_mmio_t* mdev, uint16_t size)
{
	uint16_t maxsz;
	uint64_t addr;
	virtq_t *vq;

	/* Select next virtqueue slot */
	virtio_mmio_writeReg(mdev, REG_QUEUE_SEL, 4, mdev->vdev.nvqs);

	/* The slot should be empty */
	if (virtio_mmio_readReg(mdev, virtio_legacy(&mdev->vdev) ? REG_QUEUE_PFN : REG_QUEUE_READY, 4))
		return -EFAULT;

	/* Get max virtqueue size */
	if (!(maxsz = (uint16_t)virtio_mmio_readReg(mdev, REG_QUEUE_MAX, 4)))
		return -EFAULT;

	if (size > maxsz)
		size = maxsz;

	/* Allocate the virtqueue */
	if ((vq = virtio_allocVirtq(size)) == NULL)
		return -ENOMEM;

	/* Write virtqueue size */
	virtio_mmio_writeReg(mdev, REG_QUEUE_SIZE, 4, vq->size);

	if (virtio_legacy(&mdev->vdev)) {
		/* Legacy interface requires 32-bit descriptors page address */
		if ((addr = va2pa((void *)vq->desc) / _PAGE_SIZE) >> 32) {
			virtio_freeVirtq(vq);
			return -EFAULT;
		}

		virtio_mmio_writeReg(mdev, REG_QUEUE_ALIGN, 4, ALIGN_USED);
		virtio_mmio_writeReg(mdev, REG_QUEUE_PFN, 4, addr & 0xffffffff);
	}
	else {
		/* Write descriptors area physical address */
		addr = va2pa((void *)vq->desc);
		virtio_mmio_writeReg(mdev, REG_QUEUE_LDESC, 4, addr & 0xffffffff);
		virtio_mmio_writeReg(mdev, REG_QUEUE_HDESC, 4, addr >> 32);

		/* Write driver area physical address */
		addr = va2pa((void *)vq->avail);
		virtio_mmio_writeReg(mdev, REG_QUEUE_LDRV, 4, addr & 0xffffffff);
		virtio_mmio_writeReg(mdev, REG_QUEUE_HDRV, 4, addr >> 32);

		/* Write device area physical address */
		addr = va2pa((void *)vq->used);
		virtio_mmio_writeReg(mdev, REG_QUEUE_LDEV, 4, addr & 0xffffffff);
		virtio_mmio_writeReg(mdev, REG_QUEUE_HDEV, 4, addr >> 32);

		/* Write virtqueue ready */
		virtio_mmio_writeReg(mdev, REG_QUEUE_READY, 4, 1);
	}
	/* Add virtqueue to device virtqueues list */
	LIST_ADD(&mdev->vdev.vqs, vq);

	return mdev->vdev.nvqs++;
}


void virtio_mmio_destroyDev(virtio_mmio_t *mdev)
{
	munmap((void *)mdev->base, _PAGE_SIZE);
	virtio_destroyDev(&mdev->vdev);
}


int virtio_mmio_initDev(virtio_mmio_t *mdev, addr_t addr)
{
	uint32_t version;
	int err;

	/* Init VirtIO device */
	if ((err = virtio_initDev(&mdev->vdev)) < 0)
		return err;

	/* Map registers */
	if ((mdev->base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE, OID_PHYSMEM, addr)) == MAP_FAILED) {
		virtio_destroyDev(&mdev->vdev);
		return -ENOMEM;
	}

	do {
		/* Check VirtIO magic */
		if ((uint32_t)virtio_mmio_readReg(mdev, REG_MAGIC, 4) != VIRTIO_MAGIC) {
			err = -ENXIO;
			break;
		}

		/* Check device version */
		version = (uint32_t)virtio_mmio_readReg(mdev, REG_VERSION, 4);
		if ((version < 1) || (version > 2)) {
			err = -ENXIO;
			break;
		}

		/* Check device ID */
		if (!(mdev->vdev.id.device = (uint32_t)virtio_mmio_readReg(mdev, REG_DEV_ID, 4))) {
			err = -ENXIO;
			break;
		}
		mdev->vdev.id.vendor = (uint32_t)virtio_mmio_readReg(mdev, REG_VENDOR_ID, 4);

		/* Write guest page size (legacy devices use page-based addresing) */
		if (version == 1)
			virtio_mmio_writeReg(mdev, REG_PAGE_SIZE, 4, _PAGE_SIZE);

		/* Reset the device */
		virtio_mmio_writeReg(mdev, REG_STATUS, 4, 0);
		/* Acknowledge the device */
		virtio_mmio_writeReg(mdev, REG_STATUS, 4, virtio_mmio_readReg(mdev, REG_STATUS, 4) | CSTATUS_ACK);
		/* Pass further initialization to the driver (after saving features and veryfing device version) */
		virtio_mmio_writeReg(mdev, REG_STATUS, 4, virtio_mmio_readReg(mdev, REG_STATUS, 4) | CSTATUS_DRV);

		/* Save device features */
		mdev->vdev.features = virtio_mmio_readFeatures(mdev);

		/* Compare reported VirtIO version with supported features */
		if (virtio_legacy(&mdev->vdev) && (version == 2)) {
			err = -ENXIO;
			break;
		}

		return EOK;
	} while (0);

	virtio_mmio_destroyDev(mdev);

	return err;
}
