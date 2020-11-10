/*
 * Phoenix-RTOS
 *
 * VirtIO common definitions
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
#include <string.h>

#include <sys/mman.h>
#include <sys/threads.h>

#include "virtio.h"


#define ALIGN_MSK(addr, mask) (((addr) + (mask)) & ~(mask))
#define ALIGN(addr, align) ALIGN_MSK(addr, (typeof(addr))(align) - 1)


virtq_t *virtio_allocVirtq(uint16_t size)
{
	uint32_t doffs = ALIGN(0, ALIGN_DESC);
	uint32_t aoffs = ALIGN(doffs + size * sizeof(virtq_desc_t), ALIGN_AVAIL);
	uint32_t ueoffs = aoffs + sizeof(virtq_avail_t) + size * sizeof(uint16_t);
	uint32_t uoffs = ALIGN(ueoffs + sizeof(uint16_t), ALIGN_USED);
	uint32_t aeoffs = uoffs + sizeof(virtq_used_t) + size * sizeof(virtq_used_item_t);
	uint32_t vdoffs = ALIGN(aeoffs + sizeof(uint16_t), sizeof(void *));
	uint32_t vqsz = ALIGN(vdoffs + size * sizeof(void *), _PAGE_SIZE);
	virtq_t *vq;

	if ((vq = malloc(sizeof(virtq_t))) == NULL)
		return NULL;

	if ((vq->data = mmap(NULL, vqsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, OID_NULL, 0)) == MAP_FAILED) {
		free(vq);
		return NULL;
	}

	if (mutexCreate(&vq->lock) < 0) {
		munmap((void *)vq->data, vqsz);
		free(vq);
		return NULL;
	}

	memset((void *)vq->data, 0, vqsz);
	vq->desc = (virtq_desc_t *)((uintptr_t)vq->data + doffs);
	vq->avail = (virtq_avail_t *)((uintptr_t)vq->data + aoffs);
	vq->uevent = (uint16_t *)((uintptr_t)vq->data + ueoffs);
	vq->used = (virtq_used_t *)((uintptr_t)vq->data + uoffs);
	vq->aevent = (uint16_t *)((uintptr_t)vq->data + aeoffs);
	vq->vdesc = (void **)((uintptr_t)vq->data + vdoffs);

	vq->size = size;
	vq->last = 0;
	vq->free = 0;
	vq->prev = NULL;
	vq->next = NULL;

	return vq;
}


void virtio_freeVirtq(virtq_t *vq)
{
	resourceDestroy(vq->lock);
	munmap((void *)vq->data, ALIGN((uintptr_t)vq->vdesc - (uintptr_t)vq->data + vq->size * sizeof(void *), _PAGE_SIZE));
	free(vq);
}


int virtio_allocDesc(virtio_dev_t *vdev, virtq_t *vq, void *addr)
{
	uint16_t desc;
	uint16_t next;

	mutexLock(vq->lock);

	if ((desc = vq->free) == vq->size) {
		mutexUnlock(vq->lock);
		return -ENOSPC;
	}

	next = (uint16_t)virtio_toGuest(vdev, vq->desc[desc].next, 2);
	vq->desc[desc].addr = virtio_toHost(vdev, va2pa(addr), 8);
	vq->vdesc[desc] = addr;
	vq->free = next;

	mutexUnlock(vq->lock);

	return desc;
}


void virtio_freeDesc(virtio_dev_t *vdev, virtq_t *vq, uint16_t desc)
{
	mutexLock(vq->lock);

	vq->desc[desc].next = (uint16_t)virtio_toHost(vdev, vq->free, 2);
	vq->vdesc[desc] = NULL;
	vq->free = desc;

	mutexUnlock(vq->lock);
}


virtq_t *virtio_getVirtq(virtio_dev_t *vdev, uint8_t n)
{
	virtq_t *vq;
	uint8_t i;

	if (n + 1 > vdev->nvqs)
		return NULL;

	/* Go through virtqueues list in FIFO order */
	vq = vdev->vqs->prev;
	for (i = 0; i < n; i++)
		vq = vq->prev;

	return vq;
}


void virtio_destroyDev(virtio_dev_t *vdev)
{
	virtq_t *nvq, *vq = vdev->vqs;
	uint8_t i;

	for (i = 0; i < vdev->nvqs; i++) {
		nvq = vq->next;
		virtio_freeVirtq(vq);
		vq = nvq;
	}

	memset(vdev, 0, sizeof(virtio_dev_t));
}


int virtio_initDev(virtio_dev_t *vdev)
{
	memset(vdev, 0, sizeof(virtio_dev_t));
	return EOK;
}
