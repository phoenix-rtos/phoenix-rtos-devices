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

#ifndef _VIRTIO_MMIO_H_
#define _VIRTIO_MMIO_H_

#include <stdint.h>

#include <sys/types.h>

#include "virtio.h"


/* VirtIO device magic identifier ("virt") */
#define VIRTIO_MAGIC 0x74726976


/* VirtIO MMIO registers */
enum {
	REG_MAGIC        = 0x000, /* Read magic value */
	REG_VERSION      = 0x004, /* Read device version number */
	REG_DEV_ID       = 0x008, /* Read Virtio Subsystem Device ID */
	REG_VENDOR_ID    = 0x00c, /* Read Virtio Subsystem Vendor ID */
	REG_DEV_FEAT     = 0x010, /* Read device features */
	REG_DEV_FEAT_SEL = 0x014, /* Write device features selection */
	REG_DRV_FEAT     = 0x020, /* Write driver features */
	REG_DRV_FEAT_SEL = 0x024, /* Write driver features selection */
	REG_PAGE_SIZE    = 0x028, /* Write guest page size, LEGACY DEVICES ONLY! */ 
	REG_QUEUE_SEL    = 0x030, /* Write virtqueue selection index */
	REG_QUEUE_MAX    = 0x034, /* Read max virtqueue size */
	REG_QUEUE_SIZE   = 0x038, /* Write virtqueue size */
	REG_QUEUE_ALIGN  = 0x03c, /* Write used ring virtqueue alignment boundary, LEGACY DEVICES ONLY! */
	REG_QUEUE_PFN    = 0x040, /* Read/Write virtqueue guest physical page number, LEGACY DEVICES ONLY! */
	REG_QUEUE_READY  = 0x044, /* Read/Write virtqueue ready bit */
	REG_QUEUE_NOTIFY = 0x050, /* Write queue notification */
	REG_IRQ_STATUS   = 0x060, /* Read interrupt status */
	REG_IRQ_ACK      = 0x064, /* Write interrupt acknowledged */
	REG_STATUS       = 0x070, /* Read/Write device status */
	REG_QUEUE_LDESC  = 0x080, /* Write low 32-bits virtqueue descriptor area address */
	REG_QUEUE_HDESC  = 0x084, /* Write high 32-bits virtqueue descriptor area address */
	REG_QUEUE_LDRV   = 0x090, /* Write low 32-bits virtqueue driver area address */
	REG_QUEUE_HDRV   = 0x094, /* Write high 32-bits virtqueue driver area address */
	REG_QUEUE_LDEV   = 0x0a0, /* Write low 32-bits virtqueue device area address */
	REG_QUEUE_HDEV   = 0x0a4, /* Write high 32-bits virtqueue device area address */
	REG_CONFIG_GEN   = 0x0fc, /* Read configuration atomicity value */
	REG_CONFIG       = 0x100  /* Read/Write configuration space */
};


typedef struct {
	virtio_dev_t vdev;        /* VirtIO device */
	volatile void *base;      /* MMIO registers base address */
} virtio_mmio_t;


/* Reads from MMIO register */
extern uint64_t virtio_mmio_readReg(virtio_mmio_t *mdev, uint32_t reg, uint8_t size);


/* Writes to MMIO register */
extern void virtio_mmio_writeReg(virtio_mmio_t *mdev, uint32_t reg, uint8_t size, uint64_t val);


/* Reads VirtIO MMIO device features */
extern uint64_t virtio_mmio_readFeatures(virtio_mmio_t *mdev);


/* Writes feartures to VirtIO MMIO device */
extern int virtio_mmio_writeFeatures(virtio_mmio_t *mdev, uint64_t features);


/* Reads from MMIO configuration space */
extern uint64_t virtio_mmio_readConfig(virtio_mmio_t *mdev, uint32_t offs, uint8_t size);


/* Adds virtqueue to VirtIO MMIO device */
extern int virtio_mmio_addVirtq(virtio_mmio_t* mdev, uint16_t size);


/* Destroys VirtIO MMIO device */
extern void virtio_mmio_destroyDev(virtio_mmio_t *mdev);


/* Initializes VirtIO MMIO device */
extern int virtio_mmio_initDev(virtio_mmio_t *mdev, addr_t addr);


#endif
