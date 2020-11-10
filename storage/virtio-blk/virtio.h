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

#ifndef _VIRTIO_H_
#define _VIRTIO_H_

#include <endian.h>
#include <errno.h>
#include <stdint.h>

#include <sys/types.h>


/* VirtIO device ID */
enum {
	DEV_NET           = 0x01, /* Network card device */
	DEV_BLK           = 0x02, /* Block device */
	DEV_CONSOLE       = 0x03, /* Console device */
	DEV_ENTROPY       = 0x04, /* Entropy device */
	DEV_BALLOON_STD   = 0x05, /* Memory standard ballooning device */
	DEV_IOMEM         = 0x06, /* ioMemory device */
	DEV_RPMSG         = 0x07, /* rpmsg device */
	DEV_SCSI          = 0x08, /* SCSI host device */
	DEV_9P            = 0x09, /* 9P transport device */
	DEV_MAC80211      = 0x0a, /* mac80211 wlan device */
	DEV_RPROC         = 0x0b, /* rproc serial device */
	DEV_CAIF          = 0x0c, /* virtio CAIF device */
	DEV_BALLOON       = 0x0d, /* Memory ballon device */
	DEV_GPU           = 0x10, /* GPU device */
	DEV_CLOCK         = 0x11, /* Timer/clock device */
	DEV_INPUT         = 0x12, /* Input device */
	DEV_SOCKET        = 0x13, /* Socket device */
	DEV_CRYPTO        = 0x14, /* Crypto device */
	DEV_SIGNAL        = 0x15, /* Signal Distribution Module device */
	DEV_PSTORE        = 0x16, /* pstore device */
	DEV_IOMMU         = 0x17, /* IOMMU device */
	DEV_MEM           = 0x18  /* Memory device */
};


/* VirtIO device configuration status */
enum {
	CSTATUS_ACK       = 0x01, /* Virtio device detected */
	CSTATUS_DRV       = 0x02, /* Driver supports the device */
	CSTATUS_DRV_OK    = 0x04, /* Driver is ready to drive the device */
	CSTATUS_FEAT_OK   = 0x08, /* Driver finished feature negotiation */
	CSTATUS_RESET     = 0x40, /* Device entered invalid state, driver must reset it */
	CSTATUS_FAILED    = 0x80, /* Driver gave up on the device */
};


/* VirtIO virtqueue alignment requirements (legacy interface compatible) */
enum {
	ALIGN_DESC  = _PAGE_SIZE, /* Virtqueue descriptors alignment (0x10 for modern VirtIO devices) */
	ALIGN_AVAIL =       0x02, /* Virtqueue avail ring alignment */
	ALIGN_USED  = _PAGE_SIZE  /* Virtqueue used ring alignment (0x4 for modern VirtIO devices) */
};


/* VirtIO virtqueue descriptor flags */
enum {
	DESC_NEXT         = 0x01, /* Buffer continues via the next field */
	DESC_WRITE        = 0x02, /* Buffer is write-only (otherwise read-only) */
	DESC_INDIRECT     = 0x04  /* Buffer contains a list of buffer descriptors */
};


/* VirtIO virtqueue notification flags */
enum {
	AVAIL_NO_IRQ      = 0x01, /* Avail buffer notification suppression */
	USED_NO_NOTIFY    = 0x01  /* Used buffer notification suppression */
};


/* VirtIO device common features bits */
enum {
	FEAT_NOTIFY_EMPTY = 0x18, /* Notifications when the ring is completely used, even when suppressed */
	FEAT_ANY_LAYOUT   = 0x1b, /* Any descriptor layout support */
	FEAT_INDIRECT     = 0x1c, /* Indirect buffer descriptos support */
	FEAT_EVENT        = 0x1d, /* Notification events suppression */
	FEAT_VERSION1     = 0x20, /* VirtIO v1.0 compliance */
	FEAT_ACCESS       = 0x21, /* Device can be used on platform with limited/translated memory access (IOMMU) */
	FEAT_RING_PACKED  = 0x22, /* Packed virtqueue layout support*/
	FEAT_IN_ORDER     = 0x23, /* Buffers are used by device in order they were made available */
	FEAT_ORDER        = 0x24, /* Memory accesses are ordered in platform specific way */
	FEAT_SR_IOV       = 0x25, /* Single Root IO Virtualization support */
	FEAT_NOTIFICATION = 0x26  /* Driver passes extra data in device notifications */
};


/* VirtIO device features access helpers */
#define FEATURE_MSK(x)           (1ULL << x)
#define FEATURE_HAS(features, x) (features & FEATURE_MSK(x))
#define FEATURE_SET(features, x) (features |= FEATURE_MSK(x))
#define FEATURE_CLR(features, x) (features &= ~(FEATURE_MSK(x)))


typedef struct {
	uint64_t addr;            /* Buffer physical address */
	uint32_t len;             /* Buffer length */
	uint16_t flags;           /* Descriptor flags */
	uint16_t next;            /* Next chained field (if flags & DESC_NEXT) */
} __attribute__((packed)) virtq_desc_t;


typedef struct {
	uint16_t flags;           /* Used buffer notification suppression */
	uint16_t idx;             /* Next available request index */
	uint16_t ring[];          /* Available requests (descriptors indexes) */
} __attribute__((packed)) virtq_avail_t;


typedef struct {
	uint32_t id;              /* Descriptor chain ID */
	uint32_t len;             /* Number of bytes written into the buffer */
} __attribute__((packed)) virtq_used_item_t;


typedef struct {
	uint16_t flags;           /* Available buffer notification suppression */
	uint16_t idx;             /* Next processed (used) request descriptor index */
	virtq_used_item_t ring[]; /* Processed (used) requests */
} __attribute__((packed)) virtq_used_t;


typedef struct _virtq_t virtq_t;


struct _virtq_t {
	/* Actuall virtqueue data */
	volatile void *data;

	/* Standard split virtqueue layout */
	volatile virtq_desc_t *desc;   /* Descriptors */
	volatile virtq_avail_t *avail; /* Avail ring */
	volatile uint16_t *uevent;     /* Used event notification suppression */
	volatile virtq_used_t *used;   /* Used ring */
	volatile uint16_t *aevent;     /* Avail event notification suppression */

	/* Custom helper fields */
	void **vdesc;                  /* Descriptors buffers */
	uint16_t size;                 /* Virtqueue size */
	uint16_t last;                 /* Last processed request (used descriptor) index */
	uint16_t free;                 /* Next free desriptor index */
	handle_t lock;                 /* Descriptors mutex */
	virtq_t *prev, *next;          /* Doubly linked list */
};


typedef struct {
	uint32_t device;               /* Device ID */
	uint32_t vendor;               /* Vendor ID */
	uint64_t features;             /* Device features */
	uint8_t nvqs;                  /* Number of virtqueues */
	virtq_t *vqs;                  /* Virtqueues */
} virtio_dev_t;


/* Memory barrier */
static inline void virtio_memoryBarrier(void)
{
	__asm__ __volatile__("" ::: "memory");
}


/* VirtIO legacy device? */
static inline int virtio_legacy(virtio_dev_t *vdev)
{
	return !FEATURE_HAS(vdev->features, FEAT_VERSION1);
}


/* Converts value from VirtIO host (device) to guest CPU byte order */
static inline uint64_t virtio_toGuest(virtio_dev_t *vdev, uint64_t val, uint8_t size)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
	return val;
#else
	if (virtio_legacy(vdev))
		return val;

	switch (size) {
	case 1:
		return val;

	case 2:
		return le16toh((uint16_t)val);

	case 4:
		return le32toh((uint32_t)val);
	
	case 8:
		return le64toh(val);
	}

	return -EINVAL;
#endif
}


/* Converts value from guest CPU to VirtIO host (device) byte order */
static inline uint64_t virtio_toHost(virtio_dev_t *vdev, uint64_t val, uint8_t size)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
	return val;
#else
	if (virtio_legacy(vdev))
		return val;

	switch(size) {
	case 1:
		return val;

	case 2:
		return htole16((uint16_t)val);

	case 4:
		return htole32((uint32_t)val);

	case 8:
		return htole64(val);
	}

	return -EINVAL;
#endif
}


/* Allocates virtqueue */
extern virtq_t *virtio_allocVirtq(uint16_t size);


/* Releases given virtqueue */
extern void virtio_freeVirtq(virtq_t *vq);


/* Allocates descriptor (should be protected with mutex) */
extern int virtio_allocDesc(virtio_dev_t *vdev, virtq_t *vq, void *addr);


/* Releases given descriptor (should be protected with mutex) */
extern void virtio_freeDesc(virtio_dev_t *vdev, virtq_t *vq, uint16_t desc);


/* Returns VirtIO device n-th virtqueue */
extern virtq_t *virtio_getVirtq(virtio_dev_t *vdev, uint8_t n);


/* Destroys VirtIO device */
extern void virtio_destroyDev(virtio_dev_t *vdev);


/* Initializes VirtIO device */
extern int virtio_initDev(virtio_dev_t *vdev);


#endif
