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

#ifndef _VIRTIO_BLK_H_
#define _VIRTIO_BLK_H_

#include <stdint.h>

#include <sys/types.h>

#include "virtio-mmio.h"


/* Misc definitions */
#define SECTOR_SIZE 512            /* VirtIO block device default sector size */
#define REQ_HEADER  16             /* VirtIO block device request header size */
#define REQ_FOOTER  1              /* VirtIO block device request footer size */


/* VirtIO block device features */
enum {
	FEAT_BARRIER         = 0x0001, /* Request barriers support, LEGACY */
	FEAT_MAX_SIZE        = 0x0002, /* Max segment size */
	FEAT_MAX_SEG         = 0x0004, /* Max number of segments */
	FEAT_GEOMETRY        = 0x0010, /* Legacy geometry support */
	FEAT_RO              = 0x0020, /* Disk is read-only */
	FEAT_BLK_SIZE        = 0x0040, /* Disk block size available */
	FEAT_SCSI            = 0x0080, /* SCSI commands passthrough support, LEGACY */
	FEAT_FLUSH           = 0x0200, /* Cache flush command support */
	FEAT_TOPOLOGY        = 0x0400, /* Topology/alignment information available */
	FEAT_WCE             = 0x0800, /* Cache writeback and writethrough modes support */
	FEAT_DISCARD         = 0x2000, /* Discard command support */
	FEAT_ZEROES          = 0x4000  /* Write zeroes command support */
};


/* VirtIO block device command types */
enum {
	CMD_READ               = 0x00, /* Read command */
	CMD_WRITE              = 0x01, /* Write command */
	CMD_SCSI               = 0x02, /* SCSI command, LEGACY */
	CMD_FLUSH              = 0x04, /* Flush command (if FEAT_FLUSH) */
	CMD_ID                 = 0x08, /* Get device ID */
	CMD_DISCARD            = 0x0b, /* Discard command (if FEAT_DISCARD) */
	CMD_ZEROES             = 0x0d, /* Write zeroes command (if FEAT_ZEROES) */
	/* Requests barrier command, LEGACY */
	CMD_BARRIER            = 0x80000000
};


/* VirtIO block device request returned status */
enum {
	RSTATUS_OK             = 0x00, /* Request successfully processed */
	RSTATUS_ERR            = 0x01, /* Request failed with error */
	RSTATUS_UNSUPP         = 0x02  /* Unsupported request */
};


/* VirtIO block device configuration space */
enum {
	CONFIG_CAPACITY        = 0x00, /* Device capacity (in 512-byte sectors), 64-bit */
	CONFIG_MAX_SIZE        = 0x08, /* Max segment size (if FEAT_MAX_SIZE), 32-bit */
	CONFIG_MAX_SEG         = 0x0c, /* Max number of segments (if FEAT_MAX_SEG), 32-bit */
	CONFIG_CYLINDERS       = 0x10, /* Number of cylinders (if FEAT_GEOMETRY), 16-bit */
	CONFIG_HEADS           = 0x12, /* Number of heads (if FEAT_GEOMETRY), 8-bit */
	CONFIG_SECTORS         = 0x13, /* Number of sectors (if FEAT_GEOMETRY), 8-bit */
	CONFIG_BLK_SIZE        = 0x14, /* Block size (if FEAT_BLK_SIZE), 32-bit */
	CONFIG_PBLOCK_EXP      = 0x18, /* Number of logical blocks per physical block (log2) (if FEAT_TOPOLOGY), 8-bit */
	CONFIG_ALIGN_OFFS      = 0x19, /* Offset of first aligned logical block (if FEAT_TOPOLOGY), 8-bit */
	CONFIG_MIN_IO          = 0x1a, /* Minimum suggested I/O size in logical blocks (if FEAT_TOPOLOGY), 16-bit */
	CONFIG_OPT_IO          = 0x1c, /* Optimal sustained I/O size in logical blocks (if FEAT_TOPOLOGY), 32-bit */
	CONFIG_WCE             = 0x20, /* Writeback mode (if FEAT_WCE), 8-bit */
	CONFIG_DISCARD_MAX_SEC = 0x24, /* Max discard 512-byte sectors for one segment (if FEAT_DISCARD), 32-bit */
	CONFIG_DISCARD_MAX_SEG = 0x28, /* Max discard segments (if FEAT_DISCARD), 32-bit */
	CONFIG_DISCARD_ALIGN   = 0x2c, /* Disard command sector alignment (if FEAT_DISCARD), 32-bit */
	CONFIG_ZEROES_MAX_SEC  = 0x30, /* Max write zeroes 512-byte sectors for one segment (if FEAT_ZEROES), 32-bit */
	CONFIG_ZEROES_MAX_SEG  = 0x34, /* Max write zeroes segments (if FEAT_ZEROES), 32-bit */
	CONFIG_ZEROES_UNMAP    = 0x38  /* Set if write zeroes request may result in deallocation of one or more sectors (if FEAT_ZEROES), 32-bit */
};


typedef struct {
	/* Request header (read-only) */
	uint32_t type;             /* Request type */
	uint32_t reserved;         /* Reserved */
	uint64_t sector;           /* Sector (512-byte offset) */

	/* Request footer (write-only) */
	uint8_t status;            /* Returned request status */
} __attribute__((packed)) virtio_blkreq_t;


typedef struct _virtio_blk_t virtio_blk_t;


struct _virtio_blk_t {
	/* Device information */
	virtio_mmio_t mdev;        /* VirtIO MMIO block device */
	uint64_t size;             /* Storage size */

	/* Interrupt handling */
	unsigned int irq;          /* Interrupt number */
	handle_t lock;             /* Interrupt mutex */
	handle_t cond;             /* Interrupt condition variable */
	handle_t inth;             /* Interrupt handle */

	virtio_blk_t *prev, *next; /* Doubly linked list */
};


typedef struct {
	unsigned int ndevs;        /* Number of detected VirtIO block devices */
	virtio_blk_t *devs;        /* Detected VirtIO block devices */
} virtio_blk_common_t;


extern virtio_blk_common_t virtio_blk_common;


/* Reads from VirtIO block device */
extern ssize_t virtio_blk_read(virtio_blk_t *dev, offs_t offs, char *buff, size_t len);


/* Writes to VirtIO block device */
extern ssize_t virtio_blk_write(virtio_blk_t *dev, offs_t offs, const char *buff, size_t len);


/* Destroys VirtIO block devices */
extern void virtio_blk_destroy(void);


/* Initializes VirtIO block devices */
extern int virtio_blk_init(void);


#endif
