/*
 * Phoenix-RTOS
 *
 * Generic ATA controller driver
 *
 * Copyright 2012-2015, 2019, 2020 Phoenix Systems
 * Author: Marcin Stragowski, Kamil Amanowicz, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _ATA_H_
#define _ATA_H_

#include <stdint.h>

#include <sys/types.h>


/* Offset between standard IO-ports addresses */
#define PORT_OFFSET 0x80

/* ATA standard IO-ports */
#define ATA1_BASE 0x1f0
#define ATA1_CTRL 0x3f6
#define ATA2_BASE (ATA1_BASE - PORT_OFFSET)
#define ATA2_CTRL (ATA1_CTRL - PORT_OFFSET)

#define ATA3_BASE 0x1e8
#define ATA3_CTRL 0x3ee
#define ATA4_BASE (ATA3_BASE - PORT_OFFSET)
#define ATA4_CTRL (ATA3_CTRL - PORT_OFFSET)


/* ATA bus devices */
enum { MASTER, SLAVE };


/* ATA addressing modes */
enum { CHS, LBA28, LBA48 };


/* ATA access direction */
enum { READ, WRITE };


/* ATA PIO transfer modes */
enum {
	PIO_DEFAULT         = 0x00, /* Default mode */
	PIO_0               = 0x08, /* max 3,3  MB/s */
	PIO_1               = 0x09, /* max 5,2  MB/s */
	PIO_2               = 0x0a, /* max 8,3  MB/s */
	PIO_3               = 0x0b, /* max 11,1 MB/s */
	PIO_4               = 0x0c  /* max 16,7 MB/s */
};


/* ATA commands */
enum {
	CMD_NOP             = 0x00,
	CMD_READ_PIO        = 0x20,
	CMD_READ_PIO_EXT    = 0x24,
	CMD_READ_DMA_EXT    = 0x25,
	CMD_WRITE_PIO       = 0x30,
	CMD_WRITE_PIO_EXT   = 0x34,
	CMD_WRITE_DMA_EXT   = 0x35,
	CMD_PACKET          = 0xa0,
	CMD_IDENTIFY_PACKET = 0xa1,
	CMD_READ_DMA        = 0xc8,
	CMD_WRITE_DMA       = 0xca,
	CMD_CACHE_FLUSH     = 0xe7,
	CMD_CACHE_FLUSH_EXT = 0xea,
	CMD_IDENTIFY        = 0xec,
	CMD_SET_FEATUERS    = 0xef
};


/* ATA registers */
enum {
	REG_CTRL            = 0x0, /* Write control register */
	REG_ALTSTATUS       = 0x0, /* Read alternate status */
	REG_DATA            = 0x0, /* R/W PIO data port */
	REG_ERROR           = 0x1, /* Read error */
	REG_FEATURES        = 0x1, /* Write features */
	REG_NSECTORS        = 0x2, /* Number of sectors */
	REG_SECTOR          = 0x3, /* Sector */
	REG_LCYLINDER       = 0x4, /* Cylinder low */
	REG_HCYLINDER       = 0x5, /* Cylinder high */
	REG_DEVSEL          = 0x6, /* Device select */
	REG_CMD             = 0x7, /* Write command */
	REG_STATUS          = 0x7  /* Read status */
};


/* REG_CTRL layout */
enum {
	CTRL_NIEN           = 0x02, /* Not Interrupt ENabled */
	CTRL_SOFTRST        = 0x04, /* Software reset all ATA devices on the bus */
	CTRL_HOB            = 0x80  /* Read back the High Order Byte of the last LBA48 value sent */
};


/* REG_DEVSEL layout */
enum {
	DEVSEL_HEAD         = 0x0f, /* LBA head */
	DEVSEL_DEVNUM       = 0x10, /* 0: Master, 1: Slave */
	DEVSEL_SET0         = 0x20, /* Should always be set */
	DEVSEL_LBA          = 0x40, /* 0: CHS, 1: LBA */
	DEVSEL_SET1         = 0x80  /* Should always be set */
};


/* REG_STATUS layout */
enum {
	STATUS_ERR          = 0x01, /* Error occured for last command */
	STATUS_DRQ          = 0x08, /* PIO data is ready */
	STATUS_SRV          = 0x10, /* Overlapped mode service request */
	STATUS_DF           = 0x20, /* Device fault (doesn't set ERR) */
	STATUS_RDY          = 0x40, /* Device is ready */
	STATUS_BSY          = 0x80  /* Device is busy */
};


typedef struct _ata_dev_t ata_dev_t;
typedef struct _ata_bus_t ata_bus_t;


struct _ata_dev_t {
	/* Device configuration */
	uint8_t pio;            /* PIO mode: PIO_DEFAULT, PIO_0, PIO_1, PIO_2, PIO_3, PIO_4 */
	uint8_t mode;           /* Addressing mode: CHS, LBA28, LBA48 */

	/* Device geometry */
	uint16_t cylinders;     /* Number of cylinders */
	uint16_t heads;         /* Number of heads */
	uint16_t sectors;       /* Number of sectors */
	uint32_t sectorsz;      /* Sector size */
	uint64_t size;          /* Storage size */

	ata_bus_t *bus;         /* ATA bus the device is attached to */
	ata_dev_t *prev, *next; /* Doubly linked list */
};


struct _ata_bus_t {
	/* ATA registers access */
	void *base;             /* ATA bus base registers */
	void *ctrl;             /* ATA bus control registers */

	ata_dev_t *devs[2];     /* ATA devices attached to the bus */
	ata_bus_t *prev, *next; /* Doubly linked list */

	/* Synchronization */
	handle_t lock;          /* Access mutex */
};


typedef struct {
	unsigned int ndevs;    /* Number of detected ATA devices */
	ata_dev_t *devs;       /* Detected ATA devices */
} ata_common_t;


extern ata_common_t ata_common;


/* Reads from ATA device */
extern ssize_t ata_read(ata_dev_t *dev, offs_t offs, char *buff, size_t len);


/* Writes to ATA device */
extern ssize_t ata_write(ata_dev_t *dev, offs_t offs, const char *buff, size_t len);


/* Initializes ATA devices */
extern int ata_init(void);


#endif
