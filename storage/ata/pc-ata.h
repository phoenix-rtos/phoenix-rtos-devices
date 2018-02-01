/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Generic ata devices controller
 *
 * Copyright 2012 Phoenix Systems
 * Author: Marcin Stragowski, Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DEV_ATA_GENERIC_H_
#define _DEV_ATA_GENERIC_H_

#include <stdint.h>
#include <sys/threads.h>

#include "pc-ata_info.h"

#define ATA_MAX_PIO_DRQ 256
#define ATA_DEF_SECTOR_SIZE 512
#define ATA_DEF_INTR_PRIMARY	14
#define ATA_DEF_INTR_SECONDARY  15


/* Status register bits */
enum { ATA_SR_BSY = 0x80, ATA_SR_DRDY = 0x40, ATA_SR_DF = 0x20,
	ATA_SR_DSC = 0x10, ATA_SR_DRQ = 0x08, ATA_SR_CORR = 0x04,
	ATA_SR_IDX = 0x02, ATA_SR_ERR = 0x01 };

/* Bus master status field masks */
enum { ATA_BMR_STAT_ACT = 1, ATA_BMR_STAT_ERR = (1 << 1),
	ATA_BMR_STAT_INTR = (1 << 2), ATA_BMR_STAT_DEV0_DMA	= (1 << 5),
	ATA_BMR_STAT_DEV1_DMA = (1 << 6), ATA_BMR_STAT_SIMPLEX = (1 << 7) };

/* Bus master command field masks */
enum { ATA_BMR_CMD_WRENABLE = 0, ATA_BMR_CMD_RDENABLE = (1 << 3),
	ATA_BMR_CMD_START = 1, ATA_BMR_CMD_STOP = 0 };

/* Error status register bits */
enum { ATA_ER_BBK = 0x80, ATA_ER_UNC = 0x40, ATA_ER_MC = 0x20,
	ATA_ER_IDNF	= 0x10, ATA_ER_MCR = 0x08, ATA_ER_ABRT	= 0x04,
   	ATA_ER_TK0NF = 0x02, ATA_ER_AMNF = 0x01 };

/* ATA commands */
enum { ATA_CMD_READ_PIO = 0x20, ATA_CMD_WRITE_PIO = 0x30,
   	ATA_CMD_CACHE_FLUSH = 0xE7, ATA_CMD_PACKET = 0xA0,
	ATA_CMD_IDENTIFY_PACKET = 0xA1, ATA_CMD_IDENTIFY = 0xEC };

/* ATA register definitions */
enum { ATA_REG_DATA = 0x00, ATA_REG_ERROR = 0x01, ATA_REG_FEATURES = 0x01,
	ATA_REG_SECCOUNT0 = 0x02, ATA_REG_LBA0 = 0x03, ATA_REG_LBA1 = 0x04,
	ATA_REG_LBA2 = 0x05, ATA_REG_HDDEVSEL = 0x06, ATA_REG_COMMAND = 0x07,
	ATA_REG_STATUS = 0x07 };

/* eo. hob */
enum { ATA_REG_CONTROL = 0x0C, ATA_REG_ALTSTATUS = 0x0C,
   	ATA_REG_DEVADDRESS = 0x0D, ATA_REG_BMPRIMARY = 0x00,
	ATA_REG_BMSECONDARY = 0x08, ATA_REG_BMCOMMAND = 0x0E,
	ATA_REG_BMSTATUS = 0x10, ATA_REG_BMPRD = 0x12 };

// Channels:
enum { ATA_PRIMARY = 0x00, ATA_SECONDARY = 0x01 };

// Directions:
enum { ATA_READ = 0x00, ATA_WRITE = 0x01 };

typedef struct _ata_opt_t {
	u8 force;	/* force initialize in compatibility mode */
	u8 use_int; /* use int if possible */
	u8 use_dma; /* use dma if possible */
	u8 use_multitransfer; /* makes sense only without dma */
} ata_opt_t;

struct ata_channel;

struct ata_dev {
	u8 reserved;
	u8 channel;            /* 0 (Primary Channel) or 1 (Secondary Channel) */
	u8 drive;              /* 0 (Master Drive) or 1 (Slave Drive) */
	u8 type;               /* 0: ATA, 1:ATAPI */

	u16 signature;
	u16 capabilities;
	u32 command_sets;

	u64 size;
	u32 sector_size;
	atainfo_t info;

	struct ata_channel *ac;
};


struct ata_channel {
	u16 base;
	u16 ctrl;           // Control Base address.
	u16 bmide;          // Bus Master IDE address
	u16 reg_addr[22];
	unsigned int  irq_reg;
	u8  no_int;         // No Interrupt;

	u8 status;
	u8 altstatus;

	u8 bmstatus;
	u8 bmstatus_irq;

	handle_t irq_spin;
	volatile u8 irq_invoked;
	handle_t waitq;

	struct ata_bus *ab;
	struct ata_dev devices[2];
};


struct ata_bus {
	ata_opt_t config;
	pci_device_t *dev;
	struct ata_channel ac[2];
};

typedef struct _ata_msg_t {
    u16 bus;
    u16 channel;
    u16 device;
    offs_t offset;
    u16 len;
    char data[];
} __attribute__((packed)) ata_msg_t;

// initialize on pci_device as ata bus
int ata_init_one(pci_device_t *pdev, ata_opt_t *opt);

// initialize ata bus and search for channels and drives
int ata_init_bus(struct ata_bus *ab);

// internal read from channel
u8 ata_ch_read(struct ata_channel *ac, u8 reg);
void ata_ch_write(struct ata_channel *ac, u8 reg, u8 data);
void ata_ch_read_buffer(struct ata_channel *ac, u8 reg, void *buffer, u32 quads);


int ata_polling(struct ata_channel *ac, u8 advanced_check);
int ata_access(u8 direction, struct ata_dev *ad, u32 lba,u8 numsects, void *buffer);
int ata_read_sectors(struct ata_dev *ad, u8 numsects, u32 lba, void *buff);
int ata_write_sectors(struct ata_dev *ad, u8 numsects, u32 lba, void *buff);

#endif
