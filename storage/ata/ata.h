/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Generic ata devices controller
 *
 * Copyright 2012 Phoenix Systems
 * Author: Marcin Stragowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DEV_ATA_GENERIC_H_
#define _DEV_ATA_GENERIC_H_

#include <dev/pci/pci.h>
#include <dev/storage/ata/atainfo.h>
#include "if.h"

#define ATA_MAX_PIO_DRQ 256
#define ATA_DEF_SECTOR_SIZE 512
#define ATA_DEF_INTR_PRIMARY	14
#define ATA_DEF_INTR_SECONDARY  15


/* Status register bits */
#define ATA_SR_BSY     0x80
#define ATA_SR_DRDY    0x40
#define ATA_SR_DF      0x20
#define ATA_SR_DSC     0x10
#define ATA_SR_DRQ     0x08
#define ATA_SR_CORR    0x04
#define ATA_SR_IDX     0x02
#define ATA_SR_ERR     0x01


/* Bus master status field masks */
#define ATA_BMR_STAT_ACT      1
#define ATA_BMR_STAT_ERR      (1 << 1)
#define ATA_BMR_STAT_INTR     (1 << 2)
#define ATA_BMR_STAT_DEV0_DMA (1 << 5)
#define ATA_BMR_STAT_DEV1_DMA (1 << 6)
#define ATA_BMR_STAT_SIMPLEX  (1 << 7)


/* Bus master command field masks */
#define ATA_BMR_CMD_WRENABLE 0
#define ATA_BMR_CMD_RDENABLE (1 << 3)
#define ATA_BMR_CMD_START     1
#define ATA_BMR_CMD_STOP      0

/* Error status register bits */
#define ATA_ER_BBK      0x80
#define ATA_ER_UNC      0x40
#define ATA_ER_MC       0x20
#define ATA_ER_IDNF     0x10
#define ATA_ER_MCR      0x08
#define ATA_ER_ABRT     0x04
#define ATA_ER_TK0NF    0x02
#define ATA_ER_AMNF     0x01

/* Control register */
#define ATA_CTRL_HOB    0x80

/* ATA commands */
#define ATA_CMD_READ_PIO          0x20
#define ATA_CMD_READ_PIO_EXT      0x24
#define ATA_CMD_READ_DMA          0xC8
#define ATA_CMD_READ_DMA_EXT      0x25
#define ATA_CMD_WRITE_PIO         0x30
#define ATA_CMD_WRITE_PIO_EXT     0x34
#define ATA_CMD_WRITE_DMA         0xCA
#define ATA_CMD_WRITE_DMA_EXT     0x35
#define ATA_CMD_CACHE_FLUSH       0xE7
#define ATA_CMD_CACHE_FLUSH_EXT   0xEA
#define ATA_CMD_PACKET            0xA0
#define ATA_CMD_IDENTIFY_PACKET   0xA1
#define ATA_CMD_IDENTIFY          0xEC

/* device type */
#define IDE_ATA        0x00
#define IDE_ATAPI      0x01


/* ATA register definitions */
#define ATA_REG_DATA        0x00
#define ATA_REG_ERROR       0x01
#define ATA_REG_FEATURES    0x01
#define ATA_REG_SECCOUNT0   0x02
#define ATA_REG_LBA0        0x03
#define ATA_REG_LBA1        0x04
#define ATA_REG_LBA2        0x05
#define ATA_REG_HDDEVSEL    0x06
#define ATA_REG_COMMAND     0x07
#define ATA_REG_STATUS      0x07

/* must enable hob to access theese*/
#define ATA_REG_SECCOUNT1   0x08
#define ATA_REG_LBA3        0x09
#define ATA_REG_LBA4        0x0A
#define ATA_REG_LBA5        0x0B

/* eo. hob */
#define ATA_REG_CONTROL     0x0C
#define ATA_REG_ALTSTATUS   0x0C
#define ATA_REG_DEVADDRESS  0x0D

#define ATA_REG_BMPRIMARY   0x00
#define ATA_REG_BMSECONDARY 0x08
#define ATA_REG_BMCOMMAND   0x0E
#define ATA_REG_BMSTATUS    0x10
#define ATA_REG_BMPRD       0x12

// Channels:
#define ATA_PRIMARY      0x00
#define ATA_SECONDARY    0x01

// Directions:
#define ATA_READ         0x00
#define ATA_WRITE        0x01


struct ata_channel;

struct ata_dev {
	u8 reserved;
	u8 channel;            /* 0 (Primary Channel) or 1 (Secondary Channel) */
	u8 drive;              /* 0 (Master Drive) or 1 (Slave Drive) */
	u8 type;               /* 0: ATA, 1:ATAPI */
	u8 dma;
	
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
	u8  irq_reg;
	u8  no_int;         // No Interrupt;

	u8 status;
	u8 altstatus;

	u8 bmstatus;
	u8 bmstatus_irq;

	spinlock_t irq_spin;
	volatile u8 irq_invoked;
	thq_t waitq;
	
	/* for dma */
	volatile u8 *prd_virt;
	addr_t prd_phys;
	u32 prd_size;

	struct ata_bus *ab;
	struct ata_dev devices[2];
};


struct ata_bus {
	ata_opt_t config;
	pci_device_t *dev;
	struct ata_channel ac[2];
};


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
