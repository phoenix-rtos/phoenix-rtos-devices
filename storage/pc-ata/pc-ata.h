/*
 * Phoenix-RTOS
 *
 * Generic ata devices controller
 *
 * Copyright 2012, 2018 Phoenix Systems
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

#include <phoenix/arch/ia32.h>

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
	uint8_t force;	/* force initialize in compatibility mode */
	uint8_t use_int; /* use int if possible */
	uint8_t use_dma; /* use dma if possible */
	uint8_t use_multitransfer; /* makes sense only without dma */
} ata_opt_t;

struct ata_channel;

struct ata_dev {
	uint8_t reserved;
	uint8_t channel;            /* 0 (Primary Channel) or 1 (Secondary Channel) */
	uint8_t drive;              /* 0 (Master Drive) or 1 (Slave Drive) */
	uint8_t type;               /* 0: ATA, 1:ATAPI */

	uint16_t signature;
	uint16_t capabilities;
	uint32_t command_sets;

	uint64_t size;
	uint32_t sector_size;
	atainfo_t info;

	struct ata_channel *ac;
};


struct ata_channel {
	uint16_t base;
	uint16_t ctrl;           // Control Base address.
	uint16_t bmide;          // Bus Master IDE address
	uint16_t reg_addr[22];
	unsigned int  irq_reg;
	uint8_t  no_int;         // No Interrupt;

	uint8_t status;
	uint8_t altstatus;

	uint8_t bmstatus;
	uint8_t bmstatus_irq;

	handle_t irq_spin;
	volatile uint8_t irq_invoked;
	handle_t waitq;
	handle_t inth;

	struct ata_bus *ab;
	struct ata_dev devices[2];
};


struct ata_bus {
	ata_opt_t config;
	pci_device_t *dev;
	struct ata_channel ac[2];
};

typedef struct _ata_msg_t {
    uint16_t bus;
    uint16_t channel;
    uint16_t device;
    offs_t offset;
    uint16_t len;
    char data[];
} __attribute__((packed)) ata_msg_t;

// initialize on pci_device as ata bus
int ata_init_one(pci_device_t *pdev, ata_opt_t *opt);

// initialize ata bus and search for channels and drives
int ata_init_bus(struct ata_bus *ab);

// internal read from channel
uint8_t ata_ch_read(struct ata_channel *ac, uint8_t reg);
void ata_ch_write(struct ata_channel *ac, uint8_t reg, uint8_t data);
void ata_ch_read_buffer(struct ata_channel *ac, uint8_t reg, void *buffer, uint32_t quads);


int ata_polling(struct ata_channel *ac, uint8_t advanced_check);
int ata_access(uint8_t direction, struct ata_dev *ad, uint32_t lba,uint8_t numsects, void *buffer);
int ata_read_sectors(struct ata_dev *ad, uint8_t numsects, uint32_t lba, void *buff);
int ata_write_sectors(struct ata_dev *ad, uint8_t numsects, uint32_t lba, void *buff);

#endif
