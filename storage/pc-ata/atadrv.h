/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Generic ata controller driver
 *
 * Copyright 2012-2015, 2019 Phoenix Systems
 * Author: Marcin Stragowski, Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _PC_ATADRV_H_
#define _PC_ATADRV_H_

#include <stdint.h>
#include <sys/threads.h>

#include <arch.h>
#include <phoenix/arch/ia32.h>

#define ATA_MAX_PIO_DRQ 256
#define ATA_DEF_SECTOR_SIZE 512
#define ATA_DEF_INTR_PRIMARY	14
#define ATA_DEF_INTR_SECONDARY  15


typedef union _atainfo_t {

	/* Detailed information about these fields can be found in t13 d1699 ATA8 ACS document */
	struct {
		uint16_t general_config;
		uint16_t obsolete_1;
		uint16_t specific_config;
		uint16_t obsolete_2;
		uint32_t retired_1;
		uint16_t obsolete_3;
		uint32_t compactflash_1;
		uint16_t retired_2;
		uint8_t serial_number[20];
		uint32_t retired_3;
		uint16_t obsolete_4;
		uint8_t firmware_rev[8];
		uint8_t model[40];
		uint8_t control80h;
		uint8_t max_drq_multiple;
		uint16_t trusted_computing;
		uint16_t capabilities_1;
		uint16_t capabilities_2;
		uint32_t obsolete_5;
		uint16_t freefall_control;
		uint8_t obsolete_6[10];
		uint8_t reserved_1;
		uint8_t logicalsect_drq;
		uint32_t lba28_totalsectors;
		uint16_t obsolete_7;
		uint16_t mdma_support;
		uint8_t reserved_2;
		uint8_t pio_support;
		uint16_t min_mdma_cycle;
		uint16_t rec_mdmda_cycle;
		uint16_t min_pio_cycle;
		uint16_t min_pioirq_cycle;
		uint32_t reserved_3;
		uint8_t idf_packet_dev[8];
		uint16_t queue_depth;
		uint16_t sata_capabilities;
		uint16_t sata_reserved;
		uint16_t sata_features_sup;
		uint16_t sata_features_on;
		uint16_t ver_major;
		uint16_t ver_minor;
		uint16_t commands1_sup;
		uint16_t commands2_sup;
		uint16_t commands3_sup;
		uint16_t commands4_sup;
		uint16_t commands5_sup;
		uint16_t commands6_sup;
		uint16_t udma_modes;
		uint8_t reserved_4;
		uint8_t norm_secure_erase_time;
		uint8_t reserved_5;
		uint8_t ext_secure_erase_time;
		uint16_t current_apm;
		uint16_t master_password;
		uint16_t hwreset_result;
		uint8_t recommended_aam;
		uint8_t current_aam;
		uint16_t stream_minimum_req;
		uint16_t dma_stream_trans_time;
		uint16_t dmapio_stream_acces_lat;
		uint32_t perf_grain;
		uint64_t lba48_totalsectors;
		uint16_t pio_streaming_time;
		uint16_t reserved_6;
		uint16_t physlog_sector_size;
		uint16_t interseek_delay;
		uint8_t world_wide_name[8];
		uint8_t reserved_7[8];
		uint16_t reserved_8;
		uint32_t log_sector_size;
		uint16_t commands7_sup;
		uint16_t commands8_sup;
		uint8_t reserved_9[12];
		uint16_t obsolete_8;
		uint16_t security_status;
		uint8_t vendor_specific[62];
		uint16_t cfa_power_mode;
		uint8_t compactflash_2[14];
		uint8_t reserved_10;
		uint8_t nominal_formfactor;
		uint8_t reserved_11[14];
		uint8_t curr_media_serial[60];
		uint16_t sct_command_trans;
		uint32_t ce_ata_reserved_1;
		uint16_t loginphy_alignment;
		uint32_t wrv_sectorcountmode3;
		uint32_t wrv_sectorcountmode2;
		uint16_t nv_cache_cap;
		uint32_t nv_cache_size;
		uint16_t rotation_rate;
		uint16_t reserved_12;
		uint16_t nv_cache_opts;
		uint8_t reserved_13;
		uint8_t wrv_feature_set;
		uint16_t reserved_14;
		uint16_t transport_major_ver;
		uint16_t transport_minor_ver;
		uint8_t ce_ata_reserved_2[20];
		uint16_t min_db_microcode_dl;
		uint16_t max_db_microcode_dl;
		uint8_t reserved_15[38];
		uint8_t checksum;
		uint8_t checksum_valid;
	} __attribute__((packed));


	struct {
		uint8_t buff[512];
	} __attribute__((packed));

} atainfo_t;


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


typedef struct ata_dev {
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
} ata_dev_t;


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


int ata_init(void);


int ata_read(ata_dev_t *dev, offs_t offs, char *buff, unsigned int len);


int ata_write(ata_dev_t *dev, offs_t offs, const char *buff, unsigned int len);


#endif
