/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND tool.
 *
 * Boot control blocks
 *
 * Copyright 2018 Phoenix Systems
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#ifndef _BCB_H
#define _BCB_H

#include <stdint.h>
#include <imx6ull-flashdrv.h>

#define PAGE_SIZE 4096
#define RAW_PAGE_SIZE 4320
#define PAGES_PER_BLOCK 64
#define BLOCKS_CNT 4096
#define BB_MAX	256

#define BCB_CNT 4
#define FCB_START 0x0
#define DBBT_START 0x100

typedef struct _fcb_t {
	uint32_t checksum;
	uint32_t fingerprint;
	uint32_t version;
	uint8_t	data_setup;
	uint8_t	data_hold;
	uint8_t	address_setup;
	uint8_t	dsample_time;
	uint8_t	nand_timing_state;
	uint8_t	REA;
	uint8_t	RLOH;
	uint8_t	RHOH;
	uint32_t page_size;
	uint32_t total_page_size;
	uint32_t block_size;
	uint32_t nand_number;
	uint32_t die_number;
	uint32_t cell_type;
	uint32_t bn_ecc_type;
	uint32_t b0_ecc_size;
	uint32_t bn_ecc_size;
	uint32_t b0_ecc_type;
	uint32_t meta_size;
	uint32_t ecc_per_page;
	uint32_t bn_ecc_level_sdk;
	uint32_t b0_ecc_size_sdk;
	uint32_t bn_ecc_size_sdk;
	uint32_t b0_ecc_level_sdk;
	uint32_t ecc_per_page_sdk;
	uint32_t meta_size_sdk;
	uint32_t erase_threshold;
	uint8_t	pad[8];
	uint32_t fw1_start;
	uint32_t fw2_start;
	uint32_t fw1_size; /* pages */
	uint32_t fw2_size;
	uint32_t dbbt_start;
	uint32_t bbm_offset;
	uint32_t bbm_start;
	uint32_t bbm_phys_offset;
	uint32_t bch_type;
	uint32_t read_latency;
	uint32_t preamble_delay;
	uint32_t ce_delay;
	uint32_t postamble_delay;
	uint32_t cmd_add_pause;
	uint32_t data_pause;
	uint32_t speed;
	uint32_t busy_timeout;
	uint32_t bbm_disabled;
	uint32_t bbm_spare_offset;
	uint32_t onfi_sync_enabled;
	uint32_t onfi_sync_speed;
	uint8_t onfi_sync_nand_data[28];
	uint32_t disable_bbm_search;
	uint8_t	reserved1[64];
} fcb_t;


typedef struct _dbbt_t {
	uint32_t checksum;
	uint32_t fingerprint;
	uint32_t version;
	uint32_t reserved1;
	uint32_t size; /* pages */
	uint8_t	reserved2[((4 * 4096) - 20)];
	uint32_t reserved3;
	uint32_t entries_num;
	uint32_t bad_block[];
} dbbt_t;


int fcb_flash(flashdrv_dma_t *dma, fcb_t *fcb_ret);

int dbbt_flash(flashdrv_dma_t *dma, dbbt_t *dbbt);

int dbbt_block_is_bad(dbbt_t *dbbt, uint32_t block_num);

#endif /* _BCB_H_ */
