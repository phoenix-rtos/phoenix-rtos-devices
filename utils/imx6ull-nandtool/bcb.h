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

#include "../../storage/imx6ull-flash/flashdrv.h"

#define PAGE_SIZE 4096
#define RAW_PAGE_SIZE 4320
#define PAGES_PER_BLOCK 64
#define BLOCKS_CNT 4096
#define BB_MAX	256

#define BCB_CNT 4
#define FCB_START 0x0
#define DBBT_START 0x100

typedef struct _fcb_t {
	u32 checksum;
	u32 fingerprint;
	u32 version;
	u8	data_setup;
	u8	data_hold;
	u8	address_setup;
	u8	dsample_time;
	u8	nand_timing_state;
	u8	REA;
	u8	RLOH;
	u8	RHOH;
	u32 page_size;
	u32 total_page_size;
	u32 block_size;
	u32 nand_number;
	u32 die_number;
	u32 cell_type;
	u32 bn_ecc_type;
	u32 b0_ecc_size;
	u32 bn_ecc_size;
	u32 b0_ecc_type;
	u32 meta_size;
	u32 ecc_per_page;
	u32 bn_ecc_level_sdk;
	u32 b0_ecc_size_sdk;
	u32 bn_ecc_size_sdk;
	u32 b0_ecc_level_sdk;
	u32 ecc_per_page_sdk;
	u32 meta_size_sdk;
	u32 erase_threshold;
	u8	pad[8];
	u32 fw1_start;
	u32 fw2_start;
	u32 fw1_size; /* pages */
	u32 fw2_size;
	u32 dbbt_start;
	u32 bbm_offset;
	u32 bbm_start;
	u32 bbm_phys_offset;
	u32 bch_type;
	u32 read_latency;
	u32 preamble_delay;
	u32 ce_delay;
	u32 postamble_delay;
	u32 cmd_add_pause;
	u32 data_pause;
	u32 speed;
	u32 busy_timeout;
	u32 bbm_disabled;
	u32 bbm_spare_offset;
	u32 onfi_sync_enabled;
	u32 onfi_sync_speed;
	u8 onfi_sync_nand_data[28];
	u32 disable_bbm_search;
	u8	reserved1[64];
} fcb_t;


typedef struct _dbbt_t {
	u32 checksum;
	u32 fingerprint;
	u32 version;
	u32 reserved1;
	u32 size; /* pages */
	u8	reserved2[((4 * 4096) - 20)];
	u32 reserved3;
	u32 entries_num;
	u32 bad_block[];
} dbbt_t;


int fcb_flash(flashdrv_dma_t *dma, fcb_t *fcb_ret);

int dbbt_flash(flashdrv_dma_t *dma, dbbt_t *dbbt);

int dbbt_block_is_bad(dbbt_t *dbbt, u32 block_num);

#endif /* _BCB_H_ */
