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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>

#include "bcb.h"
#include "bch.h"

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
	u32 dbbt_size; /* pages */
	u8	reserved2[((4 * 4096) - 20)];
	u32 reserved3;
	u32 entries_no;
	u32 bad_block[];
} dbbt_t;

int dbbt_flash(flashdrv_dma_t *dma)
{
	return 0;
}


u32 fcb_checksum(u8 *fcb, int size)
{
	int i;
	u32 checksum = 0;

	for (i = 0; i <= size; i++) {
		checksum += fcb[i];
	}

	checksum ^= 0xffffffff;
	return checksum;
}


void fcb_init(fcb_t *fcb)
{
	fcb->fingerprint			= 0x20424346;
	fcb->version				= 0x01000000;
	fcb->data_setup				= 0x78;
	fcb->data_hold				= 0x3c;
	fcb->address_setup			= 0x19;
	fcb->dsample_time			= 0x6;
	fcb->nand_timing_state		= 0x0;
	fcb->REA					= 0x0;
	fcb->RLOH					= 0x0;
	fcb->RHOH					= 0x0;
	fcb->page_size				= 0x1000;
	fcb->total_page_size		= 0x10e0;
	fcb->block_size				= 64;
	fcb->b0_ecc_type			= 0x8;
	fcb->b0_ecc_size			= 0x0;
	fcb->bn_ecc_size			= 512;
	fcb->bn_ecc_type			= 0x7;
	fcb->meta_size				= 0x10;
	fcb->ecc_per_page			= 8;
	fcb->fw1_start				= 512;
	fcb->fw2_start				= 1024;
	fcb->fw1_size				= 0x10;
	fcb->fw2_size				= 0x10;
	fcb->dbbt_start				= 0x0;
	fcb->bbm_offset				= 0x1000;
	fcb->bbm_start				= 0x0;
	fcb->bbm_phys_offset		= 0x1000;
	fcb->bch_type				= 0x0;
	fcb->read_latency			= 0x0;
	fcb->preamble_delay			= 0x0;
	fcb->ce_delay				= 0x0;
	fcb->postamble_delay		= 0x0;
	fcb->cmd_add_pause			= 0x0;
	fcb->data_pause				= 0x0;
	fcb->speed					= 0x0;
	fcb->busy_timeout			= 0xffff;
	fcb->bbm_disabled			= 1;
	fcb->bbm_spare_offset		= 0;
	fcb->disable_bbm_search		= 1;

	fcb->checksum = fcb_checksum(((u8 *)fcb + 4), sizeof(fcb_t) - 4);
}

int fcb_flash(flashdrv_dma_t *dma)
{
	char *sbuf, *tbuf;
	fcb_t *fcb;
	int err = 0;

	if ((sbuf = mmap(NULL, SIZE_PAGE * 2, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0)) == MAP_FAILED)
		return 1;

	if ((tbuf = mmap(NULL, SIZE_PAGE * 2, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0)) == MAP_FAILED)
		return 1;

	memset(sbuf, 0x0, SIZE_PAGE * 2);

	fcb = (fcb_t *)(sbuf);

	fcb_init(fcb);
	memset(tbuf, 0x0, SIZE_PAGE * 2);

	encode_bch_ecc(sbuf, sizeof(fcb_t), tbuf,  4320, 3);

	err = flashdrv_writeraw(dma, 0, tbuf, 4096);
	err = flashdrv_writeraw(dma, 64, tbuf, 4096);

	return err;
}
