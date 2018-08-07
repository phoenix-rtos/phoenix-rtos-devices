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


u32 bcb_checksum(u8 *bcb, int size)
{
	int i;
	u32 checksum = 0;

	for (i = 0; i <= size; i++)
		checksum += bcb[i];

	checksum ^= 0xffffffff;
	return checksum;
}


void dbbt_fingerprint(dbbt_t *dbbt)
{
	dbbt->fingerprint = 0x54424244;//0x44424254;//
	dbbt->version = 0x01000000;
}


int dbbt_block_is_bad(dbbt_t *dbbt, u32 block_num)
{
	int i;

	if (dbbt == NULL)
		return 0;

	for (i = 0; i < dbbt->entries_num; i++) {
		if (block_num == dbbt->bad_block[i])
			return 1;
	}

	return 0;
}


int dbbt_flash(flashdrv_dma_t *dma, dbbt_t *dbbt)
{
	int i, err;
	u32 page_num = DBBT_START;
	void *data = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);
	void *meta = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);

	dbbt_fingerprint(dbbt);

	dbbt->size = 1;

	memset(meta, 0xff, PAGE_SIZE);

	for (i = 0; i < BCB_CNT; i++) {

		memcpy(data, dbbt, PAGE_SIZE);
		if ((err = flashdrv_write(dma, page_num, data, meta)) != 0)
			printf("Error writing %d dbbt page\n", page_num);

		if (dbbt->entries_num) {
			memcpy(data, ((void *)dbbt) + (4 * PAGE_SIZE), PAGE_SIZE);
			if (((err = flashdrv_write(dma, page_num + 4, data, meta)) != 0))
				printf("Error writing %d dbbt page\n", page_num + 4);
		}
		page_num += PAGES_PER_BLOCK;
	}

	munmap(data, PAGE_SIZE);
	munmap(data, PAGE_SIZE);
	return 0;
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
	fcb->fw2_start				= 1536;
	fcb->fw1_size				= 0x1;
	fcb->fw2_size				= 0x1;
	fcb->dbbt_start				= 0x100;
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

	fcb->checksum = bcb_checksum(((u8 *)fcb) + 4, sizeof(fcb_t) - 4);
}

int fcb_flash(flashdrv_dma_t *dma, fcb_t *fcb_ret)
{
	char *sbuf, *tbuf;
	fcb_t *fcb;
	int i;
	int err = 0;
	int errn = 0;

	if ((sbuf = mmap(NULL, PAGE_SIZE * 2, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0)) == MAP_FAILED)
		return 1;

	if ((tbuf = mmap(NULL, PAGE_SIZE * 2, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0)) == MAP_FAILED)
		return 1;

	memset(sbuf, 0x0, PAGE_SIZE * 2);

	fcb = (fcb_t *)(sbuf);

	fcb_init(fcb);
	memset(tbuf, 0x0, PAGE_SIZE * 2);

	encode_bch_ecc(sbuf, sizeof(fcb_t), tbuf,  RAW_PAGE_SIZE, 3);

	for (i = 0; i < BCB_CNT; i++) {
		err = flashdrv_writeraw(dma, FCB_START + (i * PAGES_PER_BLOCK), tbuf, PAGE_SIZE) & err;
		errn += !!err;
	}

	if (fcb_ret != NULL)
		memcpy(fcb_ret, sbuf, sizeof(fcb_t));

	munmap(sbuf, PAGE_SIZE * 2);
	munmap(tbuf, PAGE_SIZE * 2);

	return errn;
}
