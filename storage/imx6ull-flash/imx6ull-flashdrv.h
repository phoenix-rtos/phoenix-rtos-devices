/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND flash driver.
 * Low level API to be used by flashsrv and FS implementations.
 *
 * Copyright 2018 Phoenix Systems
 * Author: Jan Sikorski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMX6ULL_FLASHDRV_H_
#define _IMX6ULL_FLASHDRV_H_

#include <stdint.h>

typedef struct _flashdrv_dma_t flashdrv_dma_t;


enum {
	flash_reset = 0, flash_read_id, flash_read_parameter_page, flash_read_unique_id,
	flash_get_features, flash_set_features, flash_read_status, flash_read_status_enhanced,
	flash_random_data_read, flash_random_data_read_two_plane, flash_random_data_input,
	flash_program_for_internal_data_move_column, flash_read_mode, flash_read_page,
	flash_read_page_cache_sequential, flash_read_page_cache_random, flash_read_page_cache_last,
	flash_program_page, flash_program_page_cache, flash_erase_block,
	flash_read_for_internal_data_move, flash_program_for_internal_data_move,
	flash_block_unlock_low, flash_block_unlock_high, flash_block_lock, flash_block_lock_tight,
	flash_block_lock_read_status, flash_otp_data_lock_by_block, flash_otp_data_program,
	flash_otp_data_read, flash_num_commands
};


/* possible values of flashdrv_meta_t->errors[] */
enum {
	flash_no_errors = 0,
	/* 0x01 - 0x28: number of bits corrected */
	flash_uncorrectable = 0xfe,
	flash_erased = 0xff
};


typedef struct {
	uint8_t metadata[16]; /* externally usable (by FS) */
	uint8_t errors[9];    /* ECC status: (one of the above enums) */
} flashdrv_meta_t;


/* information about NAND flash configuration */
typedef struct {
	const char *name;
	uint64_t size;     /* total NAND size in bytes */
	uint32_t writesz;  /* write page DATA size in bytes */
	uint32_t metasz;   /* write page METADATA size in bytes */
	uint32_t erasesz;  /* erase block size in bytes (multiply of writesize) */
	uint32_t oobsz;    /* out-of-bound (oob) data size */
	uint32_t oobavail; /* available out-of-bound (oob) data size */
	uint32_t pbits;    /* number of meaningful page address bits */
	uint8_t chips;     /* number of internal chips */
} flashdrv_info_t;

/* paddr: page address, so NAND address / writesz */

extern flashdrv_dma_t *flashdrv_dmanew(void);


extern void flashdrv_dmadestroy(flashdrv_dma_t *dma);


extern int flashdrv_reset(flashdrv_dma_t *dma, int chip);


extern int flashdrv_write(flashdrv_dma_t *dma, uint32_t paddr, void *data, char *metadata);


extern int flashdrv_read(flashdrv_dma_t *dma, uint32_t paddr, void *data, flashdrv_meta_t *meta);


extern int flashdrv_erase(flashdrv_dma_t *dma, uint32_t paddr);


extern int flashdrv_writeraw(flashdrv_dma_t *dma, uint32_t paddr, void *data, int sz);


extern int flashdrv_readraw(flashdrv_dma_t *dma, uint32_t paddr, void *data, int sz);


extern int flashdrv_isbad(flashdrv_dma_t *dma, uint32_t paddr);


extern int flashdrv_markbad(flashdrv_dma_t *dma, uint32_t paddr);


extern void flashdrv_init(void);


extern const flashdrv_info_t *flashdrv_info(void);

#endif
