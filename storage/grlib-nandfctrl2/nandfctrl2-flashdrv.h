/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash driver.
 *
 * Low-level API for userspace driver using DMA descriptor chains (LLM).
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef NANDFCTRL2_FLASHDRV_H_
#define NANDFCTRL2_FLASHDRV_H_

#include <stdint.h>
#include <stddef.h>

#include <board_config.h>


typedef struct _flashdrv_dma_t flashdrv_dma_t;


enum {
	flash_ecc_ok = 0,        /* No errors */
	flash_ecc_corrected,     /* Errors corrected by hardware EDAC */
	flash_ecc_uncorrectable, /* Uncorrectable errors */
};


/*
 * Metadata structure populated by flashdrv_read() for each page.
 * metadata[] contains user-accessible spare area bytes (OOB).
 * eccState reflects the EDAC result for the data area read.
 */
typedef struct {
	int eccState;               /* flash_ecc_* value */
	unsigned int worstBitflips; /* Corrected bits in worst ECC chunk */
} flashdrv_eccStatus_t;


/* Information about one physical NAND target */
typedef struct {
	const char *name;       /* Flash device name string */
	uint64_t size;          /* Total NAND size in bytes */
	uint32_t writesz;       /* Page data size in bytes */
	uint32_t sparesz;       /* Total spare (OOB) bytes per page */
	uint32_t spareavail;    /* User-accessible spare bytes */
	uint32_t erasesz;       /* Erase block size in bytes */
	uint32_t pagesPerBlock; /* Pages per erase block */
} flashdrv_info_t;


typedef struct {
	flashdrv_info_t info;
	flashdrv_dma_t *dma;
	unsigned int target;
} nand_die_t;


/*
 * Allocate a handle for the given NAND die (CE# target index).
 * Copies the ONFI geometry probed by flashdrv_init() and allocates a DMA buffer.
 * Returns NULL on failure.
 */
nand_die_t *flashdrv_dieAlloc(unsigned int target);


/* Release all resources allocated by flashdrv_dieAlloc(). */
void flashdrv_dieFree(nand_die_t *die);


/* Reset the NAND device. */
int flashdrv_reset(nand_die_t *die);


/*
 * Write a page.
 *   page     - flat page index
 *   data     - pointer to writesz bytes of page data (physically contiguous)
 *
 * Returns 0 on success, negative on error.
 */
int flashdrv_write(nand_die_t *die, uint32_t page, const void *data);


/*
 * Read a page.
 *   page      - flat page index
 *   data      - buffer to receive writesz bytes of page data (physically contiguous)
 *   eccStatus - ECC status
 *
 * Returns 0, -EUCLEAN (corrected, degraded), or -EBADMSG (uncorrectable).
 */
int flashdrv_read(nand_die_t *die, uint32_t page, void *data, flashdrv_eccStatus_t *eccStatus);


/*
 * Write raw bytes at an arbitrary page+offset (EDAC disabled).
 *   page     - flat page index
 *   pageOffs - byte offset within the page (0..writesz+sparesz-1)
 *   data     - source buffer
 *   size     - number of bytes to write
 */
int flashdrv_writeraw(nand_die_t *die, uint32_t page, uint32_t pageOffs, const void *data, size_t size);


/*
 * Read raw bytes at an arbitrary page+offset (EDAC disabled).
 *   page     - flat page index
 *   pageOffs - byte offset within the page (0..writesz+sparesz-1)
 *   data     - destination buffer
 *   size     - number of bytes to read
 */
int flashdrv_readraw(nand_die_t *die, uint32_t page, uint32_t pageOffs, void *data, size_t size);


/*
 * Write spare (OOB) bytes for a page with ECC disabled.
 *   page - flat page index
 *   data - source buffer (must be physically contiguous)
 *   size - number of bytes to write (must be <= spareavail)
 *
 * Returns 0 on success, negative on error.
 */
int nandfctrl2_writemeta(nand_die_t *die, uint32_t page, const void *data, size_t size);


/*
 * Read spare (OOB) bytes for a page with ECC disabled.
 *   page - flat page index
 *   data - destination buffer (must be physically contiguous)
 *   size - number of bytes to read (must be <= spareavail)
 *
 * Returns 0 on success, negative on error.
 */
int nandfctrl2_readmeta(nand_die_t *die, uint32_t page, void *data, size_t size);


/*
 * Erase a block.
 *   block - block index
 *
 * Returns 0 on success, negative on error.
 */
int flashdrv_erase(nand_die_t *die, uint32_t block);

/*
 * Check if a block is bad (reads bad-block marker byte from OOB).
 *   block - block index
 *
 * Returns 1 if bad, 0 if good, negative on error.
 */
int flashdrv_isbad(const nand_die_t *die, uint32_t block);


/*
 * Mark a block as bad (writes bad-block marker to OOB).
 *   block - block index
 */
int flashdrv_markbad(const nand_die_t *die, uint32_t block);


/*
 * Initialize the NAND controller.  Must be called once before creating any
 * DMA contexts.  Discovers the NANDFCTRL2 device via AMBA Plug-and-Play,
 * maps registers, registers the interrupt handler and queries ONFI parameters
 * for each die defined in board_config.h.
 *
 * Returns 0 on success, negative on error.
 */
int flashdrv_init(void);


#endif /* NANDFCTRL2_FLASHDRV_H_ */
