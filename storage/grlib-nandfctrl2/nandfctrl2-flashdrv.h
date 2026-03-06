/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash driver.
 * Low-level API for userspace driver using DMA descriptor chains (LLM).
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _NANDFCTRL2_FLASHDRV_H_
#define _NANDFCTRL2_FLASHDRV_H_

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

	
} nand_die_t;



/*
 * Allocate a DMA descriptor chain for the given target (NAND die index).
 * Returns NULL on failure.
 */
flashdrv_dma_t *flashdrv_dmanew(unsigned int target);


/* Free a previously allocated DMA descriptor chain. */
void flashdrv_dmadestroy(flashdrv_dma_t *dma);


/* Reset the NAND device associated with this DMA context. */
int flashdrv_reset(flashdrv_dma_t *dma);


/*
 * Write a page.
 *   page     - flat page index
 *   data     - pointer to writesz bytes of page data (must be physically contiguous)
 *   metadata - optional: pointer to NAND_SPARE_AVAIL bytes of OOB data; NULL to skip
 *
 * Returns 0 on success, negative on error.
 */
int flashdrv_write(flashdrv_dma_t *dma, uint32_t page, const void *data, const void *metadata);


/*
 * Read a page.
 *   page     - flat page index
 *   data  - buffer to receive writesz bytes of page data (contiguous); NULL to skip data read
 *   meta  - optional: populated with OOB data and ECC status; NULL to skip
 *
 * Returns 0, -EUCLEAN (corrected, degraded), or -EBADMSG (uncorrectable).
 */
int flashdrv_read(flashdrv_dma_t *dma, uint32_t page, void *data, flashdrv_eccStatus_t *eccStatus);


/*
 * Erase a block.
 *   block - block index
 *
 * Returns 0 on success, negative on error.
 */
int flashdrv_erase(flashdrv_dma_t *dma, uint32_t block);


/*
 * Write raw bytes at an arbitrary page+offset (EDAC disabled).
 *   page     - flat page index
 *   pageOffs - byte offset within the page (0..writesz+sparesz-1)
 *   data     - source buffer
 *   size     - number of bytes to write
 */
int flashdrv_writeraw(flashdrv_dma_t *dma, uint32_t page, uint32_t pageOffs, const void *data, size_t size);


/*
 * Read raw bytes at an arbitrary page+offset (EDAC disabled).
 *   page     - flat page index
 *   pageOffs - byte offset within the page (0..writesz+sparesz-1)
 *   data     - destination buffer
 *   size     - number of bytes to read
 */
int flashdrv_readraw(flashdrv_dma_t *dma, uint32_t page, uint32_t pageOffs, void *data, size_t size);


/*
 * Check if a block is bad (reads bad-block marker byte from OOB).
 *   block - block index
 *
 * Returns 1 if bad, 0 if good, negative on error.
 */
int flashdrv_isbad(flashdrv_dma_t *dma, uint32_t block);


/*
 * Mark a block as bad (writes bad-block marker to OOB).
 *   block - block index
 */
int flashdrv_markbad(flashdrv_dma_t *dma, uint32_t block);


/*
 * Initialize the NAND controller.  Must be called once before creating any
 * DMA contexts.  Discovers the NANDFCTRL2 device via AMBA Plug-and-Play,
 * maps registers, registers the interrupt handler and queries ONFI parameters
 * for each die defined in board_config.h.
 *
 * Returns 0 on success, negative on error.
 */
int flashdrv_init(void);


/* Return flash parameters for the given target (die) index. */
const flashdrv_info_t *flashdrv_info(unsigned int target);


/* Return the number of NAND targets (dies) available. */
unsigned int flashdrv_ndies(void);


#endif /* _NANDFCTRL2_FLASHDRV_H_ */
