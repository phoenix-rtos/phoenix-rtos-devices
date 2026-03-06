/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash device interface.
 *
 * MTD storage device context and initialization API.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef NANDFCTRL2_FLASHDEV_H_
#define NANDFCTRL2_FLASHDEV_H_

#include <storage/storage.h>

#include "nandfctrl2-flashdrv.h"


/* Per-device context stored in storage_dev_t::ctx */
typedef struct _storage_devCtx_t {
	nand_die_t *die;
	void *databuf; /* Uncached contiguous buffer, >= writesz bytes */
	void *metabuf; /* Uncached contiguous buffer, >= NAND_SPARE_AVAIL bytes */
	handle_t lock; /* Serialises concurrent access to databuf/metabuf */
} storage_devCtx_t;


/* Initialise the storage_dev_t and MTD layer for strg.  Must be called after
 * flashdrv_init().  For a root (non-partition) storage object strg->parent must
 * be NULL; start/size are set automatically from flash_info. */
int flashdev_init(storage_t *strg, unsigned int target);


/* Release all resources allocated by flashdev_init(). */
void flashdev_done(storage_t *strg);


#endif /* NANDFCTRL2_FLASHDEV_H_ */
