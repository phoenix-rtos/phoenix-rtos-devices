/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND flash device interface
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMX6ULL_FLASHDEV_H_
#define _IMX6ULL_FLASHDEV_H_

#include <storage/storage.h>
#include "imx6ull-flashdrv.h"


/* Storage device context definition */
typedef struct _storage_devCtx_t {
	flashdrv_dma_t *dma;
	void *databuf; /* at least writesz + metasz */
	void *metabuf; /* at least metasz */
	handle_t lock;
} storage_devCtx_t;


typedef struct {
	unsigned int npages; /* Cache size */
	unsigned int reads;  /* Pages requested through the cache */
	unsigned int misses; /* Pages read from the flash to fill the cache */
} flashdev_cachestats_t;


/* Creates the page read cache filled through the given (whole chip) device, npages < 4 leaves it disabled */
extern int flashdev_cacheInit(storage_t *strg, size_t npages);


extern void flashdev_cacheInvalidate(off_t offs, size_t size);


extern void flashdev_cacheStats(flashdev_cachestats_t *stats);


extern int flashdev_done(storage_t *strg);


extern int flashdev_init(storage_t *strg);


#endif
