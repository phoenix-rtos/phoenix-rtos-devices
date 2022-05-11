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


extern int flashdev_done(storage_t *strg);


extern int flashdev_init(storage_t *strg);


#endif
