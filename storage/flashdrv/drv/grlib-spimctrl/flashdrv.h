/*
 * Phoenix-RTOS
 *
 * GRLIB SPIMCTRL Flash driver
 *
 * Copyright 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _SPIMCTRL_FLASHDRV_H_
#define _SPIMCTRL_FLASHDRV_H_


#include <cache.h>
#include <storage/storage.h>
#include <flashdrv/cfi.h>
#include <flashdrv/flashsrv.h>


struct cache_devCtx_s {
	struct _storage_t *strg;
};


struct _storage_devCtx_t {
	cfi_info_t cfi;
	const struct flash_dev *dev;

	struct spimctrl *spimctrl;

	handle_t lock;
	size_t sectorsz;

	cachectx_t *cache;
	cache_devCtx_t cacheCtx;
};


#endif
