/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * Copyright 2023-2025 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _FTMCTRL_FLASHDRV_H_
#define _FTMCTRL_FLASHDRV_H_


#include <cache.h>
#include <storage/storage.h>
#include <flashdrv/cfi.h>
#include <flashdrv/flashsrv.h>


struct cache_devCtx_s {
	struct _storage_t *strg;
};


struct _storage_devCtx_t {
	cfi_info_t cfi;
	const struct _flash_dev_t *dev;

	void *ftmctrl;

	handle_t lock;
	size_t sectorsz;

	cachectx_t *cache;
	cache_devCtx_t cacheCtx;
};


#endif
