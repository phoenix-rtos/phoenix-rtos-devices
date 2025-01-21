/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHDRV_H_
#define _FLASHDRV_H_

#include "cfi.h"

#include <cache.h>
#include <stdio.h>
#include <sys/types.h>
#include <storage/storage.h>

/* clang-format off */
#define LOG(fmt, ...) do { (void)fprintf(stdout, "ftmctrl-flashsrv: " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(fmt, ...) do { (void)fprintf(stdout, "ftmctrl-flashsrv:%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } while (0)
#define TRACE(fmt, ...) do { if (0) { (void)fprintf(stdout, "ftmctrl-flashsrv:%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } } while (0)
/* clang-format on */


struct cache_devCtx_s {
	struct _storage_t *strg;
};


struct _storage_devCtx_t {
	cfi_info_t cfi;
	const struct _flash_dev_t *dev;

	handle_t lock;
	void *ftmctrl;
	size_t sectorsz;

	cachectx_t *cache;
	cache_devCtx_t cacheCtx;
};


const storage_mtdops_t *flashdrv_getMtdOps(void);


struct _storage_devCtx_t *flashdrv_contextInit(void);


int flashdrv_cacheInit(storage_t *strg);


void flashdrv_contextDestroy(struct _storage_devCtx_t *ctx);


#endif
