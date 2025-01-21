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

#include <stdio.h>
#include <sys/types.h>
#include <storage/storage.h>

/* clang-format off */
#define LOG(fmt, ...) do { (void)fprintf(stdout, "ftmctrl-flashsrv: " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(fmt, ...) do { (void)fprintf(stdout, "ftmctrl-flashsrv:%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } while (0)
#define TRACE(fmt, ...) do { if (0) { (void)fprintf(stdout, "ftmctrl-flashsrv:%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } } while (0)
/* clang-format on */


struct _storage_devCtx_t {
	cfi_info_t cfi;
	const struct _flash_dev_t *dev;

	handle_t lock;
	void *ftmctrl;
	size_t sectorsz;
};


const storage_mtdops_t *flashdrv_getMtdOps(void);


struct _storage_devCtx_t *flashdrv_contextInit(void);


void flashdrv_contextDestroy(struct _storage_devCtx_t *ctx);


#endif
