/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * Internal flash functions
 *
 * Copyright 2023, 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASH_H_
#define _FLASH_H_


#include <sys/types.h>

#include "flashdrv.h"


static inline off_t flash_getSectorOffset(const struct _storage_devCtx_t *ctx, off_t offs)
{
	return offs & ~(ctx->sectorsz - 1);
}


int flash_writeBuffer(const struct _storage_devCtx_t *ctx, off_t offs, const uint8_t *data, size_t len, time_t timeout);

/* Timeout in us */
int flash_sectorErase(const struct _storage_devCtx_t *ctx, off_t sectorOffs, time_t timeout);


int flash_chipErase(const struct _storage_devCtx_t *ctx, time_t timeout);


void flash_read(const struct _storage_devCtx_t *ctx, off_t offs, void *buff, size_t len);


void flash_printInfo(const struct _storage_devCtx_t *ctx);


int flash_init(struct _storage_devCtx_t *ctx);


void flash_register(const cfi_dev_t *dev);


#endif
