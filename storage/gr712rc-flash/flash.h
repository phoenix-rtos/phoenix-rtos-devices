/*
 * Phoenix-RTOS
 *
 * GR712RC Flash driver
 *
 * Internal flash functions
 *
 * Copyright 2023 Phoenix Systems
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

/* Timeouts in us */
#define CFI_TIMEOUT_MAX_PROGRAM(typical, maximum) ((1u << typical) * (1u << maximum))
#define CFI_TIMEOUT_MAX_ERASE(typical, maximum)   ((1u << typical) * (1u << maximum) * 1024u)


int flash_writeBuffer(const struct _storage_devCtx_t *ctx, off_t offs, const uint8_t *data, uint8_t len);

/* Timeout in us */
int flash_blockErase(off_t blkAddr, time_t timeout);


void flash_read(const struct _storage_devCtx_t *ctx, off_t offs, void *buff, size_t len);


void flash_printInfo(flash_cfi_t *ctx);


int flash_init(struct _storage_devCtx_t *ctx);


#endif
