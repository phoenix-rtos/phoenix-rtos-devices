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

#ifndef _FTMCTRL_FLASH_H_
#define _FTMCTRL_FLASH_H_


#include <sys/types.h>

#include "flashdrv.h"


typedef struct {
	uint8_t (*statusRead)(volatile uint8_t *base);
	int (*statusCheck)(volatile uint8_t *base, const char *msg);
	void (*statusClear)(volatile uint8_t *base);
	void (*issueReset)(volatile uint8_t *base);
	void (*issueWriteBuffer)(volatile uint8_t *base, off_t sectorOffs, off_t programOffs, size_t len);
	void (*issueWriteConfirm)(volatile uint8_t *base, off_t sectorOffs);
	void (*issueSectorErase)(volatile uint8_t *base, off_t sectorOffs);
	void (*issueChipErase)(volatile uint8_t *base);
	void (*enterQuery)(volatile uint8_t *base, off_t sectorOffs);
	void (*exitQuery)(volatile uint8_t *base);
} flash_ops_t;


typedef struct _flash_dev_t {
	const uint8_t statusRdyMask;
	const uint8_t usePolling;
	const uint8_t chipWidth;
	const uint8_t vendor;
	const uint16_t device;
	const char *name;
	const flash_ops_t *ops;
} flash_dev_t;


int ftmctrl_flash_writeBuffer(const struct _storage_devCtx_t *ctx, off_t offs, const uint8_t *data, size_t len, time_t timeout);

/* Timeout in us */
int ftmctrl_flash_sectorErase(const struct _storage_devCtx_t *ctx, off_t sectorOffs, time_t timeout);


int ftmctrl_flash_chipErase(const struct _storage_devCtx_t *ctx, time_t timeout);


void ftmctrl_flash_read(const struct _storage_devCtx_t *ctx, off_t offs, void *buff, size_t len);


void ftmctrl_flash_printInfo(const struct _storage_devCtx_t *ctx);


int ftmctrl_flash_init(struct _storage_devCtx_t *ctx, addr_t flashBase);


void ftmctrl_flash_destroy(struct _storage_devCtx_t *ctx);


void ftmctrl_flash_register(const flash_dev_t *dev);


void ftmctrl_amd_register(void);


void ftmctrl_intel_register(void);


#endif
