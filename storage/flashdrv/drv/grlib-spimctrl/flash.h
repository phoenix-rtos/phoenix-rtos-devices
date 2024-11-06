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

#ifndef _SPIMCTRL_FLASH_H_
#define _SPIMCTRL_FLASH_H_


#include <storage/storage.h>

#include <flashdrv/cfi.h>

#include "spimctrl.h"

#define NOR_ERASED_STATE 0xffu
#define NOR_SECTORSZ_MAX 0x10000u
#define NOR_PAGESZ_MAX   0x100u


struct flash_cmds {
	uint8_t rdsr;  /* Read status register */
	uint8_t wren;  /* Write enable */
	uint8_t wrdi;  /* Write disable */
	uint8_t rdear; /* Read bank/extended address register */
	uint8_t wrear; /* Write bank/extended address register */
	uint8_t ce;    /* Chip erase */
	uint8_t se;    /* Sector erase */
	uint8_t pp;    /* Page program */
	uint8_t read;
};


struct flash_dev {
	const char *name;
	const uint8_t vendor;
	const uint16_t device;

	const struct flash_cmds *cmds;
};


int spimctrl_flash_chipErase(const struct _storage_devCtx_t *ctx, time_t timeout);


int spimctrl_flash_sectorErase(const struct _storage_devCtx_t *ctx, addr_t addr, time_t timeout);


int spimctrl_flash_pageProgram(const struct _storage_devCtx_t *ctx, addr_t addr, const void *src, size_t len, time_t timeout);


ssize_t spimctrl_flash_readData(const struct _storage_devCtx_t *ctx, addr_t addr, void *buff, size_t len);


void spimctrl_flash_printInfo(const struct _storage_devCtx_t *ctx);


int spimctrl_flash_init(struct _storage_devCtx_t *ctx, addr_t flashBase);


void spimctrl_flash_destroy(struct _storage_devCtx_t *ctx);


#endif /* _NOR_H_ */
