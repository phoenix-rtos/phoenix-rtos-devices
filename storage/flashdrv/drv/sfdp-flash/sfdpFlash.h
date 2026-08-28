/*
 * Phoenix-RTOS
 *
 * Flash driver for flash devices utilizing SFDP (Serial Flash Discoverable Parameters) table,
 * according to JEDEC Standard JESD216
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski, Amelia Waszkowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _SFDP_FLASH_H_
#define _SFDP_FLASH_H_


#include <storage/storage.h>
#include <flashdrv/sfdp.h>
#include "../grlib-spimctrl/spimctrl.h"


#define NOR_ERASED_STATE 0xffu
#define NOR_SECTORSZ_MAX 0x10000u
#define NOR_PAGESZ_MAX   0x100u


int nor_waitBusy(struct spimctrl *spimctrl, time_t timeout);


int nor_eraseDie(struct spimctrl *spimctrl, time_t timeout, uint8_t selDie);


int nor_eraseChip(struct spimctrl *spimctrl, time_t timeout);


int nor_eraseSubSector(struct spimctrl *spimctrl, addr_t addr, time_t timeout);


int nor_eraseSector(struct spimctrl *spimctrl, addr_t addr, time_t timeout);


int nor_pageProgram(struct spimctrl *spimctrl, addr_t addr, const void *src, size_t len, time_t timeout);


ssize_t nor_readData(struct spimctrl *spimctrl, addr_t addr, void *buff, size_t len);


void nor_printInfo(const struct _storage_devCtx_t *ctx);


int nor_flash_init(struct _storage_devCtx_t *ctx, addr_t flashBase);


void nor_destroy(struct _storage_devCtx_t *ctx);


#endif /* _SFDP_FLASH_H_ */