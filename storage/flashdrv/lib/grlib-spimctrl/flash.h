/*
 * Phoenix-RTOS
 *
 * GR716 flash driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _NOR_H_
#define _NOR_H_


#include <storage/storage.h>

#include <cfi.h>

#include "spimctrl.h"

#define NOR_ERASED_STATE 0xffu
#define NOR_SECTORSZ_MAX 0x10000u
#define NOR_PAGESZ_MAX   0x100u


struct nor_cmds {
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


struct nor_dev {
	cfi_info_t cfi;
	const struct nor_cmds *cmds;

	const char *name;
	const uint8_t vendor;
	const uint16_t device;
};


int flash_waitBusy(struct nor_dev *dev, struct spimctrl *spimctrl, time_t timeout);


int flash_eraseChip(struct nor_dev *dev, struct spimctrl *spimctrl, time_t timeout);


int flash_eraseSector(struct nor_dev *dev, struct spimctrl *spimctrl, addr_t addr, time_t timeout);


int flash_pageProgram(struct nor_dev *dev, struct spimctrl *spimctrl, addr_t addr, const void *src, size_t len, time_t timeout);


ssize_t flash_readData(struct nor_dev *dev, struct spimctrl *spimctrl, addr_t addr, void *buff, size_t len);


int flash_init(struct _storage_devCtx_t *ctx);


void flash_destroy(struct _storage_devCtx_t *ctx);


#endif /* _NOR_H_ */
