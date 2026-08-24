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
#include "../grlib-spimctrl/spimctrl.h"


#define NOR_ERASED_STATE 0xffu
#define NOR_SECTORSZ_MAX 0x10000u
#define NOR_PAGESZ_MAX   0x100u

struct nor_info {
	uint32_t jedecId;
	const char *name;
	size_t totalSz;
	size_t pageSz;
	size_t sectorSz;
	time_t tPP; /* Page Program Cycle time */
	time_t tSE; /* Sector Erase Cycle time */
	time_t tCE; /* Chip Erase Cycle time */
    uint8_t stacked; /* number of stacked dice */
};


extern int nor_waitBusy(struct spimctrl *spimctrl, time_t timeout);


extern int nor_eraseDie(struct spimctrl *spimctrl, time_t timeout, uint8_t selDie);


extern int nor_eraseChip(struct spimctrl *spimctrl, time_t timeout);


extern int nor_eraseSector(struct spimctrl *spimctrl, addr_t addr, time_t timeout);


extern int nor_pageProgram(struct spimctrl *spimctrl, addr_t addr, const void *src, size_t len, time_t timeout);


extern ssize_t nor_readData(struct spimctrl *spimctrl, addr_t addr, void *buff, size_t len);


extern int nor_probe(struct spimctrl *spimctrl, const struct nor_info **nor, const char **pVendor);

#endif /* _SFDP_FLASH_H_ */