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

#include "../spimctrl.h"

#define NOR_ERASED_STATE 0xff
#define NOR_SECTORSZ_MAX 0x1000
#define NOR_PAGESZ_MAX   0x100


struct nor_info {
	uint32_t jedecId;
	const char *name;
	size_t totalSz;
	size_t pageSz;
	size_t sectorSz;
	time_t tPP;
	time_t tSE;
	time_t tCE;
};


extern int nor_waitBusy(spimctrl_t *spimctrl, time_t timeout);


extern int nor_eraseChip(spimctrl_t *spimctrl, time_t timeout);


extern int nor_eraseSector(spimctrl_t *spimctrl, addr_t addr, time_t timeout);


extern int nor_pageProgram(spimctrl_t *spimctrl, addr_t addr, const void *src, size_t len, time_t timeout);


extern ssize_t nor_readData(spimctrl_t *spimctrl, addr_t addr, void *buff, size_t len);


extern int nor_probe(spimctrl_t *spimctrl, const struct nor_info **nor, const char **pVendor);


#endif /* _NOR_H_ */
