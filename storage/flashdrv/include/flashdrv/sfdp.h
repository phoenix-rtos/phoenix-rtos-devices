/*
 * Phoenix-RTOS
 *
 * SFDP compatible flash info 
 * TBD: leverage SFDP parser to parse data automatically
 *
 * Copyright 2026 Phoenix Systems
 * Author: Amelia Waszkowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHDRV_SFDP_H_
#define _FLASHDRV_SFDP_H_


#include <sys/types.h>

struct nor_info {
	uint32_t jedecId;
	const char *name;
	size_t totalSz;
	size_t pageSz;
	size_t subsectorSz;
	size_t sectorSz;
	time_t tPP; /* Page Program Cycle time */
	time_t tSE; /* Sector Erase Cycle time */
	time_t tCE; /* Chip Erase Cycle time */
    uint8_t stacked; /* number of stacked dice */
};


#endif /* _FLASHDRV_SFDP_H_ */