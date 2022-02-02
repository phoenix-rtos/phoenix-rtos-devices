/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL ECSPI NOR flash driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHNOR_ECSPI_H_
#define _FLASHNOR_ECSPI_H_

#include <unistd.h>

#include "storage.h"


extern ssize_t flashnor_ecspiRead(unsigned int addr, void *buff, size_t bufflen);


extern ssize_t flashnor_ecspiWrite(unsigned int addr, void *buff, size_t bufflen);


extern int flashnor_ecspiEraseSector(unsigned int addr);


extern int flashnor_ecspiInit(unsigned int ndev, storage_t *dev);


#endif
