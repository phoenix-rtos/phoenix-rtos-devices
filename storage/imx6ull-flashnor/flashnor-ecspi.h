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

#include "storage.h"


extern int flashnor_ecspiInit(unsigned int ndev, storage_t *dev);


#endif
