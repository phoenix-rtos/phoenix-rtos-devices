/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL ECSPI NOR flash driver
 *
 * Copyright 2021, 2023 Phoenix Systems
 * Author: Lukasz Kosinski, Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHNOR_ECSPI_H_
#define _FLASHNOR_ECSPI_H_

#include <unistd.h>

#include "imx6ull-flashnor-drv.h"


extern int flashnor_ecspiInit(int ndev, flashnor_info_t *info);


#endif
