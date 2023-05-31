/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL QSPI NOR flash driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHNOR_QSPI_H_
#define _FLASHNOR_QSPI_H_

#include "imx6ull-flashnor-drv.h"


extern int flashnor_qspiInit(int ndev, flashnor_info_t *info);


#endif
