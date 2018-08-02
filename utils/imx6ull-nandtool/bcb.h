/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND tool.
 *
 * Boot control blocks
 *
 * Copyright 2018 Phoenix Systems
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#ifndef _BCB_H
#define _BCB_H

#include "../../storage/imx6ull-flash/flashdrv.h"


int fcb_flash(flashdrv_dma_t *dma);

int dbbt_flash(flashdrv_dma_t *dma);

#endif /* _BCB_H_ */
