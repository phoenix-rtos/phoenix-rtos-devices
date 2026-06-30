/*
 * Phoenix-RTOS
 *
 * Zynq Flash forwarder storage device
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jakub Klimek
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef _ZYNQ_FLASH_FWD_H_
#define _ZYNQ_FLASH_FWD_H_


#include <storage/storage.h>
#include <time.h>


typedef struct {
	storage_t *targetStrg;
	int (*limitF)(void *data, size_t wsize, size_t rsize);
	struct timespec lastOp;
	size_t readbwBudget;
	size_t readmaxBudget;
	size_t writebwBudget;
	size_t writemaxBudget;
	size_t rateBudget;
	size_t maxRate;
} devlimit_data_t;


extern const storage_mtdops_t mtdFwOps;


extern const storage_blkops_t blkFwOps;


#endif
