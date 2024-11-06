/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * Common auxiliary functions
 *
 * Copyright 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHDRV_COMMON_H_
#define _FLASHDRV_COMMON_H_


#include <stdbool.h>
#include <storage/storage.h>


off_t common_getSectorOffset(size_t sectorsz, off_t offs);


bool common_isValidAddress(size_t memsz, off_t offs, size_t len);


#endif
