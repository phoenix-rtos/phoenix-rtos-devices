/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * riscv64 cache management
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski, Amelia Waszkowska
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _CACHE_MANAGE_H_
#define _CACHE_MANAGE_H_


#include <stddef.h>


void hal_cpuDCacheInval(void *va, size_t size);


void hal_cpuDCacheFlush(void *va, size_t size);


#endif /* _CACHE_MANAGE_H_ */
