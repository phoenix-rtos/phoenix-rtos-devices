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

#include <stdint.h>
#include <board_config.h>
#include <flashdrv/cacheManage.h>

#include <stdio.h>

/* void hal_cpuDCacheInval(void *va, size_t size)
 * va must have write access
 */

void hal_cpuDCacheInval(void *va, size_t size)
{
#ifdef DCACHE_BLOCK_SIZE
	uintptr_t addr = (uintptr_t)va;
	size_t blockSize = DCACHE_BLOCK_SIZE;

	uintptr_t alignedAddr = addr & ~(blockSize - 1U);
	uintptr_t end = (addr + size + blockSize - 1U) & ~(blockSize - 1U);

	__asm__ __volatile__("fence w, r" ::: "memory");

	while (alignedAddr < end) {
		__asm__ __volatile__("cbo.inval (%0)" ::"r"(alignedAddr) : "memory");
		alignedAddr += blockSize;
	}

	// printf("cache invalidated \n");

	__asm__ __volatile__("fence r, r" ::: "memory");
#else
	(void)va;
	(void)size;
#endif
}


/* void hal_cpuDCacheFlush(void *va, size_t size)
 * va must have write access
 */
void hal_cpuDCacheFlush(void *va, size_t size)
{
#ifdef DCACHE_BLOCK_SIZE
	uintptr_t addr = (uintptr_t)va;
	size_t blockSize = DCACHE_BLOCK_SIZE;

	/* Align size to cache block size */
	size = (size + blockSize - 1) & ~(blockSize - 1);

	uintptr_t end = addr + size;

	while (addr < end) {
		__asm__ __volatile__("cbo.flush (%0)" ::"r"(addr) : "memory");
		addr += blockSize;
	}
#else
	(void)va;
	(void)size;
#endif
}
