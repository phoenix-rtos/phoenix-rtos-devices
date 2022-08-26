/*
 * Phoenix-RTOS
 *
 * UART 16550 device driver
 *
 * Hardware abstraction layer (riscv64-generic)
 *
 * Copyright 2020-2022 Phoenix Systems
 * Author: Julia Kosowska, Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <sys/mman.h>

#include "uart16550.h"


typedef struct {
	uintptr_t base;
	uint8_t irq;
} uarthw_ctx_t;


uint8_t uarthw_read(void *hwctx, unsigned int reg)
{
	volatile uintptr_t p = (((uarthw_ctx_t *)hwctx)->base + reg);

	return (*(volatile uint8_t *)p);
}


void uarthw_write(void *hwctx, unsigned int reg, uint8_t val)
{
	volatile uint64_t p = (uint64_t)(((uarthw_ctx_t *)hwctx)->base + reg);

	*(volatile uint8_t *)p = val;
}


char *uarthw_dump(void *hwctx, char *s, size_t sz)
{
	snprintf(s, sz, "base=%p irq=%u", ((uarthw_ctx_t *)hwctx)->base, ((uarthw_ctx_t *)hwctx)->irq);
	return s;
}


unsigned int uarthw_irq(void *hwctx)
{
	return ((uarthw_ctx_t *)hwctx)->irq;
}


int uarthw_init(unsigned int uartn, void *hwctx, size_t hwctxsz, unsigned int *fclk)
{
	void *base;

	if (hwctxsz < sizeof(uarthw_ctx_t)) {
		return -EINVAL;
	}

	if (uartn >= 1) {
		return -ENODEV;
	}

	base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, (offs_t)0x10000000);
	if (base == MAP_FAILED) {
		return -ENOMEM;
	}

	((uarthw_ctx_t *)hwctx)->base = (uintptr_t)base;
	((uarthw_ctx_t *)hwctx)->irq = 0xa;

	/* Detect device presence */
	if (uarthw_read(hwctx, REG_IIR) == 0xff) {
		munmap(base, _PAGE_SIZE);
		return -ENODEV;
	}

	*fclk = 115200 * 16;

	return EOK;
}
