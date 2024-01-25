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

#include <board_config.h>

#include "uart16550.h"


typedef struct {
	volatile uint8_t *base;
	uint8_t irq;
} uarthw_ctx_t;


uint8_t uarthw_read(void *hwctx, unsigned int reg)
{
	return *(((uarthw_ctx_t *)hwctx)->base + reg);
}


void uarthw_write(void *hwctx, unsigned int reg, uint8_t val)
{
	*(((uarthw_ctx_t *)hwctx)->base + reg) = val;
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

	base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)UART16550_BASE);
	if (base == MAP_FAILED) {
		return -ENOMEM;
	}

	((uarthw_ctx_t *)hwctx)->base = base;
	((uarthw_ctx_t *)hwctx)->irq = UART16550_IRQ;

	/* Detect device presence */
	if (uarthw_read(hwctx, REG_IIR) == 0xff) {
		munmap(base, _PAGE_SIZE);
		return -ENODEV;
	}

	*fclk = 115200 * 16;

	return EOK;
}
