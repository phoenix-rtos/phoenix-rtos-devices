/*
 * Phoenix-RTOS
 *
 * UART 16550 device driver
 *
 * Hardware abstracion layer (riscv64-virt)
 *
 * Copyright 2020 Phoenix Systems
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
	volatile uint8_t *base;
	uint8_t irq;
} uarthw_ctx_t;


uint8_t uarthw_read(void *hwctx, unsigned int reg)
{
	uint8_t b;

	b = *(((uarthw_ctx_t *)hwctx)->base + reg);
	return b;
}


void uarthw_write(void *hwctx, unsigned int reg, uint8_t val)
{
	*(((uarthw_ctx_t *)hwctx)->base + reg) = val;
	return;
}


char *uarthw_dump(void *hwctx, char *s, size_t sz)
{
	snprintf(s, sz, "base=%p irq=%d", ((uarthw_ctx_t *)hwctx)->base, ((uarthw_ctx_t *)hwctx)->irq);
	return s;
}


unsigned int uarthw_irq(void *hwctx)
{
	return ((uarthw_ctx_t *)hwctx)->irq;
}


int uarthw_init(unsigned int uartn, void *hwctx, size_t hwctxsz)
{
	if (hwctxsz < sizeof(uarthw_ctx_t))
		return -EINVAL;

	if (uartn >= 1)
		return -ENOENT;

	if ((((uarthw_ctx_t *)hwctx)->base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, (offs_t)0x10000000)) == NULL)
		return -ENOMEM;

	((uarthw_ctx_t *)hwctx)->irq = 0xa;

	/* Detect device presence */
	if (uarthw_read(hwctx, REG_IIR) == 0xff) {
		munmap((void *)((uarthw_ctx_t *)hwctx)->base, _PAGE_SIZE);
		return -ENOENT;
	}

	return EOK;
}
