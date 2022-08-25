/*
 * Phoenix-RTOS
 *
 * UART 16550 device driver
 *
 * Hardware abstracion layer (ia32-generic)
 *
 * Copyright 2020-2022 Phoenix Systems
 * Author: Julia Kosowska, Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <sys/io.h>
#include <errno.h>

#include "uart16550.h"


typedef struct {
	void *base;
	unsigned int irq;
} uarthw_ctx_t;


uint8_t uarthw_read(void *hwctx, unsigned int reg)
{
	return inb(((uarthw_ctx_t *)hwctx)->base + reg);
}


void uarthw_write(void *hwctx, unsigned int reg, uint8_t val)
{
	outb(((uarthw_ctx_t *)hwctx)->base + reg, val);
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
	static struct {
		void *base;
		unsigned int irq;
	} uarts[] = { { (void *)0x3f8, 4 }, { (void *)0x2f8, 3 }, { (void *)0x3e8, 4 }, { (void *)0x2e8, 3 } };

	if (hwctxsz < sizeof(uarthw_ctx_t)) {
		return -EINVAL;
	}

	if (uartn >= 4) {
		return -ENODEV;
	}

	((uarthw_ctx_t *)hwctx)->base = uarts[uartn].base;
	((uarthw_ctx_t *)hwctx)->irq = uarts[uartn].irq;

	/* Detect device presence */
	if (uarthw_read(hwctx, REG_IIR) == 0xff) {
		return -ENODEV;
	}

	*fclk = 115200 * 16;

	return EOK;
}
