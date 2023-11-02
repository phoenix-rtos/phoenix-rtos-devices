/*
 * Phoenix-RTOS
 *
 * UART 16550 device driver
 *
 * Hardware abstraction layer (armv7a9-zynq7000)
 *
 * Copyright 2022 Phoenix Systems
 * Author: Aleksander Kaminski
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

#ifndef UART16550_BASE0
#define UART16550_BASE0 0x43c10000
#endif

#ifndef UART16550_IRQ0
#define UART16550_IRQ0 61
#endif

#ifndef UART16550_BASE1
#define UART16550_BASE1 0x43c20000
#endif

#ifndef UART16550_IRQ1
#define UART16550_IRQ1 62
#endif

#ifndef UART16550_BASE2
#define UART16550_BASE2 0x43c30000
#endif

#ifndef UART16550_IRQ2
#define UART16550_IRQ2 63
#endif

#ifndef UART16550_BASE3
#define UART16550_BASE3 0
#endif

#ifndef UART16550_IRQ3
#define UART16550_IRQ3 0
#endif


typedef struct {
	volatile uint32_t *base;
	unsigned int irq;
} uarthw_ctx_t;


uint8_t uarthw_read(void *hwctx, unsigned int reg)
{
	volatile uint32_t *p = (((uarthw_ctx_t *)hwctx)->base + reg);

	return ((uint8_t)(*p));
}


void uarthw_write(void *hwctx, unsigned int reg, uint8_t val)
{
	volatile uint32_t *p = (((uarthw_ctx_t *)hwctx)->base + reg);

	*p = val;
}


char *uarthw_dump(void *hwctx, char *s, size_t sz)
{
	snprintf(s, sz, "base=0x%p irq=%u", (volatile void *)(((uarthw_ctx_t *)hwctx)->base), ((uarthw_ctx_t *)hwctx)->irq);
	return s;
}


unsigned int uarthw_irq(void *hwctx)
{
	return ((uarthw_ctx_t *)hwctx)->irq;
}


int uarthw_init(unsigned int uartn, void *hwctx, size_t hwctxsz, unsigned int *fclk)
{
	void *base;
	static const struct {
		uint32_t base;
		unsigned int irqno;
	} uarthw_info[] = {
		{ .base = UART16550_BASE0, .irqno = UART16550_IRQ0 },
		{ .base = UART16550_BASE1, .irqno = UART16550_IRQ1 },
		{ .base = UART16550_BASE2, .irqno = UART16550_IRQ2 },
		{ .base = UART16550_BASE3, .irqno = UART16550_IRQ3 },
	};

	if (hwctxsz < sizeof(uarthw_ctx_t)) {
		return -EINVAL;
	}

	if (uartn >= sizeof(uarthw_info) / sizeof(uarthw_info[0]) || uarthw_info[uartn].base == 0) {
		return -ENODEV;
	}

	base = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, uarthw_info[uartn].base + 0x1000);
	if (base == MAP_FAILED) {
		return -ENOMEM;
	}

	((uarthw_ctx_t *)hwctx)->base = base;
	((uarthw_ctx_t *)hwctx)->irq = uarthw_info[uartn].irqno;

	/* Detect device presence */
	if (uarthw_read(hwctx, REG_IIR) == 0xff) {
		munmap(base, _PAGE_SIZE);
		return -ENODEV;
	}

	*fclk = 100 * 1000 * 1000;

	return EOK;
}
