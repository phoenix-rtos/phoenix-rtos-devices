/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * UART 16450 driver
 *
 * Copyright 2012-2015 Phoenix Systems
 * Copyright 2001, 2005, 2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Pawel Kolodziej
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "uart16550.h"
#include <sys/mman.h>

typedef struct {
	void* base;
} uartriscv_t;

uartriscv_t uartriscv;

unsigned char uart_speed;


void uart_reginit(void)
{
	void *uart_addr = (void *)0x10000000;
	uartriscv.base = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, (offs_t)uart_addr);
	uart_speed = BPS_38400;
}

unsigned char uart_regrd(unsigned int reg)
{
	unsigned char b;
	b = *((char*)uartriscv.base + reg);
	return b;
}

void uart_regwr(unsigned int reg, unsigned char val)
{
	*((char*)uartriscv.base + reg) = val;
}