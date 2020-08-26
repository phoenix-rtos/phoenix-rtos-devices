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

#include <sys/io.h>
#include "uart16550.h"

typedef struct {
	void* base;
} uartpc_t;

static uartpc_t uartpc;

unsigned char uart_speed;


void uart_reginit(void)
{
	uartpc.base = (void *)0x3f8;
	uart_speed = BPS_115200;
}

unsigned char uart_regrd(unsigned int reg)
{
	return inb(uartpc.base + reg);
}

void uart_regwr(unsigned int reg, unsigned char val)
{
	outb(uartpc.base + reg, val);
}