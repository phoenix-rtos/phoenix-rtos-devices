/*
 * Phoenix-RTOS
 *
 * i.MX RT117x Cortex-M4 core start code
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stddef.h>

#include "cm4.h"
#include "gpio.h"
#include "string.h"
#include "mu.h"

extern char __bss_start;
extern char _end;
extern int main(int argc, char *argv[]);


__attribute__((noreturn)) void _startc(void)
{
	memset(&__bss_start, 0, &_end - &__bss_start);

	/* TODO .data init */

	/* Platform init */
	cm4_init();
	gpio_init();
	mu_init();

	__asm__ volatile ("cpsie if");

	main(0, NULL);

	for (;;)
		;
}
