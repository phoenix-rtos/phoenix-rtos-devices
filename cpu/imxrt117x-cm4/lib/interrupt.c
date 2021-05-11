/*
 * Phoenix-RTOS
 *
 * i.MX RT117x Cortex-M4 interrupt dispatcher
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stddef.h>
#include "interrupt.h"
#include "cm4.h"

#define SIZE_HANDLERS 4

/* .bss, doesn't need init */
static void (*callbacks[SIZE_INTERRUPTS][SIZE_HANDLERS])(int n);


void interrupt_dispatch(int n)
{
	int i;

	n -= 16;

	for (i = 0; i < SIZE_HANDLERS; ++i) {
		if (callbacks[n][i] != NULL)
			callbacks[n][i](n);
		else
			break;
	}
}


int interrupt_register(int which, void (*callback)(int n), int priority)
{
	int i;

	which -= 16;

	if (which < 0 || which > SIZE_INTERRUPTS || priority > 15)
		return -1;

	for (i = 0; i < SIZE_HANDLERS; ++i) {
		if (callbacks[which][i] == NULL)
			break;
	}

	if (i >= SIZE_HANDLERS)
		return -1;

	callbacks[which][i] = callback;

	if (priority >= 0)
		cm4_nvicSetPriority(which, priority);

	cm4_nvicSetIRQ(which, 1);

	return 0;
}
