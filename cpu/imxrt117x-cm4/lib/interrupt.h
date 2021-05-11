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

#ifndef M4_INTERRUPT_H_
#define M4_INTERRUPT_H_

#define SIZE_INTERRUPTS 217

#ifndef __ASSEMBLY__

int interrupt_register(int which, void (*callback)(int n), int priority);

#endif

#endif
