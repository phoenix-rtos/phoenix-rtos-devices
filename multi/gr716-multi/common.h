/*
 * Phoenix-RTOS
 *
 * GR716 multi driver - common definitions
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _GR716_MULTI_COMMON_H_
#define _GR716_MULTI_COMMON_H_


#include <stdint.h>


#define SYSCLK_FREQ (50 * 1000 * 1000)

/* Atomic access offsets (APB peripherals) */

#define ATOMIC_AND_OFFS 0x20000
#define ATOMIC_OR_OFFS  0x40000
#define ATOMIC_XOR_OFFS 0x60000
#define ATOMIC_SNC_OFFS 0x80000


static inline void common_atomicAnd(volatile uint32_t *reg, uint32_t val)
{
	*(reg + (ATOMIC_AND_OFFS / sizeof(*reg))) = val;
}


static inline void common_atomicOr(volatile uint32_t *reg, uint32_t val)
{
	*(reg + (ATOMIC_OR_OFFS / sizeof(*reg))) = val;
}


static inline void common_atomicXor(volatile uint32_t *reg, uint32_t val)
{
	*(reg + (ATOMIC_XOR_OFFS / sizeof(*reg))) = val;
}


#endif
