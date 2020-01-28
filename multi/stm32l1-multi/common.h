/*
 * Phoenix-RTOS
 *
 * STM32L1 multidriver common
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>
#include <stdio.h>
#include <phoenix/arch/stm32l1.h>

#include "config.h"


#define max(a, b) ({ \
	__typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
	_a > _b ? _a : _b; \
})


#define min(a, b) ({ \
	__typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
	_a > _b ? _b : _a; \
})


#ifdef NDEBUG
#define DEBUG(format, ...)
#else
#define DEBUG(format, ...) printf("%s: "format, drvname, ##__VA_ARGS__)
#endif


static inline void dataBarier(void)
{
	__asm__ volatile ("dmb");
}


static inline uint32_t getPC(void)
{
	uint32_t ret;

	__asm__ volatile ("mov %0, pc" : "=r" (ret));

	return ret;
}

#endif
