/*
 * Phoenix-RTOS
 *
 * STM32L4 multidriver common
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
#include <sys/platform.h>

#if defined(__CPU_STM32L4X6)
#include "stm32l4x6_base.h"
#include <phoenix/arch/armv7m/stm32/l4/stm32l4.h>
#elif defined(__CPU_STM32N6)
#include "stm32n6_base.h"
#include <phoenix/arch/armv8m/stm32/n6/stm32n6.h>
#else
#error "Unknown platform"
#endif


#define NELEMS(x) (sizeof(x) / sizeof(x[0]))


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
#define DEBUG(format, ...) printf("%s: " format, drvname, ##__VA_ARGS__)
#endif

/* 'Barrier' is spelled with double r. Consider renaming the function */
static inline void dataBarier(void)
{
	__asm__ volatile("dmb");
}

static inline void syncBarrier(void)
{
	__asm__ volatile("dsb");
}

static inline int devClk(int dev, int state)
{
	int ret;
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_devclk;
	pctl.devclk.dev = dev;
	pctl.devclk.state = state;
#if defined(__CPU_STM32N6)
	pctl.devclk.lpState = state;
#endif

	ret = platformctl(&pctl);

	return ret;
}


static inline unsigned int getCpufreq(void)
{
	platformctl_t pctl;

	pctl.action = pctl_get;
	pctl.type = pctl_cpuclk;
	platformctl(&pctl);

	return pctl.cpuclk.hz;
}

#endif
