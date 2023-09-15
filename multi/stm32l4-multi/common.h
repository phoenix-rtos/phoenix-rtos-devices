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
#include <phoenix/arch/stm32l4.h>
#include <sys/platform.h>


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
#define DEBUG(format, ...) printf("%s: "format, drvname, ##__VA_ARGS__)
#endif


static inline void dataBarier(void)
{
	__asm__ volatile ("dmb");
}


static inline int devClk(int dev, int state)
{
	int ret;
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_devclk;
	pctl.devclk.dev = dev;
	pctl.devclk.state = state;

	ret = platformctl(&pctl);

	return ret;
}


static inline int getCpufreq(void)
{
	platformctl_t pctl;

	pctl.action = pctl_get;
	pctl.type = pctl_cpuclk;
	platformctl(&pctl);

	return pctl.cpuclk.hz;
}

#endif
