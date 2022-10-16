/*
 * Phoenix-RTOS
 *
 * nRF91 multidriver common
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
#include <phoenix/arch/nrf9160.h>
#include <sys/platform.h>

#include "config.h"

#ifdef NDEBUG
#define DEBUG(format, ...)
#else
#define DEBUG(format, ...) printf("%s: "format, drvname, ##__VA_ARGS__)
#endif


static inline void dataBarier(void)
{
	__asm__ volatile ("dmb");
}

#endif
