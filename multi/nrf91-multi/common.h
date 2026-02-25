/*
 * Phoenix-RTOS
 *
 * nRF91 multidriver common
 *
 * Copyright 2017, 2018, 2023 Phoenix Systems
 * Author: Aleksander Kaminski, Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>
#include <stdio.h>
#include <phoenix/arch/armv8m/nrf/91/nrf9160.h>
#include <sys/platform.h>

#include <board_config.h>


static inline void dataBarier(void)
{
	__asm__ volatile("dmb");
}

#endif
