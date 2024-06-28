/*
 * Phoenix-RTOS
 *
 * MCX N94x common
 *
 * Copyright 2017, 2018, 2024 Phoenix Systems
 * Author: Aleksander Kaminski, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <arch.h>
#include <board_config.h>
#include <phoenix/arch/armv8m/mcx/n94x/mcxn94x.h>


#define NELEMS(x) (sizeof(x) / sizeof(*(x)))


static inline void common_dataBarrier(void)
{
	__asm__ volatile ("dmb");
}


static inline void common_dataSyncBarrier(void)
{
	__asm__ volatile ("dsb");
}


static inline void common_instrBarrier(void)
{
	__asm__ volatile ("isb");
}


#endif
