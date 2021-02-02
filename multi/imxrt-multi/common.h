/*
 * Phoenix-RTOS
 *
 * i.MX RT common
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <arch.h>

#include "config.h"
#include "imxrt-multi.h"

#define CONCATENATE(x, y) x##y
#define PIN2MUX(x) CONCATENATE(pctl_mux_gpio_, x)
#define PIN2PAD(x) CONCATENATE(pctl_pad_gpio_, x)


extern unsigned int multi_port;


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
