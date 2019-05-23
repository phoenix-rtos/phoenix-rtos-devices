/*
 * Phoenix-RTOS
 *
 * i.MX RT common
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

#include <phoenix/arch/imxrt.h>
#include <arch.h>

#include "config.h"
#include "imxrt-multi.h"


extern unsigned int multi_port;


/* IDs of special files OIDs */
enum { id_uart1 = 1, id_uart2, id_uart3, id_uart4, id_uart5, id_uart6, id_uart7, id_uart8 };


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


int common_setClock(int dev, unsigned int state);


int common_setMux(int mux, char sion, char mode);


int common_setPad(int pad, char hys, char pus, char pue, char pke, char ode, char speed, char dse, char sre);


int common_setInput(int isel, char daisy);

#endif
