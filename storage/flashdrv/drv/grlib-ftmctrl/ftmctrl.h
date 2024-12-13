/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * FTMCTRL routines
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FTMCTRL_H_
#define _FTMCTRL_H_


#include <stdint.h>


static inline void ftmctrl_WrEn(volatile uint32_t *ftmctrl)
{
	*ftmctrl |= (1 << 11);
}


static inline void ftmctrl_WrDis(volatile uint32_t *ftmctrl)
{
	*ftmctrl &= ~(1 << 11);
}


static inline void ftmctrl_ioEn(volatile uint32_t *ftmctrl)
{
	*ftmctrl |= (1 << 19);
}


static inline void ftmctrl_ioDis(volatile uint32_t *ftmctrl)
{
	*ftmctrl &= ~(1 << 19);
}


static inline int ftmctrl_portWidth(volatile uint32_t *ftmctrl)
{
	return (((*ftmctrl >> 8) & 0x3) == 0) ? 8 : 16;
}


#endif
