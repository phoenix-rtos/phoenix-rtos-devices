/*
 * Phoenix-RTOS
 *
 * Operating system loader
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


#define FTMCTRL_BASE 0x80000000


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


#endif
