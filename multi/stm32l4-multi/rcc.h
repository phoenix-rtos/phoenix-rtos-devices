/*
 * Phoenix-RTOS
 *
 * STM32L4/N6 reset and clock controller driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Aleksander Kaminski, Mateusz Karcz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef RCC_H_
#define RCC_H_


#include <stdint.h>

#if defined(__CPU_STM32N6)
#include "clockdef_n6.h"


int rcc_setClksel(enum ipclks ipclk, enum clock_ids clkID);
#endif


void pwr_lock(void);


void pwr_lockFromIRQ(uint32_t previous);


void pwr_unlock(void);


uint32_t pwr_unlockFromIRQ(void);


int rcc_init(void);


typedef struct {
	volatile unsigned int *base;
	volatile unsigned int *pwr;

	handle_t lock;
} rcc_common_t;


extern rcc_common_t rcc_common;


#endif
