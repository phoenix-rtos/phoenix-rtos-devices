/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4/N6/U3 reset and clock controller driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Aleksander Kaminski, Mateusz Karcz, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef RCC_COMMON_H_
#define RCC_COMMON_H_


#include <phoenix/types.h>
#include <stdint.h>


void pwr_lock(void);


void pwr_lockFromIRQ(uint32_t previous);


void pwr_unlock(void);


uint32_t pwr_unlockFromIRQ(void);


typedef struct {
	volatile unsigned int *base;
	volatile unsigned int *pwr;

	handle_t lock;
} rcc_common_t;


extern rcc_common_t rcc_common;


#endif
