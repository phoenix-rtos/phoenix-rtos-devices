/*
 * Phoenix-RTOS
 *
 * STM32L4/N6 reset and clock controller driver common code
 *
 * Copyright 2017, 2018, 2020, 2025 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Aleksander Kaminski, Jacek Maksymowicz, Mateusz Karcz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <sys/threads.h>

#include "../common.h"
#include "../rcc.h"


rcc_common_t rcc_common;


void pwr_lock(void)
{
	mutexLock(rcc_common.lock);
	pwr_lockFromIRQ(0);
	mutexUnlock(rcc_common.lock);
}


void pwr_unlock(void)
{
	mutexLock(rcc_common.lock);
	(void)pwr_unlockFromIRQ();
	mutexUnlock(rcc_common.lock);
}


int rcc_init(void)
{
	rcc_common.base = RCC_BASE;
	rcc_common.pwr = PWR_BASE;

	mutexCreate(&rcc_common.lock);

	return 0;
}
