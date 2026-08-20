/*
 * Phoenix-RTOS
 *
 * STM32H5 reset and clock controller driver
 *
 * Copyright 2017, 2018, 2020, 2025, 2026 Phoenix Systems
 * Author: Aleksander Kaminski, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/interrupt.h>
#include <sys/threads.h>
#include <sys/pwman.h>
#include <sys/platform.h>

#include "stm32h5_regs.h"
#include "common.h"
#include "rcc.h"
#include "rtc.h"


struct {
	volatile unsigned int *base;
	volatile unsigned int *pwr;

	handle_t lock;
} rcc_common;


int rcc_setClksel(enum ipclks ipclk, unsigned setting)
{
	if ((ipclk < 0) || (ipclk > pctl_ipclks_count)) {
		return -EINVAL;
	}

	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_ipclk,
		.ipclk = {
			.ipclk = ipclk,
			.setting = setting,
		}
	};

	return platformctl(&pctl);
}


void pwr_lockFromIRQ(uint32_t previous)
{
	/* Disable writing to backup domain if it was disabled previously */
	uint32_t tmp = *(rcc_common.pwr + pwr_dbpcr) & ~1u;
	*(rcc_common.pwr + pwr_dbpcr) = tmp | (previous & 1);
}


uint32_t pwr_unlockFromIRQ(void)
{
	uint32_t previous = *(rcc_common.pwr + pwr_dbpcr);
	/* Enable writing to backup domain */
	*(rcc_common.pwr + pwr_dbpcr) = previous | 1;
	return previous & 1;
}


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
