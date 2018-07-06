/*
 * Phoenix-RTOS
 *
 * STM32L1 reset and clock controler driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski
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

#include "common.h"
#include "rtc.h"


struct {
	volatile unsigned int *base;
	volatile unsigned int *pwr;

	handle_t cond;
	handle_t lock;
	handle_t inth;

	int hsiState;
} rcc_common;


enum { cr = 0, icscr, cfgr, cir, ahbrstr, apb2rstr, apb1rstr, ahbenr, apb2enr,
	apb1enr, ahblpenr, apb2lpenr, apb1lpenr, csr };


enum { pwr_cr = 0, pwr_csr };


static int rcc_irqHsiReady(unsigned int n, void *arg)
{
	if ((*(rcc_common.base + cr) & 3) == 3) {
		/* Clear IRQ */
		*(rcc_common.base + cir) |= 1 << 18;
		dataBarier();

		return 1;
	}

	return -1;
}


static inline void _pwr_lock(void)
{
	*(rcc_common.pwr + pwr_cr) &= ~(1 << 8);
}


static inline void _pwr_unlock(void)
{
	*(rcc_common.pwr + pwr_cr) |= 1 << 8;
}


int rcc_setHsi(int state)
{
	mutexLock(rcc_common.lock);
	if (!rcc_common.hsiState == !state) {
		/* HSI's already in the desired state */
		mutexUnlock(rcc_common.lock);
		return EOK;
	}

	if (!state) {
		*(rcc_common.base + cr) &= ~1;
		dataBarier();
		keepidle(0);
	}
	else if (!rcc_common.hsiState) {
		keepidle(1);

		/* Enable the HSI ready interrupt */
		*(rcc_common.base + cir) |= 1 << 10;
		dataBarier();

		/* Enable the HSI clock */
		*(rcc_common.base + cr) |= 1;
		dataBarier();

		/* Wait for the clock to stabilize */
		while (!(*(rcc_common.base + cr) & 2))
			condWait(rcc_common.cond, rcc_common.lock, 0);

		/* Disable the HSI ready interrupt */
		*(rcc_common.base + cir) &= ~(1 << 10);
		dataBarier();
	}

	rcc_common.hsiState = state;

	mutexUnlock(rcc_common.lock);

	return EOK;
}


int rcc_devClk(int dev, int state)
{
	int ret;
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_devclk;
	pctl.devclk.dev = dev;
	pctl.devclk.state = state;

	ret = platformctl(&pctl);

	return ret;
}


int rcc_getCpufreq(void)
{
	platformctl_t pctl;

	pctl.action = pctl_get;
	pctl.type = pctl_cpuclk;
	platformctl(&pctl);

	return pctl.cpuclk.hz;
}


void pwr_lock(void)
{
	mutexLock(rcc_common.lock);
	_pwr_lock();
	mutexUnlock(rcc_common.lock);
}


void pwr_unlock(void)
{
	mutexLock(rcc_common.lock);
	_pwr_unlock();
	mutexUnlock(rcc_common.lock);
}


int rcc_init(void)
{
	rcc_common.base = (void *)0x40023800;
	rcc_common.pwr = (void *)0x40007000;
	rcc_common.hsiState = 0;

	_pwr_unlock();

	/* Select LSE as clock source for RTC and LCD. */
	*(rcc_common.base + csr) |= 1 << 16;
	dataBarier();

	_pwr_lock();

	condCreate(&rcc_common.cond);
	mutexCreate(&rcc_common.lock);

	interrupt(rcc_irq, rcc_irqHsiReady, NULL, rcc_common.cond, &rcc_common.inth);

	return 0;
}
