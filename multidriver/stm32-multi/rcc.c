/*
 * Phoenix-RTOS
 *
 * Operating system kernel
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

#include ARCH
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

	handle_t hsiCond;
	handle_t hsiLock;

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

	return 0;
}


int rcc_setHsi(int state)
{
	mutexLock(rcc_common.hsiLock);
	if (!rcc_common.hsiState == !state) {
		/* HSI's already in the desired state */
		mutexUnlock(rcc_common.hsiLock);
		return EOK;
	}

	if (!state) {
		*(rcc_common.base + cr) &= ~1;
		dataBarier();
		keepidle(0);
		mutexUnlock(rcc_common.hsiLock);

		return EOK;
	}

	if (!(*(rcc_common.base + cr) & 2)) {
		keepidle(1);

		/* Enable HSI ready interrupt */
		*(rcc_common.base + cir) |= 1 << 10;
		dataBarier();

		/* Enable HSI clock */
		*(rcc_common.base + cr) |= 1;
		dataBarier();

		/* Wait for clock to stabilize */
		while (!(*(rcc_common.base + cr) & 2))
			condWait(rcc_common.hsiCond, rcc_common.hsiLock, 0);

		/* Disable HSI ready interrupt */
		*(rcc_common.base + cir) &= ~(1 << 10);
		dataBarier();
	}

	mutexUnlock(rcc_common.hsiLock);

	return EOK;
}


int rcc_devClk(int dev, int state)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_devclk;
	pctl.devclock.dev = dev;
	pctl.devclock.state = state;

	return platformctl(&pctl);
}


inline void pwr_lock(void)
{
	*(rcc_common.pwr + pwr_cr) &= ~(1 << 8);
}


inline void pwr_unlock(void)
{
	*(rcc_common.pwr + pwr_cr) |= 1 << 8;
}


int rcc_init(void)
{
	rcc_common.base = (void *)0x40023800;
	rcc_common.pwr = (void *)0x40007000;
	rcc_common.hsiState = 0;

	pwr_unlock();

	/* Select LSE as clock source for RTC and LCD. */
	*(rcc_common.base + csr) |= 1 << 16;

	dataBarier();

	pwr_lock();

	if (mutexCreate(&rcc_common.hsiLock) != EOK)
		return -ENOMEM;

	if (condCreate(&rcc_common.hsiCond) != EOK) {
		/* TODO - free mutex */
		return -ENOMEM;
	}

	if (interrupt(rcc_irq, rcc_irqHsiReady, NULL, rcc_common.hsiCond) != EOK) {
		/* TODO - free cond and mutex */
		return -ENOMEM;
	}

	return 0;
}
