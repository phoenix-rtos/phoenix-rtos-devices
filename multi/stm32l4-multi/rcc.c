/*
 * Phoenix-RTOS
 *
 * STM32L4 reset and clock controler driver
 *
 * Copyright 2017, 2018, 2020 Phoenix Systems
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

	handle_t lock;
} rcc_common;


enum { cr = 0, icscr, cfgr, cir, ahbrstr, apb2rstr, apb1rstr, ahbenr, apb2enr,
	apb1enr, ahblpenr, apb2lpenr, apb1lpenr, csr };


enum { pwr_cr1 = 0, pwr_cr2, pwr_cr3, pwr_cr4, pwr_sr1, pwr_sr2, pwr_scr, pwr_pucra, pwr_pdcra, pwr_pucrb,
	pwr_pdcrb, pwr_pucrc, pwr_pdcrc, pwr_pucrd, pwr_pdcrd, pwr_pucre, pwr_pdcre, pwr_pucrf, pwr_pdcrf,
	pwr_pucrg, pwr_pdcrg, pwr_pucrh, pwr_pdcrh, pwr_pucri, pwr_pdcri };


static inline void _pwr_lock(void)
{
	*(rcc_common.pwr + pwr_cr1) &= ~(1 << 8);
}


static inline void _pwr_unlock(void)
{
	*(rcc_common.pwr + pwr_cr1) |= 1 << 8;
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
	rcc_common.base = (void *)0x40021000;
	rcc_common.pwr = (void *)0x40007000;

	mutexCreate(&rcc_common.lock);

	return 0;
}
