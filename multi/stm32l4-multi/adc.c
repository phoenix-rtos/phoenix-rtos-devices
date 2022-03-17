/*
 * Phoenix-RTOS
 *
 * STM32L4 ADC driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <unistd.h>
#include <sys/threads.h>
#include <sys/platform.h>
#include <sys/pwman.h>

#include "common.h"
#include "rcc.h"
#include "stm32l4-multi.h"

enum { adc1_offs = 0, adc2_offs = 64, adc3_offs = 128, common_offs = 192 };

enum { isr = 0, ier, cr, cfgr, cfgr2, smpr1, smpr2, tr1 = smpr2 + 2, tr2, tr3, sqr1 = tr3 + 2, sqr2, sqr3, sqr4, dr,
	jsqr = dr + 3, ofr1 = jsqr + 5, ofr2, ofr3, ofr4, jdr1 = ofr4 + 5, jdr2, jdr3, jdr4, awd2cr = jdr4 + 5 , awd3cr,
	difsel = awd3cr + 3, calfact };

enum { common_csr = common_offs, common_ccr = common_csr + 2, common_cdr };


//static const unsigned short * const ts_cal1 = (void *)0x1fff75a8;
//static const unsigned short * const ts_cal2 = (void *)0x1fff75ca;
static const unsigned short * const vrefint = (void *)0x1fff75aa;


struct {
	volatile unsigned int *base;
	unsigned int calibration[3];

	handle_t lock[3];
} adc_common;


static void adc_delay(int delay)
{
	volatile int i;

	for (i = 0; i < delay; ++i)
		__asm__ volatile ("nop");
}


static int adc_getOffs(int adc)
{
	if (adc == adc1)
		return adc1_offs;
	else if (adc == adc2)
		return adc2_offs;
	else
		return adc3_offs;
}


static void adc_wakeup(int adc)
{
	volatile unsigned int *base = adc_common.base + adc_getOffs(adc);

	mutexLock(adc_common.lock[adc]);
	keepidle(1);

	/* Exit deep power down */
	*(base + cr) &= ~(1 << 29);
	dataBarier();
	*(base + cr) |= 1 << 28;
	dataBarier();

	/* Wait for ~20 us */
	adc_delay(100);

	*(base + cfgr) = (1 << 31) | (1 << 12);
	*(base + smpr1) = 0xbfffffff;
	*(base + smpr2) = 0x07ffffff;
}


static void adc_disable(int adc)
{
	volatile unsigned int *base = adc_common.base + adc_getOffs(adc);

	if (*(base + cr) & 1) {
		*(base + cr) |= 1 << 1;
		dataBarier();

		while (*(base + cr) & 1)
			usleep(0);
	}

	*(base + cr) |= 1 << 29;
	dataBarier();

	keepidle(0);
	mutexUnlock(adc_common.lock[adc]);
}


static void adc_calibration(int adc)
{
	volatile unsigned int *base = adc_common.base + adc_getOffs(adc);
	useconds_t sleep = 2 * 1000;

	*(base + cr) &= ~(1 << 30);
	*(base + cr) |= 1 << 31;

	/* No intterupt for calibration done. Have to wait manually... */
	while (*(base + cr) & (1 << 31)) {
		usleep(sleep);
		if (sleep < 200 * 1000)
			sleep *= 2;
	}

	adc_common.calibration[adc] = *(base + calfact);
}


static unsigned short adc_probeChannel(int adc, char chan)
{
	volatile unsigned int *base = adc_common.base + adc_getOffs(adc);

	/* Two conversions in regular sequence */
	*(base + sqr1) = (chan << 12) | (chan << 6) | 1;
	dataBarier();

	*(base + cr) |= 1 << 2;
	dataBarier();

	/* Errata DM00264473 2.7.2 - we need to get data withing 1 ms of previous conversion */
	/* so we make two conversion and allow overrun, removing result from the first one. */

	/* Wait for overrun */
	while (!(*(base + isr) & (1 << 4)))
		usleep(1000);

	/* Clear flags */
	*(base + isr) |= 0x7ff;

	return *(base + dr);
}


static void adc_enable(int adc)
{
	volatile unsigned int *base = adc_common.base + adc_getOffs(adc);

	/* Enable ADC */
	*(base + cr) |= 1;
	dataBarier();

	/* It's quick, no need for an interrupt */
	while (!(*(base + isr) & 1))
		usleep(0);

	/* Inject calibration */
	*(base + calfact) = adc_common.calibration[adc];
}


unsigned short adc_conversion(int adc, char chan)
{
	unsigned short vref, val;
	unsigned int out;

	adc_wakeup(adc);
	adc_calibration(adc);
	adc_enable(adc);

	if (adc != adc1) {
		adc_wakeup(adc1);
		adc_calibration(adc1);
		adc_enable(adc1);
	}

	chan &= 0x1f;

	vref = adc_probeChannel(adc1, 0);

	out = (3000 * (*vrefint)) / vref;

	if (adc != adc1 || chan != 0) {
		val = adc_probeChannel(adc, chan);
		out = (out * val) / ((1 << 12) - 1);
	}

	adc_disable(adc);
	if (adc != adc1)
		adc_disable(adc1);

	return out;
}


int adc_init(void)
{
	int i;

	adc_common.base = (void *)0x50040000;
	devClk(pctl_adc, 1);

	*(adc_common.base + common_ccr) = (1 << 22) | (0xe << 18) | (0x3 << 16) | (0xf << 8);

	for (i = adc1; i <= adc3; ++i) {
		mutexCreate(&adc_common.lock[i]);
		adc_wakeup(i);
		adc_calibration(i);
		adc_disable(i);
	}

	return EOK;
}
