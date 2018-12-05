/*
 * Phoenix-RTOS
 *
 * STM32L1 ADC driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#include "common.h"
#include "rcc.h"


enum { sr = 0, cr1, cr2, smpr1, smpr2, smpr3, jofr1, jofr2, jofr3, jofr4, htr, ltr, sqr1,
	sqr2, sqr3, sqr4, sqr5, jsqr, jdr1, jdr2, jdr3, jdr4, dr, smpr0, csr = 192, ccr };


struct {
	volatile unsigned int *base;

	handle_t lock;
	handle_t irqLock;
	handle_t cond;
} adc_common;


static int adc_irqEoc(unsigned int n, void *arg)
{
	*(adc_common.base + cr1) &= ~(1 << 5);

	return 1;
}


unsigned short adc_conversion(char channel)
{
	unsigned short conv;
	unsigned int t;

	mutexLock(adc_common.lock);

	/* Enable HSI */
	rcc_setHsi(1);

	/* Enable ADC */
	*(adc_common.base + cr2) |= 1;

	/* Wait for ADONS to be one */
	while (!(*(adc_common.base + sr) & (1 << 6)));

	/* Internal voltage and temperature measurement on */
	if (channel == 16 || channel == 17)
		*(adc_common.base + ccr) |= 1 << 23;

	/* Select bank */
	t = *(adc_common.base + cr2) & ~(!(channel & 0x20) << 2);
	*(adc_common.base + cr2) = t | (!!(channel & 0x20) << 2);

	/* Select channel */
	t = *(adc_common.base + sqr5) & 0xc0000000;
	*(adc_common.base + sqr5) = t | (channel & 0x1f);

	/* Clear flags */
	*(adc_common.base + sr) &= ~((1 << 2) | (1 << 1));

	/* Enable interrupt */
	*(adc_common.base + cr1) |= 1 << 5;

	/* Start conversion */
	*(adc_common.base + cr2) |= 1 << 30;

	mutexLock(adc_common.irqLock);
	while (!(*(adc_common.base + sr) & ((1 << 2) | (1 << 1))))
		condWait(adc_common.cond, adc_common.lock, 0);
	mutexUnlock(adc_common.irqLock);

	/* Read result */
	conv = *(adc_common.base + dr) & 0xffff;

	/* Internal voltage and temperature measurement off */
	*(adc_common.base + ccr) &= ~(1 << 23);

	/* Turn off everything */
	*(adc_common.base + cr2) &= ~1;

	/* Disable HSI clock */
	rcc_setHsi(0);

	mutexUnlock(adc_common.lock);

	return conv;
}


int adc_init(void)
{
	int i = 0;

	adc_common.base = (void *)0x40012400;

	/* Enable HSI and ADC clock */
	rcc_setHsi(1);
	rcc_devClk(pctl_adc1, 1);

	mutexCreate(&adc_common.lock);
	mutexCreate(&adc_common.irqLock);
	condCreate(&adc_common.cond);

	/* 12 bit resolution, power down when idle, interrupts on, */
	*(adc_common.base + cr1) |= (1 << 17) | (1 << 7);
	*(adc_common.base + cr1) &= ~(1 << 8);

	*(adc_common.base + sr) |= 1 << 5;

	/* Data aligned to the left, EOC at the end of conversion, single conversion mode */
	*(adc_common.base + cr2) |= (1 << 11) | (1 << 10);
	*(adc_common.base + cr2) &= ~2;

	/* One conversion in sequence */
	*(adc_common.base + sqr1) &= ~(0x1f << 20);

	/* Set sampling time to 48 cycles */
	*(adc_common.base + smpr0) = 055;
	*(adc_common.base + smpr1) = 05555555555;
	*(adc_common.base + smpr2) = 05555555555;
	*(adc_common.base + smpr3) = 05555555555;

	/* Turn off ADC */
	*(adc_common.base + cr2) &= ~1;
	while (*(adc_common.base + sr) & (1 << 6)) {
		if (++i > 500)
			break;
	}

	/* Disable HSI clock */
	rcc_setHsi(0);

	/* Register end of conversion interrupt */
	interrupt(adc1_irq, adc_irqEoc, NULL, adc_common.cond, NULL);

	return EOK;
}
