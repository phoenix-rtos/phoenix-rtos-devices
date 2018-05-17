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


#include ARCH
#include <errno.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#include "common.h"
#include "rcc.h"


#ifndef NDEBUG
static const char drvname[] = "adc: ";
#endif


enum { sr = 0, cr1, cr2, smpr1, smpr2, smpr3, jofr1, jofr2, jofr3, jofr4, htr, ltr, sqr1,
	sqr2, sqr3, sqr4, sqr5, jsqr, jdr1, jdr2, jdr3, jdr4, dr, smpr0, csr = 192, ccr };


struct {
	volatile unsigned int *base;

	handle_t lock;
	handle_t cond;
	handle_t inth;
} adc_common;


static int adc_irqEoc(unsigned int n, void *arg)
{
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

	/* Start conversion */
	*(adc_common.base + cr2) |= 1 << 30;

	while (!(*(adc_common.base + sr) & ((1 << 2) | (1 << 1))))
		condWait(adc_common.cond, adc_common.lock, 0);

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
	if (rcc_setHsi(1) != EOK || rcc_devClk(pctl_adc1, 1) != EOK) {
		DEBUG("ADC initial clock config failed");
		return -EIO;
	}

	if (mutexCreate(&adc_common.lock) != EOK) {
		DEBUG("ADC lock create failed\n");
		return -ENOMEM;
	}

	if (condCreate(&adc_common.cond) != EOK) {
		DEBUG("ADC cond create failed\n");
		resourceDestroy(adc_common.lock);
		return -ENOMEM;
	}

	/* 12 bit resolution, power down when idle, interrupts on, */
	*(adc_common.base + cr1) |= (1 << 17) | (1 << 7) | (1 << 5);
	*(adc_common.base + cr1) &= ~(1 << 8);

	*(adc_common.base + sr) |= 1 << 5;

	/* Data aligned to the left, EOC at the end of conversion, single conversion mode */
	*(adc_common.base + cr2) |= (1 << 11) | (1 << 10);
	*(adc_common.base + cr2) &= ~2;

	/* One conversion in sequence */
	*(adc_common.base + sqr1) &= ~(0x1f << 20);

	/* Turn off ADC */
	*(adc_common.base + cr2) &= ~1;
	while (*(adc_common.base + sr) & (1 << 6)) {
		if (++i > 500) {
			DEBUG("ADC turn off timed out\n");
			break;
		}
	}

	/* Disable HSI clock */
	if (rcc_setHsi(0) != EOK)
		DEBUG("ADC HSI turn off failed\n");

	/* Register end of conversion interrupt */
	if (interrupt(adc1_irq, adc_irqEoc, NULL, adc_common.lock, &adc_common.inth) != EOK) {
		DEBUG("ADC interrupt register failed\n");
		resourceDestroy(adc_common.lock);
		resourceDestroy(adc_common.cond);
		return -ENOMEM;
	}

	return EOK;
}
