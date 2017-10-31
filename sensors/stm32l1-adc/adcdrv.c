/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 ADC driver
 *
 * Copyright 2017 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include HAL
#include "adcdrv.h"
#include "proc/threads.h"
#include "proc/msg.h"


/* Temporary solution */
unsigned int adcdrv_id;

struct {
	volatile unsigned int *base;
	volatile unsigned int *rcc;

	thread_t *eocEv;
	thread_t *hsiEv;

	spinlock_t eocSp;
	spinlock_t hsiSp;

	intr_handler_t EndOfConversionHandler;
	intr_handler_t HsiReadyHandler;
} adcdrv_common;


enum { rcc_cr = 0, rcc_icscr, rcc_cfgr, rcc_cir, rcc_ahbrstr, rcc_apb2rstr, rcc_apb1rstr,
	rcc_ahbenr, rcc_apb2enr, rcc_apb1enr, rcc_ahblpenr, rcc_apb2lpenr, rcc_apb1lpenr, rcc_csr };


enum { adc_sr = 0, adc_cr1, adc_cr2, adc_smpr1, adc_smpr2, adc_smpr3, adc_jofr1, adc_jofr2,
	adc_jofr3, adc_jofr4, adc_htr, adc_ltr, adc_sqr1, adc_sqr2, adc_sqr3, adc_sqr4, adc_sqr5,
	adc_jsqr, adc_jdr1, adc_jdr2, adc_jdr3, adc_jdr4, adc_dr, adc_smpr0, adc_csr = 192, adc_ccr };


static int adcdrv_irqEndOfConversion(unsigned int n, cpu_context_t *context, void *arg)
{
	/* Clear IRQ */
	*(adcdrv_common.base + adc_sr) &= ~((1 << 2) | (1 << 1));

	proc_threadWakeup(&adcdrv_common.eocEv);

	return 0;
}


static int adcdrv_irqHsiReady(unsigned int n, cpu_context_t *context, void *arg)
{
	if ((*(adcdrv_common.rcc + rcc_cr) & 3) == 3) {
		/* Clear IRQ */
		*(adcdrv_common.rcc + rcc_cir) |= 1 << 18;
		hal_cpuDataBarrier();

		proc_threadWakeup(&adcdrv_common.hsiEv);
	}

	return 0;
}


static unsigned short adcdrv_conversion(char channel)
{
	unsigned short conv;
	unsigned int t;

	/* Start HSI clock */
	hal_spinlockSet(&adcdrv_common.hsiSp);
	hal_cpuSetDevBusy(1);
	if (!(*(adcdrv_common.rcc + rcc_cr) & 2)) {
		/* Enable HSI ready interrupt */
		*(adcdrv_common.rcc + rcc_cir) |= 1 << 10;
		hal_cpuDataBarrier();

		/* Enable HSI clock */
		*(adcdrv_common.rcc + rcc_cr) |= 1;
		hal_cpuDataBarrier();

		/* Wait for clock to stabilize */
		while (!(*(adcdrv_common.rcc + rcc_cr) & 2))
			proc_threadWait(&adcdrv_common.hsiEv, &adcdrv_common.hsiSp, 0);
	}
	hal_spinlockClear(&adcdrv_common.hsiSp);

	/* Enable ADC */
	*(adcdrv_common.base + adc_cr2) |= 1;

	/* Wait for ADONS to be one */
	while (!(*(adcdrv_common.base + adc_sr) & (1 << 6)));

	/* Internal voltage and temperature measurement on */
	if (channel == 16 || channel == 17)
		*(adcdrv_common.base + adc_ccr) |= 1 << 23;

	/* Select bank */
	t = *(adcdrv_common.base + adc_cr2) & ~(!(channel & 0x20) << 2);
	*(adcdrv_common.base + adc_cr2) = t | (!!(channel & 0x20) << 2);

	/* Select channel */
	t = *(adcdrv_common.base + adc_sqr5) & 0xc0000000;
	*(adcdrv_common.base + adc_sqr5) = t | (channel & 0x1f);

	hal_spinlockSet(&adcdrv_common.eocSp);
	/* Start conversion */
	*(adcdrv_common.base + adc_cr2) |= 1 << 30;
	hal_cpuDataBarrier();

	proc_threadWait(&adcdrv_common.eocEv, &adcdrv_common.eocSp, 0);

	/* Read result */
	conv = *(adcdrv_common.base + adc_dr) & 0xffff;

	/* Internal voltage and temperature measurement off */
	*(adcdrv_common.base + adc_ccr) &= ~(1 << 23);

	/* Turn off everything */
	*(adcdrv_common.base + adc_cr2) &= ~1;
	hal_spinlockClear(&adcdrv_common.eocSp);

	hal_spinlockSet(&adcdrv_common.hsiSp);
	/* Disable HSI clock */
	*(adcdrv_common.rcc + rcc_cr) &= ~1;

	hal_cpuDataBarrier();

	hal_cpuSetDevBusy(0);
	hal_spinlockClear(&adcdrv_common.hsiSp);

	return conv;
}


static void adcdrv_thread(void *arg)
{
	unsigned int msgsz;
	int err;
	unsigned int resp;
	msghdr_t hdr;
	adcdrv_devctl_t devctl;

	for (;;) {
		err = EINVAL;
		resp = 0;
		msgsz = proc_recv(adcdrv_id, &devctl, sizeof(devctl), &hdr);

		switch (hdr.op) {
			case MSG_DEVCTL:
				if (msgsz == sizeof(devctl) && devctl.type == ADCDRV_GET) {
					resp = adcdrv_conversion(devctl.channel);
					err = EOK;
				}
				break;

			case MSG_READ:
			case MSG_WRITE:
			default:
				break;
		}

		if (hdr.type == MSG_NORMAL)
			proc_respond(adcdrv_id, err, &resp, sizeof(resp));
	}
}


int adcdrv_gpio2chan(char port, char pin)
{
	switch (port) {
	case 0: /* PORT A */
		if (pin >= 0 && pin <= 7)
			return pin;
		break;

	case 1: /* PORT B */
		if (pin >= 0 && pin <= 1)
			return pin + 8;
		else if (pin == 2)
			return 0x20;
		else if (pin >= 12 && pin <= 15)
			return pin + 6;
		break;

	case 2: /* PORT C */
		if (pin >= 0 && pin <= 5)
			return pin + 10;
		break;

	case 4: /* PORT E */
		if (pin >= 7 && pin <= 10)
			return pin + 15;
		break;

	case 5: /* PORT F */
		if (pin >= 6 && pin <= 10)
			return pin + 21;
		if (pin >= 11 && pin <= 12)
			return (pin - 10) | 0x20;
		else if (pin == 15)
			return 7 | 0x20;
		break;

	case 6: /* PORT G */
		if (pin >= 0 && pin <= 4)
			return (pin + 8) | 0x20;
		break;
	}

	return -1;
}


void adcdrv_init(void)
{
	adcdrv_common.base = (void *)0x40012400;
	adcdrv_common.rcc = (void *)0x40023800;

	adcdrv_common.eocEv = NULL;
	adcdrv_common.hsiEv = NULL;

	hal_spinlockCreate(&adcdrv_common.eocSp, "ADC EOC");
	hal_spinlockCreate(&adcdrv_common.hsiSp, "HSI");

	/* Enable HSI clock */
	*(adcdrv_common.rcc + rcc_cr) |= 1;
	while (!(*(adcdrv_common.rcc + rcc_cr) & 2));

	hal_cpuDataBarrier();

	/* Enable ADC clock */
	*(adcdrv_common.rcc + rcc_apb2enr) |= (1 << 9);
	while (!(*(adcdrv_common.rcc + rcc_apb2enr) & (1 << 9)));

	hal_cpuDataBarrier();

	/* 12 bit resolution, power down when idle, interrupts on, */
	*(adcdrv_common.base + adc_cr1) |= (1 << 17) | (1 << 7) | (1 << 5);
	*(adcdrv_common.base + adc_cr1) &= ~(1 << 8);

	*(adcdrv_common.base + adc_sr) |= 1 << 5;

	/* Data aligned to the left, EOC at the end of conversion, single conversion mode */
	*(adcdrv_common.base + adc_cr2) |= (1 << 11) | (1 << 10);
	*(adcdrv_common.base + adc_cr2) &= ~2;

	/* One conversion in sequence */
	*(adcdrv_common.base + adc_sqr1) &= ~(0x1f << 20);

	/* Turn off ADC */
	*(adcdrv_common.base + adc_cr2) &= ~1;
	while (*(adcdrv_common.base + adc_sr) & (1 << 6));

	/* Disable HSI clock */
	*(adcdrv_common.rcc + rcc_cr) &= ~1;

	hal_cpuDataBarrier();

	proc_portCreate(&adcdrv_id);
	proc_portRegister(adcdrv_id, "/adcdrv");

	adcdrv_common.EndOfConversionHandler.next = NULL;
	adcdrv_common.EndOfConversionHandler.prev = NULL;
	adcdrv_common.EndOfConversionHandler.f = adcdrv_irqEndOfConversion;
	adcdrv_common.EndOfConversionHandler.data = NULL;
	adcdrv_common.EndOfConversionHandler.pmap = NULL;

	adcdrv_common.HsiReadyHandler.next = NULL;
	adcdrv_common.HsiReadyHandler.prev = NULL;
	adcdrv_common.HsiReadyHandler.f = adcdrv_irqHsiReady;
	adcdrv_common.HsiReadyHandler.data = NULL;
	adcdrv_common.HsiReadyHandler.pmap = NULL;

	hal_interruptsSetHandler(34, &adcdrv_common.EndOfConversionHandler);
	hal_interruptsSetHandler(21, &adcdrv_common.HsiReadyHandler);
	proc_threadCreate(0, adcdrv_thread, 2, 512, 0, 0);
}

