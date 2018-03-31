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

#include ARCH

#include <errno.h>

#include <stdlib.h>
#include <unistd.h>

#include <sys/threads.h>
#include <sys/msg.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#include "adcdrv.h"


struct {
	unsigned int port;

	volatile unsigned int *base;
	volatile unsigned int *rcc;
	volatile unsigned int *comp;
	volatile unsigned int *ri;

	volatile unsigned int done;

	handle_t cond;
	handle_t mutex;
} adcdrv_common;


enum { rcc_cr = 0, rcc_icscr, rcc_cfgr, rcc_cir, rcc_ahbrstr, rcc_apb2rstr, rcc_apb1rstr,
	rcc_ahbenr, rcc_apb2enr, rcc_apb1enr, rcc_ahblpenr, rcc_apb2lpenr, rcc_apb1lpenr, rcc_csr };


enum { adc_sr = 0, adc_cr1, adc_cr2, adc_smpr1, adc_smpr2, adc_smpr3, adc_jofr1, adc_jofr2,
	adc_jofr3, adc_jofr4, adc_htr, adc_ltr, adc_sqr1, adc_sqr2, adc_sqr3, adc_sqr4, adc_sqr5,
	adc_jsqr, adc_jdr1, adc_jdr2, adc_jdr3, adc_jdr4, adc_dr, adc_smpr0, adc_csr = 192, adc_ccr };


enum { ri_icr = 0, ri_ascr1, ri_ascr2, ri_hyscr1, ri_hyscr2, ri_hyscr3, ri_hyscr4, ri_asmr1,
       ri_cmr1, ri_cicr1, ri_asmr2, ri_cmr2, ri_cicr2, ri_asmr3, ri_cmr3, ri_cicr3, ri_asmr4,
       ri_cmr4, ri_cicr4, ri_asmr5, ri_cmr5, ri_cicr5 };


/* STM32 peripherals */
enum { msi = 97, hsi };


typedef struct {
	enum { PLATCTL_SET = 0, PLATCTL_GET } action;
	enum { PLATCTL_DEVCLOCK = 0, PLATCTL_CPUCLOCK } type;

	union {
		struct {
			unsigned dev, state;
		} devclock;

		struct {
			unsigned hz;
		} cpuclock;
	};
} __attribute__((packed)) platformctl_t;


static int adcdrv_irqEndOfConversion(unsigned int n, void *arg)
{
	/* Clear IRQ */
	*(adcdrv_common.base + adc_sr) &= ~((1 << 2) | (1 << 1));
	adcdrv_common.done = 1;

	return 1;
}


static int adcdrv_irqHsiReady(unsigned int n, void *arg)
{
	int release = 0;

	if ((*(adcdrv_common.rcc + rcc_cr) & 3) == 3) {
		/* Clear IRQ */
		*(adcdrv_common.rcc + rcc_cir) |= 1 << 18;
		__asm__ volatile ("dmb");

		release = 1;
	}

	return release;
}


static unsigned short adcdrv_conversion(char channel)
{
	unsigned short conv;
	unsigned int t;
	platformctl_t pctl;

	pctl.action = PLATCTL_SET;
	pctl.type = PLATCTL_DEVCLOCK;
	pctl.devclock.dev = hsi;

	keepidle(1);

	/* Enable HSI clock */
	pctl.devclock.state = 1;
	platformctl(&pctl);

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

	mutexLock(adcdrv_common.mutex);

	/* Start conversion */
	adcdrv_common.done = 0;
	__asm__ volatile ("dmb");

	*(adcdrv_common.base + adc_cr2) |= 1 << 30;
	__asm__ volatile ("dmb");

	while (!adcdrv_common.done)
		condWait(adcdrv_common.cond, adcdrv_common.mutex, 0);

	/* Read result */
	conv = *(adcdrv_common.base + adc_dr) & 0xffff;

	/* Internal voltage and temperature measurement off */
	*(adcdrv_common.base + adc_ccr) &= ~(1 << 23);

	/* Turn off everything */
	*(adcdrv_common.base + adc_cr2) &= ~1;

	/* Disable HSI clock */
	pctl.devclock.state = 0;
	platformctl(&pctl);

	__asm__ volatile ("dmb");

	keepidle(0);
	mutexUnlock(adcdrv_common.mutex);

	return conv;
}


static unsigned adcdrv_compare(char channel)
{
	unsigned result;

	/* Enable COMP clock */
	*(adcdrv_common.rcc + rcc_apb1enr) |= 1 << 31;

	/* Enable COMP1 */
	*adcdrv_common.comp |= 1 << 4;

	/* Wait until ready */
	while (!(*adcdrv_common.comp & (1 << 4)));

	/* ADC Switch control mode */
	*(adcdrv_common.ri + ri_ascr1) |= 1 << 31;

	/* VCOMP switch select */
	*(adcdrv_common.ri + ri_ascr1) |= 1 << 26;

	/* Select input */
	*(adcdrv_common.ri + ri_ascr1) |= 1 << channel;

	/* TODO: use interrupts */
	usleep(100 * 1000);

	result = *adcdrv_common.comp & (1 << 7);

	*(adcdrv_common.ri + ri_ascr1) = 0;

	/* Disable COMP1 */
	*adcdrv_common.comp &= ~(1 << 4);

	/* Disable COMP clock */
	*(adcdrv_common.rcc + rcc_apb1enr) &= ~(1 << 31);

	return !!result;
}


static void adcdrv_thread(void)
{
	unsigned int msgsz;
	int err;
	unsigned int resp;
	msghdr_t hdr;
	adcdrv_devctl_t devctl;

	for (;;) {
		err = EINVAL;
		resp = 0;
		msgsz = recv(adcdrv_common.port, &devctl, sizeof(devctl), &hdr);

		switch (hdr.op) {
		case DEVCTL:
			if (msgsz == sizeof(devctl) && devctl.type == ADCDRV_GET) {
				resp = adcdrv_conversion(devctl.channel);
				err = EOK;
			}
			if (msgsz == sizeof(devctl) && devctl.type == ADCDRV_COMP) {
				if (devctl.channel == 31)
					devctl.channel = 16;
				else if (devctl.channel == 16 || devctl.channel == 17 || devctl.channel == 26)
					break;

				resp = adcdrv_compare(devctl.channel);
				err = EOK;
			}
			break;

		case READ:
		case WRITE:
		default:
			break;
		}

		if (hdr.type == NORMAL)
			respond(adcdrv_common.port, err, &resp, sizeof(resp));
	}
}


int main(void)
{
	platformctl_t pctl;

	pctl.action = PLATCTL_SET;
	pctl.type = PLATCTL_DEVCLOCK;
	pctl.devclock.dev = hsi;

	adcdrv_common.base = (void *)0x40012400;
	adcdrv_common.rcc = (void *)0x40023800;
	adcdrv_common.comp = (void *)0x40007c00;
	adcdrv_common.ri = (void *)0x40007c04;

	mutexCreate(&adcdrv_common.mutex);
	condCreate(&adcdrv_common.cond);

	/* Enable HSI clock */
	pctl.devclock.state = 1;
	platformctl(&pctl);

	/* Enable ADC clock */
	*(adcdrv_common.rcc + rcc_apb2enr) |= (1 << 9);
	while (!(*(adcdrv_common.rcc + rcc_apb2enr) & (1 << 9)));

	__asm__ volatile ("dmb");

	/* 12 bit resolution, power down when idle, interrupts on, */
	*(adcdrv_common.base + adc_cr1) |= (1 << 17) | (1 << 7) | (1 << 5);
	*(adcdrv_common.base + adc_cr1) &= ~((1 << 8) | (3 << 24));

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
	pctl.devclock.state = 0;
	platformctl(&pctl);

	portCreate(&adcdrv_common.port);
	portRegister(adcdrv_common.port, "/adcdrv");

	interrupt(34, &adcdrv_irqEndOfConversion, NULL, adcdrv_common.cond);
	interrupt(21, &adcdrv_irqHsiReady, NULL, adcdrv_common.cond);

	adcdrv_thread();

	return 0;
}
