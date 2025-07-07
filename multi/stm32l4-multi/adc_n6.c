/*
 * Phoenix-RTOS
 *
 * STM32N6 ADC driver
 *
 * Copyright 2020, 2025 Phoenix Systems
 * Author: Aleksander Kaminski, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/threads.h>
#include <sys/platform.h>
#include <sys/pwman.h>

#include "common.h"
#include "rcc.h"
#include "stm32l4-multi.h"
#include "stm32n6_regs.h"


#define ADC_CLKSEL      pctl_ipclk_adc12sel
#define ADC_DEV         pctl_adc12
#define ADC_IRQ         adc12_irq
#define MAX_ADC         adc2
#define N_ADCS          (MAX_ADC - adc1 + 1)
#define N_CHANNELS      20
#define ADC_REGS_SIZE   64
#define VREFINT_ADC     adc1 /* Which ADC the VREFINT is connected to */
#define VREFINT_CHANNEL 17   /* Which channel the VREFINT is connected to */

#define VREFINT_MV           800  /* Value of internal reference voltage (mV) */
#define VREF_CAL_MV          1800 /* Value of Vref used during calibration test (mV) */
#define OTP_ADDR_VREFINT_CAL 110  /* Word in OTP that holds the value read for Vrefint during factory calibration test */

/* DS14791 claims max frequency is 70 MHz, but STM32CubeIDE allows you to set 130 MHz.
 * Set a maximum of 50 MHz to be safe. */
#define ADC_FREQUENCY      (50 * 1000 * 1000)
#define RESOLUTION_BITS    12
#define FULL_SCALE         ((1 << RESOLUTION_BITS) - 1)
#define CALIBRATION_ROUNDS 8    /* Number taken from documentation, not sure how it was chosen */
#define IRQ_CONVERSION     true /* Use IRQ to wait for conversion end */

/* Definitions of bits in registers */
#define ADC_INTR_ADRDY (1u << 0)
#define ADC_INTR_EOSMP (1u << 1)
#define ADC_INTR_EOC   (1u << 2)
#define ADC_INTR_EOS   (1u << 3)
#define ADC_INTR_OVR   (1u << 4)
#define ADC_INTR_JEOC  (1u << 5)
#define ADC_INTR_JEOS  (1u << 6)
#define ADC_INTR_AWD1  (1u << 7)
#define ADC_INTR_AWD2  (1u << 8)
#define ADC_INTR_AWD3  (1u << 9)
#define ADC_INTR_JQOVF (1u << 10)

#define ADC_CR_ADEN     (1u << 0)
#define ADC_CR_ADDIS    (1u << 1)
#define ADC_CR_ADSTART  (1u << 2)
#define ADC_CR_JADSTART (1u << 3)
#define ADC_CR_ADSTP    (1u << 4)
#define ADC_CR_JADSTP   (1u << 5)
#define ADC_CR_DEEPPWD  (1u << 29)
#define ADC_CR_ADCALDIF (1u << 30)
#define ADC_CR_ADCAL    (1u << 31)

#define ADC_CFGR1_OVRMOD (1 << 12)

#define ADC_CALFACT_CALADDOS (1 << 31)


enum {
	common_csr = 0xc0,
	common_ccr = common_csr + 2,
	common_cdr,
};

static struct {
	volatile unsigned int *base;
	handle_t irqCond;
	handle_t irqMutex;
	uint32_t calcVrefValue;
	struct {
		unsigned int calibration;
		handle_t lock;
	} per[N_ADCS];
} adc_common;


static int adc_getOffs(int adc)
{
	return adc * ADC_REGS_SIZE;
}


int adc_irqHandler(unsigned int n, void *arg)
{
	(void)n;
	(void)arg;
	for (int i = adc1; i <= MAX_ADC; i++) {
		volatile unsigned int *base = adc_common.base + adc_getOffs(i);
		uint32_t intrs_active = *(base + adc_isr);
		*(base + adc_ier) &= ~(intrs_active);
	}

	return 1;
}


static void adc_enableInternal(volatile unsigned int *base)
{
	*(base + adc_cr) |= ADC_CR_ADEN;
	dataBarier();

	while ((*(base + adc_isr) & ADC_INTR_ADRDY) == 0) {
		/* Takes 5 microseconds according to datasheet, but in testing it was
		 * a lot faster (about 10 iterations of the loop) */
	}
}


static void adc_disableInternal(volatile unsigned int *base)
{
	*(base + adc_cr) |= ADC_CR_ADDIS;
	dataBarier();

	while ((*(base + adc_cr) & ADC_CR_ADEN) != 0) {
		/* Wait for ADC switch off, only takes about 2 iterations */
	}
}


static void adc_wakeup(int adc)
{
	volatile unsigned int *base = adc_common.base + adc_getOffs(adc);

	mutexLock(adc_common.per[adc - adc1].lock);
	keepidle(1);

	/* Exit deep power down */
	*(base + adc_cr) &= ~ADC_CR_DEEPPWD;
	dataBarier();

	uint32_t res;
	switch (RESOLUTION_BITS) {
		case 12: res = 0; break;
		case 10: res = 1; break;
		case 8: res = 2; break;
		case 6: res = 3; break;
		default: assert(0); break;
	}

	/* Set to no analog watchdog, single conversion mode, selected resolution, data stored to DR */
	*(base + adc_cfgr1) = res << 2;
	/* Set maximum conversion time for all channels (1499.5 ADC clock cycles) */
	*(base + adc_smpr1) = 0x3fffffff;
	*(base + adc_smpr2) = 0x3fffffff;
	*(base + adc_difsel) &= ~((1 << 20) - 1); /* Set every channel to single-ended */
}


static void adc_disable(int adc)
{
	volatile unsigned int *base = adc_common.base + adc_getOffs(adc);

	if (*(base + adc_cr) & ADC_CR_ADEN) {
		adc_disableInternal(base);
	}

	*(base + adc_cr) |= ADC_CR_DEEPPWD;
	dataBarier();

	keepidle(0);
	mutexUnlock(adc_common.per[adc - adc1].lock);
}


static void adc_waitForConversion(const int *adcs, uint32_t *results, size_t n_conversions)
{
	volatile unsigned int *bases[MAX_ADC + 1];
	const uint32_t bit = ADC_INTR_EOS;
	for (int i = adc1; i <= MAX_ADC; i++) {
		bases[i] = NULL;
	}

	for (size_t i = 0; i < n_conversions; i++) {
		bases[adcs[i]] = adc_common.base + adc_getOffs(adcs[i]);
	}

	if (IRQ_CONVERSION) {
		mutexLock(adc_common.irqMutex);
	}

	for (int i = adc1; i <= MAX_ADC; i++) {
		if (bases[i] == NULL) {
			continue;
		}

		if (IRQ_CONVERSION) {
			*(bases[i] + adc_isr) = 0x3ff; /* Clear any interrupts that may have been set previously */
			*(bases[i] + adc_ier) |= bit;  /* Interrupt enable has to be set while ADSTART == 0 */
		}

		*(bases[i] + adc_cr) |= ADC_CR_ADSTART;
	}

	bool all_ready;
	do {
		if (IRQ_CONVERSION) {
			condWait(adc_common.irqCond, adc_common.irqMutex, 0);
		}
		else {
			usleep(150);
		}

		all_ready = true;
		for (int i = adc1; i <= MAX_ADC; i++) {
			if (bases[i] == NULL) {
				continue;
			}

			all_ready = all_ready && ((*(bases[i] + adc_isr) & bit) != 0);
		}
	} while (!all_ready);

	if (IRQ_CONVERSION) {
		mutexUnlock(adc_common.irqMutex);
	}

	for (size_t i = 0; i < n_conversions; i++) {
		results[i] = *(bases[adcs[i]] + adc_dr);
	}
}


static uint32_t adc_calibrationMeasure(int adc)
{
	uint32_t accumulator = 0;
	uint32_t result;
	for (int i = 0; i < CALIBRATION_ROUNDS; i++) {
		adc_waitForConversion(&adc, &result, 1);
		accumulator += result;
	}

	return accumulator / CALIBRATION_ROUNDS;
}


static int adc_calibrateMode(int adc, bool diff)
{
	const uint32_t minimumValidValue = diff ? 0x7ff : 0x1;
	const int resultOffset = diff ? 16 : 0;
	volatile unsigned int *base = adc_common.base + adc_getOffs(adc);

	if (diff) {
		*(base + adc_cr) |= ADC_CR_ADCALDIF; /* Set to differential input */
	}
	else {
		*(base + adc_cr) &= ~ADC_CR_ADCALDIF; /* Set to single-ended input */
	}

	uint32_t meas = adc_calibrationMeasure(adc);
	if (meas < minimumValidValue) {
		return -1;
	}

	if (diff) {
		meas -= minimumValidValue;
	}

	uint32_t v;
	v = *(base + adc_calfact) & ~(0x1ff << resultOffset);
	*(base + adc_calfact) = v | ((meas & 0x1ff) << resultOffset);
	return 0;
}


static void adc_calibration(int adc)
{
	volatile unsigned int *base = adc_common.base + adc_getOffs(adc);

	adc_enableInternal(base);
	*(base + adc_cr) |= ADC_CR_ADCAL;
	/* Try without additional offset */
	*(base + adc_calfact) &= ~ADC_CALFACT_CALADDOS;
	if (adc_calibrateMode(adc, false) < 0) { /* Do single-ended calibration */
		/* Additional offset necessary */
		*(base + adc_calfact) |= ADC_CALFACT_CALADDOS;
		adc_calibrateMode(adc, false);
		adc_calibrateMode(adc, true);
	}
	else if (adc_calibrateMode(adc, true) < 0) { /* Do differential calibration */
		/* Additional offset necessary, also we need to measure single-ended value again */
		*(base + adc_calfact) |= ADC_CALFACT_CALADDOS;
		adc_calibrateMode(adc, false);
		adc_calibrateMode(adc, true);
	}

	adc_common.per[adc - adc1].calibration = *(base + adc_calfact);
	dataBarier();
	*(base + adc_cr) &= ~ADC_CR_ADCAL;
}


static void adc_enable(int adc)
{
	volatile unsigned int *base = adc_common.base + adc_getOffs(adc);
	adc_enableInternal(base);
	/* Inject calibration */
	*(base + adc_cr) |= ADC_CR_ADCAL;
	dataBarier();
	*(base + adc_calfact) = adc_common.per[adc - adc1].calibration;
	dataBarier();
	*(base + adc_cr) &= ~ADC_CR_ADCAL;
	dataBarier();
}


unsigned short adc_conversion(int adc, char chan)
{
	unsigned int out;
	if ((adc > MAX_ADC) || (chan >= N_CHANNELS)) {
		return 0;
	}

	volatile unsigned int *base_vrefint = adc_common.base + adc_getOffs(VREFINT_ADC);
	volatile unsigned int *base_conv = adc_common.base + adc_getOffs(adc);

	adc_wakeup(adc);
	adc_enable(adc);

	int adcs[2] = { VREFINT_ADC, adc };
	uint32_t results[2];
	size_t n_conversions;

	if (adc == VREFINT_ADC) {
		*(base_conv + adc_pcsel) = (1 << VREFINT_CHANNEL) | (1 << chan);
		if (chan != VREFINT_CHANNEL) {
			*(base_conv + adc_sqr1) = (chan << 12) | (VREFINT_CHANNEL << 6) | 1;
			n_conversions = 2;
		}
		else {
			*(base_conv + adc_sqr1) = (VREFINT_CHANNEL << 6);
			n_conversions = 1;
		}
	}
	else {
		adc_wakeup(VREFINT_ADC);
		adc_enable(VREFINT_ADC);
		*(base_vrefint + adc_pcsel) = (1 << VREFINT_CHANNEL);
		*(base_vrefint + adc_sqr1) = (VREFINT_CHANNEL << 6);
		*(base_conv + adc_pcsel) = (1 << chan);
		*(base_conv + adc_sqr1) = (chan << 6);
		n_conversions = 2;
	}

	adc_waitForConversion(adcs, results, n_conversions);
	/* Check vrefint just to avoid possible division by 0 */
	if (results[0] != 0) {
		uint32_t vref = adc_common.calcVrefValue / results[0];
		if ((adc == VREFINT_ADC) && (chan == VREFINT_CHANNEL)) {
			out = vref;
		}
		else {
			out = (vref * results[1]) / FULL_SCALE;
		}
	}
	else {
		/* TODO: we should probably return some error value */
		out = 0;
	}

	adc_disable(adc);
	if (adc != VREFINT_ADC) {
		adc_disable(VREFINT_ADC);
	}

	return out;
}


int adc_init(void)
{
	adc_common.base = ADC_BASE;

	/* On STM32N6 the ADC clocks have to be supplied from RCC */
	if (rcc_setClksel(ADC_CLKSEL, clkid_hclk) < 0) {
		return -1;
	}

	uint64_t freq_out;
	if (clockdef_getClock(clkid_hclk, &freq_out) < 0) {
		return -1;
	}

	uint32_t freq = (uint32_t)freq_out;
	uint32_t prescaler = (freq + ADC_FREQUENCY - 1) / ADC_FREQUENCY;
	if ((prescaler == 0) || (prescaler > 256)) {
		return -1;
	}

	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_ipclk,
		.ipclk = {
			.ipclk = pctl_ipclk_adcpre,
			.setting = prescaler - 1,
		},
	};

	if (platformctl(&pctl) < 0) {
		return -1;
	}

	/* Undocumented requirement - ADCs needs to be set for secure accesses only,
	 * otherwise they will only be able to read internal channels (Vref, Vbat, Vrefint)
	 * and any other channel will appear to be floating. */
	pctl.action = pctl_set;
	pctl.type = pctl_risup;
	pctl.risup.index = pctl_risup_adc12;
	pctl.risup.secure = 1;
	pctl.risup.privileged = 0;
	pctl.risup.lock = 0;
	if (platformctl(&pctl) < 0) {
		return -1;
	}

	/* Get calibration value from OTP. We need to ask kernel to do it for us
	 * (it requires privileged execution) */
	pctl.action = pctl_get;
	pctl.type = pctl_otp;
	pctl.otp.addr = OTP_ADDR_VREFINT_CAL;
	if (platformctl(&pctl) < 0) {
		return -1;
	}

	adc_common.calcVrefValue = VREF_CAL_MV * pctl.otp.val;
	devClk(ADC_DEV, 1);
	/* Enable VREFINT */
	*(adc_common.base + common_ccr) = (1 << 22) | (0xf << 8);

	mutexCreate(&adc_common.irqMutex);
	condCreate(&adc_common.irqCond);
	if (IRQ_CONVERSION) {
		interrupt(ADC_IRQ, adc_irqHandler, NULL, adc_common.irqCond, NULL);
	}

	for (int i = adc1; i <= MAX_ADC; ++i) {
		mutexCreate(&adc_common.per[i - adc1].lock);
		adc_wakeup(i);
		adc_calibration(i);
		adc_disable(i);
	}

	return EOK;
}
