/*
 * Phoenix-RTOS
 *
 * STM32N6 PWM TIM driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Krzysztof Radzewicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include "pwm_n6.h"
#include <stdio.h>
#include "common.h"

typedef struct pwm_common_t {
	volatile uint32_t *const timBase[PWM_TIMER_NUM];
	const uint8_t timChannelSet[PWM_TIMER_NUM];
	uint16_t timArr[PWM_TIMER_NUM];
} pwm_common_t;

/* clang-format off */
static pwm_common_t pwm_common = {
	/* Basic timers TIM6/TIM7/TIM18 not available for PWM */
	.timBase = {
		TIM1_BASE,
		TIM2_BASE,
		TIM3_BASE,
		TIM4_BASE,
		TIM5_BASE,
		TIM8_BASE,
		TIM9_BASE,
		TIM10_BASE,
		TIM11_BASE,
		TIM12_BASE,
		TIM13_BASE,
		TIM14_BASE,
		TIM15_BASE,
		TIM16_BASE,
		TIM17_BASE,
	},
	.timChannelSet = {
		(0x1 << pwm_ch1) | (0x1 << pwm_ch2) | (0x1 << pwm_ch3) | (0x1 << pwm_ch4) | \
        (0x1 << pwm_ch1n) | (0x1 << pwm_ch2n) | (0x1 << pwm_ch3n) | (0x1 << pwm_ch4n),  // TIM1
		(0x1 << pwm_ch1) | (0x1 << pwm_ch2) | (0x1 << pwm_ch3) | (0x1 << pwm_ch4),      // TIM2
		(0x1 << pwm_ch1) | (0x1 << pwm_ch2) | (0x1 << pwm_ch3) | (0x1 << pwm_ch4),      // TIM3
		(0x1 << pwm_ch1) | (0x1 << pwm_ch2) | (0x1 << pwm_ch3) | (0x1 << pwm_ch4),      // TIM4
		(0x1 << pwm_ch1) | (0x1 << pwm_ch2) | (0x1 << pwm_ch3) | (0x1 << pwm_ch4),      // TIM5
		(0x1 << pwm_ch1) | (0x1 << pwm_ch2) | (0x1 << pwm_ch3) | (0x1 << pwm_ch4) | \
        (0x1 << pwm_ch1n) | (0x1 << pwm_ch2n) | (0x1 << pwm_ch3n) | (0x1 << pwm_ch4n),  // TIM8
		(0x1 << pwm_ch1) | (0x1 << pwm_ch2),                                            // TIM9
		(0x1 << pwm_ch1),                                                               // TIM10
		(0x1 << pwm_ch1),                                                               // TIM11
		(0x1 << pwm_ch1),                                                               // TIM12
		(0x1 << pwm_ch1),                                                               // TIM13
		(0x1 << pwm_ch1),                                                               // TIM14
		(0x1 << pwm_ch1) | (1 << pwm_ch2) | (0x1 << pwm_ch1n),                          // TIM15
		(0x1 << pwm_ch1) | (0x1 << pwm_ch1n),                                           // TIM16
		(0x1 << pwm_ch1) | (0x1 << pwm_ch1n),                                           // TIM17
	},
	.timArr = {}
};
/* clang-format on */


static int pwm_validateTimer(pwm_tim_id_t timer)
{
	if (timer < 0 || timer >= PWM_TIMER_NUM) {
		return -EINVAL;
	}
	return EOK;
}


static int pwm_validateChannel(pwm_tim_id_t timer, pwm_ch_id_t chn)
{
	/* Assume timer is already correct */
	if (chn < 0 || chn >= PWM_CHN_NUM) {
		return -EINVAL;
	}
	if ((pwm_common.timChannelSet[timer] & (0x1 << chn)) == 0) {
		return -EINVAL;
	}
	return EOK;
}

static int pwm_channelHasComplement(pwm_tim_id_t timer, pwm_ch_id_t chn)
{
	/* Assume timer/channel is already correct */
	if ((pwm_common.timChannelSet[timer] & (0x1 << (chn + 4))) == 0) {
		return 0;
	}
	return 1;
}


// static void pwm_enable(pwm_tim_id_t timer, pwm_ch_id_t chn)
// {
// 	return;
// }

// /* Returns errors */
// static int pwm_disable(pwm_tim_id_t timer)
// {
// 	int res;
// 	if ((res = pwm_validateTimer(timer)) < 0) {
// 		return res;
// 	}

// 	/* Clear CEN in CR1 */
// 	*(pwm_common.timBase[timer] + tim_cr1) &= ~(0x1);
// }


uint64_t pwm_getBaseFrequency(pwm_tim_id_t timer)
{
	uint64_t baseFreq = 0ull;
	clockdef_getClock(clkid_timg, &baseFreq);
	return baseFreq;
}


/* Configure a compatible timer for pwm. Returns errors */
int pwm_configure(pwm_tim_id_t timer, uint32_t prescaler, uint16_t top)
{
	/* TODO: For now assuming full configuration such as in TIM1/TIM8.
	 * Later check if each step makes sense for simpler timers and 'if out' advanced configuration
	 * TODO: Ask somebody if its ok to set a 16 bit register () via a u32 pointer or if it's gonna result in setting reserved bits
	 */
	int res;
	uint32_t t;
	if ((res = pwm_validateTimer(timer)) < 0) {
		return res;
	}

	/* In CR1 set prescaler, countermode, autoreload, clockdivision, repetition counter */
	t = *(pwm_common.timBase[timer] + tim_cr1);
	/* Set counter mode UP in CR1 => clear DIR and CMS */
	t &= ~((0x3 << 5) | (0x1 << 4));
	/* Set clock division 1 in CR1 => clear CKD */
	t &= ~(0x3 << 8);
	*(pwm_common.timBase[timer] + tim_cr1) = t;

	/* Set autoreload in ARR */
	pwm_common.timArr[timer] = top;
	*(pwm_common.timBase[timer] + tim_arr) = top;

	/* Set prescaler in PSC */
	*(pwm_common.timBase[timer] + tim_psc) = prescaler;
	/* Set repetition counter to 0 in RCR */  // only if it exists in this timer
	*(pwm_common.timBase[timer] + tim_rcr) = 0;

	/* Generate update event (UG bit in EGR) to reload prescaler and repetition counter */
	*(pwm_common.timBase[timer] + tim_egr) |= 0x1;

	/* Enable buffering in CR1 ARPE */
	*((volatile uint16_t *)(pwm_common.timBase[timer] + tim_cr1)) |= (0x1 << 7);

	printf("DEBUG: CR1=%x\n", *(pwm_common.timBase[timer] + tim_cr1));
	return EOK;
}


/* Set output pwm channel on configured timer. Returns errors */
int pwm_set(pwm_tim_id_t timer, pwm_ch_id_t chn, uint16_t compare)
{
	/* For now this function reconfigures the entire channel each time it's called.
	 * In the future change it so that it only changes parameters once running
	 */
	int res;
	volatile uint32_t *reg;
	uint32_t tmpcr2, tmpccmr, tmpccer, tmpbdtr;

	/* Validate channel/timer compatibility */
	if ((res = pwm_validateChannel(timer, chn)) < 0) {
		return res;
	}
	if (compare > pwm_common.timArr[timer]) {
		return -EINVAL;
	}

	/* Set output channel preloading in CCMRx OCyPE */
	reg = (pwm_common.timBase[timer] + PWM_CCMR_REG(chn));
	*reg |= (0x1 << PWM_CCMR_OCPE_OFF(chn));
	/* TODO: Find out what's the deal with channel 5,6. Why are they not in the pin table, but in the register description? */

	/* Disable the channel. Clear CCxE in CCER */
	*(pwm_common.timBase[timer] + tim_ccer) &= ~(0x1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		*(pwm_common.timBase[timer] + tim_ccer) &= ~(0x1 << PWM_CCER_CCNE_OFF(chn));
	}

	// NOTE: For OC with compliments enabling also requires a trillion other bits in BDTR or sth
	tmpcr2 = *(pwm_common.timBase[timer] + tim_cr2);
	tmpccmr = *(pwm_common.timBase[timer] + PWM_CCMR_REG(chn));
	tmpccer = *(pwm_common.timBase[timer] + tim_ccer);

	// NOTE: CCMR works differently on TIM10-14 :(
	/* Set channel as output. Clear CCyS */
	tmpccmr &= ~(0x3 << PWM_CCMR_CCS_OFF(chn));
	/* Set output compare mode to PWM 1 mode (0110 bits 16, 6:4)*/
	tmpccmr = (tmpccmr & ~((0x1 << PWM_CCMR_OCMH_OFF(chn)) | (0x7 << PWM_CCMR_OCML_OFF(chn))));
	tmpccmr |= (0x6 << PWM_CCMR_OCML_OFF(chn));
	/* Set output compare polarity to active high (0)*/
	tmpccer &= ~(0x1 << PWM_CCER_CCP_OFF(chn));

	if (pwm_channelHasComplement(timer, chn)) {
		/* Disable the complimentary channel */
		tmpccer &= ~(0x1 << PWM_CCER_CCNE_OFF(chn));
		/* Set complimentary output polarity (to active high) */
		tmpccer &= ~(0x1 << PWM_CCER_CCNP_OFF(chn));
		/* Set output idle state to low */
		tmpcr2 &= ~(0x1 << PWM_CR2_OIS_OFF(chn));
		/* Set complimentary output idle state to low */
		tmpcr2 &= ~(0x1 << PWM_CR2_OISN_OFF(chn));
	}

	*(pwm_common.timBase[timer] + tim_cr2) = tmpcr2;
	*(pwm_common.timBase[timer] + PWM_CCMR_REG(chn)) = tmpccmr;
	/* Set compare value in CCRx */
	*(pwm_common.timBase[timer] + PWM_CCR_REG(chn)) = compare;
	*(pwm_common.timBase[timer] + tim_ccer) = tmpccer;

	/* Disable fast output */
	*(pwm_common.timBase[timer] + PWM_CCMR_REG(chn)) &= ~(0x1 << PWM_CCMR_OCFE_OFF(chn));
	/* Set master-slave mode to disabled (I think?) For now TIM1/8 */
	*(pwm_common.timBase[timer] + tim_cr2) &= ~((0xF << 4) | (0xF << 20) | (0x1 << 25));
	*(pwm_common.timBase[timer] + tim_smcr) &= ~(0x1 << 7);

	/* Initialize BDTR register. We don't need the dead time feature. */
	tmpbdtr = 0;
	tmpbdtr |= (0x1 << 13) | (0x1 << 25);
	*(pwm_common.timBase[timer] + tim_bdtr) = tmpbdtr;

	// Also simplify control flow when the pwm channel is already set, to just modify the compare value

	/* Enable campture/compare interrupt */
	*(pwm_common.timBase[timer] + tim_dier) |= (0x1 << PWM_DIER_CCIE_OFF(chn));
	/* Enable output channel (and it's compliment) */
	*(pwm_common.timBase[timer] + tim_ccer) |= (0x1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		*(pwm_common.timBase[timer] + tim_ccer) |= (0x1 << PWM_CCER_CCNE_OFF(chn));
	}
	/* Enable counter if disabled (CR1 CER) */
	*(pwm_common.timBase[timer] + tim_cr1) |= 0x1;
	/* Force an update event */
	*(pwm_common.timBase[timer] + tim_egr) |= 0x1;

	return EOK;
}


/* Returns current duty cycle percentage. Or errors */
int pwm_get(pwm_tim_id_t timer, pwm_ch_id_t chn)
{
	return -ENOSYS;
}


int pwm_setBitSequence(void)
{
	return -ENOSYS;
}


void pwm_init(void)
{
	return;
}
