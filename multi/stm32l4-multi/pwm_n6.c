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

/* IMPORTANT NOTE:
 * The TIMG timer feeding TIMx peripherals is clocked at 1/4 of the HCLK (cpu),
 * furthermore TIMx is connected via APB which causes register writes to be lost,
 * as they are coming in too fast. For that reason, writes have to be guarded by while loops. (temporary fix)
 * Also, for now the driver is not synchronized, consider adding channel/timer level locking.
 */

#include <stdio.h>
#include <errno.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <sys/pwman.h>
#include "pwm_n6.h"

#include <unistd.h>

#include "common.h"

/* Interrupt data for one timer */
typedef struct pwm_irq_t {
	unsigned int uevirq; /* Update Event interrupt request */
	uint8_t initialized; /* Cond and mutex initialized */
	uint8_t uevReceived;
	handle_t uevlock;
	handle_t uevcond;
} pwm_irq_t;

/* TODO: Consider moving all data related to a single timer to one struct.
 * Downside is that it would be more annoying to innitialize the array of timer related data
 */
typedef struct pwm_common_t {
	volatile uint32_t *const timBase[PWM_TIMER_NUM];
	const uint8_t timChannelSet[PWM_TIMER_NUM];
	uint16_t timArr[PWM_TIMER_NUM];  /* Auto Reload Register (top) */
	pwm_irq_t timIrq[PWM_TIMER_NUM]; /* Interrupt request data */
	uint8_t timChnOn[PWM_TIMER_NUM]; /* Is given channel on (0x1 << channel_id)*/
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
	.timArr = {},
	.timIrq = {
		{.uevirq = tim1_up_irq},
		{.uevirq = tim2_irq},
		{.uevirq = tim3_irq},
		{.uevirq = tim4_irq},
		{.uevirq = tim5_irq},
		{.uevirq = tim8_up_irq},
		{.uevirq = tim9_irq},
		{.uevirq = tim10_irq},
		{.uevirq = tim11_irq},
		{.uevirq = tim12_irq},
		{.uevirq = tim13_irq},
		{.uevirq = tim14_irq},
		{.uevirq = tim15_irq},
		{.uevirq = tim16_irq},
		{.uevirq = tim17_irq},
	},
};
/* clang-format on */


// /* This is necessary to slow down the cpu enough for APB bus to catch up */
// static inline void pwm_nop(int32_t times)
// {
// 	while (times-- > 0) {
// 		__asm__ volatile ("nop");
// 	}
// }


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


static int pwm_updateEventIrq(unsigned int n, void *arg)
{
	/* Cursed pointer to int cast */
	pwm_tim_id_t timer = (pwm_tim_id_t)arg;
	/* Disable interrupts by clearing UIE */
	while ((*(pwm_common.timBase[timer] + tim_dier) & 0x1) != 0) {
		*(pwm_common.timBase[timer] + tim_dier) &= ~0x1;
		*(pwm_common.timBase[timer] + tim_dier) &= ~0x1;
		*(pwm_common.timBase[timer] + tim_dier) &= ~0x1;
		*(pwm_common.timBase[timer] + tim_sr) &= ~0x1;
	}
	/* Clear UIF in SR */
	while ((*(pwm_common.timBase[timer] + tim_sr) & 0x1) != 0) {
		*(pwm_common.timBase[timer] + tim_sr) &= ~0x1;
		*(pwm_common.timBase[timer] + tim_sr) &= ~0x1;
		*(pwm_common.timBase[timer] + tim_sr) &= ~0x1;
		*(pwm_common.timBase[timer] + tim_sr) &= ~0x1;
	}
	pwm_common.timIrq[timer].uevReceived = 1;
	// printf("INTR\n"); // DEBUG
	return 0;
}


// static void pwm_enable(pwm_tim_id_t timer, pwm_ch_id_t chn)
// {
// 	return;
// }


__attribute__((unused)) static void pwm_printRegisters(pwm_tim_id_t timer)
{
	/* Assume correct timer */
	printf("tim_cr1: %x\n", *((pwm_common.timBase[timer] + tim_cr1)));
	printf("tim_cr2: %x\n", *(pwm_common.timBase[timer] + tim_cr2));
	printf("tim_smcr: %x\n", *(pwm_common.timBase[timer] + tim_smcr));
	printf("tim_dier: %x\n", *(pwm_common.timBase[timer] + tim_dier));
	printf("tim_sr: %x\n", *(pwm_common.timBase[timer] + tim_sr));
	printf("tim_egr: %x\n", *((pwm_common.timBase[timer] + tim_egr)));
	printf("tim_ccmr1: %x\n", *(pwm_common.timBase[timer] + tim_ccmr1));
	printf("tim_ccmr2: %x\n", *(pwm_common.timBase[timer] + tim_ccmr2));
	printf("tim_ccer: %x\n", *(pwm_common.timBase[timer] + tim_ccer));
	printf("tim_cnt: %x\n", *(pwm_common.timBase[timer] + tim_cnt));
	printf("tim_psc: %x\n", *((pwm_common.timBase[timer] + tim_psc)));
	printf("tim_arr: %x\n", *(pwm_common.timBase[timer] + tim_arr));
	printf("tim_rcr: %x\n", *((pwm_common.timBase[timer] + tim_rcr)));
	printf("tim_ccr1: %x\n", *(pwm_common.timBase[timer] + tim_ccr1));
	printf("tim_ccr2: %x\n", *(pwm_common.timBase[timer] + tim_ccr2));
	printf("tim_ccr3: %x\n", *(pwm_common.timBase[timer] + tim_ccr3));
	printf("tim_ccr4: %x\n", *(pwm_common.timBase[timer] + tim_ccr4));
	printf("tim_bdtr: %x\n", *(pwm_common.timBase[timer] + tim_bdtr));
	printf("tim_ccr5: %x\n", *(pwm_common.timBase[timer] + tim_ccr5));
	printf("tim_ccr6: %x\n", *(pwm_common.timBase[timer] + tim_ccr6));
	printf("tim_ccmr3: %x\n", *(pwm_common.timBase[timer] + tim_ccmr3));
	printf("tim_dtr2: %x\n", *(pwm_common.timBase[timer] + tim_dtr2));
	printf("tim_ecr: %x\n", *(pwm_common.timBase[timer] + tim_ecr));
	printf("tim_tisel: %x\n", *(pwm_common.timBase[timer] + tim_tisel));
	printf("tim_af1: %x\n", *(pwm_common.timBase[timer] + tim_af1));
	printf("tim_af2: %x\n", *(pwm_common.timBase[timer] + tim_af2));
	printf("tim_dcr: %x\n", *(pwm_common.timBase[timer] + tim_dcr));
	printf("tim_dmar: %x\n", *(pwm_common.timBase[timer] + tim_dmar));
}


/* Returns errors */
int pwm_disableTimer(pwm_tim_id_t timer)
{
	int res;
	if ((res = pwm_validateTimer(timer)) < 0) {
		return res;
	}

	/* Clear CEN in CR1 */
	while ((*(pwm_common.timBase[timer] + tim_cr1) & 0x1) != 0)
		*(pwm_common.timBase[timer] + tim_cr1) &= ~0x1;

	return EOK;
}


int pwm_disableChannel(pwm_tim_id_t timer, pwm_ch_id_t chn)
{
	int res;
	if ((res = pwm_validateTimer(timer)) < 0) {
		return res;
	}
	if ((res = pwm_validateChannel(timer, chn)) < 0) {
		return res;
	}

	/* Disable the channel. Clear CCxE in CCER */
	while ((*(pwm_common.timBase[timer] + tim_ccer) & (0x1 << PWM_CCER_CCE_OFF(chn))) != 0)
		*(pwm_common.timBase[timer] + tim_ccer) &= ~(0x1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		while ((*(pwm_common.timBase[timer] + tim_ccer) & (0x1 << PWM_CCER_CCNE_OFF(chn))) != 0)
			*(pwm_common.timBase[timer] + tim_ccer) &= ~(0x1 << PWM_CCER_CCNE_OFF(chn));
	}
	/* Notify that channel is closed */
	pwm_common.timChnOn[timer] &= ~(0x1 << pwm_ch1);
	return EOK;
}


uint64_t pwm_getBaseFrequency(pwm_tim_id_t timer)
{
	uint64_t baseFreq = 0ull;
	clockdef_getClock(clkid_timg, &baseFreq);
	return baseFreq;
}


/* Configure a compatible timer for pwm. Returns errors */
int pwm_configure(pwm_tim_id_t timer, uint16_t prescaler, uint16_t top)
{
	/* TODO: For now assuming full configuration such as in TIM1/TIM8.
	 * Later check if each step makes sense for simpler timers and 'if out' advanced configuration
	 * NOTE: All registers are 4-byte aligned. Acces them via uint32*.
	 * NOTE: APB clocking mismatch with HCLK cause unreliavble reads and writes.
	 */
	int res;
	uint16_t t16;
	// uint32_t t32;
	if ((res = pwm_validateTimer(timer)) < 0) {
		return res;
	}

	if (!pwm_common.timIrq[timer].initialized) {
		mutexCreate(&pwm_common.timIrq[timer].uevlock);
		condCreate(&pwm_common.timIrq[timer].uevcond);
		res = interrupt(pwm_common.timIrq[timer].uevirq, pwm_updateEventIrq, (void *)timer, pwm_common.timIrq[timer].uevcond, NULL);
		pwm_common.timIrq[timer].initialized = 1;
	}

	/* Fail if one of the channels hasn't been disabled */
	if (pwm_common.timChnOn[timer] != 0) {
		return -EPERM;
	}

	/* In CR1 set prescaler, countermode, autoreload, clockdivision, repetition counter */
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	t16 = *((pwm_common.timBase[timer] + tim_cr1));
	syncBarrier();
	/* Set counter mode UP in CR1 => clear DIR and CMS */
	t16 &= ~((0x3 << 5) | (0x1 << 4));
	/* Set clock division 1 in CR1 => clear CKD */
	t16 &= ~(0x3 << 8);
	/* Disable counter */
	t16 &= ~0x1;

	while (*(pwm_common.timBase[timer] + tim_cr1) != t16)
		*(pwm_common.timBase[timer] + tim_cr1) = t16;
	syncBarrier();

	/* In AF1/2 disable break input */
	while ((*(pwm_common.timBase[timer] + tim_af1) & 0x1) != 0)
		*(pwm_common.timBase[timer] + tim_af1) &= ~0x1;

	while ((*(pwm_common.timBase[timer] + tim_af2) & 0x1) != 0)
		*(pwm_common.timBase[timer] + tim_af2) &= ~0x1;

	/* Set autoreload in ARR */
	pwm_common.timArr[timer] = top;

	while (*(pwm_common.timBase[timer] + tim_arr) != top)
		*(pwm_common.timBase[timer] + tim_arr) = top;

	/* Set prescaler in PSC */
	while (*(pwm_common.timBase[timer] + tim_psc) != prescaler)
		*(pwm_common.timBase[timer] + tim_psc) = prescaler;

	/* Set repetition counter to 0 in RCR */  // only if it exists in this timer
	while (*(pwm_common.timBase[timer] + tim_rcr) != 0)
		*(pwm_common.timBase[timer] + tim_rcr) = 0;

	syncBarrier();

	/* Enable update event interrupt */
	while ((*(pwm_common.timBase[timer] + tim_dier) & 0x1) == 0)
		*(pwm_common.timBase[timer] + tim_dier) |= 0x1;

	syncBarrier();
	/* Generate update event (UG bit in EGR) to reload prescaler and repetition counter */
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	*(pwm_common.timBase[timer] + tim_egr) |= 0x1;
	syncBarrier();

	/* Enable autoreload buffering in CR1 ARPE */
	while ((*(pwm_common.timBase[timer] + tim_cr1) & (0x1 << 7)) == 0)
		*(pwm_common.timBase[timer] + tim_cr1) |= (0x1 << 7);

	syncBarrier();

	printf("Locking...\n");

	/* Wait for update event to load prescaler and arr */
	keepidle(1);
	mutexLock(pwm_common.timIrq[timer].uevlock);
	//(*(pwm_common.timBase[timer] + tim_arr) & 0xFFFF) != top
	while (pwm_common.timIrq[timer].uevReceived == 0) {
		// printf("ARR: %08x\n", *(pwm_common.timBase[timer] + tim_arr) & 0xFFFF);
		condWait(pwm_common.timIrq[timer].uevcond, pwm_common.timIrq[timer].uevlock, 0);
	}
	pwm_common.timIrq[timer].uevReceived = 0;
	mutexUnlock(pwm_common.timIrq[timer].uevlock);
	keepidle(0);

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
	uint32_t tmpcr2 = 0, tmpccmr = 0, tmpccer = 0, tmpbdtr = 0;
	(void)tmpbdtr;
	/* Validate channel/timer compatibility */
	if ((res = pwm_validateChannel(timer, chn)) < 0) {
		return res;
	}
	if (compare > pwm_common.timArr[timer]) {
		return -EINVAL;
	}
	/* If channel is on, then only reconfigure compare value */
	if (pwm_common.timChnOn[timer] & (0x1 << chn)) {
		while (*(pwm_common.timBase[timer] + PWM_CCR_REG(chn)) != compare)
			*(pwm_common.timBase[timer] + PWM_CCR_REG(chn)) = compare;
		return EOK;
	}

	/* Set output channel preloading in CCMRx OCyPE */
	reg = (pwm_common.timBase[timer] + PWM_CCMR_REG(chn));
	while ((*reg & (0x1 << PWM_CCMR_OCPE_OFF(chn))) == 0)
		*reg |= (0x1 << PWM_CCMR_OCPE_OFF(chn));
	/* TODO: Find out what's the deal with channel 5,6. Why are they not in the pin table, but in the register description? */

	syncBarrier();

	/* Disable the channel. Clear CCxE in CCER */
	while ((*(pwm_common.timBase[timer] + tim_ccer) & (0x1 << PWM_CCER_CCE_OFF(chn))) != 0)
		*(pwm_common.timBase[timer] + tim_ccer) &= ~(0x1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		while ((*(pwm_common.timBase[timer] + tim_ccer) & (0x1 << PWM_CCER_CCNE_OFF(chn))) != 0)
			*(pwm_common.timBase[timer] + tim_ccer) &= ~(0x1 << PWM_CCER_CCNE_OFF(chn));
	}

	syncBarrier();

	// NOTE: For OC with complements enabling also requires a trillion other bits in BDTR or sth
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	tmpcr2 = *(pwm_common.timBase[timer] + tim_cr2);
	syncBarrier();
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	tmpccmr = *(pwm_common.timBase[timer] + PWM_CCMR_REG(chn));
	syncBarrier();
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	tmpccer = *(pwm_common.timBase[timer] + tim_ccer);
	syncBarrier();

	// NOTE: CCMR works differently on TIM10-14 :(
	/* Set channel as output. Clear CCyS */
	tmpccmr &= ~(0x3 << PWM_CCMR_CCS_OFF(chn));
	/* Set output compare mode to PWM 1 mode (0110 bits 16, 6:4)*/
	tmpccmr &= ~((0x1 << PWM_CCMR_OCMH_OFF(chn)) | (0x7 << PWM_CCMR_OCML_OFF(chn)));
	tmpccmr |= (0x6 << PWM_CCMR_OCML_OFF(chn));
	/* Set output compare polarity to active high (0)*/
	tmpccer &= ~(0x1 << PWM_CCER_CCP_OFF(chn));

	// This isn't necessarily a check if complement exists, but if there is break mode
	if (pwm_channelHasComplement(timer, chn)) {
		/* Disable the complementary channel */
		tmpccer &= ~(0x1 << PWM_CCER_CCNE_OFF(chn));
		/* Set complementary output polarity (to active high) */
		tmpccer &= ~(0x1 << PWM_CCER_CCNP_OFF(chn));
		/* Set output idle state to low */
		tmpcr2 &= ~(0x1 << PWM_CR2_OIS_OFF(chn));
		/* Set complementary output idle state to low */
		tmpcr2 &= ~(0x1 << PWM_CR2_OISN_OFF(chn));
	}

	while (*(pwm_common.timBase[timer] + tim_cr2) != tmpcr2)
		*(pwm_common.timBase[timer] + tim_cr2) = tmpcr2;
	while (*(pwm_common.timBase[timer] + PWM_CCMR_REG(chn)) != tmpccmr)
		*(pwm_common.timBase[timer] + PWM_CCMR_REG(chn)) = tmpccmr;
	/* Set compare value in CCRx */
	while (*(pwm_common.timBase[timer] + PWM_CCR_REG(chn)) != compare)
		*(pwm_common.timBase[timer] + PWM_CCR_REG(chn)) = compare;
	while (*(pwm_common.timBase[timer] + tim_ccer) != tmpccer)
		*(pwm_common.timBase[timer] + tim_ccer) = tmpccer;

	syncBarrier();

	/* Disable fast output */
	while ((*(pwm_common.timBase[timer] + PWM_CCMR_REG(chn)) & (0x1 << PWM_CCMR_OCFE_OFF(chn))) != 0)
		*(pwm_common.timBase[timer] + PWM_CCMR_REG(chn)) &= ~(0x1 << PWM_CCMR_OCFE_OFF(chn));
	/* Set master-slave mode to disabled (I think?) For now TIM1/8 */
	while ((*(pwm_common.timBase[timer] + tim_cr2) & ((0x7 << 4) | (0xF << 20) | (0x1 << 25))) != 0)
		*(pwm_common.timBase[timer] + tim_cr2) &= ~((0x7 << 4) | (0xF << 20) | (0x1 << 25));
	while ((*(pwm_common.timBase[timer] + tim_smcr) & (0x1 << 7)) != 0)
		*(pwm_common.timBase[timer] + tim_smcr) &= ~(0x1 << 7);

	/* Initialize BDTR register. We don't need the dead time feature. */
	tmpbdtr = 0;
	tmpbdtr |= (0x1 << 13) | (0x1 << 25);
	/* Enable main output (MOE) */
	tmpbdtr |= (0x1 << 15);
	while (*(pwm_common.timBase[timer] + tim_bdtr) != tmpbdtr)
		*(pwm_common.timBase[timer] + tim_bdtr) = tmpbdtr;

	syncBarrier();
	// // Also simplify control flow when the pwm channel is already set, to just modify the compare value

	// /* Enable capture/compare interrupt */
	// *(pwm_common.timBase[timer] + tim_dier) |= (0x1 << PWM_DIER_CCIE_OFF(chn));
	/* Enable output channel (and it's complement) */
	while ((*(pwm_common.timBase[timer] + tim_ccer) & (0x1 << PWM_CCER_CCE_OFF(chn))) == 0)
		*(pwm_common.timBase[timer] + tim_ccer) |= (0x1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		while ((*(pwm_common.timBase[timer] + tim_ccer) & (0x1 << PWM_CCER_CCNE_OFF(chn))) == 0)
			*(pwm_common.timBase[timer] + tim_ccer) |= (0x1 << PWM_CCER_CCNE_OFF(chn));
	}

	syncBarrier();
	/* Enable counter if disabled (CR1 CER) */
	while (((*pwm_common.timBase[timer] + tim_cr1) & (0x1)) == 0)
		*(pwm_common.timBase[timer] + tim_cr1) |= 0x1;

	syncBarrier();
	/* Force an update event */
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	__asm__ volatile("nop");
	*(pwm_common.timBase[timer] + tim_egr) |= 0x1;

	syncBarrier();

	/* Notify that channel is on */
	pwm_common.timChnOn[timer] |= (0x1 << pwm_ch1);
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


int pwm_init(void)
{
	uint64_t timgFreq, hclkFreq;
	if (clockdef_getClock(clkid_timg, &timgFreq) < 0) {
		return -1;
	}

	if (clockdef_getClock(clkid_hclk, &hclkFreq) < 0) {
		return -1;
	}

	/* Calculate the required slowdown factor */
	// pwm_common.requiredNops = 2 * (hclkFreq / timgFreq);


	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_devclk,
		.devclk = {
			.dev = pctl_tim1,
			.state = 1,
			.lpState = 1,
		}
	};

	if (platformctl(&pctl) < 0) {
		printf("pctl failed\n");  // Debug only
		return -1;
	}

	platformctl_t pctl1 = {
		.action = pctl_get,
		.type = pctl_devclk,
		.devclk = {
			.dev = pctl_tim1 }
	};

	if (platformctl(&pctl1) < 0) {
		printf("pctl failed\n");  // Debug only
		return -1;
	}

	printf("TIM1 clock: %u %u\n", pctl1.devclk.state, pctl1.devclk.lpState);

	return 0;
}
