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


#include <stdio.h>
#include <errno.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <sys/pwman.h>
#include "pwm_n6.h"

#include "common.h"

#include <unistd.h>
#include <stdlib.h>
#include <time.h>


#define PWM_IRQ_INITIALIZED (1 << 0)
#define PWM_IRQ_UEVRECIEVED (1 << 1)
#define PWM_IRQ_DSHOT_UEV   (1 << 2) /* IRQ used to reconfigure duty cycle */
#define PWM_IRQ_DSHOT_END   (1 << 3) /* DSHOT bit sequence complete */


/* Save dshot context to avoid having to malloc a buffor of compare values */
typedef struct pwm_dshot_ctx_t {
	pwm_ch_id_t chn;   /* Channel used for pwm */
	uint16_t compare0; /* Compare val if 0 bit */
	uint16_t compare1; /* Compare val if 1 bit */
	uint32_t bitSize;  /* Number of bits in sequence */
	uint32_t bitPos;   /* Current bit to process */
	uint8_t *data;     /* Array of data bits */
} pwm_dshot_ctx_t;

/* Interrupt data for one timer */
typedef struct pwm_irq_t {
	unsigned int uevirq; /* Update Event interrupt request */
	uint32_t flags;
	handle_t uevlock;
	handle_t uevcond;
	pwm_dshot_ctx_t dshot;
} pwm_irq_t;

// typedef struct pwm_common_t {
// 	volatile uint32_t *const timBase[PWM_TIMER_NUM];
// 	const uint8_t timChannelSet[PWM_TIMER_NUM];
// 	uint16_t timArr[PWM_TIMER_NUM];  /* Auto Reload Register (top) */
// 	pwm_irq_t timIrq[PWM_TIMER_NUM]; /* Interrupt request data */
// 	uint8_t timChnOn[PWM_TIMER_NUM]; /* Is given channel on (1 << channel_id) */
// } pwm_common_t;

typedef struct pwm_tim_setup_t {
	volatile uint32_t *const base;
	const uint8_t channels; /* If channel available (1 << channel_id) set */
} pwm_tim_setup_t;

typedef struct pwm_tim_data_t {
	pwm_irq_t irq;
	uint16_t arr;
	uint8_t channelOn;

} pwm_tim_data_t;

static const struct {
	pwm_tim_setup_t timer[pwm_tim_count]
} pwm_setup = {
	.timer = {
		[pwm_tim1] = {
			.base = { TIM1_BASE },
			.channels = (1 << pwm_ch1) | (1 << pwm_ch1n) | (1 << pwm_ch2) | (1 << pwm) } }
};

static struct {
	pwm_tim_data_t timer[pwm_tim_count]
} pwm_common = {};

/* clang-format off */
// static pwm_common_t pwm_common = {
// 	/* Basic timers TIM6/TIM7/TIM18 not available for PWM */
// 	.timBase = {
// 		TIM1_BASE,
// 		TIM2_BASE,
// 		TIM3_BASE,
// 		TIM4_BASE,
// 		TIM5_BASE,
// 		TIM8_BASE,
// 		TIM9_BASE,
// 		TIM10_BASE,
// 		TIM11_BASE,
// 		TIM12_BASE,
// 		TIM13_BASE,
// 		TIM14_BASE,
// 		TIM15_BASE,
// 		TIM16_BASE,
// 		TIM17_BASE,
// 	},
// 	.timChannelSet = {
// 		(1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4) | \
//         (1 << pwm_ch1n) | (1 << pwm_ch2n) | (1 << pwm_ch3n) | (1 << pwm_ch4n),  // TIM1
// 		(1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4),      // TIM2
// 		(1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4),      // TIM3
// 		(1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4),      // TIM4
// 		(1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4),      // TIM5
// 		(1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4) | \
//         (1 << pwm_ch1n) | (1 << pwm_ch2n) | (1 << pwm_ch3n) | (1 << pwm_ch4n),  // TIM8
// 		(1 << pwm_ch1) | (1 << pwm_ch2),                                            // TIM9
// 		(1 << pwm_ch1),                                                               // TIM10
// 		(1 << pwm_ch1),                                                               // TIM11
// 		(1 << pwm_ch1),                                                               // TIM12
// 		(1 << pwm_ch1),                                                               // TIM13
// 		(1 << pwm_ch1),                                                               // TIM14
// 		(1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch1n),                          // TIM15
// 		(1 << pwm_ch1) | (1 << pwm_ch1n),                                           // TIM16
// 		(1 << pwm_ch1) | (1 << pwm_ch1n),                                           // TIM17
// 	},
// 	.timArr = {},
// 	.timIrq = {
// 		{.uevirq = tim1_up_irq},
// 		{.uevirq = tim2_irq},
// 		{.uevirq = tim3_irq},
// 		{.uevirq = tim4_irq},
// 		{.uevirq = tim5_irq},
// 		{.uevirq = tim8_up_irq},
// 		{.uevirq = tim9_irq},
// 		{.uevirq = tim10_irq},
// 		{.uevirq = tim11_irq},
// 		{.uevirq = tim12_irq},
// 		{.uevirq = tim13_irq},
// 		{.uevirq = tim14_irq},
// 		{.uevirq = tim15_irq},
// 		{.uevirq = tim16_irq},
// 		{.uevirq = tim17_irq},
// 	},
// };
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
	if ((pwm_common.timChannelSet[timer] & (1 << chn)) == 0) {
		return -EINVAL;
	}
	return EOK;
}

static int pwm_channelHasComplement(pwm_tim_id_t timer, pwm_ch_id_t chn)
{
	/* Assume timer/channel is already correct */
	if ((pwm_common.timChannelSet[timer] & (1 << (chn + 4))) == 0) {
		return 0;
	}
	return 1;
}


static void pwm_disableMasterSlave(pwm_tim_id_t timer)
{
	uint32_t cr2 = 0;
	uint32_t smcr = (0x7) | (1 << 7) | (1 << 16);
	/* Assume timer/channel is already correct */
	if ((1 << timer) & 0x6D80) {
		return;
	}
	/* Clear MSM and SMS in SMCR */
	*(pwm_common.timBase[timer] + tim_smcr) &= ~smcr;
	/* TIM1/8 */
	if ((1 << timer) & 0x21) {
		cr2 = (0x7 << 4) | (0xF << 20) | (1 << 25);
	}
	/* TIM2..5 */
	else if ((1 << timer) & 0x1e) {
		cr2 = (0x7 << 4) | (1 << 25);
	}
	/* TIM9/12/15 */
	else if ((1 << timer) & 0x1240) {
		cr2 = 0x7 << 4;
	}
	*(pwm_common.timBase[timer] + tim_cr2) &= ~cr2;
}


__attribute__((unused)) static int pwm_updateEventIrq(unsigned int n, void *arg)
{
	/* Cursed pointer to int cast */
	pwm_tim_id_t timer = (pwm_tim_id_t)arg;

	/* If irq doesn't come from DSHOT, didsable future UEV interrupts. Clear UIE */
	if (!(pwm_common.timIrq[timer].flags & PWM_IRQ_DSHOT_UEV)) {
		*(pwm_common.timBase[timer] + tim_dier) &= ~1u;
	}
	else {
		pwm_ch_id_t chn = pwm_common.timIrq[timer].dshot.chn;
		/* If last bit then preload compare value to 0 to end the sequence */
		if (pwm_common.timIrq[timer].dshot.bitPos > pwm_common.timIrq[timer].dshot.bitSize - 1) {
			/* Also preload compare value of 0 to make sure the signal ends after the last bit */
			*(pwm_common.timBase[timer] + PWM_CCR_REG(chn)) = 0;
			pwm_common.timIrq[timer].dshot.bitPos++;
		}
		/* If the last bit was already emitted, can safely disable further UEV IRQ */
		else if (pwm_common.timIrq[timer].dshot.bitPos > pwm_common.timIrq[timer].dshot.bitSize) {
			*(pwm_common.timBase[timer] + tim_dier) &= ~1u;
			pwm_common.timIrq[timer].flags |= PWM_IRQ_DSHOT_END;
		}
		else {
			/* Preload next compare value on the correct channel. */
			uint32_t bitPos = pwm_common.timIrq[timer].dshot.bitPos;
			uint8_t bit = (*(pwm_common.timIrq[timer].dshot.data + bitPos / 8) >> (bitPos & 0x7)) & 0x1;
			uint16_t compare = (bit == 0) ? pwm_common.timIrq[timer].dshot.compare0 : pwm_common.timIrq[timer].dshot.compare1;
			*(pwm_common.timBase[timer] + PWM_CCR_REG(chn)) = compare;
			pwm_common.timIrq[timer].dshot.bitPos++;
		}
	}

	/* Clear UIF in SR */
	*(pwm_common.timBase[timer] + tim_sr) &= ~1u;
	pwm_common.timIrq[timer].flags |= PWM_IRQ_UEVRECIEVED;
	return 0;
}


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

	/* Disable all enabled channels */
	for (pwm_ch_id_t chn = 0; chn < PWM_CHN_NUM; chn++) {
		if (pwm_common.timChnOn[timer] & (1 << chn)) {
			pwm_disableChannel(timer, chn);
		}
	}

	/* Clear CEN in CR1 */
	while ((*(pwm_common.timBase[timer] + tim_cr1) & 0x1) != 0)
		*(pwm_common.timBase[timer] + tim_cr1) &= ~1u;

	return EOK;
}

/* Returns errors */
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
	*(pwm_common.timBase[timer] + tim_ccer) &= ~(1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		*(pwm_common.timBase[timer] + tim_ccer) &= ~(1 << PWM_CCER_CCNE_OFF(chn));
	}
	/* Notify that channel is closed */
	pwm_common.timChnOn[timer] &= ~(1 << pwm_ch1);
	return EOK;
}


uint64_t pwm_getBaseFrequency(pwm_tim_id_t timer)
{
	uint64_t baseFreq = 0ull;
	clockdef_getClock(clkid_timg, &baseFreq);
	return baseFreq;
}


__attribute__((unused)) static void pwm_apbTest(pwm_tim_id_t timer, uint16_t prescaler, uint16_t top)
{
	srand((unsigned)time(NULL));
	uint16_t val = rand() & 0xFFFF;
	printf("Rand value: 0x%04x\n", val);
	*((volatile uint32_t *)0x5200002c) = val;

	dataBarier();

	sleep(2);

	pwm_printRegisters(timer);
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
	if ((res = pwm_validateTimer(timer)) < 0) {
		return res;
	}

	if (!(pwm_common.timIrq[timer].flags & PWM_IRQ_INITIALIZED)) {
		mutexCreate(&pwm_common.timIrq[timer].uevlock);
		condCreate(&pwm_common.timIrq[timer].uevcond);
		interrupt(pwm_common.timIrq[timer].uevirq, pwm_updateEventIrq, (void *)timer, pwm_common.timIrq[timer].uevcond, NULL);
		pwm_common.timIrq[timer].flags |= PWM_IRQ_INITIALIZED;
	}

	/* Fail if one of the channels hasn't been disabled */
	if (pwm_common.timChnOn[timer] != 0) {
		return -EPERM;
	}

	/* In CR1 set prescaler, countermode, autoreload, clockdivision, repetition counter */
	t16 = *((pwm_common.timBase[timer] + tim_cr1));
	dataBarier();

	/* Set counter mode UP in CR1 => clear DIR and CMS */
	if (PWM_TIM_COUNTER_MODE_SELECT(timer)) {
		t16 &= ~((0x3 << 5) | (1 << 4));
	}
	/* Set clock division 1 in CR1 => clear CKD */
	t16 &= ~(0x3 << 8);
	/* Disable counter */
	t16 &= ~1u;

	*(pwm_common.timBase[timer] + tim_cr1) = t16;
	dataBarier();

	/* In AF1/2 disable break input */
	if (PWM_TIM_BREAK1_MODE(timer)) {
		*(pwm_common.timBase[timer] + tim_af1) &= ~1u;
	}
	if (PWM_TIM_BREAK2_MODE(timer)) {
		*(pwm_common.timBase[timer] + tim_af2) &= ~1u;
	}

	/* Set autoreload in ARR */
	pwm_common.timArr[timer] = top;

	*(pwm_common.timBase[timer] + tim_arr) = top;

	/* Set prescaler in PSC */
	*(pwm_common.timBase[timer] + tim_psc) = prescaler;

	/* Set repetition counter to 0 in RCR */
	if (PWM_TIM_REP_COUNTER(timer)) {
		*(pwm_common.timBase[timer] + tim_rcr) = 0;
	}

	dataBarier();

	/* Enable update event interrupt */
	*(pwm_common.timBase[timer] + tim_dier) |= 0x1;

	dataBarier();
	/* Generate update event (UG bit in EGR) to reload prescaler and repetition counter */
	*(pwm_common.timBase[timer] + tim_egr) |= 0x1;

	dataBarier();

	/* Enable autoreload buffering in CR1 ARPE */
	*(pwm_common.timBase[timer] + tim_cr1) |= (1 << 7);

	dataBarier();

	printf("Locking...\n");

	/* Wait for update event to load prescaler and arr */
	mutexLock(pwm_common.timIrq[timer].uevlock);
	while (!(pwm_common.timIrq[timer].flags & PWM_IRQ_UEVRECIEVED)) {
		condWait(pwm_common.timIrq[timer].uevcond, pwm_common.timIrq[timer].uevlock, 0);
	}
	pwm_common.timIrq[timer].flags &= ~PWM_IRQ_UEVRECIEVED;
	mutexUnlock(pwm_common.timIrq[timer].uevlock);

	return EOK;
}


/* Set output pwm channel on configured timer. Returns errors */
int pwm_set(pwm_tim_id_t timer, pwm_ch_id_t chn, uint16_t compare)
{
	int res;
	volatile uint32_t *reg;
	uint32_t tmpcr2 = 0, tmpccmr = 0, tmpccer = 0, tmpbdtr = 0;
	/* Validate channel/timer compatibility */
	if ((res = pwm_validateChannel(timer, chn)) < 0) {
		return res;
	}
	if (compare > pwm_common.timArr[timer]) {
		return -EINVAL;
	}
	/* If channel is on, then only reconfigure compare value */
	if (pwm_common.timChnOn[timer] & (1 << chn)) {
		*(pwm_common.timBase[timer] + PWM_CCR_REG(chn)) = compare;
		return EOK;
	}

	/* Set output channel preloading in CCMRx OCyPE */
	reg = (pwm_common.timBase[timer] + PWM_CCMR_REG(chn));
	*reg |= (1 << PWM_CCMR_OCPE_OFF(chn));

	/* TODO: Find out what's the deal with channel 5,6. Why are they not in the pin table, but in the register description? */

	dataBarier();

	/* Disable the channel. Clear CCxE in CCER */
	*(pwm_common.timBase[timer] + tim_ccer) &= ~(1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		*(pwm_common.timBase[timer] + tim_ccer) &= ~(1 << PWM_CCER_CCNE_OFF(chn));
	}

	dataBarier();

	tmpcr2 = *(pwm_common.timBase[timer] + tim_cr2);
	tmpccmr = *(pwm_common.timBase[timer] + PWM_CCMR_REG(chn));
	tmpccer = *(pwm_common.timBase[timer] + tim_ccer);

	/* Set channel as output. Clear CCyS */
	tmpccmr &= ~(0x3 << PWM_CCMR_CCS_OFF(chn));
	/* Set output compare mode to PWM 1 mode (0110 bits 16, 6:4)*/
	tmpccmr &= ~((1 << PWM_CCMR_OCMH_OFF(chn)) | (0x7 << PWM_CCMR_OCML_OFF(chn)));
	tmpccmr |= (0x6 << PWM_CCMR_OCML_OFF(chn));
	/* Set output compare polarity to active high (0)*/
	tmpccer &= ~(1 << PWM_CCER_CCP_OFF(chn));

	// This isn't necessarily a check if complement exists, but if there is break mode
	if (pwm_channelHasComplement(timer, chn)) {
		/* Disable the complementary channel */
		tmpccer &= ~(1 << PWM_CCER_CCNE_OFF(chn));
		/* Set complementary output polarity (to active high) */
		tmpccer &= ~(1 << PWM_CCER_CCNP_OFF(chn));
		/* Set output idle state to low */
		tmpcr2 &= ~(1 << PWM_CR2_OIS_OFF(chn));
		/* Set complementary output idle state to low */
		tmpcr2 &= ~(1 << PWM_CR2_OISN_OFF(chn));

		*(pwm_common.timBase[timer] + tim_cr2) = tmpcr2;
	}

	/* Write changes */
	*(pwm_common.timBase[timer] + PWM_CCMR_REG(chn)) = tmpccmr;
	*(pwm_common.timBase[timer] + tim_ccer) = tmpccer;
	/* Set compare value in CCRx */
	*(pwm_common.timBase[timer] + PWM_CCR_REG(chn)) = compare;

	dataBarier();

	/* Disable fast output */
	*(pwm_common.timBase[timer] + PWM_CCMR_REG(chn)) &= ~(1 << PWM_CCMR_OCFE_OFF(chn));

	pwm_disableMasterSlave(timer);

	/* Initialize BDTR register. Dead time feature is unnecessary. Disable breaks */
	if (PWM_TIM_BREAK1_MODE(timer) || PWM_TIM_BREAK2_MODE(timer)) {
		tmpbdtr = 0;
		/* Enable main output (MOE) */
		tmpbdtr |= (1 << 15);
		*(pwm_common.timBase[timer] + tim_bdtr) = tmpbdtr;
	}

	dataBarier();

	/* Enable output channel (and it's complement) */
	*(pwm_common.timBase[timer] + tim_ccer) |= (1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		*(pwm_common.timBase[timer] + tim_ccer) |= (1 << PWM_CCER_CCNE_OFF(chn));
	}

	dataBarier();
	/* Enable update event interrupt */
	*(pwm_common.timBase[timer] + tim_dier) |= 0x1;

	/* Temporarily clear the DSHOT_UEV flag to run the regular UEV handler */
	uint32_t oldflags = pwm_common.timIrq[timer].flags;
	pwm_common.timIrq[timer].flags &= ~PWM_IRQ_DSHOT_UEV;
	dataBarier();

	/* Force an update event */
	*(pwm_common.timBase[timer] + tim_egr) |= 0x1;

	/* Wait for the update event */
	mutexLock(pwm_common.timIrq[timer].uevlock);
	while (!(pwm_common.timIrq[timer].flags & PWM_IRQ_UEVRECIEVED)) {
		condWait(pwm_common.timIrq[timer].uevcond, pwm_common.timIrq[timer].uevlock, 0);
	}
	pwm_common.timIrq[timer].flags &= ~PWM_IRQ_UEVRECIEVED;
	mutexUnlock(pwm_common.timIrq[timer].uevlock);

	dataBarier();
	/* Restore the old flags */
	pwm_common.timIrq[timer].flags = oldflags;
	dataBarier();
	/* Enable subsequent UEV interrupts if in dshot mode */
	if (pwm_common.timIrq[timer].flags & PWM_IRQ_DSHOT_UEV) {
		*(pwm_common.timBase[timer] + tim_dier) |= 0x1;
	}

	/* Preload the second compare value if in dshot mode */
	if (pwm_common.timIrq[timer].flags & PWM_IRQ_DSHOT_UEV) {
		uint32_t bitPos = pwm_common.timIrq[timer].dshot.bitPos;
		uint8_t bit = (*(pwm_common.timIrq[timer].dshot.data + bitPos / 8) >> (bitPos & 0x7)) & 0x1;
		compare = (bit == 0) ? pwm_common.timIrq[timer].dshot.compare0 : pwm_common.timIrq[timer].dshot.compare1;
		*(pwm_common.timBase[timer] + PWM_CCR_REG(chn)) = compare;
		pwm_common.timIrq[timer].dshot.bitPos++;
	}

	dataBarier();
	/* Enable counter if disabled (CR1 CER) */
	*(pwm_common.timBase[timer] + tim_cr1) |= 0x1;

	dataBarier();

	/* Mark that channel is on */
	pwm_common.timChnOn[timer] |= (1 << chn);
	return EOK;
}


/* Returns current duty cycle percentage. Or errors */
int pwm_get(pwm_tim_id_t timer, pwm_ch_id_t chn, uint16_t *top, uint16_t *compare)
{
	int res;
	if ((res = pwm_validateTimer(timer)) < 0) {
		return res;
	}
	if ((res = pwm_validateChannel(timer, chn)) < 0) {
		return res;
	}
	if ((pwm_common.timChnOn[timer] & (1 << chn)) == 0) {
		return -EPERM;
	}
	/* Load top and compare. Return compare/top */
	*top = *(pwm_common.timBase[timer] + tim_arr);
	*compare = *(pwm_common.timBase[timer] + PWM_CCR_REG(chn));
	return EOK;
}


int pwm_setBitSequence(pwm_tim_id_t timer, pwm_ch_id_t chn, uint16_t compare0, uint16_t compare1, uint32_t nbits, uint8_t *data)
{
	int res;
	uint16_t firstCompare;
	if ((res = pwm_validateTimer(timer)) < 0) {
		return res;
	}
	if ((res = pwm_validateChannel(timer, chn)) < 0) {
		return res;
	}
	if (nbits < 2) {
		return -EINVAL;
	}
	pwm_common.timIrq[timer].dshot = (pwm_dshot_ctx_t) {
		.chn = chn,
		.bitPos = 1, /* The first compare value is passed via pwm_set parameter */
		.bitSize = nbits,
		.compare0 = compare0,
		.compare1 = compare1,
		.data = data
	};
	pwm_common.timIrq[timer].flags |= PWM_IRQ_DSHOT_UEV;
	pwm_common.timIrq[timer].flags &= ~PWM_IRQ_DSHOT_END;

	dataBarier();

	firstCompare = (((*data) & 0x1) == 0) ? compare0 : compare1;
	pwm_set(timer, chn, firstCompare);


	/* Wait for the DSHOT sequence to end */
	mutexLock(pwm_common.timIrq[timer].uevlock);
	while (!(pwm_common.timIrq[timer].flags & PWM_IRQ_DSHOT_END)) {
		condWait(pwm_common.timIrq[timer].uevcond, pwm_common.timIrq[timer].uevlock, 0);
	}
	pwm_common.timIrq[timer].flags &= ~PWM_IRQ_UEVRECIEVED;
	pwm_common.timIrq[timer].flags &= ~PWM_IRQ_DSHOT_UEV;
	pwm_common.timIrq[timer].flags &= ~PWM_IRQ_DSHOT_END;
	mutexUnlock(pwm_common.timIrq[timer].uevlock);

	pwm_disableChannel(timer, chn);

	return EOK;
}


int pwm_init(void)
{
	// printf("TIMG: %llu MHz\n", timgFreq / 1000000);
	// printf("HCLK: %llu MHz\n", hclkFreq / 1000000);

	/* NOTE: Should enabling the device clocks be done on the fly, or as part of the config? */

	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_devclk,
		.devclk = {
			.dev = pctl_tim16,
			.state = 1,
			.lpState = 1,
		}
	};

	if (platformctl(&pctl) < 0) {
		printf("pctl failed\n");  // Debug only
		return -1;
	}
	return 0;
}
