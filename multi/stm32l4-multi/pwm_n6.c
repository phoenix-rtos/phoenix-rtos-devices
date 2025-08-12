/*
 * Phoenix-RTOS
 *
 * STM32N6 Pulse Width Modulation driver
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
	uint32_t compare0; /* Compare val if 0 bit */
	uint32_t compare1; /* Compare val if 1 bit */
	uint32_t bitSize;  /* Number of bits in sequence */
	uint32_t bitPos;   /* Current bit to process */
	uint8_t *data;     /* Array of data bits */
} pwm_dshot_ctx_t;

/* Interrupt data for one timer */
typedef struct pwm_irq_t {
	uint32_t flags;
	handle_t uevlock;
	handle_t uevcond;
	pwm_dshot_ctx_t dshot;
} pwm_irq_t;

typedef struct pwm_tim_setup_t {
	volatile uint32_t *const base;
	const unsigned int uevirq; /* Update Event interrupt request */
	const uint32_t pctl;
	const uint8_t channels; /* If channel available (1 << channel_id) set */
} pwm_tim_setup_t;

typedef struct pwm_tim_data_t {
	pwm_irq_t irq;
	uint32_t arr;
	uint8_t channelOn;
	uint8_t devOn;

} pwm_tim_data_t;

/* clang-format off */
static const struct {
	pwm_tim_setup_t timer[pwm_tim_count];
} pwm_setup = {
	.timer = {
		[pwm_tim1] = {
			.base = TIM1_BASE,
			.channels = (1 << pwm_ch1) | (1 << pwm_ch1n) | (1 << pwm_ch2) | (1 << pwm_ch2n) |
					    (1 << pwm_ch3) | (1 << pwm_ch3n) | (1 << pwm_ch4) | (1 << pwm_ch4n),
			.uevirq = tim1_up_irq,
			.pctl = pctl_tim1 
		},
		[pwm_tim2] = { 
			.base = TIM2_BASE, 
			.channels = (1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4), 
			.uevirq = tim2_irq,
			 .pctl = pctl_tim2 
		},
		[pwm_tim3] = { 
			.base = TIM3_BASE, 
			.channels = (1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4),
			.uevirq = tim3_irq,
			.pctl = pctl_tim3 
		},
		[pwm_tim4] = { 
			.base = TIM4_BASE, 
			.channels = (1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4), 
			.uevirq = tim4_irq, .pctl = pctl_tim4 
		},
		[pwm_tim5] = { 
			.base = TIM5_BASE, 
			.channels = (1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4), 
			.uevirq = tim4_irq, 
			.pctl = pctl_tim4 },
		[pwm_tim6] = {
			.base = TIM6_BASE,
		},
		[pwm_tim7] = {
			.base = TIM7_BASE,
		},
		[pwm_tim8] = { 
			.base = TIM8_BASE, 
			.channels = (1 << pwm_ch1) | (1 << pwm_ch1n) | (1 << pwm_ch2) | (1 << pwm_ch2n) |
			            (1 << pwm_ch3) | (1 << pwm_ch3n) | (1 << pwm_ch4) | (1 << pwm_ch4n), 
			.uevirq = tim8_up_irq, 
			.pctl = pctl_tim8 
		},
		[pwm_tim9] = { 
			.base = TIM9_BASE, 
			.channels = (1 << pwm_ch1) | (1 << pwm_ch2), 
			.uevirq = tim9_irq, 
			.pctl = pctl_tim9 
		},
		[pwm_tim10] = { 
			.base = TIM10_BASE, 
			.channels = (1 << pwm_ch1), 
			.uevirq = tim10_irq, 
			.pctl = pctl_tim10 
		},
		[pwm_tim11] = { 
			.base = TIM11_BASE, 
			.channels = (1 << pwm_ch1), 
			.uevirq = tim11_irq, 
			.pctl = pctl_tim11 
		},
		[pwm_tim12] = { 
			.base = TIM12_BASE, 
			.channels = (1 << pwm_ch1) | (1 << pwm_ch2), 
			.uevirq = tim12_irq, 
			.pctl = pctl_tim12 
		},
		[pwm_tim13] = { 
			.base = TIM13_BASE, 
			.channels = (1 << pwm_ch1), 
			.uevirq = tim13_irq, 
			.pctl = pctl_tim13 
		},
		[pwm_tim14] = { 
			.base = TIM14_BASE, 
			.channels = (1 << pwm_ch1), 
			.uevirq = tim14_irq, 
			.pctl = pctl_tim14 
		},
		[pwm_tim15] = { 
			.base = TIM15_BASE, 
			.channels = (1 << pwm_ch1) | (1 << pwm_ch1n) | (1 << pwm_ch2), 
			.uevirq = tim15_irq, 
			.pctl = pctl_tim15 
		},
		[pwm_tim16] = { 
			.base = TIM16_BASE, 
			.channels = (1 << pwm_ch1) | (1 << pwm_ch1n), 
			.uevirq = tim16_irq, 
			.pctl = pctl_tim16 
		},
		[pwm_tim17] = { 
			.base = TIM17_BASE, 
			.channels = (1 << pwm_ch1) | (1 << pwm_ch1n), 
			.uevirq = tim17_irq, 
			.pctl = pctl_tim17 
		},
		[pwm_tim18] = {
			.base = TIM18_BASE,
		} }
};
/* clang-format on */

static struct {
	pwm_tim_data_t timer[pwm_tim_count];
} pwm_common = {
	.timer = {}
};


static int pwm_validateTimer(pwm_tim_id_t timer)
{
	if (timer < 0 || timer >= pwm_tim_count) {
		return -EINVAL;
	}
	if (((PWM_TIM_BASIC >> timer) & 1) != 0) {
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
	if ((pwm_setup.timer[timer].channels & (1 << chn)) == 0) {
		return -EINVAL;
	}
	return EOK;
}

static int pwm_channelHasComplement(pwm_tim_id_t timer, pwm_ch_id_t chn)
{
	/* Assume timer/channel is already correct */
	if ((pwm_setup.timer[timer].channels & (1 << (chn + 4))) == 0) {
		return 0;
	}
	return 1;
}


static void pwm_disableMasterSlave(pwm_tim_id_t timer)
{
	uint32_t cr2 = 0;
	uint32_t smcr = (0x7) | (1 << 7) | (1 << 16);
	/* Assume timer/channel is already correct */
	if (((PWM_TIM_NO_MASTER_SLAVE >> timer) & 1) != 0) {
		return;
	}
	/* Clear MSM and SMS in SMCR */
	*(pwm_setup.timer[timer].base + tim_smcr) &= ~smcr;
	/* TIM1/8 */
	if (((PWM_TIM_ADVANCED >> timer) & 1) != 0) {
		cr2 = (0x7 << 4) | (0xF << 20) | (1 << 25);
	}
	/* TIM2..5 */
	else if (((PWM_TIM_GP1 >> timer) & 1) != 0) {
		cr2 = (0x7 << 4) | (1 << 25);
	}
	/* TIM9/12/15 */
	else if (((PWM_TIM_GP2 >> timer) & 1) != 0) {
		cr2 = 0x7 << 4;
	}
	*(pwm_setup.timer[timer].base + tim_cr2) &= ~cr2;
}


__attribute__((unused)) static int pwm_updateEventIrq(unsigned int n, void *arg)
{
	/* Cursed pointer to int cast */
	pwm_tim_id_t timer = (pwm_tim_id_t)arg;
	volatile uint32_t *base = pwm_setup.timer[timer].base;
	pwm_irq_t *irq = &pwm_common.timer[timer].irq;
	pwm_dshot_ctx_t *dshot = &irq->dshot;

	/* If irq doesn't come from DSHOT, didsable future UEV interrupts. Clear UIE */
	if (!(irq->flags & PWM_IRQ_DSHOT_UEV)) {
		*(base + tim_dier) &= ~1u;
	}
	else {
		pwm_ch_id_t chn = dshot->chn;
		/* If the last bit was already emitted, can safely disable further UEV IRQ */
		if (dshot->bitPos > dshot->bitSize) {
			*(base + tim_dier) &= ~1u;
			irq->flags |= PWM_IRQ_DSHOT_END;
		}
		/* If last bit then preload compare value to 0 to end the sequence */
		else if (dshot->bitPos > dshot->bitSize - 1) {
			/* Also preload compare value of 0 to make sure the signal ends after the last bit */
			*(base + PWM_CCR_REG(chn)) = 0;
			dshot->bitPos++;
		}
		else {
			/* Preload next compare value on the correct channel. */
			uint8_t bit = ((dshot->data[dshot->bitPos / 8]) >> (dshot->bitPos % 8)) & 1;
			uint16_t compare = (bit == 0) ? dshot->compare0 : dshot->compare1;
			*(base + PWM_CCR_REG(chn)) = compare;
			dshot->bitPos++;
		}
	}

	/* Clear UIF in SR */
	*(base + tim_sr) &= ~1u;
	irq->flags |= PWM_IRQ_UEVRECIEVED;
	return 0;
}


__attribute__((unused)) static void pwm_printRegisters(pwm_tim_id_t timer)
{
	/* Assume correct timer */
	volatile uint32_t *base = pwm_setup.timer[timer].base;

	printf("tim_cr1: %x\n", *((base + tim_cr1)));
	printf("tim_cr2: %x\n", *(base + tim_cr2));
	printf("tim_smcr: %x\n", *(base + tim_smcr));
	printf("tim_dier: %x\n", *(base + tim_dier));
	printf("tim_sr: %x\n", *(base + tim_sr));
	printf("tim_egr: %x\n", *((base + tim_egr)));
	printf("tim_ccmr1: %x\n", *(base + tim_ccmr1));
	printf("tim_ccmr2: %x\n", *(base + tim_ccmr2));
	printf("tim_ccer: %x\n", *(base + tim_ccer));
	printf("tim_cnt: %x\n", *(base + tim_cnt));
	printf("tim_psc: %x\n", *((base + tim_psc)));
	printf("tim_arr: %x\n", *(base + tim_arr));
	printf("tim_rcr: %x\n", *((base + tim_rcr)));
	printf("tim_ccr1: %x\n", *(base + tim_ccr1));
	printf("tim_ccr2: %x\n", *(base + tim_ccr2));
	printf("tim_ccr3: %x\n", *(base + tim_ccr3));
	printf("tim_ccr4: %x\n", *(base + tim_ccr4));
	printf("tim_bdtr: %x\n", *(base + tim_bdtr));
	printf("tim_ccr5: %x\n", *(base + tim_ccr5));
	printf("tim_ccr6: %x\n", *(base + tim_ccr6));
	printf("tim_ccmr3: %x\n", *(base + tim_ccmr3));
	printf("tim_dtr2: %x\n", *(base + tim_dtr2));
	printf("tim_ecr: %x\n", *(base + tim_ecr));
	printf("tim_tisel: %x\n", *(base + tim_tisel));
	printf("tim_af1: %x\n", *(base + tim_af1));
	printf("tim_af2: %x\n", *(base + tim_af2));
	printf("tim_dcr: %x\n", *(base + tim_dcr));
	printf("tim_dmar: %x\n", *(base + tim_dmar));
}


/* Returns errors */
int pwm_disableTimer(pwm_tim_id_t timer)
{
	int res = pwm_validateTimer(timer);
	if (res < 0) {
		return res;
	}
	/* Disable all enabled channels */
	for (pwm_ch_id_t chn = 0; chn < PWM_CHN_NUM; chn++) {
		if (((pwm_common.timer[timer].channelOn >> chn) & 1) != 0) {
			pwm_disableChannel(timer, chn);
		}
	}

	/* Clear CEN in CR1 */
	*(pwm_setup.timer[timer].base + tim_cr1) &= ~1u;

	return EOK;
}

/* Returns errors */
int pwm_disableChannel(pwm_tim_id_t timer, pwm_ch_id_t chn)
{
	int res = pwm_validateTimer(timer);
	if (res < 0) {
		return res;
	}
	res = pwm_validateChannel(timer, chn);
	if (res < 0) {
		return res;
	}

	/* Disable the channel. Clear CCxE in CCER */
	*(pwm_setup.timer[timer].base + tim_ccer) &= ~(1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		*(pwm_setup.timer[timer].base + tim_ccer) &= ~(1 << PWM_CCER_CCNE_OFF(chn));
	}
	/* Notify that channel is closed */
	pwm_common.timer[timer].channelOn &= ~(1 << chn);
	return EOK;
}


uint64_t pwm_getBaseFrequency(pwm_tim_id_t timer)
{
	uint64_t baseFreq = 0ull;
	clockdef_getClock(clkid_timg, &baseFreq);
	return baseFreq;
}


__attribute__((unused)) static void pwm_apbTest(pwm_tim_id_t timer, uint16_t prescaler, uint32_t top)
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
int pwm_configure(pwm_tim_id_t timer, uint16_t prescaler, uint32_t top)
{
	/* NOTE: All registers are 4-byte aligned. Acces them via uint32*.
	 * NOTE: APB clocking mismatch with HCLK cause unreliavble reads and writes.
	 */

	uint16_t t16;
	volatile uint32_t *base;
	pwm_irq_t *irq;
	int res = pwm_validateTimer(timer);

	if (res < 0) {
		return res;
	}
	if ((((PWM_TIM_32BIT >> timer) & 1) == 0) && ((top & 0xFFFF0000) != 0)) {
		return -EINVAL;
	}
	base = pwm_setup.timer[timer].base;
	irq = &pwm_common.timer[timer].irq;
	if (pwm_common.timer[timer].devOn == 0) {
		devClk(pwm_setup.timer[timer].pctl, 1);
		pwm_common.timer[timer].devOn = 1;
	}
	if ((irq->flags & PWM_IRQ_INITIALIZED) == 0) {
		mutexCreate(&irq->uevlock);
		condCreate(&irq->uevcond);
		interrupt(pwm_setup.timer[timer].uevirq, pwm_updateEventIrq, (void *)timer, irq->uevcond, NULL);
		irq->flags |= PWM_IRQ_INITIALIZED;
	}

	/* Fail if one of the channels hasn't been disabled */
	if (pwm_common.timer[timer].channelOn != 0) {
		return -EPERM;
	}

	/* In CR1 set prescaler, countermode, autoreload, clockdivision, repetition counter */
	t16 = *((base + tim_cr1));
	dataBarier();

	/* Set counter mode UP in CR1 => clear DIR and CMS */
	if (((PWM_TIM_CNT_MODE_SELECT >> timer) & 1) != 0) {
		t16 &= ~((0x3 << 5) | (1 << 4));
	}
	/* Set clock division 1 in CR1 => clear CKD */
	t16 &= ~(0x3 << 8);
	/* Disable counter */
	t16 &= ~1u;

	*(base + tim_cr1) = t16;
	dataBarier();

	/* In AF1/2 disable break input */
	if (((PWM_TIM_BREAK1_MODE >> timer) & 1) != 0) {
		*(base + tim_af1) &= ~1u;
	}
	if (((PWM_TIM_BREAK2_MODE >> timer) & 1) != 0) {
		*(base + tim_af2) &= ~1u;
	}

	/* Set autoreload in ARR */
	pwm_common.timer[timer].arr = top;

	*(base + tim_arr) = top;

	/* Set prescaler in PSC */
	*(base + tim_psc) = prescaler;

	/* Set repetition counter to 0 in RCR */
	if (((PWM_TIM_REP_CNT >> timer) & 1) != 0) {
		*(base + tim_rcr) = 0;
	}

	dataBarier();

	/* Enable update event interrupt */
	*(base + tim_dier) |= 0x1;

	dataBarier();
	/* Generate update event (UG bit in EGR) to reload prescaler and repetition counter */
	*(base + tim_egr) |= 0x1;

	dataBarier();

	/* Enable autoreload buffering in CR1 ARPE */
	*(base + tim_cr1) |= (1 << 7);

	dataBarier();

	/* Wait for update event to load prescaler and arr */
	mutexLock(irq->uevlock);
	while (!(irq->flags & PWM_IRQ_UEVRECIEVED)) {
		condWait(irq->uevcond, irq->uevlock, 0);
	}
	irq->flags &= ~PWM_IRQ_UEVRECIEVED;
	mutexUnlock(irq->uevlock);

	return EOK;
}


/* Set output pwm channel on configured timer. Returns errors */
int pwm_set(pwm_tim_id_t timer, pwm_ch_id_t chn, uint32_t compare)
{
	volatile uint32_t *reg;
	uint32_t tmpcr2 = 0, tmpccmr = 0, tmpccer = 0, tmpbdtr = 0;
	volatile uint32_t *base;
	pwm_irq_t *irq;
	pwm_dshot_ctx_t *dshot;

	int res = pwm_validateTimer(timer);
	if (res < 0) {
		return res;
	}
	res = pwm_validateChannel(timer, chn);
	if (res < 0) {
		return res;
	}
	if ((((PWM_TIM_32BIT >> timer) & 1) == 0) && ((compare & 0xFFFF0000) != 0)) {
		return -EINVAL;
	}
	if (compare > pwm_common.timer[timer].arr) {
		return -EINVAL;
	}
	base = pwm_setup.timer[timer].base;
	irq = &pwm_common.timer[timer].irq;
	dshot = &irq->dshot;

	/* If channel is on, then only reconfigure compare value */
	if (((pwm_common.timer[timer].channelOn >> chn) & 1) != 0) {
		*(base + PWM_CCR_REG(chn)) = compare;
		return EOK;
	}

	/* Set output channel preloading in CCMRx OCyPE */
	reg = (base + PWM_CCMR_REG(chn));
	*reg |= (1 << PWM_CCMR_OCPE_OFF(chn));

	/* TODO: Find out what's the deal with channel 5,6. Why are they not in the pin table, but in the register description? */

	dataBarier();

	/* Disable the channel. Clear CCxE in CCER */
	*(base + tim_ccer) &= ~(1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		*(base + tim_ccer) &= ~(1 << PWM_CCER_CCNE_OFF(chn));
	}

	dataBarier();

	tmpcr2 = *(base + tim_cr2);
	tmpccmr = *(base + PWM_CCMR_REG(chn));
	tmpccer = *(base + tim_ccer);

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

		*(base + tim_cr2) = tmpcr2;
	}

	/* Write changes */
	*(base + PWM_CCMR_REG(chn)) = tmpccmr;
	*(base + tim_ccer) = tmpccer;
	/* Set compare value in CCRx */
	*(base + PWM_CCR_REG(chn)) = compare;

	dataBarier();

	/* Disable fast output */
	*(base + PWM_CCMR_REG(chn)) &= ~(1 << PWM_CCMR_OCFE_OFF(chn));

	pwm_disableMasterSlave(timer);

	/* Initialize BDTR register. Dead time feature is unnecessary. Disable breaks */
	if ((((PWM_TIM_BREAK1_MODE | PWM_TIM_BREAK2_MODE) >> timer) & 1) != 0) {
		tmpbdtr = 0;
		/* Enable main output (MOE) */
		tmpbdtr |= (1 << 15);
		*(base + tim_bdtr) = tmpbdtr;
	}

	dataBarier();

	/* Enable output channel (and it's complement) */
	*(base + tim_ccer) |= (1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		*(base + tim_ccer) |= (1 << PWM_CCER_CCNE_OFF(chn));
	}

	dataBarier();
	/* Enable update event interrupt */
	*(base + tim_dier) |= 0x1;

	/* Temporarily clear the DSHOT_UEV flag to run the regular UEV handler */
	uint32_t oldflags = irq->flags;
	irq->flags &= ~PWM_IRQ_DSHOT_UEV;
	dataBarier();

	/* Force an update event */
	*(base + tim_egr) |= 0x1;

	/* Wait for the update event */
	mutexLock(irq->uevlock);
	while (!(irq->flags & PWM_IRQ_UEVRECIEVED)) {
		condWait(irq->uevcond, irq->uevlock, 0);
	}
	irq->flags &= ~PWM_IRQ_UEVRECIEVED;
	mutexUnlock(irq->uevlock);

	dataBarier();
	/* Restore the old flags */
	irq->flags = oldflags;
	dataBarier();
	/* Enable subsequent UEV interrupts if in dshot mode */
	if ((irq->flags & PWM_IRQ_DSHOT_UEV) != 0) {
		*(base + tim_dier) |= 0x1;
	}

	/* Preload the second compare value if in dshot mode */
	if ((irq->flags & PWM_IRQ_DSHOT_UEV) != 0) {
		uint8_t bit = ((dshot->data[dshot->bitPos / 8]) >> (dshot->bitPos % 8)) & 1;
		compare = (bit == 0) ? dshot->compare0 : dshot->compare1;
		*(base + PWM_CCR_REG(chn)) = compare;
		dshot->bitPos++;
	}

	dataBarier();
	/* Enable counter if disabled (CR1 CER) */
	*(base + tim_cr1) |= 0x1;

	dataBarier();

	/* Mark that channel is on */
	pwm_common.timer[timer].channelOn |= (1 << chn);
	return EOK;
}


/* Returns current duty cycle percentage. Or errors */
int pwm_get(pwm_tim_id_t timer, pwm_ch_id_t chn, uint32_t *top, uint32_t *compare)
{
	int res = pwm_validateTimer(timer);
	if (res < 0) {
		return res;
	}
	res = pwm_validateChannel(timer, chn);
	if (res < 0) {
		return res;
	}
	if (((pwm_common.timer[timer].channelOn >> chn) & 1) == 0) {
		return -EPERM;
	}
	/* Load top and compare. Return compare/top */
	*top = *(pwm_setup.timer[timer].base + tim_arr);
	*compare = *(pwm_setup.timer[timer].base + PWM_CCR_REG(chn));
	return EOK;
}


int pwm_setBitSequence(pwm_tim_id_t timer, pwm_ch_id_t chn, uint32_t compare0, uint32_t compare1, uint32_t nbits, uint8_t *data)
{
	int res;
	uint16_t firstCompare;
	pwm_irq_t *irq;
	pwm_dshot_ctx_t *dshot;
	res = pwm_validateTimer(timer);
	if (res < 0) {
		return res;
	}
	res = pwm_validateChannel(timer, chn);
	if (res < 0) {
		return res;
	}
	if ((((PWM_TIM_32BIT >> timer) & 1) == 0) && (((compare0 & 0xFFFF0000) != 0) || ((compare1 & 0xFFFF0000) != 0))) {
		return -EINVAL;
	}
	if (nbits < 2) {
		return -EINVAL;
	}
	irq = &pwm_common.timer[timer].irq;
	dshot = &irq->dshot;
	*dshot = (pwm_dshot_ctx_t) {
		.chn = chn,
		.bitPos = 1, /* The first compare value is passed via pwm_set parameter */
		.bitSize = nbits,
		.compare0 = compare0,
		.compare1 = compare1,
		.data = data
	};
	irq->flags |= PWM_IRQ_DSHOT_UEV;
	irq->flags &= ~PWM_IRQ_DSHOT_END;

	dataBarier();

	firstCompare = (((*data) & 0x1) == 0) ? compare0 : compare1;
	pwm_set(timer, chn, firstCompare);

	/* Wait for the DSHOT sequence to end */
	mutexLock(irq->uevlock);
	while ((irq->flags & PWM_IRQ_DSHOT_END) == 0) {
		condWait(irq->uevcond, irq->uevlock, 0);
	}
	irq->flags &= ~PWM_IRQ_DSHOT_UEV;
	irq->flags &= ~PWM_IRQ_DSHOT_END;
	mutexUnlock(irq->uevlock);

	pwm_disableChannel(timer, chn);

	return EOK;
}


int pwm_init(void)
{
	return 0;
}
