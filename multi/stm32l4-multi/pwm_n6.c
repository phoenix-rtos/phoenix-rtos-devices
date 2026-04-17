/*
 * Phoenix-RTOS
 *
 * STM32N6 Pulse Width Modulation driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Krzysztof Radzewicz, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <stdio.h>
#include <errno.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <sys/pwman.h>
#include "pwm_n6.h"
#include "libmulti/libdma.h"

#include "common.h"

#include <unistd.h>
#include <stdlib.h>
#include <time.h>

#define USE_DSHOT_DMA 1
#define MAX_DSHOT_DMA 32

#define TIM_DIER_UIE   (1 << 0)  /* Update IRQ enable */
#define TIM_DIER_CC1IE (1 << 1)  /* Capture/compare 1 IRQ enable */
#define TIM_DIER_CC2IE (1 << 2)  /* Capture/compare 2 IRQ enable */
#define TIM_DIER_CC3IE (1 << 3)  /* Capture/compare 3 IRQ enable */
#define TIM_DIER_CC4IE (1 << 4)  /* Capture/compare 4 IRQ enable */
#define TIM_DIER_UDE   (1 << 8)  /* Update DMA request enable */
#define TIM_DIER_CC1DE (1 << 9)  /* Capture/compare 1 DMA request enable */
#define TIM_DIER_CC2DE (1 << 10) /* Capture/compare 2 DMA request enable */
#define TIM_DIER_CC3DE (1 << 11) /* Capture/compare 3 DMA request enable */
#define TIM_DIER_CC4DE (1 << 12) /* Capture/compare 4 DMA request enable */

/* Note: status registers are write 0 to clear, write 1 indifferent */

#define TIM_SR_UIF   (1 << 0)  /* Update flag */
#define TIM_SR_CC1IF (1 << 1)  /* Capture/compare 1 flag */
#define TIM_SR_CC2IF (1 << 2)  /* Capture/compare 2 flag */
#define TIM_SR_CC3IF (1 << 3)  /* Capture/compare 3 flag */
#define TIM_SR_CC4IF (1 << 4)  /* Capture/compare 4 flag */
#define TIM_SR_TIF   (1 << 6)  /* Trigger event flag */
#define TIM_SR_CC1OF (1 << 9)  /* Capture/compare 1 overcapture flag */
#define TIM_SR_CC2OF (1 << 10) /* Capture/compare 2 overcapture flag */
#define TIM_SR_CC3OF (1 << 11) /* Capture/compare 3 overcapture flag */
#define TIM_SR_CC4OF (1 << 12) /* Capture/compare 4 overcapture flag */

#define TIM_EGR_UG (1 << 0) /* Generate update event */

/* DMA capability flags - there exists a DMA request of this type for this timer */
#define DMA_CAP_CC1 (1 << 0)
#define DMA_CAP_CC2 (1 << 1)
#define DMA_CAP_CC3 (1 << 2)
#define DMA_CAP_CC4 (1 << 3)
#define DMA_CAP_UPD (1 << 4)
#define DMA_CAP_TRG (1 << 5)
#define DMA_CAP_COM (1 << 6)


/* Data used in IRQ dshot. Limited to one channel per timer. */
typedef struct {
	pwm_ch_id_t chn;  /* Channel used for pwm */
	uint32_t bitSize; /* Number of bits in sequence */
	uint32_t bitPos;  /* Current bit to process */
	void *data;       /* Array of data bits */
	uint8_t dataSize; /* Number of bytes per compare value */
} pwm_dshot_ctx_t;

/* Interrupt data for one timer */
typedef struct {
	bool flag_initialized;
	volatile uint32_t flag_uevreceived;
	volatile uint32_t flag_dshot_uev; /* IRQ used to reconfigure duty cycle */
	volatile uint32_t flag_dshot_end; /* DSHOT bit sequence complete */
	handle_t uevlock;
	handle_t uevcond;
#if USE_DSHOT_DMA == 0
	pwm_dshot_ctx_t dshot;
#endif
} pwm_irq_t;

typedef struct {
	volatile uint32_t *const base;
	const unsigned int uevirq; /* Update Event interrupt request */
	const uint32_t pctl;
	const uint8_t channels; /* If channel available (1 << channel_id) set */
	const uint8_t dmaCaps;  /* DMA capabilities */
} pwm_tim_setup_t;

typedef struct {
	pwm_irq_t irq;
	uint32_t arr;
	atomic_uint_least8_t channelOn;
	bool initialized;
#if USE_DSHOT_DMA
	handle_t dmaMutex;
	const struct libdma_per *dma_per[PWM_CHN_NUM];
	const struct libdma_per *dma_multiChannel;
	const struct libdma_per *dma_capture[PWM_CHN_NUM];
#endif
} pwm_tim_data_t;

static const struct {
	pwm_tim_setup_t timer[pwm_tim_count];
} pwm_setup = {
	.timer = {
		[pwm_tim1] = {
			.base = TIM1_BASE,
			.pctl = pctl_tim1,
			.channels = (1 << pwm_ch1) | (1 << pwm_ch1n) | (1 << pwm_ch2) | (1 << pwm_ch2n) |
					(1 << pwm_ch3) | (1 << pwm_ch3n) | (1 << pwm_ch4) | (1 << pwm_ch4n),
			.uevirq = tim1_up_irq,
			.dmaCaps = DMA_CAP_CC1 | DMA_CAP_CC2 | DMA_CAP_CC3 | DMA_CAP_CC4 | DMA_CAP_UPD | DMA_CAP_TRG | DMA_CAP_COM,
		},
		[pwm_tim2] = {
			.base = TIM2_BASE,
			.pctl = pctl_tim2,
			.channels = (1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4),
			.uevirq = tim2_irq,
			.dmaCaps = DMA_CAP_CC1 | DMA_CAP_CC2 | DMA_CAP_CC3 | DMA_CAP_CC4 | DMA_CAP_UPD | DMA_CAP_TRG,
		},
		[pwm_tim3] = {
			.base = TIM3_BASE,
			.pctl = pctl_tim3,
			.channels = (1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4),
			.uevirq = tim3_irq,
			.dmaCaps = DMA_CAP_CC1 | DMA_CAP_CC2 | DMA_CAP_CC3 | DMA_CAP_CC4 | DMA_CAP_UPD | DMA_CAP_TRG,
		},
		[pwm_tim4] = {
			.base = TIM4_BASE,
			.pctl = pctl_tim4,
			.channels = (1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4),
			.uevirq = tim4_irq,
			.dmaCaps = DMA_CAP_CC1 | DMA_CAP_CC2 | DMA_CAP_CC3 | DMA_CAP_CC4 | DMA_CAP_UPD | DMA_CAP_TRG,
		},
		[pwm_tim5] = {
			.base = TIM5_BASE,
			.pctl = pctl_tim5,
			.channels = (1 << pwm_ch1) | (1 << pwm_ch2) | (1 << pwm_ch3) | (1 << pwm_ch4),
			.uevirq = tim5_irq,
			.dmaCaps = DMA_CAP_CC1 | DMA_CAP_CC2 | DMA_CAP_CC3 | DMA_CAP_CC4 | DMA_CAP_UPD | DMA_CAP_TRG,
		},
		[pwm_tim6] = {
			.base = TIM6_BASE,
			.pctl = pctl_tim6,
			.uevirq = tim6_irq,
			.dmaCaps = DMA_CAP_UPD,
		},
		[pwm_tim7] = {
			.base = TIM7_BASE,
			.pctl = pctl_tim7,
			.uevirq = tim7_irq,
			.dmaCaps = DMA_CAP_UPD,
		},
		[pwm_tim8] = {
			.base = TIM8_BASE,
			.pctl = pctl_tim8,
			.channels = (1 << pwm_ch1) | (1 << pwm_ch1n) | (1 << pwm_ch2) | (1 << pwm_ch2n) | (1 << pwm_ch3) | (1 << pwm_ch3n) | (1 << pwm_ch4) | (1 << pwm_ch4n),
			.uevirq = tim8_up_irq,
			.dmaCaps = DMA_CAP_CC1 | DMA_CAP_CC2 | DMA_CAP_CC3 | DMA_CAP_CC4 | DMA_CAP_UPD | DMA_CAP_TRG | DMA_CAP_COM,
		},
		[pwm_tim9] = {
			.base = TIM9_BASE,
			.pctl = pctl_tim9,
			.channels = (1 << pwm_ch1) | (1 << pwm_ch2),
			.uevirq = tim9_irq,
		},
		[pwm_tim10] = {
			.base = TIM10_BASE,
			.pctl = pctl_tim10,
			.channels = (1 << pwm_ch1),
			.uevirq = tim10_irq,
		},
		[pwm_tim11] = {
			.base = TIM11_BASE,
			.pctl = pctl_tim11,
			.channels = (1 << pwm_ch1),
			.uevirq = tim11_irq,
		},
		[pwm_tim12] = {
			.base = TIM12_BASE,
			.pctl = pctl_tim12,
			.channels = (1 << pwm_ch1) | (1 << pwm_ch2),
			.uevirq = tim12_irq,
		},
		[pwm_tim13] = {
			.base = TIM13_BASE,
			.pctl = pctl_tim13,
			.channels = (1 << pwm_ch1),
			.uevirq = tim13_irq,
		},
		[pwm_tim14] = {
			.base = TIM14_BASE,
			.pctl = pctl_tim14,
			.channels = (1 << pwm_ch1),
			.uevirq = tim14_irq,
		},
		[pwm_tim15] = {
			.base = TIM15_BASE,
			.pctl = pctl_tim15,
			.channels = (1 << pwm_ch1) | (1 << pwm_ch1n) | (1 << pwm_ch2),
			.uevirq = tim15_irq,
			.dmaCaps = DMA_CAP_CC1 | DMA_CAP_CC2 | DMA_CAP_UPD | DMA_CAP_TRG | DMA_CAP_COM,
		},
		[pwm_tim16] = {
			.base = TIM16_BASE,
			.pctl = pctl_tim16,
			.channels = (1 << pwm_ch1) | (1 << pwm_ch1n),
			.uevirq = tim16_irq,
			.dmaCaps = DMA_CAP_CC1 | DMA_CAP_UPD | DMA_CAP_COM,
		},
		[pwm_tim17] = {
			.base = TIM17_BASE,
			.pctl = pctl_tim17,
			.channels = (1 << pwm_ch1) | (1 << pwm_ch1n),
			.uevirq = tim17_irq,
			.dmaCaps = DMA_CAP_CC1 | DMA_CAP_UPD | DMA_CAP_COM,
		},
		[pwm_tim18] = {
			.base = TIM18_BASE,
			.pctl = pctl_tim18,
			.uevirq = tim18_irq,
			.dmaCaps = DMA_CAP_CC1 | DMA_CAP_UPD | DMA_CAP_COM,
		} }
};

static struct {
	pwm_tim_data_t timer[pwm_tim_count];
	uint32_t baseFreq; /* Timer base frequency, 32-bit range is acceptable on this platform */
} pwm_common;


static int pwm_validateTimer(pwm_tim_id_t timer, bool checkInitialized, uint8_t checkDMAFlags)
{
	if (timer < 0 || timer >= pwm_tim_count) {
		return -EINVAL;
	}

	if (((PWM_TIM_BASIC >> timer) & 1) != 0) {
		return -EINVAL;
	}

	if (checkInitialized && !pwm_common.timer[timer].initialized) {
		return -EINVAL;
	}

	if ((pwm_setup.timer[timer].dmaCaps & checkDMAFlags) != checkDMAFlags) {
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


static inline uint32_t pwm_getUserCompareVal(void *data, uint32_t i, uint8_t datasize)
{
	/* Internal function. Assume correct arguments. */
	uint32_t compare = 0;
	switch (datasize) {
		case 1:
			compare = ((uint8_t *)data)[i];
			break;
		case 2:
			compare = ((uint16_t *)data)[i];
			break;
		case 4:
			compare = ((uint32_t *)data)[i];
			break;
	}
	return compare;
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


static int pwm_updateEventIrq(unsigned int n, void *arg)
{
	/* Cursed pointer to int cast */
	pwm_tim_id_t timer = (pwm_tim_id_t)arg;
	volatile uint32_t *base = pwm_setup.timer[timer].base;
	pwm_irq_t *irq = &pwm_common.timer[timer].irq;
#if USE_DSHOT_DMA == 0
	pwm_dshot_ctx_t *dshot = &irq->dshot;
#endif

	/* If irq isn't used to load next DSHOT bit, didsable future UEV interrupts. */
	if (!irq->flag_dshot_uev) {
		*(base + tim_dier) &= ~TIM_DIER_UIE;
	}
	else {
#if USE_DSHOT_DMA == 0
		pwm_ch_id_t chn = dshot->chn;
		/* If the last bit was already emitted, can safely disable further UEV IRQ */
		if (dshot->bitPos > dshot->bitSize) {
			*(base + tim_dier) &= ~TIM_DIER_UIE;
			irq->flag_dshot_end = 1;
		}
		/* If last bit then preload compare value to 0 to end the sequence */
		else if (dshot->bitPos > dshot->bitSize - 1) {
			*(base + PWM_CCR_REG(chn)) = 0;
			dshot->bitPos++;
		}
		else {
			/* Preload next compare value on the correct channel. */
			*(base + PWM_CCR_REG(chn)) = pwm_getUserCompareVal(dshot->data, dshot->bitPos, dshot->dataSize);
			dshot->bitPos++;
		}
#endif
	}

	*(base + tim_sr) = ~TIM_SR_UIF;
	irq->flag_uevreceived = 1;
	return 0;
}


int pwm_disableTimer(pwm_tim_id_t timer)
{
	int res = pwm_validateTimer(timer, true, 0);
	if (res < 0) {
		return res;
	}
	/* Disable all enabled channels */
	for (pwm_ch_id_t chn = 0; chn < PWM_CHN_NUM; chn++) {
		pwm_disableChannel(timer, chn);
	}

	/* Clear CEN in CR1 */
	*(pwm_setup.timer[timer].base + tim_cr1) &= ~1u;

	return EOK;
}


int pwm_disableChannel(pwm_tim_id_t timer, pwm_ch_id_t chn)
{
	int res = pwm_validateTimer(timer, true, 0);
	if (res < 0) {
		return res;
	}
	res = pwm_validateChannel(timer, chn);
	if (res < 0) {
		return res;
	}

	uint8_t prevChannelOn = atomic_fetch_and(&pwm_common.timer[timer].channelOn, ~(1 << chn));
	if (((prevChannelOn >> chn) & 1) == 0) {
		/* Already disabled */
		return 0;
	}

	/* Disable the channel. Clear CCxE in CCER */
	*(pwm_setup.timer[timer].base + tim_ccer) &= ~(1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		*(pwm_setup.timer[timer].base + tim_ccer) &= ~(1 << PWM_CCER_CCNE_OFF(chn));
	}

	return EOK;
}


uint64_t pwm_getBaseFrequency(pwm_tim_id_t timer)
{
	int res = pwm_validateTimer(timer, false, 0);
	if (res < 0) {
		return 0;
	}
	uint64_t baseFreq = 0ull;
	res = clockdef_getClock(clkid_timg, &baseFreq);
	if (res < 0) {
		return 0;
	}
	pwm_common.baseFreq = (uint32_t)baseFreq; /* Update stored base frequency */
	return baseFreq;
}

/* Generate update event (UG bit in EGR) to update prescaler, autoreload value and clear counters */
static void pwm_forceUpdateEvent(pwm_tim_id_t timer)
{
	volatile uint32_t *base = pwm_setup.timer[timer].base;
	*(base + tim_sr) = ~TIM_SR_UIF; /* Clear update event flag */
	/* Force an update event */
	*(base + tim_egr) = TIM_EGR_UG;
	while ((*(base + tim_sr) & TIM_SR_UIF) == 0) {
		/* Wait for timer to have an update event, it should be very short (~50 ns) */
	}

	*(base + tim_sr) = ~TIM_SR_UIF; /* Clear update event flag */
}


int pwm_configure(pwm_tim_id_t timer, uint16_t prescaler, uint32_t top)
{
	/* Although some of the registers are 16-bit, all of them are 4-byte aligned.
	 * Access registers via (uint32_t *). */

	uint16_t t16;
	volatile uint32_t *base;
	pwm_irq_t *irq;
	int res = pwm_validateTimer(timer, false, 0);

	if (res < 0) {
		return res;
	}
	if ((((PWM_TIM_32BIT >> timer) & 1) == 0) && ((top & 0xFFFF0000) != 0)) {
		return -EINVAL;
	}
	base = pwm_setup.timer[timer].base;
	irq = &pwm_common.timer[timer].irq;
	if (!pwm_common.timer[timer].initialized) {
		devClk(pwm_setup.timer[timer].pctl, 1);
		if (pwm_setup.timer[timer].dmaCaps != 0) {
			if (mutexCreate(&pwm_common.timer[timer].dmaMutex) < 0) {
				return -ENOMEM;
			}
		}

		pwm_common.timer[timer].initialized = true;
	}

	if (!irq->flag_initialized) {
		mutexCreate(&irq->uevlock);
		condCreate(&irq->uevcond);
		irq->flag_initialized = true;
		interrupt(pwm_setup.timer[timer].uevirq, pwm_updateEventIrq, (void *)timer, irq->uevcond, NULL);
	}

	/* Fail if one of the channels hasn't been disabled */
	if (atomic_load(&pwm_common.timer[timer].channelOn) != 0) {
		return -EPERM;
	}

	dataBarier();

	/* In CR1 set prescaler, countermode, autoreload, clockdivision, repetition counter */
	t16 = *((base + tim_cr1));

	/* Set counter mode UP in CR1 => clear DIR and CMS */
	if (((PWM_TIM_CNT_MODE_SELECT >> timer) & 1) != 0) {
		t16 &= ~((0x3 << 5) | (1 << 4));
	}
	/* Set clock division 1 in CR1 => clear CKD */
	t16 &= ~(0x3 << 8);
	/* Disable counter */
	t16 &= ~1u;

	*(base + tim_cr1) = t16;

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

	/* Enable autoreload buffering in CR1 ARPE */
	*(base + tim_cr1) |= (1 << 7);

	pwm_disableMasterSlave(timer);

	/* Initialize BDTR register. Dead time feature is unnecessary. Disable breaks */
	if ((((PWM_TIM_BREAK1_MODE | PWM_TIM_BREAK2_MODE) >> timer) & 1) != 0) {
		/* Enable main output (MOE) */
		*(base + tim_bdtr) = (1 << 15);
	}

	pwm_forceUpdateEvent(timer);

	dataBarier();

	return EOK;
}


static int pwm_setInternal(pwm_tim_id_t timer, pwm_ch_id_t chn, uint32_t compare, uint8_t activeLow)
{
	volatile uint32_t *base = pwm_setup.timer[timer].base;

	/* If channel is on, then only reconfigure compare value */
	uint8_t prevChannelOn = atomic_fetch_or(&pwm_common.timer[timer].channelOn, (1 << chn));
	if (((prevChannelOn >> chn) & 1) != 0) {
		*(base + PWM_CCR_REG(chn)) = compare;
		return EOK;
	}

	dataBarier();

	/* Disable the channel. Clear CCxE in CCER */
	uint32_t tmpccer = *(base + tim_ccer);
	tmpccer &= ~(1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		tmpccer &= ~(1 << PWM_CCER_CCNE_OFF(chn));
	}

	*(base + tim_ccer) = tmpccer;

	uint32_t tmpccmr = *(base + PWM_CCMR_REG(chn));
	/* Set channel as output. Clear CCyS */
	tmpccmr &= ~(0x3 << PWM_CCMR_CCS_OFF(chn));
	/* Set output compare mode to PWM 1 mode (0110 bits 16, 6:4)*/
	tmpccmr &= ~((1 << PWM_CCMR_OCMH_OFF(chn)) | (0x7 << PWM_CCMR_OCML_OFF(chn)));
	tmpccmr |= (0x6 << PWM_CCMR_OCML_OFF(chn));
	/* Set output channel preloading in CCMRx OCyPE */
	tmpccmr |= (1 << PWM_CCMR_OCPE_OFF(chn));
	/* Disable fast output */
	tmpccmr &= ~(1 << PWM_CCMR_OCFE_OFF(chn));
	/* Set output compare polarity */
	tmpccer &= ~(1 << PWM_CCER_CCP_OFF(chn));
	tmpccer |= (activeLow != 0) ? (1 << PWM_CCER_CCP_OFF(chn)) : 0;
	/* Enable output channel */
	tmpccer |= (1 << PWM_CCER_CCE_OFF(chn));

	// This isn't necessarily a check if complement exists, but if there is break mode
	if (pwm_channelHasComplement(timer, chn)) {
		/* Enable the complementary channel */
		tmpccer |= (1 << PWM_CCER_CCNE_OFF(chn));
		/* Set complementary output polarity (same as normal channel) */
		tmpccer &= ~(1 << PWM_CCER_CCNP_OFF(chn));
		tmpccer |= (activeLow != 0) ? (1 << PWM_CCER_CCNP_OFF(chn)) : 0;
		/* Set output idle state to low */
		uint32_t tmpcr2 = *(base + tim_cr2);
		tmpcr2 &= ~(1 << PWM_CR2_OIS_OFF(chn));
		/* Set complementary output idle state to low */
		tmpcr2 &= ~(1 << PWM_CR2_OISN_OFF(chn));
		*(base + tim_cr2) = tmpcr2;
	}

	/* Write configuration changes */
	*(base + PWM_CCMR_REG(chn)) = tmpccmr;
	/* Set compare value in CCRx */
	*(base + PWM_CCR_REG(chn)) = compare;

	pwm_forceUpdateEvent(timer);

	/* Write changes to CCER (also enables channel) */
	*(base + tim_ccer) = tmpccer;

#if USE_DSHOT_DMA == 0
	pwm_irq_t *irq = &pwm_common.timer[timer].irq;
	pwm_dshot_ctx_t *dshot = &irq->dshot;

	/* Enable subsequent UEV interrupts if in dshot mode */
	if (irq->flag_dshot_uev) {
		*(base + tim_dier) |= TIM_DIER_UIE;
	}

	/* Preload the second compare value if in dshot mode */
	if (irq->flag_dshot_uev) {
		*(base + PWM_CCR_REG(chn)) = pwm_getUserCompareVal(dshot->data, dshot->bitPos, dshot->dataSize);
		dshot->bitPos++;
	}

	dataBarier();
#endif
	/* Enable counter if disabled (CR1 CER) */
	*(base + tim_cr1) |= 0x1;

	dataBarier();
	return EOK;
}


int pwm_set(pwm_tim_id_t timer, pwm_ch_id_t chn, uint32_t compare, uint8_t activeLow)
{
	int res = pwm_validateTimer(timer, true, 0);
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
	return pwm_setInternal(timer, chn, compare, activeLow);
}


int pwm_get(pwm_tim_id_t timer, pwm_ch_id_t chn, uint32_t *top, uint32_t *compare)
{
	int res = pwm_validateTimer(timer, true, 0);
	if (res < 0) {
		return res;
	}
	res = pwm_validateChannel(timer, chn);
	if (res < 0) {
		return res;
	}

	/* If channel is off, we don't allow reading of current timer settings */
	if (((atomic_load(&pwm_common.timer[timer].channelOn) >> chn) & 1) == 0) {
		return -EPERM;
	}

	/* Load top and compare. Return compare/top */
	*top = *(pwm_setup.timer[timer].base + tim_arr);
	*compare = *(pwm_setup.timer[timer].base + PWM_CCR_REG(chn));
	return EOK;
}

#if USE_DSHOT_DMA
enum pwm_dmaAllocType {
	pwm_dmaUpd,
	pwm_dmaUpdMultiChannel,
	pwm_dmaCapture,
};

static int pwm_initDMA(pwm_tim_id_t timer, pwm_ch_id_t chn, enum pwm_dmaAllocType type)
{
	const struct libdma_per **target;
	switch (type) {
		case pwm_dmaUpd:
			target = &pwm_common.timer[timer].dma_per[chn];
			break;
		case pwm_dmaUpdMultiChannel:
			target = &pwm_common.timer[timer].dma_multiChannel;
			break;
		case pwm_dmaCapture:
			target = &pwm_common.timer[timer].dma_capture[chn];
			break;
		default:
			/* Should never happen except for programmer error */
			return -EINVAL;
	}

	/* Check again, as it may have changed since last check */
	if (*target != NULL) {
		return 0;
	}

	int per = (type == pwm_dmaCapture) ? (dma_tim_cap1 + chn) : dma_tim_upd;
	int dir = (type == pwm_dmaCapture) ? dma_per2mem : dma_mem2per;
	int res = libxpdma_acquirePeripheral(per, timer, 0, target);
	if ((res < 0) || (*target == NULL)) {
		return res;
	}

	volatile uint32_t *destination = (type == pwm_dmaUpdMultiChannel) ?
			(pwm_setup.timer[timer].base + tim_dmar) :
			(pwm_setup.timer[timer].base + PWM_CCR_REG(chn));
	res = (res < 0) ? res : libxpdma_configureChannel(*target, dir, dma_priorityVeryHigh, NULL);
	libdma_peripheral_config_t cfg = {
		.addr = (void *)destination,
		.elSize_log = (((PWM_TIM_32BIT >> timer) & 1) != 0) ? 2 : 1,
		.burstSize = 1,
		.increment = 0,
	};

	res = (res < 0) ? res : libxpdma_configurePeripheral(*target, dir, &cfg);
	return res;
}
#endif

static void pwm_fillBitSequence(uint16_t value, uint16_t hcmp, uint16_t lcmp, uint16_t seq[PWM_BITSEQ4_BITS + 2])
{
	int bit;

	for (bit = 0; bit < PWM_BITSEQ4_BITS; ++bit) {
		uint16_t mask = (uint16_t)(1u << (PWM_BITSEQ4_BITS - 1 - bit));
		seq[bit] = ((value & mask) != 0u) ? hcmp : lcmp;
	}

	/* Add two "idle" values at the end of the DMA buffer.
	 * One value is necessary because after DMA writes the last compare value, `libdma_tx` exits and user may
	 * disable the timer immediately without waiting for the final cycle to finish.
	 * Another value is added because compare values are "delayed" - a write to the CCR register after an update event
	 * will not take effect immediately, but during the next cycle.
	 * Without those two "idle" values, the last value would not be transmitted and second-to-last value
	 * would not be transmitted completely.
	 */
	seq[PWM_BITSEQ4_BITS] = 0;
	seq[PWM_BITSEQ4_BITS + 1] = 0;
}


static int pwm_callBitSequence(pwm_tim_id_t timer, pwm_ch_id_t chn[PWM_CHN_NUM], uint16_t seq[PWM_CHN_NUM][PWM_BITSEQ4_BITS + 2])
{
#if USE_DSHOT_DMA
	volatile uint32_t *base = pwm_setup.timer[timer].base;
	const struct libdma_per *per[PWM_CHN_NUM];
	volatile int done[PWM_CHN_NUM] = { 0 };
	uint8_t started = 0;
	bool udeMasked = false;
	int res = 0, i;

	mutexLock(pwm_common.timer[timer].dmaMutex);

	for (i = 0; i < PWM_CHN_NUM; ++i) {
		libdma_transfer_buffer_t buf = {
			.buf = seq[i],
			.bufSize = sizeof(seq[i]),
			.elSize_log = 1,
			.burstSize = 1,
			.increment = 1,
			.isCached = 1,
			.transform = LIBXPDMA_TRANSFORM_ALIGNR0,
		};

		per[i] = pwm_common.timer[timer].dma_per[chn[i]];
		if (per[i] == NULL) {
			res = pwm_initDMA(timer, chn[i], pwm_dmaUpd);
			per[i] = pwm_common.timer[timer].dma_per[chn[i]];
		}

		if (res < 0) {
			break;
		}

		res = libxpdma_configureMemory(per[i], dma_mem2per, 0, &buf, 1);
		if (res < 0) {
			break;
		}

		/* Start/arm the PWM channel in idle state. DMA will update CCR on timer update events. */
		res = pwm_setInternal(timer, chn[i], 0, 0);
		if (res < 0) {
			break;
		}
	}

	if (res >= 0) {
		/* Request is disabled before DMA start and enabled after to prevent a possible race condition
		 * between DMA start and timer update request. */
		*(base + tim_dier) &= ~TIM_DIER_UDE;
		udeMasked = true;

		for (i = 0; i < PWM_CHN_NUM; ++i) {
			res = libxpdma_startTransferWithFlag(per[i], dma_mem2per, &done[i]);
			if (res < 0) {
				break;
			}
			started++;
		}
	}

	if (udeMasked) {
		*(base + tim_dier) |= TIM_DIER_UDE;
		udeMasked = false;
	}

	if (res >= 0) {
		/* Wait for all transfers. Preserve the first error, but still drain all transactions. */
		res = 0;
		for (i = 0; i < PWM_CHN_NUM; ++i) {
			int waitRes = libxpdma_waitForTransaction(per[i], &done[i], NULL, 0);
			if ((res >= 0) && (waitRes < 0)) {
				res = waitRes;
			}
		}
	}
	else if (started != 0u) {
		for (i = 0; i < started; ++i) {
			(void)libxpdma_waitForTransaction(per[i], &done[i], NULL, 0);
		}
	}

	mutexUnlock(pwm_common.timer[timer].dmaMutex);
	return res;
#else
	return -ENOSYS;
#endif
}


int pwm_setBitSequence(pwm_tim_id_t timer, pwm_ch_id_t chn, void *data, uint32_t nbits, uint8_t datasize, int flags)
{
	int res;
	res = pwm_validateTimer(timer, true, DMA_CAP_UPD);
	if (res < 0) {
		return res;
	}
	res = pwm_validateChannel(timer, chn);
	if (res < 0) {
		return res;
	}
	if (data == NULL) {
		return -EINVAL;
	}
	if (datasize != 1 && datasize != 2 && datasize != 4) {
		return -EINVAL;
	}
	if ((datasize == 4) && (((PWM_TIM_32BIT >> timer) & 1) == 0)) {
		return -EINVAL;
	}
	if (nbits == 0) {
		/* Treat as no-op */
		return 0;
	}

#if USE_DSHOT_DMA
	mutexLock(pwm_common.timer[timer].dmaMutex);
	const struct libdma_per *per = pwm_common.timer[timer].dma_per[chn];
	if (per == NULL) {
		res = (res < 0) ? res : pwm_initDMA(timer, chn, pwm_dmaUpd);
		per = pwm_common.timer[timer].dma_per[chn];
	}

	static const uint8_t zero_value = 0;
	const uint8_t datasize_log = (datasize == 1) ? 0 : ((datasize == 2) ? 1 : 2);
	libdma_transfer_buffer_t bufs[2] = {
		{
			.buf = data,
			.bufSize = nbits << datasize_log,
			.elSize_log = datasize_log,
			.burstSize = 1,
			.increment = 1,
			.isCached = 1,
			.transform = LIBXPDMA_TRANSFORM_ALIGNR0,
		},
		/* Add two "idle" values at the end of the DMA buffer.
		 * One value is necessary because after DMA writes the last compare value, `libdma_tx` exits and user may
		 * disable the timer immediately without waiting for the final cycle to finish.
		 * Another value is added because compare values are "delayed" - a write to the CCR register after an update event
		 * will not take effect immediately, but during the next cycle.
		 * Without those two "idle" values, the last value would not be transmitted and second-to-last value
		 * would not be transmitted completely.
		 * TODO: this could be fixed by waiting two timer cycles after DMA finishes - but this may not be enough depending
		 * on how fast `libdma_tx` returns after DMA is finished.
		 */
		{
			.buf = (void *)&zero_value,
			.bufSize = 2,
			.elSize_log = 0,
			.burstSize = 1,
			.increment = 0,
			.isCached = 0,
			.transform = LIBXPDMA_TRANSFORM_ALIGNR0,
		},
	};
	res = (res < 0) ? res : libxpdma_configureMemory(per, dma_mem2per, 0, bufs, 2);
	/* Start the PWM channel. Set first duty cycle to 0 - idle state. It will be output until DMA takes over. */
	res = (res < 0) ? res : pwm_setInternal(timer, chn, 0, 0);
	volatile int done = 0;
	/* Request is disabled before DMA start and enabled after to prevent a possible race condition
	 * between DMA start and timer update request. */
	*(pwm_setup.timer[timer].base + tim_dier) &= ~TIM_DIER_UDE;
	res = (res < 0) ? res : libxpdma_startTransferWithFlag(per, dma_mem2per, &done);
	*(pwm_setup.timer[timer].base + tim_dier) |= TIM_DIER_UDE;
	/* TODO: we can calculate a timeout here */
	res = (res < 0) ? res : libxpdma_waitForTransaction(per, &done, NULL, 0);
	mutexUnlock(pwm_common.timer[timer].dmaMutex);
#else
	uint16_t firstCompare;
	pwm_irq_t *irq = &pwm_common.timer[timer].irq;
	pwm_dshot_ctx_t *dshot = &irq->dshot;
	*dshot = (pwm_dshot_ctx_t) {
		.chn = chn,
		.bitPos = 1, /* The first compare value is passed via pwm_set parameter */
		.bitSize = nbits,
		.data = data,
		.dataSize = datasize
	};
	irq->flag_dshot_uev = 1;
	irq->flag_dshot_end = 0;

	dataBarier();

	firstCompare = pwm_getUserCompareVal(data, 0, datasize);
	pwm_setInternal(timer, chn, firstCompare, 0);

	/* Wait for the DSHOT sequence to end */
	mutexLock(irq->uevlock);
	while (!irq->flag_dshot_end) {
		condWait(irq->uevcond, irq->uevlock, 0);
	}
	irq->flag_dshot_uev = 0;
	irq->flag_dshot_end = 0;
	mutexUnlock(irq->uevlock);
#endif

	return res;
}


int pwm_setBitSequence4(pwm_tim_id_t timer, const uint16_t chnRaw[4], const uint16_t val16[4], uint16_t hcmp, uint16_t lcmp, int flags)
{
	pwm_ch_id_t chn[PWM_CHN_NUM];
	uint16_t seq[PWM_CHN_NUM][PWM_BITSEQ4_BITS + 2];
	int res, i, j;

	(void)flags;

	res = pwm_validateTimer(timer, true, DMA_CAP_UPD);
	if (res < 0) {
		return res;
	}

	for (i = 0; i < PWM_CHN_NUM; ++i) {
		chn[i] = (pwm_ch_id_t)chnRaw[i];

		res = pwm_validateChannel(timer, chn[i]);
		if (res < 0) {
			return res;
		}

		for (j = 0; j < i; ++j) {
			if (chn[i] == chn[j]) {
				return -EINVAL;
			}

			if (PWM_CCR_REG(chn[i]) == PWM_CCR_REG(chn[j])) {
				return -EINVAL;
			}
		}

		pwm_fillBitSequence(val16[i], hcmp, lcmp, seq[i]);
	}

	return pwm_callBitSequence(timer, chn, seq);
}


int pwm_init(void)
{
#if USE_DSHOT_DMA
	libdma_init();
#endif

	/* All timers have the same base frequency on this platform */
	pwm_getBaseFrequency(pwm_tim1);
	return 0;
}
