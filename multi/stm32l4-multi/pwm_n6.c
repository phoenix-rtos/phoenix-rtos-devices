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
} pwm_tim_setup_t;

typedef struct {
	pwm_irq_t irq;
	uint32_t arr;
	uint8_t channelOn;
	uint8_t devOn;
#if USE_DSHOT_DMA
	const struct libdma_per *dma_per[PWM_CHN_NUM];
#endif
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
			.uevirq = tim5_irq,
			.pctl = pctl_tim5 },
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
#if USE_DSHOT_DMA
	uint32_t dmaTransferBuffer[MAX_DSHOT_DMA + 3];
	handle_t dmalock;
#endif
} pwm_common;


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
		*(base + tim_dier) &= ~1u;
	}
	else {
#if USE_DSHOT_DMA == 0
		pwm_ch_id_t chn = dshot->chn;
		/* If the last bit was already emitted, can safely disable further UEV IRQ */
		if (dshot->bitPos > dshot->bitSize) {
			*(base + tim_dier) &= ~1u;
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

	/* Clear UIF in SR */
	*(base + tim_sr) &= ~1u;
	irq->flag_uevreceived = 1;
	return 0;
}


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
	/* Remember that channel is closed */
	pwm_common.timer[timer].channelOn &= ~(1 << chn);
	return EOK;
}


uint64_t pwm_getBaseFrequency(pwm_tim_id_t timer)
{
	int res = pwm_validateTimer(timer);
	if (res < 0) {
		return 0;
	}
	uint64_t baseFreq = 0ull;
	res = clockdef_getClock(clkid_timg, &baseFreq);
	if (res < 0) {
		return 0;
	}
	return baseFreq;
}


int pwm_configure(pwm_tim_id_t timer, uint16_t prescaler, uint32_t top)
{
	/* Although some of the registers are 16-bit, all of them are 4-byte aligned.
	 * Access registers via (uint32_t *). */

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
	if (!irq->flag_initialized) {
		mutexCreate(&irq->uevlock);
		condCreate(&irq->uevcond);
		irq->flag_initialized = true;
		interrupt(pwm_setup.timer[timer].uevirq, pwm_updateEventIrq, (void *)timer, irq->uevcond, NULL);
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
	while (!irq->flag_uevreceived) {
		condWait(irq->uevcond, irq->uevlock, 0);
	}
	irq->flag_uevreceived = 0;
	mutexUnlock(irq->uevlock);

	return EOK;
}


int pwm_set(pwm_tim_id_t timer, pwm_ch_id_t chn, uint32_t compare)
{
	volatile uint32_t *reg;
	uint32_t tmpcr2 = 0, tmpccmr = 0, tmpccer = 0, tmpbdtr = 0;
	volatile uint32_t *base;
	pwm_irq_t *irq;
#if USE_DSHOT_DMA == 0
	pwm_dshot_ctx_t *dshot;
#endif

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
#if USE_DSHOT_DMA == 0
	dshot = &irq->dshot;
#endif

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
	uint32_t oldflag = irq->flag_dshot_uev;
	irq->flag_dshot_uev = 0;
	dataBarier();

	/* Force an update event */
	*(base + tim_egr) |= 0x1;

	/* Wait for the update event */
	mutexLock(irq->uevlock);
	while (!irq->flag_uevreceived) {
		condWait(irq->uevcond, irq->uevlock, 0);
	}
	irq->flag_uevreceived = 0;
	mutexUnlock(irq->uevlock);

	dataBarier();
	/* Restore the old flags */
	irq->flag_dshot_uev = oldflag;
	dataBarier();

#if USE_DSHOT_DMA == 0
	/* Enable subsequent UEV interrupts if in dshot mode */
	if (irq->flag_dshot_uev) {
		*(base + tim_dier) |= 0x1;
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

	/* Mark that channel is on */
	pwm_common.timer[timer].channelOn |= (1 << chn);
	return EOK;
}


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

int pwm_setBitSequence(pwm_tim_id_t timer, pwm_ch_id_t chn, void *data, uint32_t nbits, uint8_t datasize, int flags)
{
	int res;
	res = pwm_validateTimer(timer);
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
		return EOK;
	}

#if USE_DSHOT_DMA
	/* Limit number of bits so we don't overflow the buffer */
	if (nbits > MAX_DSHOT_DMA) {
		return -EINVAL;
	}

	mutexLock(pwm_common.dmalock);

	/* TODO: adding 0 value in the front as a workaround to the rare issue
	 * of the first value not being emitted. */
	pwm_common.dmaTransferBuffer[0] = 0;
	for (unsigned int i = 0; i < nbits; i++) {
		pwm_common.dmaTransferBuffer[i + 1] = pwm_getUserCompareVal(data, i, datasize);
	}

	if (pwm_common.timer[timer].dma_per[chn] == NULL) {
		int ret = libdma_acquirePeripheral(dma_tim_upd, timer, &(pwm_common.timer[timer].dma_per[chn]));
		if ((ret < 0) || (pwm_common.timer[timer].dma_per[chn] == NULL)) {
			mutexUnlock(pwm_common.dmalock);
			return ret;
		}

		int pSize = (((PWM_TIM_32BIT >> timer) & 1) != 0) ? 2 : 1;
		/* You're supposed to set control DMA-TIM transfers via TIM_DCR and TIM_DMAR registers
		 * but that failed in testing, so we just give DMA the destination address directly. */
		volatile uint32_t *destination = pwm_setup.timer[timer].base + PWM_CCR_REG(chn);
		ret = libdma_configurePeripheral(
				pwm_common.timer[timer].dma_per[chn],
				dma_mem2per,
				dma_priorityVeryHigh,
				(void *)(destination),
				2,
				pSize,
				1,
				0,
				NULL);

		if (ret < 0) {
			mutexUnlock(pwm_common.dmalock);
			return ret;
		}

		/* Enable TIM DMA request */
		*(pwm_setup.timer[timer].base + tim_dier) |= (1 << 8);
	}

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
	pwm_common.dmaTransferBuffer[nbits + 1] = 0;
	pwm_common.dmaTransferBuffer[nbits + 2] = 0;

	/* Start the PWM channel. Set first duty cycle to 0 - idle state. It will be output until DMA takes over. */
	pwm_set(timer, chn, 0);
	/* TODO: we can calculate a timeout here */
	libdma_tx(pwm_common.timer[timer].dma_per[chn], pwm_common.dmaTransferBuffer, (nbits + 3) * 4, 0, 0);
	/* TODO: we may need to wait for the last cycle to finish */
	mutexUnlock(pwm_common.dmalock);
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
	pwm_set(timer, chn, firstCompare);

	/* Wait for the DSHOT sequence to end */
	mutexLock(irq->uevlock);
	while (!irq->flag_dshot_end) {
		condWait(irq->uevcond, irq->uevlock, 0);
	}
	irq->flag_dshot_uev = 0;
	irq->flag_dshot_end = 0;
	mutexUnlock(irq->uevlock);
#endif

	return EOK;
}


int pwm_init(void)
{
#if USE_DSHOT_DMA
	mutexCreate(&pwm_common.dmalock);
	libdma_init();
#endif
	return 0;
}
