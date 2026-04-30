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
#include <board_config.h>

#include <unistd.h>
#include <stdlib.h>
#include <time.h>

#define CACHE_LINE_ALIGN(sz) (((sz) + CACHE_LINE_SIZE - 1) & ~(CACHE_LINE_SIZE - 1))

/* Values below are meant to be compile-time configurable */

#ifndef DSHOT_RX_MARGIN_US
/* Additional time to wait for end of reception in microseconds.
 * Note that returning to userspace after condWait() will take another few microseconds,
 * so in practice this margin becomes larger than the value here. */
#define DSHOT_RX_MARGIN_US 5
#endif

#ifndef CACHE_LINE_SIZE
/* WHAT IF YOU
 * wanted to check cache line size from userspace
 * BUT ARM SAID
 * "Privileged access permitted only. Unprivileged accesses generate a fault." */
#define CACHE_LINE_SIZE 32
#endif

#ifndef DSHOT_GCR_RATIO
/* Officially, GCR bit length is 4/5 of the DShot PWM length.
 * However, on some ESC it seems to be closer to 3/4, so I wanted to make it more configurable. */
#define DSHOT_GCR_RATIO(x) (((x) * 4) / 5)
#endif

/* Values below are part of DShot protocol and are not expected to change */

#define DSHOT_TX_N_BITS     16 /* Number of bits for transmission */
#define DSHOT_SWITCHOVER_US 30 /* Maximum time to switch from transmission to reception in microseconds */
#define DSHOT_GCR_N_BITS    21 /* Total of 21 bits may be received (including leading 0) */
/* When using modified GCR encoding on 16-bit numbers, we may get up to 18 signal transitions.
 * However, when only counting eRPM responses with valid checksums, up to 16 transitions will happen. */
#define DSHOT_GCR_N_TRANSITIONS 16

#define DSHOT_RX_BUF_SIZE (DSHOT_GCR_N_TRANSITIONS * PWM_CHN_NUM * sizeof(uint16_t))
#define DSHOT_TX_BUF_SIZE (DSHOT_TX_N_BITS * PWM_CHN_NUM * sizeof(uint16_t))
/* DShot DMA buffer must be large enough to contain transmission or reception data (whichever is larger) */
#define DSHOT_BUF_SIZE CACHE_LINE_ALIGN(((DSHOT_RX_BUF_SIZE > DSHOT_TX_BUF_SIZE) ? (DSHOT_RX_BUF_SIZE) : (DSHOT_TX_BUF_SIZE)))

#define USE_DSHOT_DMA 1

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

#define DSHOT_UEV_NONE       0
#define DSHOT_UEV_IRQ_DRIVEN 1
#define DSHOT_UEV_DMA_RX_END 2


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
	atomic_uint_least8_t initialized;
	handle_t mutex;
#if USE_DSHOT_DMA
	void *dshotBuf;  /* Start of buffer allocated for DShot data */
	void *dshotData; /* Start of DShot data (must be cache line aligned) */
	const struct libdma_per *dma_per[PWM_CHN_NUM];
	const struct libdma_per *dma_multiChannel;
	const struct libdma_per *dma_capture[PWM_CHN_NUM];
	uint32_t dma_skipSetup;
#endif
} pwm_tim_data_t;

/* Idle value made available as a static const, so that it can be read by DMA */
static const uint16_t pwm_idleValue = 0;

#define SKIPSETUP_DMA_PER_SHIFT      0
#define SKIPSETUP_DMA_MULTICHN_SHIFT (SKIPSETUP_DMA_PER_SHIFT + NELEMS(((pwm_tim_data_t *)NULL)->dma_per))
#define SKIPSETUP_DMA_CAPTURE_SHIFT  (SKIPSETUP_DMA_MULTICHN_SHIFT + 1)
#define SKIPSETUP_END_SHIFT          (SKIPSETUP_DMA_CAPTURE_SHIFT + NELEMS(((pwm_tim_data_t *)NULL)->dma_capture))

#define TIMER_INIT_NONE        0
#define TIMER_INIT_IN_PROGRESS (1 << 0)
#define TIMER_INIT_DONE        (1 << 1)

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

/* Get log2(size) for sizes of 4, 2 or 1. */
static inline uint8_t sizeLog(size_t size)
{
	return (size == 4) ? 2 : ((size == 2) ? 1 : 0);
}


/* Decode timer counts into eRPM value.
 * First it turns timer counts into a stream of bits, then it undoes the modified GCR encoding to get the raw value.
 * * bitlen - expected time per bit (unit: timer tick)
 * * counts - array of timer counts taken at edges of signal
 * * n_counts - number of timer counts that were captured
 * * val_out - output of decoded binary value
 * returns:
 * * 0 on success
 * * -EILSEQ on unsuccessful decode
 * NOTE: we assume the timer won't overflow - it shouldn't happen because DMA requests are turned off upon RX timeout.
 */
static int dshot_decodeRxData(uint16_t bitlen, const uint16_t *counts, size_t nCounts, uint16_t *val_out)
{
	static const size_t MAX_BITS = DSHOT_GCR_N_BITS;
	if (nCounts < 2) {
		/* Nothing was captured */
		return -ENODATA;
	}

	if (((nCounts % 2) != 0) || (nCounts < 10)) {
		/* The last rising edge of the signal was not captured or
		 * less than minimum number of transitions was captured */
		return -EILSEQ;
	}

	uint32_t val = 1; /* Signal always starts in high state, so starting value is 1 */
	size_t totalBits = 0;
	for (size_t i = 1; i <= nCounts; i++) {
		uint32_t nBits;
		if (totalBits > MAX_BITS) {
			/* We decoded too many bits - signal glitch or clock drifted too far */
			return -EILSEQ;
		}
		else if (totalBits == MAX_BITS) {
			break;
		}
		else if (i == nCounts) {
			nBits = MAX_BITS - totalBits;
		}
		else {
			uint16_t diff = counts[i] - counts[i - 1];
			nBits = (diff + (bitlen / 2)) / bitlen; /* Divide with rounding to get the expected length */
		}

		totalBits += nBits;
		uint32_t new_bits = (~val & 1) << nBits;
		val <<= nBits;
		val |= (new_bits != 0) ? (new_bits - 1) : 0;
	}

	/* Decode modified GCR into regular GCR. Note that because bit 21 is set, bit 20 will also be set,
	 * but it's okay because we only care about bits 0..19. */
	uint32_t gcr = val ^ (val >> 1);

	static const int8_t gcrDecodeLookup[32] = {
		[0x00] = -EILSEQ,
		[0x01] = -EILSEQ,
		[0x02] = -EILSEQ,
		[0x03] = -EILSEQ,
		[0x04] = -EILSEQ,
		[0x05] = -EILSEQ,
		[0x06] = -EILSEQ,
		[0x07] = -EILSEQ,
		[0x08] = -EILSEQ,
		[0x09] = 9,
		[0x0a] = 10,
		[0x0b] = 11,
		[0x0c] = -EILSEQ,
		[0x0d] = 13,
		[0x0e] = 14,
		[0x0f] = 15,
		[0x10] = -EILSEQ,
		[0x11] = -EILSEQ,
		[0x12] = 2,
		[0x13] = 3,
		[0x14] = -EILSEQ,
		[0x15] = 5,
		[0x16] = 6,
		[0x17] = 7,
		[0x18] = -EILSEQ,
		[0x19] = 0,
		[0x1a] = 8,
		[0x1b] = 1,
		[0x1c] = -EILSEQ,
		[0x1d] = 4,
		[0x1e] = 12,
		[0x1f] = -EILSEQ,
	};

	uint16_t res = 0;
	for (int i = 3; i >= 0; i--) {
		int8_t lu = gcrDecodeLookup[(gcr >> (i * 5)) & 0x1f];
		if (lu < 0) {
			return (int)lu;
		}

		res <<= 4;
		res |= (uint8_t)lu;
	}

	*val_out = res;
	return 0;
}


static int pwm_validateTimer(pwm_tim_id_t timer, bool checkInitialized, uint8_t checkDMAFlags)
{
	if (timer < 0 || timer >= pwm_tim_count) {
		return -EINVAL;
	}

	if (((PWM_TIM_BASIC >> timer) & 1) != 0) {
		return -EINVAL;
	}

	if (checkInitialized && (atomic_load(&pwm_common.timer[timer].initialized) & TIMER_INIT_DONE) == 0) {
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
	uint32_t flag_dshot_uev = irq->flag_dshot_uev;
	if (flag_dshot_uev == DSHOT_UEV_NONE) {
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
#else
		*(base + tim_dier) &= ~(TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_CC3DE | TIM_DIER_CC4DE | TIM_DIER_UIE);
		irq->flag_dshot_uev = DSHOT_UEV_NONE;
#endif
	}

	*(base + tim_sr) = ~TIM_SR_UIF;
	irq->flag_uevreceived = 1;
	return 0;
}


static void pwm_disableChannelInternal(pwm_tim_id_t timer, pwm_ch_id_t chn)
{
	uint8_t prevChannelOn = atomic_fetch_and(&pwm_common.timer[timer].channelOn, ~(1 << chn));
	if (((prevChannelOn >> chn) & 1) == 0) {
		/* Already disabled */
		return;
	}

	/* Disable the channel. Clear CCxE in CCER */
	*(pwm_setup.timer[timer].base + tim_ccer) &= ~(1 << PWM_CCER_CCE_OFF(chn));
	if (pwm_channelHasComplement(timer, chn)) {
		*(pwm_setup.timer[timer].base + tim_ccer) &= ~(1 << PWM_CCER_CCNE_OFF(chn));
	}
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

	mutexLock(pwm_common.timer[timer].mutex);
	pwm_disableChannelInternal(timer, chn);
	mutexUnlock(pwm_common.timer[timer].mutex);
	return 0;
}


int pwm_disableTimer(pwm_tim_id_t timer)
{
	int res = pwm_validateTimer(timer, true, 0);
	if (res < 0) {
		return res;
	}

	mutexLock(pwm_common.timer[timer].mutex);
	/* Disable all enabled channels */
	for (pwm_ch_id_t chn = 0; chn < PWM_CHN_NUM; chn++) {
		pwm_disableChannelInternal(timer, chn);
	}

	/* Clear CEN in CR1 */
	*(pwm_setup.timer[timer].base + tim_cr1) &= ~1u;
	mutexUnlock(pwm_common.timer[timer].mutex);
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
	uint8_t timer_inited = atomic_fetch_or(&pwm_common.timer[timer].initialized, TIMER_INIT_IN_PROGRESS);
	if (timer_inited == TIMER_INIT_NONE) {
		devClk(pwm_setup.timer[timer].pctl, 1);
		if (mutexCreate(&pwm_common.timer[timer].mutex) < 0) {
			atomic_store(&pwm_common.timer[timer].initialized, TIMER_INIT_NONE);
			return -ENOMEM;
		}

		if (mutexCreate(&irq->uevlock) < 0) {
			resourceDestroy(pwm_common.timer[timer].mutex);
			atomic_store(&pwm_common.timer[timer].initialized, TIMER_INIT_NONE);
			return -ENOMEM;
		}

		if (condCreate(&irq->uevcond) < 0) {
			resourceDestroy(pwm_common.timer[timer].mutex);
			resourceDestroy(irq->uevlock);
			atomic_store(&pwm_common.timer[timer].initialized, TIMER_INIT_NONE);
			return -ENOMEM;
		}

		interrupt(pwm_setup.timer[timer].uevirq, pwm_updateEventIrq, (void *)timer, irq->uevcond, NULL);
		atomic_fetch_or(&pwm_common.timer[timer].initialized, TIMER_INIT_DONE);
	}
	else if ((timer_inited & TIMER_INIT_DONE) == 0) {
		return -EBUSY;
	}

	/* Fail if one of the channels hasn't been disabled */
	if (atomic_load(&pwm_common.timer[timer].channelOn) != 0) {
		return -EPERM;
	}

	mutexLock(pwm_common.timer[timer].mutex);

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

	mutexUnlock(pwm_common.timer[timer].mutex);
	return EOK;
}


static int pwm_setInternal(pwm_tim_id_t timer, pwm_ch_id_t chn, uint32_t compare, uint8_t activeLow, bool doMutex)
{
	volatile uint32_t *base = pwm_setup.timer[timer].base;

	/* If channel is on, then only reconfigure compare value */
	uint8_t prevChannelOn = atomic_fetch_or(&pwm_common.timer[timer].channelOn, (1 << chn));
	if (((prevChannelOn >> chn) & 1) != 0) {
		*(base + PWM_CCR_REG(chn)) = compare;
		return EOK;
	}

	if (doMutex) {
		mutexLock(pwm_common.timer[timer].mutex);
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
	if (irq->flag_dshot_uev == DSHOT_UEV_IRQ_DRIVEN) {
		*(base + tim_dier) |= TIM_DIER_UIE;
		/* Preload the second compare value if in dshot mode */
		*(base + PWM_CCR_REG(chn)) = pwm_getUserCompareVal(dshot->data, dshot->bitPos, dshot->dataSize);
		dshot->bitPos++;
	}

	dataBarier();
#endif
	/* Enable counter if disabled (CR1 CER) */
	*(base + tim_cr1) |= 0x1;

	dataBarier();
	if (doMutex) {
		mutexUnlock(pwm_common.timer[timer].mutex);
	}

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
	return pwm_setInternal(timer, chn, compare, activeLow, true);
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

	mutexLock(pwm_common.timer[timer].mutex);
#if USE_DSHOT_DMA
	res = (res < 0) ? res : pwm_initDMA(timer, chn, pwm_dmaUpd);

	const struct libdma_per *per = pwm_common.timer[timer].dma_per[chn];
	libdma_transfer_buffer_t bufs[2] = {
		{
			.buf = data,
			.bufSize = nbits * datasize,
			.elSize_log = sizeLog(datasize),
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
			.buf = (void *)&pwm_idleValue,
			.bufSize = 2 * sizeof(pwm_idleValue),
			.elSize_log = sizeLog(sizeof(pwm_idleValue)),
			.burstSize = 1,
			.increment = 0,
			.isCached = 0,
			.transform = LIBXPDMA_TRANSFORM_ALIGNR0,
		},
	};
	res = (res < 0) ? res : libxpdma_configureMemory(per, dma_mem2per, 0, bufs, 2);
	/* Start the PWM channel. Set first duty cycle to 0 - idle state. It will be output until DMA takes over. */
	res = (res < 0) ? res : pwm_setInternal(timer, chn, pwm_idleValue, 0, false);
	volatile int done = 0;
	/* Request is disabled before DMA start and enabled after to prevent a possible race condition
	 * between DMA start and timer update request. */
	*(pwm_setup.timer[timer].base + tim_dier) &= ~TIM_DIER_UDE;
	res = (res < 0) ? res : libxpdma_startTransferWithFlag(per, dma_mem2per, &done);
	*(pwm_setup.timer[timer].base + tim_dier) |= TIM_DIER_UDE;
	/* TODO: we can calculate a timeout here */
	res = (res < 0) ? res : libxpdma_waitForTransaction(per, &done, NULL, 0);
#else
	(void)pwm_idleValue;
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
	irq->flag_dshot_uev = DSHOT_UEV_IRQ_DRIVEN;
	irq->flag_dshot_end = 0;

	dataBarier();

	firstCompare = pwm_getUserCompareVal(data, 0, datasize);
	pwm_setInternal(timer, chn, firstCompare, 0, false);

	/* Wait for the DSHOT sequence to end */
	mutexLock(irq->uevlock);
	while (!irq->flag_dshot_end) {
		condWait(irq->uevcond, irq->uevlock, 0);
	}
	irq->flag_dshot_uev = DSHOT_UEV_NONE;
	irq->flag_dshot_end = 0;
	mutexUnlock(irq->uevlock);
#endif
	mutexUnlock(pwm_common.timer[timer].mutex);

	return res;
}


#if USE_DSHOT_DMA
/* Render data given in `val16` into an array of timer compare values.
 * * `output` - array of timer values.
 *   Array must have (`n_bits * n_channels`) elements.
 * * `n_bits` - total number of bits to render (0..16)
 * * `valIdx` - mapping from physical channel to indices into `val16`.
 *   Index of < 0 indicates the channel will be inactive.
 *   Array must have `n_channels` elements.
 * * `n_channels` - total number of channels to output (0..`PWM_CHN_NUM`)
 * * `val16` - data for channels.
 *   Array must have `max(valIdx[...])` elements.
 * * `hcmp` - timer compare value for 1 bit
 * * `lcmp` - timer compare value for 0 bit
 */
static void pwm_renderBitSequence(void *output, size_t n_bits, int *valIdx, size_t n_channels, const uint16_t *val16, uint16_t hcmp, uint16_t lcmp)
{
	if ((n_bits <= 16) && (n_channels == 4)) {
		/* If using 16 bits and 4 channels we can render data using this pseudo-vectorized code.
		 * It doesn't use MVE, but is still 3x as performant as naive code. */
		uint64_t *out64 = output;
		uint64_t dataVec = 0, lowVec = 0;
		for (int i = 0; i < 4; i++) {
			if (valIdx[i] >= 0) {
				uint64_t val = val16[valIdx[i]] ^ ((hcmp >= lcmp) ? 0 : 0xffff);
				dataVec |= val << (i * 16);
				lowVec |= (uint64_t)min(hcmp, lcmp) << (i * 16);
			}
			else {
				lowVec |= (uint64_t)(pwm_idleValue & 0xffff) << (i * 16);
			}
		}

		const uint32_t diff = (hcmp >= lcmp) ? (hcmp - lcmp) : (lcmp - hcmp);
		const uint32_t oneVec = 0x00010001UL;
		for (int i = (int)n_bits - 1; i >= 0; i--) {
			/* Split into top and bottom 32 bits - we're working on a 32-bit CPU, so 64-bit values are actually stored
			 * as pairs of registers.  We know no carry can happen when multiplying or adding and we don't care about
			 * carrying when shifting - the casts to uint32_t are there to convince the compiler of that.
			 * Eventually, this should compile into two sequences of `lsr`, `and`, `mla`, `str`. */
			uint32_t tmp_h = ((uint32_t)(dataVec >> 32) >> i) & oneVec;
			uint32_t tmp_l = ((uint32_t)(dataVec & 0xffffffffUL) >> i) & oneVec;
			tmp_h *= diff;
			tmp_l *= diff;
			tmp_h += lowVec >> 32;
			tmp_l += lowVec & 0xffffffffUL;
			out64[n_bits - 1 - i] = ((uint64_t)tmp_h << 32) | tmp_l;
		}
	}
	else {
		uint16_t *out16 = output;
		for (size_t bitIdx = 0; bitIdx < n_bits; bitIdx++) {
			uint16_t mask = (1U << (n_bits - 1 - bitIdx));
			for (size_t chnIdx = 0; chnIdx < n_channels; chnIdx++) {
				int idx = valIdx[chnIdx];
				uint16_t cmp = pwm_idleValue;
				if (idx >= 0) {
					cmp = ((val16[idx] & mask) != 0) ? hcmp : lcmp;
				}

				out16[(bitIdx * n_channels) + chnIdx] = cmp;
			}
		}
	}
}


/* Initialize DMA */
static int pwm_bitSequenceInitDMA(pwm_tim_id_t timer, uint32_t doRx)
{
	pwm_tim_data_t *t = &pwm_common.timer[timer];
	int res = pwm_initDMA(timer, 0, pwm_dmaUpdMultiChannel);
	if (res < 0) {
		return res;
	}

	if (t->dshotData == NULL) {
		/* malloc() always (or almost always) returns non-cache-line-aligned pointer here
		 * so we preemptively allocate more than necessary */
		void *mem = malloc(DSHOT_BUF_SIZE + CACHE_LINE_SIZE);
		if (mem == NULL) {
			return -ENOMEM;
		}

		t->dshotBuf = mem;
		if (((uintptr_t)mem % CACHE_LINE_SIZE) != 0) {
			mem -= ((uintptr_t)mem % CACHE_LINE_SIZE);
			mem += CACHE_LINE_SIZE;
		}

		t->dshotData = mem;
	}

	for (int chn = pwm_ch1; chn <= pwm_ch4; chn++) {
		if (((doRx >> chn) & 1) == 0) {
			continue;
		}

		res = pwm_initDMA(timer, chn, pwm_dmaCapture);
		if (res < 0) {
			return res;
		}
	}

	return 0;
}

/* Set up the memory side of the transfer and start RX transfers.
 * Because our DMA API stores the setup parameters internally and the buffers are the same every time,
 * there is no need to call libxpdma_configureMemory again unless there's been a change of parameters in the meantime.
 * Skipping this setup step saves us 2 microseconds. */
static int pwm_bitSequenceSetupDMA(pwm_tim_id_t timer, uint32_t doRx)
{
	pwm_tim_data_t *t = &pwm_common.timer[timer];
	*(pwm_setup.timer[timer].base + tim_dier) &= ~(TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_CC3DE | TIM_DIER_CC4DE);
	int res;
	if ((t->dma_skipSetup & (1 << SKIPSETUP_DMA_MULTICHN_SHIFT)) == 0) {
		/* Note: the buffers are actually in cached memory, but we don't want the DMA driver to perform cache operations,
		 * so we set buffer descriptors to "not cached". Otherwise, the DMA driver would perform cache ops 5 times,
		 * when in reality we just need 1. */
		libdma_transfer_buffer_t txBufs[2] = {
			{
				.buf = t->dshotData,
				.bufSize = DSHOT_TX_N_BITS * PWM_CHN_NUM * sizeof(uint16_t),
				.elSize_log = sizeLog(sizeof(uint16_t)),
				.burstSize = PWM_CHN_NUM, /* Use burst transaction when reading to reduce bus contention a bit */
				.increment = 1,
				.isCached = 0,
				.transform = LIBXPDMA_TRANSFORM_ALIGNR0,
			},
			/* Insert the two idle values (see above) */
			{
				.buf = (void *)&pwm_idleValue,
				.bufSize = 2 * PWM_CHN_NUM * sizeof(pwm_idleValue),
				.elSize_log = sizeLog(sizeof(pwm_idleValue)),
				.burstSize = PWM_CHN_NUM,
				.increment = 0,
				.isCached = 0,
				.transform = LIBXPDMA_TRANSFORM_ALIGNR0,
			},
		};

		res = libxpdma_configureMemory(t->dma_multiChannel, dma_mem2per, 0, txBufs, 2);
		if (res < 0) {
			return res;
		}

		t->dma_skipSetup |= (1 << SKIPSETUP_DMA_MULTICHN_SHIFT);
	}

	/* Configure DMA burst mechanism */
	*(pwm_setup.timer[timer].base + tim_dcr) =
			(1 << 16) |                /* DMA burst source: update event */
			((PWM_CHN_NUM - 1) << 8) | /* DMA burst length: all available channels */
			(tim_ccr1 << 0);           /* DMA burst start at CCR1 register */

	libdma_transfer_buffer_t rxBuf = {
		.bufSize = DSHOT_GCR_N_TRANSITIONS * sizeof(uint16_t),
		.elSize_log = sizeLog(sizeof(uint16_t)),
		.burstSize = 1,
		.increment = 1,
		.isCached = 0,
		.transform = LIBXPDMA_TRANSFORM_ALIGNR0,
	};

	for (int chn = pwm_ch1; chn <= pwm_ch4; chn++) {
		if (((doRx >> chn) & 1) == 0) {
			continue;
		}

		if ((t->dma_skipSetup & (1 << (SKIPSETUP_DMA_CAPTURE_SHIFT + chn))) == 0) {
			rxBuf.buf = t->dshotData + (DSHOT_GCR_N_TRANSITIONS * sizeof(uint16_t) * chn);
			res = libxpdma_configureMemory(t->dma_capture[chn], dma_per2mem, 0, &rxBuf, 1);
			if (res < 0) {
				return res;
			}

			t->dma_skipSetup |= (1 << (SKIPSETUP_DMA_CAPTURE_SHIFT + chn));
		}

		res = libxpdma_startTransferPollingOnly(t->dma_capture[chn], dma_per2mem);
		if (res < 0) {
			return res;
		}
	}

	return 0;
}


static void pwm_bitSequenceCleanup(pwm_tim_id_t timer, uint32_t doRx)
{
	volatile uint32_t *base = pwm_setup.timer[timer].base;
	pwm_tim_data_t *t = &pwm_common.timer[timer];
	for (int chn = pwm_ch1; chn <= pwm_ch4; chn++) {
		if (((doRx >> chn) & 1) != 0) {
			/* Channels may have been cancelled before, but there's no harm doing it again */
			libxpdma_cancelTransfer(t->dma_capture[chn], dma_per2mem);
		}
	}

	/* If any channels were receiving, flag that they need to be re-configured by the next pwm_setInternal call */
	atomic_fetch_and(&t->channelOn, ~doRx);
	*(base + tim_dier) &= ~(TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_CC3DE | TIM_DIER_CC4DE);
	mutexUnlock(t->mutex);
}


typedef struct
{
	pwm_tim_id_t timer;
	volatile int done; /* TX complete flag */
	uint32_t dier;     /* Bits to set in DIER register to enable reception. If == 0, reception disabled. */
	struct {
		uint32_t ccerOff; /* CCER value which switches selected channels off */
		uint32_t ccmr[2]; /* CCMRx values for receiving */
		uint32_t ccer;    /* CCER value which enables receiving on selected channels */
		uint32_t srClear; /* Bits to clear in SR register before reception can start */
		uint32_t arrRx;   /* ARR value for RX timeout */
		uint32_t arrPrev; /* ARR value to restore after RX timeout */
	} regs;               /* Pre-rendered register values. May be uninitialized when TX only. */
} pwm_bitSequenceSwitchArg_t;


/* Render timer configuration data that will be written to registers during ISR */
static int pwm_bitSequenceTimerReconfig(pwm_tim_id_t timer, uint32_t doRx, uint32_t gcrBitTime, pwm_bitSequenceSwitchArg_t *arg)
{
	volatile uint32_t *base = pwm_setup.timer[timer].base;
	const uint32_t timerMax = (((PWM_TIM_32BIT >> timer) & 1) != 0) ? 0xFFFFFFFF : 0xFFFF;
	/* Calculate RX timeout. We will always wait at least this time, so it should be more or less optimal. */
	uint32_t timerFreq = pwm_common.baseFreq / (*(base + tim_psc) + 1);
	arg->regs.arrRx = (((DSHOT_SWITCHOVER_US + DSHOT_RX_MARGIN_US) * (timerFreq / 1000000)) + (DSHOT_GCR_N_BITS * gcrBitTime)) - 1;
	if (arg->regs.arrRx > timerMax) {
		return -EINVAL;
	}

	arg->regs.arrPrev = pwm_common.timer[timer].arr;
	arg->regs.srClear = 0;
	arg->regs.ccerOff = *(base + tim_ccer);
	arg->regs.ccer = arg->regs.ccerOff;
	arg->regs.ccmr[0] = *(base + tim_ccmr1);
	arg->regs.ccmr[1] = *(base + tim_ccmr2);
	for (int chn = pwm_ch1; chn <= pwm_ch4; chn++) {
		if (((doRx >> chn) & 1) == 0) {
			continue;
		}

		arg->dier |= TIM_DIER_CC1DE << chn;
		arg->regs.srClear |= TIM_SR_CC1IF << chn;
		/* Disable output, set input sensitivity to both edges */
		arg->regs.ccerOff &= ~((1 << PWM_CCER_CCE_OFF(chn)) | (1 << PWM_CCER_CCNE_OFF(chn)));
		arg->regs.ccerOff |= (1 << PWM_CCER_CCP_OFF(chn)) | (1 << PWM_CCER_CCNP_OFF(chn));

		uint32_t v = arg->regs.ccmr[PWM_CCMR_REG(chn) - tim_ccmr1];
		/* Switch mode to input from tim_tiX where X == channel number */
		v &= ~(0x3 << PWM_CCMR_CCS_OFF(chn));
		v |= (0x1 << PWM_CCMR_CCS_OFF(chn));
		/* Capture every event */
		v &= ~(0x3 << PWM_CCMR_ICPSC_OFF(chn));
		/* Set digital filter to 5 samples @ f(DTS)/32 frequency -> reject events shorter than 0.4 us */
		v &= ~(0xf << PWM_CCMR_ICF_OFF(chn));
		v |= (13 << PWM_CCMR_ICF_OFF(chn));
		arg->regs.ccmr[PWM_CCMR_REG(chn) - tim_ccmr1] = v;

		/* Enable input */
		arg->regs.ccer &= ~(1 << PWM_CCER_CCNE_OFF(chn));
		arg->regs.ccer |= (1 << PWM_CCER_CCP_OFF(chn)) | (1 << PWM_CCER_CCNP_OFF(chn)) | (1 << PWM_CCER_CCE_OFF(chn));
	}

	return 0;
}


/* Signal end of TX DMA and reconfigure timer for receiving if RX was requested */
static void pwm_bitSequenceSwitchISR(void *cbArg, int type)
{
	(void)type;
	pwm_bitSequenceSwitchArg_t *arg = cbArg;
	volatile uint32_t *base = pwm_setup.timer[arg->timer].base;

	if (arg->dier != 0) {
		/* Switch timer parameters */
		*(base + tim_ccer) = arg->regs.ccerOff;
		*(base + tim_ccmr1) = arg->regs.ccmr[0];
		*(base + tim_ccmr2) = arg->regs.ccmr[1];
		*(base + tim_ccer) = arg->regs.ccer;
		*(base + tim_arr) = arg->regs.arrRx;
		/* Clear update event flags */
		*(base + tim_sr) = ~TIM_SR_UIF;
		*(base + tim_egr) = TIM_EGR_UG; /* Force an update event */
		while ((*(base + tim_sr) & TIM_SR_UIF) == 0) {
			/* Wait for timer to have an update event, it's very short (~50 ns) */
		}

		*(base + tim_sr) = ~(arg->regs.srClear | TIM_SR_UIF); /* Clear update event flag */
		*(base + tim_dier) |= TIM_DIER_UIE | arg->dier;
		/* Preload the previously set autoreload value, i.e. cleanup */
		*(base + tim_arr) = arg->regs.arrPrev;
	}

	arg->done = 1;
}


static void pwm_bitSequenceFinalizeRx(pwm_tim_id_t timer, uint32_t gcrBitTime, uint32_t doRx, int valIdx[PWM_CHN_NUM], int rxOutput[4])
{
	pwm_tim_data_t *t = &pwm_common.timer[timer];
	int res;

	int rxLen[PWM_CHN_NUM];
	for (int chn = pwm_ch1; chn <= pwm_ch4; chn++) {
		if (((doRx >> chn) & 1) == 0) {
			rxLen[chn] = -ENODATA;
			continue;
		}

		res = libxpdma_bufferRemaining(t->dma_capture[chn], dma_per2mem);
		if (res < 0) {
			rxLen[chn] = res;
		}
		else if (res > (DSHOT_GCR_N_TRANSITIONS * sizeof(uint16_t))) {
			rxLen[chn] = -EIO;
		}
		else {
			rxLen[chn] = DSHOT_GCR_N_TRANSITIONS - (res / sizeof(uint16_t));
		}

		libxpdma_cancelTransfer(t->dma_capture[chn], dma_per2mem);
	}

	for (int chn = pwm_ch1; chn <= pwm_ch4; chn++) {
		int outIdx = valIdx[chn];
		if (outIdx < 0) {
			continue;
		}

		if (rxLen[chn] < 0) {
			rxOutput[outIdx] = rxLen[chn];
			continue;
		}

		uint16_t *counts = t->dshotData + (DSHOT_GCR_N_TRANSITIONS * sizeof(uint16_t) * chn);
		uint16_t val;
		res = dshot_decodeRxData(gcrBitTime, counts, rxLen[chn], &val);
		rxOutput[outIdx] = (res < 0) ? res : (int)val;
	}
}


int pwm_setBitSequence4(pwm_tim_id_t timer, const uint16_t chnRaw[4], const uint16_t val16[4], uint16_t hcmp, uint16_t lcmp, int flags, int rxOutput[4])
{
	(void)flags;
	int res;
	/* Mapping from physical channels (pwm_ch1 .. pwm_ch4) to indices in val16 array */
	int valIdx[PWM_CHN_NUM] = { -1, -1, -1, -1 };
	bool actLow[PWM_CHN_NUM] = { false, false, false, false };
	uint32_t doRx = 0;
	uint8_t reqDmaCaps = DMA_CAP_UPD;
	/* TODO: it would be nice to support timers with < 4 channels */
	reqDmaCaps |= DMA_CAP_CC1 | DMA_CAP_CC2 | DMA_CAP_CC3 | DMA_CAP_CC4;

	res = pwm_validateTimer(timer, true, reqDmaCaps);
	if (res < 0) {
		return res;
	}

	volatile uint32_t *base = pwm_setup.timer[timer].base;
	pwm_bitSequenceSwitchArg_t arg = {
		.timer = timer,
		.done = 0,
		.dier = 0,
	};

	pwm_tim_data_t *t = &pwm_common.timer[timer];
	for (int i = 0; i < 4; i++) {
		if ((chnRaw[i] & PWM_BITSEQ4_CHN_INVALID) != 0) {
			continue;
		}

		pwm_ch_id_t chn = (pwm_ch_id_t)(chnRaw[i] & PWM_BITSEQ4_CHN_MASK);
		res = pwm_validateChannel(timer, chn);
		if (res < 0) {
			return res;
		}

		if (valIdx[chn] != -1) {
			/* One of the channels was given twice */
			return -EINVAL;
		}

		valIdx[chn] = i;
		actLow[chn] = (chnRaw[i] & PWM_BITSEQ4_CHN_ACT_LOW) != 0;
		doRx |= ((chnRaw[i] & PWM_BITSEQ4_CHN_DO_RX) != 0) ? (1 << chn) : 0;
	}

	if ((doRx != 0) && (rxOutput == NULL)) {
		return -EINVAL;
	}

	if (pwm_common.baseFreq == 0) {
		return -EIO;
	}

	mutexLock(t->mutex);
	const uint32_t gcrBitTime = DSHOT_GCR_RATIO(t->arr);
	if (doRx != 0) {
		res = pwm_bitSequenceTimerReconfig(timer, doRx, gcrBitTime, &arg);
		if (res < 0) {
			pwm_bitSequenceCleanup(timer, doRx);
			return res;
		}

		t->irq.flag_dshot_uev = DSHOT_UEV_DMA_RX_END;
	}

	res = pwm_bitSequenceInitDMA(timer, doRx);
	if (res < 0) {
		pwm_bitSequenceCleanup(timer, doRx);
		return res;
	}

	pwm_renderBitSequence(t->dshotData, DSHOT_TX_N_BITS, valIdx, PWM_CHN_NUM, val16, hcmp, lcmp);

	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_cleanInvalDCache;
	pctl.opDCache.addr = t->dshotData;
	pctl.opDCache.sz = DSHOT_BUF_SIZE;
	res = platformctl(&pctl);
	if (res < 0) {
		pwm_bitSequenceCleanup(timer, doRx);
		return res;
	}

	res = pwm_bitSequenceSetupDMA(timer, doRx);
	if (res < 0) {
		pwm_bitSequenceCleanup(timer, doRx);
		return res;
	}

	/* Start PWM channels. Set first duty cycle to 0 - idle state. It will be output until DMA takes over. */
	for (int i = pwm_ch1; i <= pwm_ch4; i++) {
		if (valIdx[i] < 0) {
			continue;
		}

		res = pwm_setInternal(timer, i, pwm_idleValue, actLow[i], false);
		if (res < 0) {
			pwm_bitSequenceCleanup(timer, doRx);
			return res;
		}
	}

	res = libxpdma_startTransferWithCallback(t->dma_multiChannel, dma_mem2per, dma_tc, pwm_bitSequenceSwitchISR, &arg);
	if (res < 0) {
		pwm_bitSequenceCleanup(timer, doRx);
		return res;
	}

	/* Start TX DMA */
	*(base + tim_dier) |= TIM_DIER_UDE;
	/* If only transmitting, wait for DMA to end and exit.
	 * Otherwise, don't wait for DMA, only for RX finished - this way we can yield the CPU for longer. */
	if (doRx == 0) {
		/* Calculate TX timeout as 2x max transmission time (timeout should never be hit) */
		const time_t timeoutTx = (2 * DSHOT_TX_N_BITS * t->arr) / pwm_common.baseFreq;
		res = libxpdma_waitForTransaction(t->dma_multiChannel, &arg.done, NULL, timeoutTx);
		pwm_bitSequenceCleanup(timer, doRx);
		return res;
	}
	else {
		pwm_irq_t *irq = &pwm_common.timer[timer].irq;
		/* Wait for update event that signals end of RX */
		mutexLock(irq->uevlock);
		while (!irq->flag_uevreceived) {
			condWait(irq->uevcond, irq->uevlock, 0);
		}

		irq->flag_uevreceived = 0;
		mutexUnlock(irq->uevlock);

		/* If TX DMA hasn't finished yet, it's way past timeout - cleanup and bail out */
		if (arg.done == 0) {
			libxpdma_cancelTransfer(t->dma_multiChannel, dma_mem2per);
			pwm_bitSequenceCleanup(timer, doRx);
			return -ETIME;
		}
	}

	pwm_bitSequenceFinalizeRx(timer, gcrBitTime, doRx, valIdx, rxOutput);
	pwm_bitSequenceCleanup(timer, doRx);
	return 0;
}
#else
int pwm_setBitSequence4(pwm_tim_id_t timer, const uint16_t chn[PWM_CHN_NUM], const uint16_t val16[PWM_CHN_NUM], uint16_t hcmp, uint16_t lcmp, int flags, int rxOutput[PWM_CHN_NUM])
{
	(void)dshot_decodeRxData;
	return -ENOSYS;
}
#endif


int pwm_init(void)
{
#if USE_DSHOT_DMA
	libdma_init();
#endif

	/* All timers have the same base frequency on this platform */
	pwm_getBaseFrequency(pwm_tim1);
	return 0;
}
