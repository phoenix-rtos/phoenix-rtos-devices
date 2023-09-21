/*
 * Phoenix-RTOS
 *
 * GR716 ADC driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <board_config.h>
#include <errno.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <phoenix/arch/gr716.h>

#include "adc.h"
#include "common.h"
#include "gr716-multi.h"


#define ADC_CNT 8

#define ADC0_BASE ((void *)0x80400000)
#define ADC1_BASE ((void *)0x80401000)
#define ADC2_BASE ((void *)0x80402000)
#define ADC3_BASE ((void *)0x80403000)
#define ADC4_BASE ((void *)0x80404000)
#define ADC5_BASE ((void *)0x80405000)
#define ADC6_BASE ((void *)0x80406000)
#define ADC7_BASE ((void *)0x80407000)

#define ADC0_IRQ 28
#define ADC1_IRQ 29
#define ADC2_IRQ 30
#define ADC3_IRQ 31
#define ADC4_IRQ 28
#define ADC5_IRQ 29
#define ADC6_IRQ 30
#define ADC7_IRQ 31

#define ADC_DEFAULT_SR (200 * 1000) /* 200 kSps */

/* ADC registers */
#define ADC_CTRL      0  /* Control register : 0x00 */
#define ADC_SAMP_CTRL 1  /* Sampling control register : 0x04 */
#define ADC_SEQ_CTRL  2  /* Sequence control register : 0x08 */
#define ADC_SEQ_SYNC  3  /* Sequence sync register : 0x0C */
#define ADC_STATUS    4  /* Status register : 0x10 */
#define ADC_IRQ       5  /* Interrupt register : 0x14 */
#define ADC_IRQ_MSK   6  /* Interrupt mask register : 0x18 */
#define ADC_PRE_AMP   7  /* Pre-amplifier control register : 0x1C */
#define ADC_HI_LVL    8  /* High level detection register : 0x20 */
#define ADC_LO_LVL    9  /* Low level detection register : 0x24 */
#define ADC_SEQ_STS   10 /* Sequence status register : 0x2C - 0x38 */

/* Control register */
#define ADC_CONV_START (1 << 14)
#define ADC_ENABLE     (1 << 15)

/* Interrupt register */
#define ADC_IRQ_CONV_END (1 << 0)

/* Status register */
#define ADC_STS_CONV_END (1 << 31)
#define ADC_STS_VAL      0x7ffffu

/* Pre-amp control register */
#define ADC_PREAMP_BYPASS (1 << 2)


static const struct {
	volatile uint32_t *base;
	uint32_t irq;
	int active;
} adc_info[ADC_CNT] = {
	{ ADC0_BASE, ADC0_IRQ, 1 },
	{ ADC1_BASE, ADC1_IRQ, 0 },
	{ ADC2_BASE, ADC2_IRQ, 0 },
	{ ADC3_BASE, ADC3_IRQ, 0 },
	{ ADC4_BASE, ADC4_IRQ, 0 },
	{ ADC5_BASE, ADC5_IRQ, 0 },
	{ ADC6_BASE, ADC6_IRQ, 0 },
	{ ADC7_BASE, ADC7_IRQ, 0 }
};


static struct {
	handle_t mutex;
	handle_t irqLock;
	handle_t cond;
	handle_t inth;
} adc_common[ADC_CNT];


static int adc_irqHandler(unsigned int n, void *arg)
{
	int adc = (int)arg;
	volatile uint32_t *adc_base = adc_info[adc].base;

	(void)n;

	if ((*(adc_base + ADC_IRQ) & ADC_IRQ_CONV_END) != 0) {
		*(adc_base + ADC_IRQ) = ADC_IRQ_CONV_END;
		return 1;
	}

	return 0;
}


static void adc_convert(int dev, uint32_t *value)
{
	volatile uint32_t *adc_base = adc_info[dev].base;

	mutexLock(adc_common[dev].mutex);

	/* Start conversion */
	common_atomicOr(adc_base + ADC_CTRL, ADC_CONV_START);

	/* Wait for conversion to end */
	while ((*(adc_base + ADC_STATUS) & ADC_STS_CONV_END) == 0) {
		condWait(adc_common[dev].cond, adc_common[dev].mutex, 0);
	}

	*value = *(adc_base + ADC_STATUS) & ADC_STS_VAL;

	mutexUnlock(adc_common[dev].mutex);
}


static void adc_configure(int dev, adc_config_t *config)
{
	volatile uint32_t *adc_base = adc_info[dev].base;
	uint32_t scaler = (SYSCLK_FREQ / (config->sampleRate * 20));

	uint32_t ctrl = (scaler << 16) | ((dev & 0xf) << 2) | (config->mode & 0x1) | ADC_ENABLE;

	mutexLock(adc_common[dev].mutex);

	*(adc_base + ADC_CTRL) = ctrl;

	/* Disable pre-amp */
	*(adc_base + ADC_PRE_AMP) = ADC_PREAMP_BYPASS;

	/* Enable end of conversion interrupt */
	*(adc_base + ADC_IRQ_MSK) = ADC_IRQ_CONV_END;

	mutexUnlock(adc_common[dev].mutex);
}


static void adc_handleDevCtl(msg_t *msg, int dev)
{
	multi_i_t *idevctl = (multi_i_t *)msg->i.raw;
	multi_o_t *odevctl = (multi_o_t *)msg->o.raw;

	switch (idevctl->adc.type) {
		case adc_config:
			adc_configure(dev, &idevctl->adc.config);
			odevctl->err = EOK;
			break;

		default:
			odevctl->err = -EINVAL;
			break;
	}
}


void adc_handleMsg(msg_t *msg, int dev)
{
	dev -= id_adc0;

	if ((dev < 0) || (dev >= ADC_CNT) || (adc_info[dev].active == 0)) {
		if (msg->type == mtDevCtl) {
			((multi_o_t *)msg->o.raw)->err = -EINVAL;
		}
		else {
			msg->o.io.err = -EINVAL;
		}
		return;
	}

	switch (msg->type) {
		case mtOpen:
		case mtClose:
		case mtWrite:
			msg->o.io.err = EOK;
			break;

		case mtRead:
			if (msg->o.size != sizeof(uint32_t)) {
				msg->o.io.err = -EINVAL;
				break;
			}
			adc_convert(dev, (uint32_t *)msg->o.data);
			msg->o.io.err = EOK;
			break;

		case mtDevCtl:
			adc_handleDevCtl(msg, dev);
			break;

		default:
			msg->o.io.err = -EINVAL;
			break;
	}
}


int adc_init(void)
{
	int res = 0;
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_cguctrl,
		.cguctrl = {
			.state = enable,
			.cgu = cgu_secondary,
		}
	};

	adc_config_t defaultConf = {
		.sampleRate = ADC_DEFAULT_SR,
		.mode = adc_mode_single,
		.sampleCnt = adc_sampleCnt_1
	};

	for (int i = 0; i < ADC_CNT; i++) {
		if (adc_info[i].active == 0) {
			continue;
		}

		/* Enable clock */
		pctl.cguctrl.cgudev = cgudev_gradc0 + i;
		res = platformctl(&pctl);
		if (res < 0) {
			return res;
		}

		res = condCreate(&adc_common[i].cond);
		if (res < 0) {
			return -ENOENT;
		}

		res = mutexCreate(&adc_common[i].mutex);
		if (res < 0) {
			resourceDestroy(adc_common[i].cond);
			return -ENOENT;
		}

		res = mutexCreate(&adc_common[i].irqLock);
		if (res < 0) {
			resourceDestroy(adc_common[i].cond);
			resourceDestroy(adc_common[i].mutex);
			return -ENOENT;
		}

		adc_configure(i, &defaultConf);

		interrupt(adc_info[i].irq, adc_irqHandler, (void *)i, adc_common[i].cond, &adc_common[i].inth);
	}
	return 0;
}
