/*
 * Phoenix-RTOS
 *
 * GRLIB ADC driver
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
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <posix/utils.h>

#if defined(__CPU_GR716)
#include <phoenix/arch/gr716.h>
#elif defined(__CPU_GR712RC)
#include <phoenix/arch/gr712rc.h>
#else
#error "Unsupported target"
#endif

#include <phoenix/arch/sparcv8leon3.h>

#include "adc.h"
#include "grlib-multi.h"

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


typedef struct {
	volatile uint32_t *vbase;

	handle_t mutex;
	handle_t irqLock;
	handle_t cond;
} adc_dev_t;


static struct {
	volatile uint32_t *base;
	uint32_t irq;
	int active;
} adc_info[] = {
	{ .active = ADC0_ACTIVE },
	{ .active = ADC1_ACTIVE },
	{ .active = ADC2_ACTIVE },
	{ .active = ADC3_ACTIVE },
	{ .active = ADC4_ACTIVE },
	{ .active = ADC5_ACTIVE },
	{ .active = ADC6_ACTIVE },
	{ .active = ADC7_ACTIVE }
};


static struct {
	adc_dev_t adc[ADC_CNT];
} adc_common;


static int adc_irqHandler(unsigned int n, void *arg)
{
	int dev = (int)arg;
	volatile uint32_t *adc_base = adc_common.adc[dev].vbase;

	(void)n;

	if ((*(adc_base + ADC_IRQ) & ADC_IRQ_CONV_END) != 0) {
		*(adc_base + ADC_IRQ) = ADC_IRQ_CONV_END;
		return 1;
	}

	return 0;
}


static void adc_convert(int dev, uint32_t *value)
{
	volatile uint32_t *adc_base = adc_common.adc[dev].vbase;

	mutexLock(adc_common.adc[dev].mutex);

	/* Start conversion */
	*(adc_base + ADC_CTRL) |= ADC_CONV_START;

	/* Wait for conversion to end */
	mutexLock(adc_common.adc[dev].irqLock);
	while ((*(adc_base + ADC_STATUS) & ADC_STS_CONV_END) == 0) {
		condWait(adc_common.adc[dev].cond, adc_common.adc[dev].irqLock, 0);
	}
	mutexUnlock(adc_common.adc[dev].irqLock);

	*value = *(adc_base + ADC_STATUS) & ADC_STS_VAL;

	mutexUnlock(adc_common.adc[dev].mutex);
}


static void adc_configure(int dev, adc_config_t *config)
{
	volatile uint32_t *adc_base = adc_common.adc[dev].vbase;
	uint32_t scaler = (SYSCLK_FREQ / (config->sampleRate * 20));

	uint32_t ctrl = (scaler << 16) | ((dev & 0xf) << 2) | (config->mode & 0x1) | ADC_ENABLE;

	mutexLock(adc_common.adc[dev].mutex);

	*(adc_base + ADC_CTRL) = ctrl;

	/* Disable pre-amp */
	*(adc_base + ADC_PRE_AMP) = ADC_PREAMP_BYPASS;

	/* Enable end of conversion interrupt */
	*(adc_base + ADC_IRQ_MSK) = ADC_IRQ_CONV_END;

	mutexUnlock(adc_common.adc[dev].mutex);
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


static int adc_cguInit(int dev)
{
#if defined(__CPU_GR716)
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_cguctrl,
		.cguctrl = {
			.state = enable,
			.cgu = cgu_secondary,
			.cgudev = cgudev_gradc0 + dev }
	};

	return platformctl(&pctl);
#else
	return 0;
#endif
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


int adc_createDevs(oid_t *oid)
{
	for (unsigned int i = 0; i < ADC_CNT; i++) {
		if (adc_info[i].active == 0) {
			continue;
		}

		char buf[8];
		if (snprintf(buf, sizeof(buf), "adc%u", i) >= sizeof(buf)) {
			return -1;
		}

		if (create_dev(oid, buf) < 0) {
			return -1;
		}
	}
	return 0;
}


int adc_init(void)
{
	adc_config_t defaultConf = {
		.sampleRate = ADC_DEFAULT_SR,
		.mode = adc_modeSingle,
		.sampleCnt = adc_sampleCnt1
	};

	for (unsigned int i = 0; i < ADC_CNT; i++) {
		if (adc_info[i].active == 0) {
			continue;
		}

		unsigned int instance = i;
		ambapp_dev_t dev = { .devId = CORE_ID_GRADCDAC };
		platformctl_t pctl = {
			.action = pctl_get,
			.type = pctl_ambapp,
			.ambapp = {
				.dev = &dev,
				.instance = &instance,
			}
		};

		if (platformctl(&pctl) < 0) {
			return -1;
		}

		if (dev.bus != BUS_AMBA_APB) {
			/* GRADCDAC should be on APB bus */
			return -1;
		}
		adc_info[i].base = dev.info.apb.base;
		adc_info[i].irq = dev.irqn;

		if (adc_cguInit(i) < 0) {
			return -1;
		}

		uintptr_t base = ((uintptr_t)adc_info[i].base & ~(_PAGE_SIZE - 1));
		adc_common.adc[i].vbase = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
		if (adc_common.adc[i].vbase == MAP_FAILED) {
			return -1;
		}

		if (condCreate(&adc_common.adc[i].cond) < 0) {
			munmap((void *)adc_common.adc[i].vbase, _PAGE_SIZE);
			return -1;
		}

		if (mutexCreate(&adc_common.adc[i].mutex) < 0) {
			munmap((void *)adc_common.adc[i].vbase, _PAGE_SIZE);
			resourceDestroy(adc_common.adc[i].cond);
			return -1;
		}

		if (mutexCreate(&adc_common.adc[i].irqLock) < 0) {
			munmap((void *)adc_common.adc[i].vbase, _PAGE_SIZE);
			resourceDestroy(adc_common.adc[i].cond);
			resourceDestroy(adc_common.adc[i].mutex);
			return -1;
		}

		adc_common.adc[i].vbase += ((uintptr_t)adc_info[i].base - base) / sizeof(uintptr_t);

		adc_configure(i, &defaultConf);

		interrupt(adc_info[i].irq, adc_irqHandler, (void *)i, adc_common.adc[i].cond, NULL);
	}
	return 0;
}
