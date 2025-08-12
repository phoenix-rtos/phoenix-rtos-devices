/*
 * Phoenix-RTOS
 *
 * STM32N6 external interrupts driver
 *
 * Copyright 2019, 2020, 2025 Phoenix Systems
 * Author: Daniel Sawka, Aleksander Kaminski, Jacek Maksymowicz
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include "stm32l4-multi.h"
#include "common.h"

#include "exti.h"


struct {
	volatile unsigned int *base;
	handle_t lock;
} exti_common;


#define EXTI_LINES 78
#define N_EXTI     16
#define MAX_GPIO   gpioq

static const int8_t gpio_to_mux_setting[] = {
	[gpioa] = 0x00,
	[gpiob] = 0x01,
	[gpioc] = 0x02,
	[gpiod] = 0x03,
	[gpioe] = 0x04,
	[gpiof] = 0x05,
	[gpiog] = 0x06,
	[gpioh] = 0x07,
	[gpioi] = -1,
	[gpioj] = -1,
	[gpiok] = -1,
	[gpiol] = -1,
	[gpiom] = -1,
	[gpion] = 0x08,
	[gpioo] = 0x09,
	[gpiop] = 0x0a,
	[gpioq] = 0x0b,
};


enum exti_regs {
	exti_rtsr1 = 0x0,
	exti_ftsr1,
	exti_swier1,
	exti_rpr1,
	exti_fpr1,
	exti_seccfgr1,
	exti_privcfgr1,
	exti_rtsr2 = 0x8,
	exti_ftsr2,
	exti_swier2,
	exti_rpr2,
	exti_fpr2,
	exti_seccfgr2,
	exti_privcfgr2,
	exti_rtsr3 = 0x10,
	exti_ftsr3,
	exti_swier3,
	exti_rpr3,
	exti_fpr3,
	exti_seccfgr3,
	exti_privcfgr3,
	exti_exticr1 = 0x18,
	exti_exticr2,
	exti_exticr3,
	exti_exticr4,
	exti_lockr,
	exti_imr1 = 0x20,
	exti_emr1,
	exti_imr2 = 0x24,
	exti_emr2,
	exti_imr3 = 0x28,
	exti_emr3,
};


static int exti_lineToRegBit(uint32_t line, uint32_t *reg_offs, uint32_t *bit)
{
	if (line >= EXTI_LINES) {
		return -1;
	}

	*reg_offs = (line / 32) * 8;
	*bit = (1u << (line % 32));
	return 0;
}


static int exti_handler(unsigned int n, void *arg)
{
	if ((n < exti0_irq) || (n > exti15_irq)) {
		return 0;
	}

	uint32_t line = n - exti0_irq;
	uint32_t reg, bit;
	exti_lineToRegBit(line, &reg, &bit);
	*(exti_common.base + exti_rpr1 + reg) = bit;
	*(exti_common.base + exti_fpr1 + reg) = bit;
	return -1;
}


int exti_configure(unsigned int line, unsigned char mode, unsigned char edge)
{
	uint32_t regMR, regTSR, bit;
	if ((mode > exti_disabled) || (edge > exti_risingfalling)) {
		return -EINVAL;
	}

	if (exti_lineToRegBit(line, &regTSR, &bit) < 0) {
		return -EINVAL;
	}

	regMR = (line / 32) * 4;
	mutexLock(exti_common.lock);
	switch (mode) {
		case exti_irq:
			*(exti_common.base + exti_imr1 + regMR) |= bit;
			*(exti_common.base + exti_emr1 + regMR) &= ~bit;
			break;

		case exti_event:
			*(exti_common.base + exti_imr1 + regMR) &= ~bit;
			*(exti_common.base + exti_emr1 + regMR) |= bit;
			break;

		case exti_irqevent:
			*(exti_common.base + exti_imr1 + regMR) |= bit;
			*(exti_common.base + exti_emr1 + regMR) |= bit;
			break;

		case exti_disabled:
			*(exti_common.base + exti_imr1 + regMR) &= ~bit;
			*(exti_common.base + exti_emr1 + regMR) &= ~bit;
			break;

		default:
			return -EINVAL;
	}

	switch (edge) {
		case exti_rising:
			*(exti_common.base + exti_rtsr1 + regTSR) |= bit;
			*(exti_common.base + exti_ftsr1 + regTSR) &= ~bit;
			break;

		case exti_falling:
			*(exti_common.base + exti_rtsr1 + regTSR) &= ~bit;
			*(exti_common.base + exti_ftsr1 + regTSR) |= bit;
			break;

		case exti_risingfalling:
			*(exti_common.base + exti_rtsr1 + regTSR) |= bit;
			*(exti_common.base + exti_ftsr1 + regTSR) |= bit;
			break;

		default:
			return -EINVAL;
	};

	mutexUnlock(exti_common.lock);

	return EOK;
}


/* NOTE: on STM32N6 external interrupt selection registers are in EXTI, not SYSCFG */
int syscfg_mapexti(unsigned int line, int port)
{
	volatile uint32_t *cr;
	uint32_t tmp;

	if ((port < gpioa) || (port > MAX_GPIO) || (line > 15)) {
		return -EINVAL;
	}

	int8_t mux_setting = gpio_to_mux_setting[port];
	if (mux_setting < 0) {
		return -EINVAL;
	}

	mutexLock(exti_common.lock);
	cr = exti_common.base + exti_exticr1 + (line / 4);
	tmp = *cr;
	tmp &= ~(0xff << ((line % 4) * 8));
	tmp |= ((uint32_t)mux_setting) << ((line % 4) * 8);
	*cr = tmp;
	mutexUnlock(exti_common.lock);

	return EOK;
}


int exti_clear_irq(unsigned int line)
{
	uint32_t reg, bit;
	if (exti_lineToRegBit(line, &reg, &bit) < 0) {
		return -EINVAL;
	}

	*(exti_common.base + exti_rpr1 + reg) = bit;
	*(exti_common.base + exti_fpr1 + reg) = bit;
	return 0;
}


int exti_init(void)
{
	exti_common.base = EXTI_BASE;

	mutexCreate(&exti_common.lock);

	for (int i = 0; i < N_EXTI; i++) {
		interrupt(exti0_irq + i, exti_handler, NULL, 0, NULL);
	}

	return 0;
}
