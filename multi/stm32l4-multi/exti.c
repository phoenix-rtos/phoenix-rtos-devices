/*
 * Phoenix-RTOS
 *
 * STM32 extended interrupts and event controller driver
 *
 * Copyright 2019, 2020, 2025 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Daniel Sawka, Aleksander Kaminski, Jacek Maksymowicz, Mateusz Karcz
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include "stm32l4-multi.h"
#include "common.h"
#include "exti.h"
#include "rcc.h"


/*
 * EXTI_OLD_LAYOUT - selection registers in SYSCFG, no rising/falling pending registers
 * EXTI_NEW_LAYOUT - selection registers in EXTI, rising/falling pending registers present
 */

#define EXTI_IRQS 16

#if defined(__CPU_STM32L4X6)
#define EXTI_OLD_LAYOUT
#define EXTI_CONTINUOUS_IRQS 5
#define EXTI_LINES           41
#define MAX_GPIO             gpioi
#endif

#if defined(__CPU_STM32N6)
#define EXTI_NEW_LAYOUT
#define EXTI_LINES 78
#define MAX_GPIO   gpioq
#endif

#if defined(__CPU_STM32U3)
#define EXTI_NEW_LAYOUT
#define EXTI_LINES 23
#define MAX_GPIO   gpioh
#endif

#if !defined(EXTI_CONTINUOUS_IRQS)
#define EXTI_CONTINUOUS_IRQS EXTI_IRQS
#endif


struct {
	volatile unsigned int *base;
#if defined(EXTI_OLD_LAYOUT)
	volatile unsigned int *syscfg;
#endif

	handle_t lock;
} exti_common;


#define EXTI_REG_SPACING 8
#if defined(EXTI_OLD_LAYOUT)
#define EXTI_xMREG_SPACING 8


enum {
	exti_imr1 = 0x0,
	exti_emr1,
	exti_rtsr1,
	exti_ftsr1,
	exti_swier1,
	exti_pr1,
};


enum {
	syscfg_memrmp = 0x0,
	syscfg_cfgr1,
	syscfg_exticr1,
};
#else
#define EXTI_xMREG_SPACING 4


enum exti_regs {
	exti_rtsr1 = 0x0,
	exti_ftsr1,
	exti_swier1,
	exti_rpr1,
	exti_fpr1,
	exti_seccfgr1,
	exti_privcfgr1,
	exti_exticr1 = 0x18,
	exti_lockr = 0x1c,
	exti_imr1 = 0x20,
	exti_emr1,
};
#endif


static void exti_lineToRegBit(uint32_t line, uint32_t *reg_offs, uint32_t *bit)
{
	*reg_offs = (line / 32) * EXTI_REG_SPACING;
	*bit = (1u << (line % 32));
}


static void exti_lineToXmregBit(uint32_t line, uint32_t *reg_offs, uint32_t *bit)
{
	*reg_offs = (line / 32) * EXTI_xMREG_SPACING;
	*bit = (1u << (line % 32));
}


static int exti_handler(unsigned int n, void *arg)
{
	if ((n < exti0_irq) || (n >= (exti0_irq + EXTI_CONTINUOUS_IRQS))) {
		return 0;
	}

	uint32_t line = n - exti0_irq;
	uint32_t reg, bit;
	exti_lineToRegBit(line, &reg, &bit);
#if defined(EXTI_OLD_LAYOUT)
	*(exti_common.base + exti_pr1 + reg) = bit;
#else
	*(exti_common.base + exti_rpr1 + reg) = bit;
	*(exti_common.base + exti_fpr1 + reg) = bit;
#endif
	return -1;
}


#if defined(__CPU_STM32L4X6)
static int exti9_5_handler(unsigned int n, void *arg)
{
	*(exti_common.base + exti_pr1) = 0x3e0;
	return -1;
}


static int exti15_10_handler(unsigned int n, void *arg)
{
	*(exti_common.base + exti_pr1) = 0xfc00;
	return -1;
}
#endif


static void _exti_setMode(unsigned int line, unsigned char mode)
{
	uint32_t reg, bit;
	exti_lineToXmregBit(line, &reg, &bit);

	switch (mode) {
		case exti_irq:
			*(exti_common.base + exti_imr1 + reg) |= bit;
			*(exti_common.base + exti_emr1 + reg) &= ~bit;
			break;

		case exti_event:
			*(exti_common.base + exti_imr1 + reg) &= ~bit;
			*(exti_common.base + exti_emr1 + reg) |= bit;
			break;

		case exti_irqevent:
			*(exti_common.base + exti_imr1 + reg) |= bit;
			*(exti_common.base + exti_emr1 + reg) |= bit;
			break;

		case exti_disabled:
			*(exti_common.base + exti_imr1 + reg) &= ~bit;
			*(exti_common.base + exti_emr1 + reg) &= ~bit;
			break;
	}
}


static void _exti_setEdge(unsigned int line, unsigned char edge)
{
	uint32_t reg, bit;
	exti_lineToRegBit(line, &reg, &bit);

	switch (edge) {
		case exti_rising:
			*(exti_common.base + exti_rtsr1 + reg) |= bit;
			*(exti_common.base + exti_ftsr1 + reg) &= ~bit;
			break;

		case exti_falling:
			*(exti_common.base + exti_rtsr1 + reg) &= ~bit;
			*(exti_common.base + exti_ftsr1 + reg) |= bit;
			break;

		case exti_risingfalling:
			*(exti_common.base + exti_rtsr1 + reg) |= bit;
			*(exti_common.base + exti_ftsr1 + reg) |= bit;
			break;
	};
}


int exti_configure(unsigned int line, unsigned char mode, unsigned char edge)
{
	if ((line >= EXTI_LINES) || (mode > exti_disabled) || (edge > exti_risingfalling)) {
		return -EINVAL;
	}

	mutexLock(exti_common.lock);
	_exti_setMode(line, mode);
	_exti_setEdge(line, edge);
	mutexUnlock(exti_common.lock);

	return EOK;
}


int syscfg_mapexti(unsigned int line, int port)
{
	static const int8_t gpio_to_mux_setting[] = {
		[gpioa] = 0x00,
		[gpiob] = 0x01,
		[gpioc] = 0x02,
		[gpiod] = 0x03,
		[gpioe] = 0x04,
		[gpiof] = 0x05,
		[gpiog] = 0x06,
		[gpioh] = 0x07,
#if defined(__CPU_STM32L4X6)
		[gpioi] = 0x08,
#elif defined(__CPU_STM32N6)
		[gpioi] = -1,
		[gpioj] = -1,
		[gpiok] = -1,
		[gpiol] = -1,
		[gpiom] = -1,
		[gpion] = 0x08,
		[gpioo] = 0x09,
		[gpiop] = 0x0a,
		[gpioq] = 0x0b,
#endif
	};

#if defined(EXTI_OLD_LAYOUT)
	static int initDone = 0;
#endif
	volatile uint32_t *cr;
	uint32_t tmp;

	if ((port < gpioa) || (port > MAX_GPIO) || (line >= EXTI_IRQS)) {
		return -EINVAL;
	}

	int8_t mux_setting = gpio_to_mux_setting[port];
	if (mux_setting < 0) {
		return -EINVAL;
	}

	mutexLock(exti_common.lock);
#if defined(EXTI_OLD_LAYOUT)
	cr = exti_common.syscfg + syscfg_exticr1 + (line / 4);
	tmp = *cr;
	tmp &= ~(0xf << ((line % 4) * 4));
	tmp |= ((uint32_t)mux_setting) << ((line % 4) * 4);

	if (!initDone) {
		devClk(pctl_syscfg, 1);
		initDone = 1;
	}
#else
	cr = exti_common.base + exti_exticr1 + (line / 4);
	tmp = *cr;
	tmp &= ~(0xff << ((line % 4) * 8));
	tmp |= ((uint32_t)mux_setting) << ((line % 4) * 8);
#endif
	*cr = tmp;
	mutexUnlock(exti_common.lock);

	return EOK;
}


int exti_clear_irq(unsigned int line)
{
	static const uint32_t validBits[] = {
#if defined(__CPU_STM32L4X6)
		0x007dffffU, 0x00000078U
#elif defined(__CPU_STM32N6)
		0x0030ffffU, 0x01480180U, 0x000007f4U
#elif defined(__CPU_STM32U3)
		0x007fffffU
#endif
	};

	uint32_t reg, bit;

	if (line >= EXTI_LINES) {
		return -EINVAL;
	}

	exti_lineToRegBit(line, &reg, &bit);
	if ((bit & validBits[line / 32]) == 0) {
		return -EINVAL;
	}

#if defined(EXTI_OLD_LAYOUT)
	*(exti_common.base + exti_pr1 + reg) = bit;
#else
	*(exti_common.base + exti_rpr1 + reg) = bit;
	*(exti_common.base + exti_fpr1 + reg) = bit;
#endif
	return 0;
}


int exti_init(void)
{
	exti_common.base = EXTI_BASE;
#if defined(EXTI_OLD_LAYOUT)
	exti_common.syscfg = SYSCFG_BASE;
#endif

	mutexCreate(&exti_common.lock);

	for (int i = 0; i < EXTI_CONTINUOUS_IRQS; i++) {
		interrupt(exti0_irq + i, exti_handler, NULL, 0, NULL);
	}

#if defined(__CPU_STM32L4X6)
	interrupt(exti9_5_irq, exti9_5_handler, NULL, 0, NULL);
	interrupt(exti15_10_irq, exti15_10_handler, NULL, 0, NULL);
#endif

	return 0;
}
