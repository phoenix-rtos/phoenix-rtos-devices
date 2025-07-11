/*
 * Phoenix-RTOS
 *
 * STM32L4 GPIO driver
 *
 * Copyright 2017, 2018, 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/threads.h>

#include "stm32l4-multi.h"
#include "common.h"
#include "rcc.h"

#if defined(__CPU_STM32L4X6)
#define GPIO_MAX gpioi
static const int gpio2pctl[GPIO_MAX + 1] = {
	[gpioa] = pctl_gpioa,
	[gpiob] = pctl_gpiob,
	[gpioc] = pctl_gpioc,
	[gpiod] = pctl_gpiod,
	[gpioe] = pctl_gpioe,
	[gpiof] = pctl_gpiof,
	[gpiog] = pctl_gpiog,
	[gpioh] = pctl_gpioh,
	[gpioi] = pctl_gpioi,
};
#elif defined(__CPU_STM32N6)
#define GPIO_MAX gpioq
static const int gpio2pctl[GPIO_MAX + 1] = {
	[gpioa] = pctl_gpioa,
	[gpiob] = pctl_gpiob,
	[gpioc] = pctl_gpioc,
	[gpiod] = pctl_gpiod,
	[gpioe] = pctl_gpioe,
	[gpiof] = pctl_gpiof,
	[gpiog] = pctl_gpiog,
	[gpioh] = pctl_gpioh,
	[gpioi] = -1,
	[gpioj] = -1,
	[gpiok] = -1,
	[gpiol] = -1,
	[gpiom] = -1,
	[gpion] = pctl_gpion,
	[gpioo] = pctl_gpioo,
	[gpiop] = pctl_gpiop,
	[gpioq] = pctl_gpioq,
};
#endif


enum {
	moder = 0,
	otyper,
	ospeedr,
	pupdr,
	idr,
	odr,
	bsrr,
	lckr,
	afrl,
	afrh,
	brr,
	ascr,
};


struct {
	volatile unsigned int *base[GPIO_MAX + 1];
	uint32_t enableMask;

	handle_t lock;
} gpio_common;


int gpio_setPort(int port, unsigned int mask, unsigned int val)
{
	unsigned int t;

	if ((port < gpioa) || (port > GPIO_MAX) || (gpio_common.base[port] == NULL)) {
		return -EINVAL;
	}

	t = (mask & val) & 0xffff;
	t |= (mask & ~val) << 16;

	*(gpio_common.base[port] + bsrr) = t;
	dataBarier();

	return EOK;
}


int gpio_getPort(int port, unsigned int *val)
{
	if ((port < gpioa) || (port > GPIO_MAX) || (gpio_common.base[port] == NULL)) {
		return -EINVAL;
	}

	dataBarier();
	(*val) = *(gpio_common.base[port] + idr) & 0xffff;

	return EOK;
}


int gpio_configPin(int port, char pin, char mode, char af, char otype, char ospeed, char pupd)
{
	volatile unsigned int *reg;
	unsigned int t;

	if ((port < gpioa) || (port > GPIO_MAX) || (gpio_common.base[port] == NULL) || (pin > 16)) {
		return -EINVAL;
	}

	if (gpio2pctl[port] < 0) {
		return -EINVAL;
	}

	reg = gpio_common.base[port];

	mutexLock(gpio_common.lock);

	/* Enable GPIO port's clock */
	if (!(gpio_common.enableMask & (1 << (port - gpioa)))) {
		devClk(gpio2pctl[port], 1);
		gpio_common.enableMask |= (1 << (port - gpioa));
	}

	t = *(reg + otyper) & ~(1 << pin);
	*(reg + otyper) = t | (otype & 1) << pin;

	t = *(reg + ospeedr) & ~(0x3 << (pin << 1));
	*(reg + ospeedr) = t | (ospeed & 0x3) << (pin << 1);

	t = *(reg + pupdr) & ~(0x03 << (pin << 1));
	*(reg + pupdr) = t | (pupd & 0x3) << (pin << 1);

	if (pin < 8) {
		t = *(reg + afrl) & ~(0xf << (pin << 2));
		*(reg + afrl) = t | (af & 0xf) << (pin << 2);
	}
	else {
		t = *(reg + afrh) & ~(0xf << ((pin - 8) << 2));
		*(reg + afrh) = t | (af & 0xf) << ((pin - 8) << 2);
	}

	if ((mode & 0x3) == 0x3) {
		*(reg + ascr) |= 1 << pin;
	}
	else {
		*(reg + ascr) &= ~(1 << pin);
	}

	dataBarier();

	t = *(reg + moder) & ~(0x3 << (pin << 1));
	*(reg + moder) = t | (mode & 0x3) << (pin << 1);

	dataBarier();

	mutexUnlock(gpio_common.lock);

	return EOK;
}


int gpio_init(void)
{
	memset(gpio_common.base, 0, sizeof(gpio_common.base));
#if defined(__CPU_STM32L4X6)
	gpio_common.base[gpioa] = GPIOA_BASE;
	gpio_common.base[gpiob] = GPIOB_BASE;
	gpio_common.base[gpioc] = GPIOC_BASE;
	gpio_common.base[gpiod] = GPIOD_BASE;
	gpio_common.base[gpioe] = GPIOE_BASE;
	gpio_common.base[gpiof] = GPIOF_BASE;
	gpio_common.base[gpiog] = GPIOG_BASE;
	gpio_common.base[gpioh] = GPIOH_BASE;
	gpio_common.base[gpioi] = GPIOI_BASE;
#elif defined(__CPU_STM32N6)
	gpio_common.base[gpioa] = GPIOA_BASE;
	gpio_common.base[gpiob] = GPIOB_BASE;
	gpio_common.base[gpioc] = GPIOC_BASE;
	gpio_common.base[gpiod] = GPIOD_BASE;
	gpio_common.base[gpioe] = GPIOE_BASE;
	gpio_common.base[gpiof] = GPIOF_BASE;
	gpio_common.base[gpiog] = GPIOG_BASE;
	gpio_common.base[gpioh] = GPIOH_BASE;
	gpio_common.base[gpion] = GPION_BASE;
	gpio_common.base[gpioo] = GPIOO_BASE;
	gpio_common.base[gpiop] = GPIOP_BASE;
	gpio_common.base[gpioq] = GPIOQ_BASE;

	for (int i = gpioa; i <= GPIO_MAX; i++) {
		if (gpio_common.base[i] != NULL) {
			platformctl_t pctl = {
				.action = pctl_set,
				.type = pctl_gpioPrivilege,
				.gpioPrivilege = {
					.port = gpio2pctl[i],
					.mask = 0, /* All GPIOs are unprivileged */
				}
			};

			platformctl(&pctl);
		}
	}
#endif

	gpio_common.enableMask = 0;
	mutexCreate(&gpio_common.lock);

	return EOK;
}
