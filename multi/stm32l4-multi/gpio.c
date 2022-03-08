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
#include <sys/threads.h>

#include "stm32l4-multi.h"
#include "common.h"
#include "rcc.h"


static const int gpio2pctl[] = { pctl_gpioa, pctl_gpiob, pctl_gpioc, pctl_gpiod, pctl_gpioe,
	pctl_gpiof, pctl_gpiog, pctl_gpioh, pctl_gpioi };


enum { moder = 0, otyper, ospeedr, pupdr, idr, odr, bsrr, lckr, afrl, afrh, brr, ascr };


struct {
	volatile unsigned int *base[9];
	uint32_t enableMask;

	handle_t lock;
} gpio_common;


int gpio_setPort(int port, unsigned int mask, unsigned int val)
{
	unsigned int t;

	if (port < gpioa || port > gpioi)
		return -EINVAL;

	t = (mask & val) & 0xffff;
	t |= (mask & ~val) << 16;

	*(gpio_common.base[port - gpioa] + bsrr) = t;
	dataBarier();

	return EOK;
}


int gpio_getPort(int port, unsigned int *val)
{
	if (port < gpioa || port > gpioi)
		return -EINVAL;

	dataBarier();
	(*val) = *(gpio_common.base[port - gpioa] + idr) & 0xffff;

	return EOK;
}


int gpio_configPin(int port, char pin, char mode, char af, char otype, char ospeed, char pupd)
{
	volatile unsigned int *reg;
	unsigned int t;

	if (port < gpioa || port > gpioi || pin > 16)
		return -EINVAL;

	reg = gpio_common.base[port - gpioa];

	mutexLock(gpio_common.lock);

	/* Enable GPIO port's clock */
	if (!(gpio_common.enableMask & (1 << (port - gpioa)))) {
		devClk(gpio2pctl[port - gpioa], 1);
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
	} else {
		t = *(reg + afrh) & ~(0xf << ((pin - 8) << 2));
		*(reg + afrh) = t | (af & 0xf) << ((pin - 8) << 2);
	}

	if ((mode & 0x3) == 0x3)
		*(reg + ascr) |= 1 << pin;
	else
		*(reg + ascr) &= ~(1 << pin);

	dataBarier();

	t = *(reg + moder) & ~(0x3 << (pin << 1));
	*(reg + moder) = t | (mode & 0x3) << (pin << 1);

	dataBarier();

	mutexUnlock(gpio_common.lock);

	return EOK;
}


int gpio_init(void)
{
	gpio_common.base[0] = (void *)0x48000000;
	gpio_common.base[1] = (void *)0x48000400;
	gpio_common.base[2] = (void *)0x48000800;
	gpio_common.base[3] = (void *)0x48000c00;
	gpio_common.base[4] = (void *)0x48001000;
	gpio_common.base[5] = (void *)0x48001400;
	gpio_common.base[6] = (void *)0x48001800;
	gpio_common.base[7] = (void *)0x48001c00;
	gpio_common.base[8] = (void *)0x48002000;

	gpio_common.enableMask = 0;
	mutexCreate(&gpio_common.lock);

	return EOK;
}
