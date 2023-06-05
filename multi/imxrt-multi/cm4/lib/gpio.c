/*
 * Phoenix-RTOS
 *
 * i.MX RT117x Cortex-M4 core gpio driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stddef.h>
#include "gpio.h"


struct {
	volatile uint32_t *gpioBase[13];
} gpio_common;


// clang-format off
enum { dr = 0, gdir, psr, icr1, icr2, imr, isr, edge_sel, dr_set = 33, dr_clr, dr_toggle };
// clang-format on


int gpio_setPin(int port, int pin, int state)
{
	if (port < gpio1 || port > gpio13 || pin < 0 || pin > 31)
		return -1;

	if (state)
		*(gpio_common.gpioBase[port] + dr_set) = 1u << pin;
	else
		*(gpio_common.gpioBase[port] + dr_clr) = 1u << pin;

	return 0;
}


int gpio_togglePin(int port, int pin)
{
	if (port < gpio1 || port > gpio13 || pin < 0 || pin > 31)
		return -1;

	*(gpio_common.gpioBase[port] + dr_toggle) = 1u << pin;

	return 0;
}


int gpio_setPort(int port, uint32_t mask, uint32_t val)
{
	if (port < gpio1 || port > gpio13)
		return -1;

	*(gpio_common.gpioBase[port] + dr_set) = val & mask;
	*(gpio_common.gpioBase[port] + dr_clr) = ~val & mask;

	return 0;
}


int gpio_getPort(int port, unsigned int *val)
{
	if (port < gpio1 || port > gpio13 || val == NULL)
		return -1;

	*val = *(gpio_common.gpioBase[port] + dr);

	return 0;
}


int gpio_setDir(int port, int pin, int dir)
{
	uint32_t t;

	if (port < gpio1 || port > gpio13 || pin < 0 || pin > 31)
		return -1;

	t = *(gpio_common.gpioBase[port] + gdir) & ~(1 << pin);
	*(gpio_common.gpioBase[port] + gdir) = t | ((!!dir) << pin);

	return 0;
}


void gpio_init(void)
{
	gpio_common.gpioBase[0] = (void *)0x4012c000;
	gpio_common.gpioBase[1] = (void *)0x40130000;
	gpio_common.gpioBase[2] = (void *)0x40134000;
	gpio_common.gpioBase[3] = (void *)0x40138000;
	gpio_common.gpioBase[4] = (void *)0x4013c000;
	gpio_common.gpioBase[5] = (void *)0x40140000;
	gpio_common.gpioBase[6] = (void *)0x40c5c000;
	gpio_common.gpioBase[7] = (void *)0x40c60000;
	gpio_common.gpioBase[8] = (void *)0x40c64000;
	gpio_common.gpioBase[9] = (void *)0x40c68000;
	gpio_common.gpioBase[10] = (void *)0x40c6c000;
	gpio_common.gpioBase[11] = (void *)0x40c70000;
	gpio_common.gpioBase[12] = (void *)0x40ca0000;
}
