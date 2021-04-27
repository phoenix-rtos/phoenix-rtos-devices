/*
 * Phoenix-RTOS
 *
 * i.MX RT117x Cortex-M4 blinky example
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <cm4.h>
#include <gpio.h>

static void wait(unsigned int howlong)
{
	unsigned int start = cm4_getJiffies(), end = start + howlong, val;

	while (1) {
		__asm__ volatile ("wfi");

		val = cm4_getJiffies();

		if ((start < end && val > end) || (val < start && val > end))
			return;
	}
}

int main(int argc, char *argv[])
{
	/* GPIO_AD_10 - GPIO9 pin 9*/
	cm4_setIOmux(pctl_mux_gpio_ad_10, 0, 10);
	cm4_setIOpad(pctl_pad_gpio_ad_10, 0, 1, 0, 0, 0, 0);

	gpio_setDir(gpio9, 9, gpio_out);
	gpio_setPin(gpio9, 9, 1);

	while (1) {
		wait(500);
		gpio_togglePin(gpio9, 9);
	}
}
