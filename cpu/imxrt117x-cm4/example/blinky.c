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
#include <mu.h>

static struct {
	unsigned int start;
	unsigned int end;
} common;


static void setTimer(unsigned int howlong)
{
	common.start = cm4_getJiffies();
	common.end = common.start + howlong;
}


static int checkTimer(void)
{
	int val;

	val = cm4_getJiffies();

	if ((common.start < common.end && val > common.end) || (val < common.start && val > common.end))
		return 1;

	return 0;
}


int main(int argc, char *argv[])
{
	char byte;
	int i;
	static const char str[] = "Hearth beat\n";

	/* GPIO_AD_10 - GPIO9 pin 9*/
	cm4_setIOmux(pctl_mux_gpio_ad_10, 0, 10);
	cm4_setIOpad(pctl_pad_gpio_ad_10, 0, 1, 0, 0, 0, 0);

	gpio_setDir(gpio9, 9, gpio_out);
	gpio_setPin(gpio9, 9, 1);

	setTimer(500);

	while (1) {
		if (checkTimer()) {
			setTimer(500);
			gpio_togglePin(gpio9, 9);
			mu_write(0, str, sizeof(str) - 1);
		}

		for (i = 1; i < 4; ++i) {
			if (mu_read(i, &byte, 1)) {
				byte += i - 1;
				mu_write(i, &byte, 1);
			}
		}
	}
}
