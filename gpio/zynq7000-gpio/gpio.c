/*
 * Phoenix-RTOS
 *
 * Zynq-7000 GPIO controller
 *
 * Copyright 2022 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>

#include <sys/mman.h>
#include <sys/platform.h>

#include <phoenix/arch/zynq7000.h>
#include <board_config.h>

#include "gpio.h"


/* GPIO registers */
#define GPIO_DLSW0 0
#define GPIO_DMSW0 1
#define GPIO_DLSW1 2
#define GPIO_DMSW1 3

#define GPIO_DATA0 16
#define GPIO_DATA1 17

#define GPIO_RODATA0 24
#define GPIO_RODATA1 25

#define GPIO_DIRM0 129
#define GPIO_OEN0  130

#define GPIO_DIRM1 145
#define GPIO_OEN1  146


/* GPIO pins */
const int gpioPins[GPIO_BANKS][GPIO_PINS] = {
	{
		GPIO0_0, GPIO0_1, GPIO0_2, GPIO0_3,
		GPIO0_4, GPIO0_5, GPIO0_6, GPIO0_7,
		GPIO0_8, GPIO0_9, GPIO0_10, GPIO0_11,
		GPIO0_12, GPIO0_13, GPIO0_14, GPIO0_15,
		GPIO0_16, GPIO0_17, GPIO0_18, GPIO0_19,
		GPIO0_20, GPIO0_21, GPIO0_22, GPIO0_23,
		GPIO0_24, GPIO0_25, GPIO0_26, GPIO0_27,
		GPIO0_28, GPIO0_29, GPIO0_30, GPIO0_31
	},
	{
		GPIO1_0, GPIO1_1, GPIO1_2, GPIO1_3,
		GPIO1_4, GPIO1_5, GPIO1_6, GPIO1_7,
		GPIO1_8, GPIO1_9, GPIO1_10, GPIO1_11,
		GPIO1_12, GPIO1_13, GPIO1_14, GPIO1_15,
		GPIO1_16, GPIO1_17, GPIO1_18, GPIO1_19,
		GPIO1_20, GPIO1_21, -1, -1,
		-1, -1, -1, -1,
		-1, -1, -1, -1,
	},
};


static struct {
	volatile uint32_t *base; /* GPIO registers base address */
} gpio_common;


static int gpio_checkPin(unsigned int bank, unsigned int pin)
{
	if ((bank >= GPIO_BANKS) || (pin >= GPIO_PINS)) {
		return -EINVAL;
	}

	if (gpioPins[bank][pin] < 0) {
		return -EINVAL;
	}

	return EOK;
}


int gpio_readPort(unsigned int bank, uint32_t *val)
{
	if (bank == 0) {
		*val = *(gpio_common.base + GPIO_RODATA0);
	}
	else if (bank == 1) {
		*val = *(gpio_common.base + GPIO_RODATA1);
	}
	else {
		return -EINVAL;
	}

	return EOK;
}


int gpio_writePort(unsigned int bank, uint32_t val, uint32_t mask)
{
	if (bank == 0) {
		/* Write MSW */
		if (mask & 0xffff0000) {
			*(gpio_common.base + GPIO_DMSW0) = (~mask & 0xffff0000) | ((val >> 16) & 0x0000ffff);
		}
		/* Write LSW */
		if (mask & 0x0000ffff) {
			*(gpio_common.base + GPIO_DLSW0) = ((~mask << 16) & 0xffff0000) | (val & 0x0000ffff);
		}
	}
	else if (bank == 1) {
		/* Write MSW */
		if (mask & 0xffff0000) {
			*(gpio_common.base + GPIO_DMSW1) = (~mask & 0xffff0000) | ((val >> 16) & 0x0000ffff);
		}
		/* Write LSW */
		if (mask & 0x0000ffff) {
			*(gpio_common.base + GPIO_DLSW1) = ((~mask << 16) & 0xffff0000) | (val & 0x0000ffff);
		}
	}
	else {
		return -EINVAL;
	}

	return EOK;
}


int gpio_readDir(unsigned int bank, uint32_t *dir)
{
	if (bank == 0) {
		*dir = *(gpio_common.base + GPIO_DIRM0);
	}
	else if (bank == 1) {
		*dir = *(gpio_common.base + GPIO_DIRM1);
	}
	else {
		return -EINVAL;
	}

	return EOK;
}


int gpio_writeDir(unsigned int bank, uint32_t dir, uint32_t mask)
{
	uint32_t tmp;

	if (bank == 0) {
		tmp = *(gpio_common.base + GPIO_DIRM0);
		*(gpio_common.base + GPIO_DIRM0) = (tmp & ~mask) | (dir & mask);
	}
	else if (bank == 1) {
		tmp = *(gpio_common.base + GPIO_DIRM1);
		*(gpio_common.base + GPIO_DIRM1) = (tmp & ~mask) | (dir & mask);
	}
	else {
		return -EINVAL;
	}

	return EOK;
}


int gpio_readPin(unsigned int bank, unsigned int pin, uint32_t *val)
{
	int err;

	err = gpio_checkPin(bank, pin);
	if (err < 0) {
		return err;
	}

	err = gpio_readPort(bank, val);
	if (err < 0) {
		return err;
	}

	*val = (*val & (1 << pin)) ? 1 : 0;

	return EOK;
}


int gpio_writePin(unsigned int bank, unsigned int pin, uint32_t val)
{
	int err;

	err = gpio_checkPin(bank, pin);
	if (err < 0) {
		return err;
	}

	val = (val) ? 1 : 0;

	return gpio_writePort(bank, val << pin, 1 << pin);
}


static int gpio_setPin(unsigned int pin)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_mio;
	pctl.mio.pin = pin;
	pctl.mio.disableRcvr = 1;
	pctl.mio.pullup = 1;
	pctl.mio.ioType = 1;
	pctl.mio.speed = 0;
	pctl.mio.l3 = 0;
	pctl.mio.l2 = 0;
	pctl.mio.l1 = 0;
	pctl.mio.l0 = 0;
	pctl.mio.triEnable = 0;

	return platformctl(&pctl);
}


static int gpio_initPins(void)
{
	unsigned int i, j;
	int err;

	for (i = 0; i < GPIO_BANKS; i++) {
		for (j = 0; j < GPIO_PINS; j++) {
			/* Skip not configured pins */
			if (gpioPins[i][j] < 0) {
				continue;
			}

			err = gpio_setPin(gpioPins[i][j]);
			if (err < 0) {
				return err;
			}
		}
	}

	return EOK;
}


static int gpio_reset(void)
{
	platformctl_t pctl;
	int err;

	pctl.action = pctl_set;
	pctl.type = pctl_devreset;
	pctl.devreset.dev = pctl_ctrl_gpio_rst;
	pctl.devreset.state = 1;
	err = platformctl(&pctl);
	if (err < 0) {
		return err;
	}

	pctl.devreset.state = 0;
	err = platformctl(&pctl);
	if (err < 0) {
		return err;
	}

	return err;
}


int gpio_init(void)
{
	int err;

	/* Reset controller */
	err = gpio_reset();
	if (err < 0) {
		return err;
	}

	/* Configure pins */
	err = gpio_initPins();
	if (err < 0) {
		return err;
	}

	/* Map GPIO registers */
	gpio_common.base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0xe000a000);
	if (gpio_common.base == MAP_FAILED) {
		return -ENOMEM;
	}

	/* Enable output for all pins */
	*(gpio_common.base + GPIO_OEN0) = 0xffffffff;
	*(gpio_common.base + GPIO_OEN1) = 0xffffffff;

	return EOK;
}
