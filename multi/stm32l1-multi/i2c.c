/*
 * Phoenix-RTOS
 *
 * STM32L1 I2C driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/pwman.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include "stm32l1-multi.h"
#include "common.h"
#include "gpio.h"
#include "i2c.h"
#include "rcc.h"


enum { cr1 = 0, cr2, oar1, oar2, dr, sr1, sr2, ccr, trise };


struct {
	volatile unsigned int *base;

	handle_t lock;
	handle_t irqlock;
	handle_t irqcond;
	handle_t inth;
} i2c_common;


static int i2c_irq(unsigned int n, void *arg)
{
	/* Event irq disable */
	*(i2c_common.base + cr2) &= ~(3 << 9);

	return 1;
}


static void i2c_waitForIrq(void)
{
	mutexLock(i2c_common.irqlock);
	*(i2c_common.base + cr2) |= (3 << 9);
	condWait(i2c_common.irqcond, i2c_common.irqlock, 0);
	mutexUnlock(i2c_common.irqlock);
}


static void i2c_hwinit(void)
{
	unsigned int t;

	/* Disable I2C periph */
	*(i2c_common.base + cr1) &= ~1;
	dataBarier();

	*(i2c_common.base + cr1) |= 1 << 15;
	dataBarier();
	*(i2c_common.base + cr1) &= ~(1 << 15);
	dataBarier();

	/* Enable ACK after each byte */
	*(i2c_common.base + cr2) |= 1 << 10;

	/* Peripheral clock = 2 MHz */
	t = *(i2c_common.base + cr2) & ~0x1ff;
	*(i2c_common.base + cr2) = t | (1 << 2);

	/* 95,325 kHz SCK */
	t = *(i2c_common.base + ccr) & ~((1 << 14) | 0x7ff);
	*(i2c_common.base + ccr) = t | 0xb;

	/* 500 ns SCL rise time */
	t = *(i2c_common.base + trise) & ~0x1ff;
	*(i2c_common.base + trise) = t | 3;

	/* Enable I2C periph */
	*(i2c_common.base + cr1) |= 1;
}


unsigned int i2c_transaction(char op, char addr, char reg, void *buff, unsigned int count)
{
	int i;

	if (count < 1 || (op != _i2c_read && op != _i2c_write))
		return 0;

	mutexLock(i2c_common.lock);
	keepidle(1);

	if (*(i2c_common.base + sr2) & (1 << 1))
		i2c_hwinit();

	*(i2c_common.base + cr2) |= (3 << 9);

	/* Start condition generation */
	*(i2c_common.base + cr1) |= (1 << 8);

	while (!(*(i2c_common.base + sr1) & 1))
		i2c_waitForIrq();

	*(i2c_common.base + dr) = addr << 1;

	while (!((*(i2c_common.base + sr1) & 2) && (*(i2c_common.base + sr2) & 1)))
		i2c_waitForIrq();

	while (!(*(i2c_common.base + sr1) & (1 << 7)))
		i2c_waitForIrq();

	*(i2c_common.base + dr) = reg;

	if (op == _i2c_read) {
		*(i2c_common.base + cr1) |= (1 << 8);
		if (count > 1)
			*(i2c_common.base + cr1) |= (1 << 10);

		while (!(*(i2c_common.base + sr1) & 1))
			i2c_waitForIrq();

		*(i2c_common.base + dr) = (addr << 1) | 1;

		while (!((*(i2c_common.base + sr1) & 2) && (*(i2c_common.base + sr2) & 1)))
			i2c_waitForIrq();

		for (i = 0; i < count; ++i) {
			while (!(*(i2c_common.base + sr1) & (1 << 6)))
				i2c_waitForIrq();

			if ((i + 2) >= count)
				*(i2c_common.base + cr1) &= ~(1 << 10);

			((char *)buff)[i] = *(i2c_common.base + dr);
		}
	}
	else {
		for (i = 0; i < count; ++i) {
			while (!(*(i2c_common.base + sr1) & (1 << 7)))
				i2c_waitForIrq();

			*(i2c_common.base + dr) = ((char *)buff)[i];
		}

		while (!(*(i2c_common.base + sr1) & (1 << 7)))
			i2c_waitForIrq();
	}

	*(i2c_common.base + cr1) |= (1 << 9);

	keepidle(0);
	mutexUnlock(i2c_common.lock);

	return i;
}


int i2c_init(void)
{
	i2c_common.base = (void *)0x40005800;

	rcc_devClk(pctl_i2c2, 1);

	i2c_hwinit();

	gpio_configPin(gpiob, 10, 2, 4, 1, 0, 0);
	gpio_configPin(gpiob, 11, 2, 4, 1, 0, 0);

	mutexCreate(&i2c_common.lock);
	mutexCreate(&i2c_common.irqlock);
	condCreate(&i2c_common.irqcond);

	interrupt(49, i2c_irq, NULL, i2c_common.irqcond, &i2c_common.inth);

	return EOK;
}
