/*
 * Phoenix-RTOS
 *
 * STM32L4 external interrupts driver
 *
 * Copyright 2019, 2020 Phoenix Systems
 * Author: Daniel Sawka, Aleksander Kaminski
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


struct {
	volatile unsigned int *base;
	volatile unsigned int *syscfg;

	handle_t lock;
} exti_common;


enum { imr1 = 0, emr1, rtsr1, ftsr1, swier1, pr1, imr2 = 8, emr2, rtsr2, ftsr2, swier2, pr2 };


enum { memrmp = 0, cfgr1, exticr1, exticr2, exticr3, exticr4 };


static int exti0_handler(unsigned int n, void *arg)
{
	*(exti_common.base + pr1) = 0x1;
	return -1;
}


static int exti1_handler(unsigned int n, void *arg)
{
	*(exti_common.base + pr1) = 0x2;
	return -1;
}


static int exti2_handler(unsigned int n, void *arg)
{
	*(exti_common.base + pr1) = 0x4;
	return -1;
}


static int exti3_handler(unsigned int n, void *arg)
{
	*(exti_common.base + pr1) = 0x8;
	return -1;
}


static int exti4_handler(unsigned int n, void *arg)
{
	*(exti_common.base + pr1) = 0x10;
	return -1;
}


static int exti9_5_handler(unsigned int n, void *arg)
{
	*(exti_common.base + pr1) = 0x3e0;
	return -1;
}


static int exti15_10_handler(unsigned int n, void *arg)
{
	*(exti_common.base + pr1) = 0xfc00;
	return -1;
}


static void _exti_setMode(unsigned int line, unsigned char mode)
{
	unsigned int tmp;
	int regoffs = 0;

	if (line > 31) {
		regoffs = imr2;
		line -= 32;
	}

	tmp = *(exti_common.base + imr1 + regoffs) & ~(1 << line);
	*(exti_common.base + imr1 + regoffs) = tmp | ((mode == exti_irq || mode == exti_irqevent) << line);

	tmp = *(exti_common.base + emr1 + regoffs) & ~(1 << line);
	*(exti_common.base + emr1 + regoffs) = tmp | ((mode == exti_event || mode == exti_irqevent) << line);
}


static void _exti_setEdge(unsigned int line, unsigned char edge)
{
	unsigned int tmp;
	int regoffs = 0;

	if (line > 31) {
		regoffs = imr2;
		line -= 32;
	}

	tmp = *(exti_common.base + rtsr1 + regoffs) & ~(1 << line);
	*(exti_common.base + rtsr1 + regoffs) = tmp | ((edge != exti_falling) << line);

	tmp = *(exti_common.base + ftsr1 + regoffs) & ~(1 << line);
	*(exti_common.base + ftsr1 + regoffs) = tmp | ((edge != exti_rising) << line);
}


int exti_configure(unsigned int line, unsigned char mode, unsigned char edge)
{
	if (line > 40 || mode > exti_disabled || edge > exti_risingfalling)
		return -EINVAL;

	mutexLock(exti_common.lock);
	_exti_setMode(line, mode);
	_exti_setEdge(line, edge);
	mutexUnlock(exti_common.lock);

	return EOK;
}


int syscfg_mapexti(unsigned int line, int port)
{
	volatile unsigned int *cr;
	unsigned int value;
	unsigned int tmp;
	unsigned int mask;
	static int initDone = 0;

	if (port < gpioa || port > gpioi || line > 15)
		return -EINVAL;

	cr = exti_common.syscfg + exticr1 + line / 4;
	mask = 0xf << ((line & 0x3) << 2);

	value = port - gpioa;
	value = (value << ((line & 0x3) << 2));

	mutexLock(exti_common.lock);
	if (!initDone) {
		devClk(pctl_syscfg, 1);
		initDone = 1;
	}
	tmp = *cr & ~mask;
	*cr = tmp | value;
	mutexUnlock(exti_common.lock);

	return EOK;
}


int exti_clear_irq(unsigned int line)
{
	if (line <= 22 && line != 17)
		*(exti_common.base + pr1) = 1 << line;
	else if (line >= 35 && line <= 38)
		*(exti_common.base + pr2) = 1 << (line - 32);
	else
		return -1;

	return 0;
}


int exti_init(void)
{
	exti_common.base = (void *)0x40010400;
	exti_common.syscfg = (void *)0x40010000;

	mutexCreate(&exti_common.lock);

	interrupt(exti0_irq, exti0_handler, NULL, 0, NULL);
	interrupt(exti1_irq, exti1_handler, NULL, 0, NULL);
	interrupt(exti2_irq, exti2_handler, NULL, 0, NULL);
	interrupt(exti3_irq, exti3_handler, NULL, 0, NULL);
	interrupt(exti4_irq, exti4_handler, NULL, 0, NULL);
	interrupt(exti9_5_irq, exti9_5_handler, NULL, 0, NULL);
	interrupt(exti15_10_irq, exti15_10_handler, NULL, 0, NULL);

	return 0;
}
