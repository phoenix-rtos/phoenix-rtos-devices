/*
 * Phoenix-RTOS
 *
 * STM32L1 external interrupts driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Daniel Sawka
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include "stm32l1-multi.h"
#include "common.h"
#include "exti.h"
#include "rcc.h"


struct {
	volatile unsigned int *base;
	volatile unsigned int *syscfg;

	handle_t lock;
} exti_common;


enum { imr = 0, emr, rtsr, ftsr, swier, pr };


enum { memrmp = 0, pmc, exticr1, exticr2, exticr3, exticr4 };


static int exti0_handler(unsigned int n, void *arg)
{
	*(exti_common.base + pr) = 0x1;
	return -1;
}


static int exti1_handler(unsigned int n, void *arg)
{
	*(exti_common.base + pr) = 0x2;
	return -1;
}


static int exti2_handler(unsigned int n, void *arg)
{
	*(exti_common.base + pr) = 0x4;
	return -1;
}


static int exti3_handler(unsigned int n, void *arg)
{
	*(exti_common.base + pr) = 0x8;
	return -1;
}


static int exti4_handler(unsigned int n, void *arg)
{
	*(exti_common.base + pr) = 0x10;
	return -1;
}


static void _exti_setMode(unsigned int line, unsigned char mode)
{
	unsigned int tmp;

	tmp = *(exti_common.base + imr) & ~(1 << line);
	*(exti_common.base + imr) = tmp | ((mode == exti_irq || mode == exti_irqevent) << line);

	tmp = *(exti_common.base + emr) & ~(1 << line);
	*(exti_common.base + emr) = tmp | ((mode == exti_event || mode == exti_irqevent) << line);
}


static void _exti_setEdge(unsigned int line, unsigned char edge)
{
	unsigned int tmp;

	tmp = *(exti_common.base + rtsr) & ~(1 << line);
	*(exti_common.base + rtsr) = tmp | ((edge != exti_falling) << line);

	tmp = *(exti_common.base + ftsr) & ~(1 << line);
	*(exti_common.base + ftsr) = tmp | ((edge != exti_rising) << line);
}


int exti_configure(unsigned int line, unsigned char mode, unsigned char edge)
{
	if (line > 23 || mode > exti_disabled || edge > exti_risingfalling)
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

	if (port < gpioa || port > gpioh || line > 15)
		return -EINVAL;

	cr = exti_common.syscfg + exticr1 + line / 4;
	mask = 0xF << ((line & 0x3) << 2);

	if (port < gpioe)
		value = port;
	else if (port < gpioh)
		value = port + 1;
	else
		value = 5;

	value = (value << ((line & 0x3) << 2));

	mutexLock(exti_common.lock);
	rcc_devClk(pctl_syscfg, 1);
	tmp = *cr & ~mask;
	*cr = tmp | value;
	mutexUnlock(exti_common.lock);

	return EOK;
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

	return 0;
}
