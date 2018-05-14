/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 multi driver main
 *
 * Copyright 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include ARCH
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/threads.h>
#include <sys/msg.h>

#include "adc.h"
#include "common.h"
#include "flash.h"
#include "gpio.h"
#include "i2c.h"
#include "lcd.h"
#include "rcc.h"
#include "rtc.h"
#include "uart.h"

#define THREADS_NO 4
#define THREADS_PRIORITY 2
#define STACKSZ 512

#ifndef NDEBUG
static const char drvname[] = "multidrv: ";
#endif


struct {
	char stack[STACKSZ][THREADS_NO];

	unsigned int port;
} common;


void thread(void *arg)
{
	msg_t msg;
	unsigned int rid;

	while (1) {
		while (msgRecv(common.port, &msg, &rid) < 0)
			;



	}
}


int main(void)
{
	int i;

	if (portCreate(&common.port) != EOK) {
		DEBUG("Failed to create port\n");
		return -1;
	}

	rcc_init();
	rtc_init();
	gpio_init();
	lcd_init();
	adc_init();
	//uart_init();
	//flash_init();

	for (i = 0; i < THREADS_NO - 1; ++i) {
		if (beginthread(thread, THREADS_PRIORITY, common.stack[i], STACKSZ, (void *)i) < 0)
			DEBUG("Failed to spawn thread #%d", i);
	}

	thread((void *)THREADS_NO);

	return 0;
}
