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
										#define NULL 0
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


struct {
	char stack[STACKSZ][THREADS_NO];

	unsigned int port;
} common;


const char drvname[] = "multidriver";


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

	common.pool_root = NULL;
	common.pool_size = 0;
	common.pool_busy = 0;

	if (portCreate(&common.port) != EOK) {
		DEBUG("Failed to port\n");
		return -1;
	}

	if (mutexCreate(&common.pool_lock) != EOK) {
		DEBUG("Failed to create lock\n");
		portDestroy(common.port);
		return -1;
	}

	rcc_init();
	rtc_init();
	//lcd_init();
	//uart_init();
	//gpio_init();
	//flash_init();
	//adc_init();

	for (i = 0; i < THREADS_NO - 1; ++i) {
		if (beginthread(thread, THREADS_PRIORITY, common.stack[i], STACKSZ, i) < 0)
			DEBUG("Failed to spawn thread #%d", i);
	}

	thread();
}
