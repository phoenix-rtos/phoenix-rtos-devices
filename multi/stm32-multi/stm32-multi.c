/*
 * Phoenix-RTOS
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

#include "common.h"

#include "adc.h"
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


void handleMsg(msg_t *msg)
{

}


void thread(void *arg)
{
	msg_t msg;
	unsigned int rid;

	while (1) {
		while (msgRecv(common.port, &msg, &rid) < 0)
			;

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;

			case mtRead:
				msg.o.io.err = uart_read(uart4, msg.o.data, msg.o.size, uart_mnormal, 0);
				break;

			case mtWrite:
				msg.o.io.err = uart_write(uart4, msg.i.data, msg.i.size);
				break;

			case mtDevCtl:
				handleMsg(&msg);
				break;

			case mtCreate:
				msg.o.create.err = -EINVAL;
				break;

			case mtLookup:
				msg.o.create.err = -EINVAL;
				break;

			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(common.port, &msg, rid);
	}
}


int main(void)
{
	int i;
	oid_t oid;

	if (portCreate(&common.port) != EOK) {
		DEBUG("Failed to create port\n");
		return -1;
	}

	rcc_init();
	rtc_init();
	gpio_init();
	lcd_init();
	adc_init();
	i2c_init();
	flash_init();
	uart_init();

	portRegister(common.port, "/multi", &oid);

	for (i = 0; i < THREADS_NO - 1; ++i) {
		if (beginthread(thread, THREADS_PRIORITY, common.stack[i], STACKSZ, (void *)i) < 0)
			DEBUG("Failed to spawn thread #%d", i);
	}

	thread((void *)i);

	return 0;
}
