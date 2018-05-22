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
	multi_i_t *imsg = (multi_i_t *)(&msg->i);
	multi_o_t *omsg = (multi_o_t *)(&msg->o);

	omsg->err = EOK;

	switch (imsg->type) {
		case adc_get:
			omsg->adc_val = adc_conversion(imsg->adc_channel);
			break;

		case rtc_get:
			rtc_getTime(&omsg->rtc_timestamp);
			break;

		case rtc_set:
			rtc_setTime(&imsg->rtc_timestamp);
			break;

		case lcd_get:
			lcd_getDisplay(&imsg->lcd_msg);
			break;

		case lcd_set:
			lcd_setDisplay(&imsg->lcd_msg);
			break;

		case i2c_get:
			omsg->err = i2c_transaction(_i2c_read, imsg->i2c_msg.addr, imsg->i2c_msg.reg, msg->o.data, msg->o.size);
			break;

		case i2c_set:
			omsg->err = i2c_transaction(_i2c_write, imsg->i2c_msg.addr, imsg->i2c_msg.reg, msg->i.data, msg->i.size);
			break;

		case gpio_def:
			omsg->err = gpio_configPin(imsg->gpio_def.port, imsg->gpio_def.pin, imsg->gpio_def.mode,
				imsg->gpio_def.af, imsg->gpio_def.otype, imsg->gpio_def.ospeed, imsg->gpio_def.pupd);
			break;

		case gpio_get:
			omsg->err = gpio_getPort(imsg->gpio_get.port, &omsg->gpio_get);
			break;

		case gpio_set:
			omsg->err = gpio_setPort(imsg->gpio_set.port, imsg->gpio_set.mask, imsg->gpio_set.state);
			break;

		case gpio_seq:
			/* TODO */
			break;

		case uart_def:
			omsg->err = uart_configure(imsg->uart_def.uart, imsg->uart_def.bits, imsg->uart_def.parity,
				imsg->uart_def.baud, imsg->uart_def.enable);
			break;

		case uart_get:
			omsg->err = uart_read(imsg->uart_get.uart, msg->o.data, msg->o.size,
				imsg->uart_get.mode, imsg->uart_get.timeout);
			break;

		case uart_set:
			omsg->err = uart_write(imsg->uart_set.uart, msg->i.data, msg->i.size);
			break;

		case flash_get:
			/* TODO */
			break;

		case flash_set:
			/* TODO */
			break;

		default:
			omsg->err = -EINVAL;
	}
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
