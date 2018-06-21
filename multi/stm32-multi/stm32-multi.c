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


#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/threads.h>
#include <sys/msg.h>
#include <sys/pwman.h>

#include "common.h"

#include "adc.h"
#include "flash.h"
#include "gpio.h"
#include "i2c.h"
#include "lcd.h"
#include "rcc.h"
#include "rtc.h"
#include "uart.h"
#include "spi.h"

#define THREADS_NO 4
#define THREADS_PRIORITY 2
#define STACKSZ 512


struct {
	char stack[THREADS_NO - 1][STACKSZ] __attribute__ ((aligned(8)));

	unsigned int port;
} common;


void handleGpioSeq(gpioseq_t seq[], size_t scount, unsigned int *val)
{
	size_t i;
	volatile int j;

	for (i = 0; i < scount; ++i) {
		switch (seq[i].type) {
			case gpio_def:
				gpio_configPin(seq[i].def.port, seq[i].def.pin, seq[i].def.mode,
					seq[i].def.af, seq[i].def.otype, seq[i].def.ospeed, seq[i].def.pupd);
				break;

			case gpio_get:
				gpio_getPort(seq[i].get.port, val);
				break;

			case gpio_set:
				gpio_setPort(seq[i].set.port, seq[i].set.mask, seq[i].set.state);
				break;

			case gpio_delay:
				for (j = 0; j < seq[i].delay; ++j)
					__asm__ volatile ("nop");
				break;
		}
	}
}


int handleMsg(msg_t *msg)
{
	multi_i_t *imsg = (multi_i_t *)(&msg->i.raw);
	multi_o_t *omsg = (multi_o_t *)(&msg->o.raw);
	int err = EOK;

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
			err = i2c_transaction(_i2c_read, imsg->i2c_msg.addr, imsg->i2c_msg.reg, msg->o.data, msg->o.size);
			break;

		case i2c_set:
			err = i2c_transaction(_i2c_write, imsg->i2c_msg.addr, imsg->i2c_msg.reg, msg->i.data, msg->i.size);
			break;

		case gpio_def:
			err = gpio_configPin(imsg->gpio_def.port, imsg->gpio_def.pin, imsg->gpio_def.mode,
				imsg->gpio_def.af, imsg->gpio_def.otype, imsg->gpio_def.ospeed, imsg->gpio_def.pupd);
			break;

		case gpio_get:
			err = gpio_getPort(imsg->gpio_get.port, &omsg->gpio_get);
			break;

		case gpio_set:
			err = gpio_setPort(imsg->gpio_set.port, imsg->gpio_set.mask, imsg->gpio_set.state);
			break;

		case gpio_seq:
			handleGpioSeq(msg->i.data, msg->i.size / sizeof(gpioseq_t), &omsg->gpio_get);
			break;

		case uart_def:
			err = uart_configure(imsg->uart_def.uart, imsg->uart_def.bits, imsg->uart_def.parity,
				imsg->uart_def.baud, imsg->uart_def.enable);
			break;

		case uart_get:
			err = uart_read(imsg->uart_get.uart, msg->o.data, msg->o.size,
				imsg->uart_get.mode, imsg->uart_get.timeout);
			break;

		case uart_set:
			err = uart_write(imsg->uart_set.uart, msg->i.data, msg->i.size);
			break;

		case flash_get:
			err = flash_readData(imsg->flash_addr, msg->o.data, msg->o.size);
			break;

		case flash_set:
			err = flash_writeData(imsg->flash_addr, msg->i.data, msg->i.size);
			break;

		case spi_get:
			err = spi_transaction(imsg->spi_rw.spi, spi_read, imsg->spi_rw.cmd, imsg->spi_rw.addr,
				imsg->spi_rw.flags, msg->o.data, msg->o.size);
			break;

		case spi_set:
			err = spi_transaction(imsg->spi_rw.spi, spi_write, imsg->spi_rw.cmd, imsg->spi_rw.addr,
				imsg->spi_rw.flags, msg->i.data, msg->i.size);
			break;

		case spi_def:
			err = spi_configure(imsg->spi_def.spi, imsg->spi_def.mode, imsg->spi_def.bdiv, imsg->spi_def.enable);
			break;

		default:
			err = -EINVAL;
	}

	return err;
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
				msg.o.io.err = uart_read(UART_CONSOLE + usart1 - 1, msg.o.data, msg.o.size, uart_mnormal, 0);
				break;

			case mtWrite:
				msg.o.io.err = uart_write(UART_CONSOLE + usart1 - 1, msg.i.data, msg.i.size);
				break;

			case mtDevCtl:
				msg.o.io.err = handleMsg(&msg);
				break;

			case mtCreate:
				msg.o.create.err = -EINVAL;
				break;

			case mtLookup:
				msg.o.lookup.err = -EINVAL;
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

	rcc_init();
	rtc_init();
	gpio_init();
	lcd_init();
	adc_init();
	i2c_init();
	flash_init();
	uart_init();
	spi_init();

	portCreate(&common.port);
	portRegister(common.port, "/multi", &oid);

	for (i = 0; i < THREADS_NO - 1; ++i)
		beginthread(thread, THREADS_PRIORITY, common.stack[i], STACKSZ, (void *)i);

	thread((void *)i);

	return 0;
}
