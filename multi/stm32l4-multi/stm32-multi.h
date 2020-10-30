/*
 * Phoenix-RTOS
 *
 * STM32L1 multidriver
 *
 * Copyright 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _STM32_MULTI_H_
#define _STM32_MULTI_H_

#include <sys/interrupt.h>
#include <stm32l4-multi/libuart.h>
#include <stm32l4-multi/libspi.h>
#include <stm32l4-multi/libi2c.h>


enum { adc_get = 0, rtc_setcal, rtc_get, rtc_set, rtc_setalarm, i2c_get, i2c_getwreg,
	i2c_set, i2c_setwreg, gpio_def, gpio_get, gpio_set, uart_def, uart_get, uart_set,
	flash_get, flash_set, spi_get, spi_set, spi_rw, spi_def, exti_def, exti_map };

/* RTC */


typedef struct {
	unsigned int usecs;
	unsigned int year;
	unsigned char month;
	unsigned char day;
	unsigned char wday;

	unsigned char hours;
	unsigned char minutes;
	unsigned char seconds;
} __attribute__((packed)) rtctimestamp_t;


/* I2C */


typedef struct {
	int i2c;
	unsigned char addr;
	unsigned char reg;
} __attribute__((packed)) i2cmsg_t;


/* GPIO */


enum { gpioa = 0, gpiob, gpioc, gpiod, gpioe, gpiof, gpiog, gpioh, gpioi };


typedef struct {
	int port;
} __attribute__((packed)) gpioget_t;


typedef struct {
	int port;
	unsigned int mask;
	unsigned int state;
} __attribute__((packed)) gpioset_t;


typedef struct {
	int port;
	char pin;
	char mode;
	char af;
	char otype;
	char ospeed;
	char pupd;
} __attribute__((packed)) gpiodef_t;


/* SPI */

typedef struct {
	int spi;
	unsigned int addr;
	unsigned int flags;
	char cmd;
} __attribute__((packed)) spirw_t;


typedef struct {
	int spi;
	int enable;
	char mode;
	char bdiv;
} __attribute__((packed)) spidef_t;


/* EXTI */


enum { exti_irq = 0, exti_event, exti_irqevent, exti_disabled };


enum { exti_rising = 0, exti_falling, exti_risingfalling };


typedef struct {
	unsigned int line;
	unsigned char mode;
	unsigned char edge;
} extidef_t;


typedef struct {
	unsigned int line;
	int port;
} extimap_t;


/* ADC */


enum { adc1 = 0, adc2, adc3 };


typedef struct {
	int adcno;
	int channel;
} adcget_t;


/* MULTI */


typedef struct {
	int type;

	union {
		adcget_t adc_get;
		int rtc_calib;
		rtctimestamp_t rtc_timestamp;
		i2cmsg_t i2c_msg;
		gpiodef_t gpio_def;
		gpioget_t gpio_get;
		gpioset_t gpio_set;
		spirw_t spi_rw;
		spidef_t spi_def;
		extidef_t exti_def;
		extimap_t exti_map;
		unsigned int flash_addr;
	};
} __attribute__((packed)) multi_i_t;


typedef struct {
	int err;

	union {
		unsigned short adc_valmv;
		rtctimestamp_t rtc_timestamp;
		unsigned int gpio_get;
	};
} __attribute__((packed)) multi_o_t;


#endif
