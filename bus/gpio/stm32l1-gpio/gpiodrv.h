/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 GPIO driver
 *
 * Copyright 2017 Phoenix Systems
 * Author: Aleksander Kaminski, Adrian Kepka
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _GPIODRV_H_
#define _GPIODRV_H_

#include HAL

/* Temporary solution */
extern unsigned int gpiodrv_id;


enum { GPIO_CONFIG, GPIO_INTERRUPT, GPIO_GET, GPIO_SET, GPIO_DELAY };


enum { GPIOA = 0, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH };


enum { EDGE_RISING, EDGE_FALLING };


typedef struct {
	int mask;
	int state;
} __attribute__((packed)) gpioset_t;


typedef struct {
	char pin;
	char mode;
	char af;
	char otype;
	char ospeed;
	char pupd;
} __attribute__((packed)) gpioconfig_t;


typedef struct {
	char pin;
	char state;
	char edge;
} __attribute__((packed)) gpiointerrupt_t;


typedef struct {
	int len;
} __attribute__((packed)) gpiodelay_t;


typedef struct {
	char type;
	int port;

	union {
		gpioset_t set;
		gpioconfig_t config;
		gpiointerrupt_t interrupt;
		gpiodelay_t delay;
	};
} __attribute__((packed)) gpiomsg_t;


typedef struct {
	size_t off;
	char buff[];
} __attribute__((packed)) gpiodrv_data_t;


extern int gpio_sequence(gpiomsg_t *msg, unsigned int len, int *resp);


extern int gpio_setPort(int port, int mask, int state);


extern int gpio_getPort(int port);


extern int gpio_configPin(int port, char pin, char mode, char af, char otype, char ospeed, char pupd);


/* TODO: remove below functions */
extern void gpiodrv_configPin(int port, char pin, char mode, char af, char otype, char ospeed, char pupd);


extern void gpiodrv_init(void);


#endif
