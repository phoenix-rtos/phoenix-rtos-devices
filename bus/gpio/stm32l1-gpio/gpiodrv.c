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


#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <sys/threads.h>
#include <sys/msg.h>

#include "gpiodrv.h"


struct {
	volatile u32 *rcc;
	volatile u32 *gpio[8];
	volatile u32 *exti;
	volatile u32 *syscfg;

	unsigned int id;
} gpiodrv_common;


static const u8 port2ix[8] = {0, 1, 2, 3, 4, 6, 7, 5};
static const u8 ix2port[8] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOH, GPIOF, GPIOG};


enum { gpio_moder = 0, gpio_otyper, gpio_ospeedr, gpio_pupdr, gpio_idr,
	gpio_odr, gpio_bsrr, gpio_lckr, gpio_afrl, gpio_afrh, gpio_brr };


enum { rcc_cr = 0, rcc_icscr, rcc_cfgr, rcc_cir, rcc_ahbrstr, rcc_apb2rstr, rcc_apb1rstr,
	rcc_ahbenr, rcc_apb2enr, rcc_apb1enr, rcc_ahblpenr, rcc_apb2lpenr, rcc_apb1lpenr, rcc_csr };


enum { exti_imr = 0, exti_emr, exti_rtsr, exti_ftsr, exti_swier, exti_pr };


enum { syscfg_memrmp = 0, syscfg_pmc, syscfg_exticr };


static void gpiodrv_config(int port, u8 pin, u8 mode, u8 af, u8 otype, u8 ospeed, u8 pupd)
{
	volatile u32 *base = gpiodrv_common.gpio[port];
	u32 t;

	/* Enable clock */
	*(gpiodrv_common.rcc + rcc_ahbenr) |= 1 << port2ix[port];
	__asm__ volatile ("dmb");

	t = *(base + gpio_moder) & ~(0x3 << (pin << 1));
	*(base + gpio_moder) = t | (mode & 0x3) << (pin << 1);

	t = *(base + gpio_otyper) & ~(1 << pin);
	*(base + gpio_otyper) = t | (otype & 1) << pin;

	t = *(base + gpio_ospeedr) & ~(0x3 << (pin << 1));
	*(base + gpio_ospeedr) = t | (ospeed & 0x3) << (pin << 1);

	t = *(base + gpio_pupdr) & ~(0x03 << (pin << 1));
	*(base + gpio_pupdr) = t | (pupd & 0x3) << (pin << 1);

	if (pin < 8) {
		t = *(base + gpio_afrl) & ~(0xf << (pin << 2));
		*(base + gpio_afrl) = t | (af & 0xf) << (pin << 2);
	}
	else {
		t = *(base + gpio_afrh) & ~(0xf << ((pin - 8) << 2));
		*(base + gpio_afrh) = t | (af & 0xf) << ((pin - 8) << 2);
	}
}


static void gpiodrv_stox(char *buff, unsigned short x)
{
	int i;
	const char hex[] = {'0', '1', '2', '3', '4', '5', '6', '7',
			    '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };

	for (i = 0; i < 4; ++i)
		buff[i] = hex[(x >> (4 * (3 - i))) & 0xf];
}


static unsigned short gpiodrv_xtos(char **ptr)
{
	unsigned short x = 0;

	while (**ptr == ' ')
		++*ptr;

	if (**ptr == '0' && (*(*ptr + 1) == 'x' || *(*ptr + 1) == 'X'))
		*ptr += 2;

	while (**ptr != '\0') {
		if (**ptr >= 'a' && **ptr <= 'f')
			x = (x << 4) | (**ptr - 'a' + 10);
		else if (**ptr >= 'A' && **ptr <= 'F')
			x = (x << 4) | (**ptr - 'A' + 10);
		else if (**ptr >= '0' && **ptr <= '9')
			x = (x << 4) | (**ptr - '0');
		else
			break;
		++*ptr;
	}

	return x;
}


static void gpiodrv_uwait(unsigned time)
{
	volatile unsigned i;

	for (i = 0; i < time; ++i);
}


int main(void)
{
	msghdr_t hdr;
	union {
		char buff[41];
		gpiomsg_t devctl[8];
		gpiodrv_data_t data;
	} msg;

	int resps[8], i, len, err;

	char *tmpptr;
	unsigned short val, mask;
	unsigned msgsz, port, pin, mode, af, otype, ospeed, pupd, tmp, state;
	volatile unsigned *ptr;

	portCreate(&gpiodrv_common.id);
	portRegister(gpiodrv_common.id, "/gpiodrv");

	gpiodrv_common.rcc = (void *)0x40023800;
	gpiodrv_common.exti = (void *)0x40010400;
	gpiodrv_common.syscfg = (void *)0x40010000;

	for (i = 0; i < 8; ++i)
		gpiodrv_common.gpio[ix2port[i]] = (void *)(0x40020000 + i * 0x400);

	for (;;) {
		err = EOK;
		msgsz = recv(gpiodrv_common.id, &msg, sizeof(msg), &hdr);

		if (hdr.type != NORMAL)
			continue;

		switch (hdr.op) {
		case READ: {
			for (i = 0; i < 8; ++i) {
				val = *(u16*)(gpiodrv_common.gpio[i] + gpio_idr);
				gpiodrv_stox(msg.buff + 5 * i, val);
				msg.buff[5 * i + 4] = ' ';
			}

			msg.buff[39] = '\n';
			msg.buff[40] = '\0';

			respond(gpiodrv_common.id, EOK, msg.buff, 41);

			break;
		}

		case WRITE: {
			tmpptr = msg.data.buff;

			for (; *tmpptr == ' ' && *tmpptr != '\0'; ++tmpptr);

			if (*tmpptr == '\0') {
				respond(gpiodrv_common.id, -EINVAL, NULL, 0);
				break;
			}

			if (!strcmp("SET ", tmpptr) || !strcmp("set ", tmpptr)) {
				tmpptr += 4;
				port = gpiodrv_xtos(&tmpptr);
				mask = gpiodrv_xtos(&tmpptr);
				val = gpiodrv_xtos(&tmpptr);

				*(gpiodrv_common.gpio[port & 7] + gpio_bsrr) = ((unsigned)(~val & mask) << 16) | (val & mask);
			}
			else if (!strcmp("DEF ", tmpptr) || !strcmp("def ", tmpptr)) {
				tmpptr += 4;
				port = gpiodrv_xtos(&tmpptr);
				pin = gpiodrv_xtos(&tmpptr);
				mode = gpiodrv_xtos(&tmpptr);
				af = gpiodrv_xtos(&tmpptr);
				otype = gpiodrv_xtos(&tmpptr);
				ospeed = gpiodrv_xtos(&tmpptr);
				pupd = gpiodrv_xtos(&tmpptr);

				gpiodrv_config(port, pin, mode, af, otype, ospeed, pupd);
			}

			respond(gpiodrv_common.id, EOK, NULL, 0);
			break;
		}

		case DEVCTL: {
			if (msgsz < sizeof(gpiomsg_t)) {
				respond(gpiodrv_common.id, -EINVAL, NULL, 0);
				break;
			}

			len = msgsz / sizeof(gpiomsg_t);

			for (i = 0; i < len && err == EOK; ++i) {
				resps[i] = 0;
				port = msg.devctl[i].port;

				if (port > 7) {
					err = -EINVAL;
					break;
				}

				switch (msg.devctl[i].type) {
				case GPIO_GET:
					resps[i] = *(u16*)(gpiodrv_common.gpio[port] + gpio_idr);
					break;

				case GPIO_SET:
					val = msg.devctl[i].set.state;
					mask = msg.devctl[i].set.mask;
					*(gpiodrv_common.gpio[port] + gpio_bsrr) = ((unsigned)(~val & mask) << 16) | (val & mask);
					break;

				case GPIO_CONFIG:
					gpiodrv_config(port, msg.devctl[i].config.pin, msg.devctl[i].config.mode, msg.devctl[i].config.af, msg.devctl[i].config.otype, msg.devctl[i].config.ospeed, msg.devctl[i].config.pupd);
					break;

				case GPIO_INTERRUPT:
					pin = msg.devctl[i].interrupt.pin;
					state = msg.devctl[i].interrupt.state;

					/* Enable clock */
					*(gpiodrv_common.rcc + rcc_ahbenr) |= 1 << port2ix[port];
					__asm__ volatile ("dmb");

					/* Exti line config */
					tmp = ((u32)0x0f << (0x04 * (pin & (u8)0x03)));
					(gpiodrv_common.syscfg + syscfg_exticr)[pin >> 0x02] &= ~tmp;
					(gpiodrv_common.syscfg + syscfg_exticr)[pin >> 0x02] |= ((u32)port) << (0x04 * (pin & (u8)0x03));

					/* Exti mask interrupt */
					*(gpiodrv_common.exti + exti_imr) &= ~(!state << pin);
					*(gpiodrv_common.exti + exti_imr) |= !!state << pin;

					/* Exti set trigger */
					if (msg.devctl[i].interrupt.edge)
						ptr = gpiodrv_common.exti + exti_rtsr;
					else
						ptr = gpiodrv_common.exti + exti_ftsr;

					*ptr &= ~(!state << pin);
					*ptr |= !!state << pin;

					break;

				case GPIO_DELAY:
					gpiodrv_uwait(msg.devctl[i].delay.len);
					break;

				default:
					err = -EINVAL;
					break;
				}
			}

			if (err == EOK)
				respond(gpiodrv_common.id, EOK, resps, len * sizeof(resps[0]));
			else
				respond(gpiodrv_common.id, -EINVAL, NULL, 0);

			break;
		}

		default:
			respond(gpiodrv_common.id, -EINVAL, NULL, 0);
			break;
		}
	}
}
