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

#include HAL
#include "gpiodrv.h"
#include "proc/threads.h"
#include "proc/msg.h"
#include "lib/lib.h"


/* Temporary solution */
unsigned int gpiodrv_id;


struct {
	volatile unsigned int *rcc;
	volatile unsigned int *gpio[8];
} gpiodrv_common;


enum { gpio_moder = 0, gpio_otyper, gpio_ospeedr, gpio_pupdr, gpio_idr,
	gpio_odr, gpio_bsrr, gpio_lckr, gpio_afrl, gpio_afrh, gpio_brr };


enum { rcc_cr = 0, rcc_icscr, rcc_cfgr, rcc_cir, rcc_ahbrstr, rcc_apb2rstr, rcc_apb1rstr,
	rcc_ahbenr, rcc_apb2enr, rcc_apb1enr, rcc_ahblpenr, rcc_apb2lpenr, rcc_apb1lpenr, rcc_csr };


static void gpiodrv_setPort(int port, unsigned int mask, unsigned int val)
{
	unsigned int t;

	t = *(gpiodrv_common.gpio[port & 7] + gpio_odr) & ~(~val & mask);
	t |= val & mask;
	*(gpiodrv_common.gpio[port & 7] + gpio_odr) = t & 0xffff;
}


void gpiodrv_configPin(int port, char pin, char mode, char af, char otype, char ospeed, char pupd)
{
	volatile unsigned int *base = gpiodrv_common.gpio[port & 7];
	unsigned int t;

	/* Enable GPIO port's clock */
	if (port == 5 || port == 6)
		*(gpiodrv_common.rcc + rcc_ahbenr) |= 1 << (port + 1);
	else if (port == 7)
		*(gpiodrv_common.rcc + rcc_ahbenr) |= 1 << 5;
	else
		*(gpiodrv_common.rcc + rcc_ahbenr) |= 1 << (port & 7);

	hal_cpuDataBarrier();

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
	} else {
		t = *(base + gpio_afrh) & ~(0xf << ((pin - 8) << 2));
		*(base + gpio_afrh) = t | (af & 0xf) << ((pin - 8) << 2);
	}
}


extern void gpiodrv_configInterrupt(int port, char pin, char state, char edge)
{
	/* enable syscfg clock */
	*(gpiodrv_common.rcc + rcc_apb2enr) |= 1;
	hal_interruptsSetGpioInterrupt(port, pin, state, edge);
}


static void stox(char *buff, unsigned short x)
{
	char hex[] = {'0', '1', '2', '3', '4', '5', '6', '7',
		'8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };

	for (int i = 0; i < 4; ++i) {
		buff[i] = hex[(x >> (4 * (3 - i))) & 0xf];
	}
}


static unsigned short xtos(char **ptr)
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


static int strcmp(const char *x, const char *y)
{
	for (; *x != '\0' && *y != '\0'; ++x, ++y) {
		if (*x < *y)
			return -1;
		else if (*x > *y)
			return 1;
	}

	return 0;
}


static void gpiodrv_uwait(int time)
{
	volatile int i;

	for (i = 0; i < time; ++i);
}


static void gpiodrv_thread(void *arg)
{
	union {
		char buff[41];
		gpiomsg_t devctl[8];
		gpiodrv_data_t data;
	} msg;

	int resps[8];
	int i, len;
	unsigned int msgsz;
	msghdr_t hdr;

	for (;;) {
		msgsz = proc_recv(gpiodrv_id, &msg, sizeof(msg), &hdr);

		switch (hdr.op) {
			case MSG_READ: {
				for (i = 0; i < 8; ) {
					stox(&(msg.buff[5 * i]), *(gpiodrv_common.gpio[i] + gpio_idr));
					msg.buff[(5 * ++i) - 1] = ' ';
				}

				msg.buff[39] = '\n';
				msg.buff[40] = '\0';

				if (hdr.type != MSG_NOTIFY)
					proc_respond(gpiodrv_id, EOK, msg.buff, 41);

				break;
			}

			case MSG_WRITE: {
				char *tmpptr = msg.data.buff;

				for (; *tmpptr == ' ' && *tmpptr != '\0'; ++tmpptr);

				if (*tmpptr == '\0') {
					if (hdr.type != MSG_NOTIFY)
						proc_respond(gpiodrv_id, EINVAL, NULL, 0);

					break;
				}

				if (!strcmp("SET ", tmpptr) || !strcmp("set ", tmpptr)) {
					unsigned int port, mask, val;

					tmpptr += 4;
					port = xtos(&tmpptr);
					mask = xtos(&tmpptr);
					val = xtos(&tmpptr);

					gpiodrv_setPort(port, mask, val);
				}
				else if (!strcmp("DEF ", tmpptr) || !strcmp("def ", tmpptr)) {
					unsigned int port, pin, mode, af, otype, ospeed, pupd;

					tmpptr += 4;
					port = xtos(&tmpptr);
					pin = xtos(&tmpptr) & 0xf;
					mode = xtos(&tmpptr) & 0x3;
					af = xtos(&tmpptr) & 0xf;
					otype = xtos(&tmpptr) & 0x3;
					ospeed = xtos(&tmpptr) & 0x3;
					pupd = xtos(&tmpptr) & 0x3;

					gpiodrv_configPin(port, pin, mode, af, otype, ospeed, pupd);
				}

				if (hdr.type != MSG_NOTIFY)
					proc_respond(gpiodrv_id, EOK, NULL, 0);

				break;
			}

			case MSG_DEVCTL: {
				if (msgsz < sizeof(gpiomsg_t)) {
					if (hdr.type != MSG_NOTIFY)
						proc_respond(gpiodrv_id, EINVAL, NULL, 0);
					break;
				}
				else if (msgsz > sizeof(gpiomsg_t)) {
					len = msgsz / sizeof(gpiomsg_t);
				}
				else {
					len = 1;
				}

				for (i = 0; i < len; ++i) {
					switch (msg.devctl[i].type) {
						case GPIO_GET:
							resps[i] = *(gpiodrv_common.gpio[msg.devctl[i].port & 7] + gpio_idr) & 0xffff;

							break;

						case GPIO_SET:
							gpiodrv_setPort(msg.devctl[i].port, msg.devctl[i].set.mask, msg.devctl[i].set.state);
							resps[i] = 0;

							break;

						case GPIO_CONFIG:
							gpiodrv_configPin(msg.devctl[i].port, msg.devctl[i].config.pin, msg.devctl[i].config.mode,
								msg.devctl[i].config.af, msg.devctl[i].config.otype, msg.devctl[i].config.ospeed, msg.devctl[i].config.pupd);
							resps[i] = 0;

							break;

						case GPIO_INTERRUPT:
							gpiodrv_configInterrupt(msg.devctl[i].port, msg.devctl[i].interrupt.pin,
								msg.devctl[i].interrupt.state, msg.devctl[i].interrupt.edge);
							resps[i] = 0;

							break;

						case GPIO_DELAY:
							gpiodrv_uwait(msg.devctl[i].delay.len);
							resps[i] = 0;

							break;

						default:
							if (hdr.type != MSG_NOTIFY)
								proc_respond(gpiodrv_id, EINVAL, NULL, 0);

							break;
					}
				}

				if (hdr.type != MSG_NOTIFY)
					proc_respond(gpiodrv_id, EOK, resps, len * sizeof(resps[0]));

				break;
			}

			default: {
				if (hdr.type != MSG_NOTIFY)
					proc_respond(gpiodrv_id, EINVAL, NULL, 0);

				break;
			}
		}
	}
}


int gpio_sequence(gpiomsg_t *msg, unsigned int len, int *resp)
{
	return proc_send(gpiodrv_id, MSG_DEVCTL, msg, len * sizeof(gpiomsg_t), MSG_NORMAL, resp, len * sizeof(int));
}


int gpio_setPort(int port, int mask, int state)
{
	gpiomsg_t gpio;
	int resp;

	gpio.port = port;
	gpio.type = GPIO_SET;
	gpio.set.mask = mask;
	gpio.set.state = state;

	resp = proc_send(gpiodrv_id, MSG_DEVCTL, &gpio, sizeof(gpio), MSG_NORMAL, NULL, 0);

	return resp < 0 ? resp : 0;
}


int gpio_getPort(int port)
{
	gpiomsg_t gpio;
	int resp;

	gpio.port = port;
	gpio.type = GPIO_GET;
	proc_send(gpiodrv_id, MSG_DEVCTL, &gpio, sizeof(gpio), MSG_NORMAL, &resp, sizeof(resp));

	return resp;
}


int gpio_configPin(int port, char pin, char mode, char af, char ospeed, char otype, char pupd)
{
	gpiomsg_t gpio;
	int resp;

	gpio.port = port;
	gpio.type = GPIO_CONFIG;
	gpio.config.pin = pin;
	gpio.config.mode = mode;
	gpio.config.af = af;
	gpio.config.ospeed = ospeed;
	gpio.config.otype = otype;
	gpio.config.pupd = pupd;

	resp = proc_send(gpiodrv_id, MSG_DEVCTL, &gpio, sizeof(gpio), MSG_NORMAL, NULL, 0);

	return resp < 0 ? resp : 0;
}


void gpiodrv_init(void)
{
	gpiodrv_common.gpio[0] = (void *)0x40020000;
	gpiodrv_common.gpio[1] = (void *)0x40020400;
	gpiodrv_common.gpio[2] = (void *)0x40020800;
	gpiodrv_common.gpio[3] = (void *)0x40020c00;
	gpiodrv_common.gpio[4] = (void *)0x40021000;
	gpiodrv_common.gpio[5] = (void *)0x40021800;
	gpiodrv_common.gpio[6] = (void *)0x40021c00;
	gpiodrv_common.gpio[7] = (void *)0x40021400;

	gpiodrv_common.rcc = (void *)0x40023800;

	proc_portCreate(&gpiodrv_id);
	proc_portRegister(gpiodrv_id, "/gpiodrv");

	proc_threadCreate(NULL, gpiodrv_thread, 0, 1024, NULL, NULL);
}
