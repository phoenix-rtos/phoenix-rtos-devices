/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Tiramisu LCD driver implementation
 *
 * Copyright 2017 Phoenix Systems
 * Author: Adrian Kepka
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#include ARCH

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>

#include <sys/msg.h>
#include <sys/threads.h>

#include "gpiodrv.h"
#include "lcddrv.h"


enum stm32_registers {
	ahbenr = 7, apb1enr = 9
};


struct {
	volatile unsigned int *rcc;
	volatile unsigned int *base;

	unsigned port, gpiodrv_id;

	char str[sizeof(((lcddrv_msg_t*)NULL)->str)];
	char str_small[sizeof(((lcddrv_msg_t*)NULL)->str_small)];

	unsigned char on, backlight;

	u64 sym_state;
} lcd_common;


enum lcd_registers {
	cr = 0, fcr, sr, clr, ram = 5
};


typedef enum _lcd_coms_t {
	COM0 = 2 * 3,
	COM1 = 2 * 0,
	COM2 = 2 * 1,
	COM3 = 2 * 2
} lcd_coms_t;


enum { SEGA = 0, SEGF, SEGE, SEGD, SEGB, SEGG, SEGC, SEGP };


/*
 * SmartEMU LCD Segments are mapped like this:
 *   --A--
 *  |     |
 *  F     B
 *  |     |
 *   --G--
 *  |     |
 *  E     C
 *  |     |
 *   --D--
 *
 *  Each byte in below maps corresponds to segments: 0GFEDCBA
 */


/* 0-9 */
static const unsigned char numbers[10] = {
	0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};


/* not possible: k, m, v, w, x */
static const unsigned char lowercase_letters[] = {
	/* a     b     c     d     e     f     g     h     i     j     k     l     m */
	0x5F, 0x7C, 0x58, 0x5E, 0x7B, 0x71, 0x6F, 0x74, 0x04, 0x0E, 0x76, 0x38, 0x54,
	/* n     o     p     q     r     s     t     u     v     w     x     y     z */
	0x54, 0x5C, 0x73, 0x67, 0x50, 0x6D, 0x78, 0x1C, 0x1C, 0x1C, 0x76, 0x6E, 0x6B
};


/* not possible: K, M, Q, T, V, W, X */
static const unsigned char uppercase_letters[] = {
	/* A     B     C     D     E     F     G     H     I     J     K     L     M */
	0x77, 0x7F, 0x39, 0x3F, 0x79, 0x71, 0x7D, 0x76, 0x06, 0x0E, 0x76, 0x38, 0x37,
	/* N     O     P     Q     R     S     T     U     V     W     X     Y     Z */
	0x37, 0x3F, 0x73, 0x3F, 0x77, 0x6D, 0x78, 0x3E, 0x3E, 0x3E, 0x76, 0x6E,	0x6B
};


static const unsigned char char_dash = 0x40;
static const unsigned char char_underscore = 0x08;
static const unsigned char char_questionmark = 0x53;
static const unsigned char char_degree = 0x63;


void lcddrv_update(void)
{
	if(lcd_common.on == 0)
		return;

	/* request update */
	*(lcd_common.base + sr) |= 0x04;
	/* wait for sync */
	while (!(*(lcd_common.base + sr) & 0x08));

	*(lcd_common.base + clr) |= 0x08;
}



void lcddrv_setRamSegment(unsigned int position, unsigned int segment, unsigned int on)
{
	static const char routes[36] = {
		2, 3, 4, 22, 23, 5, 6, 10, 11, 12, 13, 14, 15, 30, 31, 32, 33, 34, 35,
		24, 25, 26, 27, 17, 16, 9, 8, 7, 36, 37, 38, 39, 40, 41, 42, 43 };
//		24, 25, 26, 27, 17, 40, 41, 42, 43, 7, 8, 9, 16, 36, 37, 38, 39 };

	static const char coms[4] = { COM3, COM2, COM1, COM0 };

	int pin = routes[2 * position + segment / 4];
	int com = coms[segment % 4];

	if (pin > 31)
		com++;

	if (on)
		(lcd_common.base + ram)[com] |= (1 << (pin % 32));
	else
		(lcd_common.base + ram)[com] &= ~(1 << (pin % 32));
}


void lcddrv_showChar(char ch, unsigned int pos)
{
	static const char segments[] = { SEGA, SEGB, SEGC, SEGD, SEGE, SEGF, SEGG, SEGP };

	unsigned char segment = 0;
	unsigned int i;

	if (ch >= '0' && ch <= '9')
		segment = numbers[ch - '0'];
	else if (ch >= 'a' && ch <= 'z')
		segment = lowercase_letters[ch - 'a'];
	else if (ch >= 'A' && ch <= 'Z')
		segment = uppercase_letters[ch - 'A'];
	else if (ch == '-')
		segment = char_dash;
	else if (ch == '_')
		segment = char_underscore;
	else if (ch == '?')
		segment = char_questionmark;
	else if (ch == LCDDRV_CHAR_DEGREE)
		segment = char_degree;

	for (i = 0; i < 7; i++)
		lcddrv_setRamSegment(pos, segments[i], segment & (1 << i));
}


void lcddrv_showString(const char *text, int sz, int offs)
{
	unsigned int i;

	for (i = 0; i < sz && text[i] != 0; i++) {
		if (text[i] != '#')
			lcddrv_showChar(lcd_common.str[i] = text[i], i + offs);
	}

	for (; i < sz; i++)
		lcddrv_showChar(lcd_common.str[i] = ' ', i + offs);
}


void lcddrv_showSymbol(u64 mask, u64 state)
{
	unsigned int pos, seg, change, on;
	int i;

	state &= LCDSYM_ALL;

	for (i = 0; i < LCDSYM_TOTAL; i++) {
		change = (mask >> i) & 1;
		on = (state >> i) & 1;

		if (!change)
			continue;

		if (i < 14) {
			pos = i;
			seg = SEGP;
		}
		else {
			pos = 14 + (i - 14) / 8;
			seg = i % 8;
		}

		lcddrv_setRamSegment(pos, seg, on);
	}

	lcd_common.sym_state &= ~mask;
	lcd_common.sym_state |= mask & state;
}


void lcddrv_enable(int on)
{
	lcd_common.on = on;

	if (on)
		*(lcd_common.base + cr) |= 1;
	else
		*(lcd_common.base + cr) &= ~1;
}


void lcddrv_initGpioPins(void)
{
	unsigned char port, i = 0;
	gpiomsg_t gpio;
	int gpio_res;

	static const char gpio_pins[] = {
		/* PORTA */
		3, 6, 7, 8, 9, 10, 15, -1,
		/* PORTB */
		0, 1, 3, 4, 5, 8, 9, 10, 11, 12, 13, 14, 15, -1,
		/* PORTC */
		4, 5, 6, 7, 8, 9, 10, 11, 12, -1,
		/* PORTD */
		2, 10, 11, 12, 13, 14, 15, -1,
		/* PORTE */
		0, 1, 2, 3, -1,
	};

	gpio.type = GPIO_CONFIG;
	gpio.config.otype = 0;
	gpio.config.pupd = 0;
	gpio.config.mode = 2;
	gpio.config.af = 11;
	gpio.config.ospeed = 1;

	for (port = 0; port < 5; i++) {
		if (gpio_pins[i] == 0xff) {
			port++;
			continue;
		}

		gpio.port = port;
		gpio.config.pin = gpio_pins[i];

		send(lcd_common.gpiodrv_id, DEVCTL, &gpio, sizeof(gpio), NORMAL, &gpio_res, sizeof(gpio_res));
	}
}


static void lcddrv_mainThread(void)
{
	msghdr_t hdr;
	lcddrv_msg_t msg;

	for (;;) {
		recv(lcd_common.port, &msg, sizeof(msg), &hdr, 0);

		if (hdr.op != DEVCTL)
			continue;

		switch (msg.type) {
		case LCD_SET:
			if (*msg.str != 0)
				lcddrv_showString(msg.str, sizeof(lcd_common.str), sizeof(lcd_common.str_small));

			if (*msg.str_small != 0)
				lcddrv_showString(msg.str_small, sizeof(lcd_common.str_small), 0);

			lcddrv_showSymbol(msg.sym_mask, msg.sym_state);

			lcddrv_enable(msg.on);
			lcddrv_update();
			/* fall through, cause we always return actual LCD state */
		case LCD_GET:
			memcpy(msg.str, lcd_common.str, sizeof(lcd_common.str));
			memcpy(msg.str_small, lcd_common.str_small, sizeof(lcd_common.str_small));
			msg.sym_mask = lcd_common.sym_state;
			msg.backlight = lcd_common.backlight;
			msg.on = lcd_common.on;
			respond(lcd_common.port, EOK, &msg, sizeof(msg));
			break;
		}
	}
}


void main(void)
{
	lcd_common.rcc = (void *)0x40023800;

	lcd_common.base = (void *)0x40002400;
	lcd_common.str[0] = 0;
	lcd_common.sym_state = 0;
	lcd_common.backlight = 0;
	lcd_common.on = 0;

	/* mask all needed registers */
	*(lcd_common.base + cr) &= 0xFFFFFF01;
	*(lcd_common.base + fcr) &= 0xFC03FFFF;

	/* enable LCD clock */
	*(lcd_common.rcc + apb1enr) |= 0x0200;

	__asm__ volatile ("dmb");

	*(lcd_common.base + cr)  |= 0x3 << 2;  /* DUTY = 1/4 */
	*(lcd_common.base + fcr) |= 0xA << 18; /* DIV  = 16  */
	*(lcd_common.base + fcr) |= 0x2 << 22; /* PS   = 4   */
	*(lcd_common.base + fcr) |= 0x6 << 10; /* CC   = 6   */
	*(lcd_common.base + fcr) |= 0x2 << 7;  /* DEAD = 2   */
	*(lcd_common.base + fcr) |= 0x3 << 4;  /* PON  = 3   */

	/* wait for FCR register sync */
	while (!(*(lcd_common.base + sr) & 0x20));

	/* clear RAM */
	memset((void *)(lcd_common.base + ram), 0, 8);

	portCreate(&lcd_common.port);
	portRegister(lcd_common.port, "/lcddrv");

	while (lookup("/gpiodrv", &lcd_common.gpiodrv_id) != EOK)
		usleep(100000);

	lcddrv_initGpioPins();
	lcddrv_mainThread();
}
