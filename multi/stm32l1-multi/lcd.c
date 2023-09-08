/*
 * Phoenix-RTOS
 *
 * STM32L1 LCD driver
 *
 * Copyright 2017, 2018 Phoenix Systems
 * Author: Adrian Kepka, Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <string.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include "common.h"
#include "config.h"
#include "stm32l1-multi.h"
#include "gpio.h"
#include "lcd.h"
#include "rcc.h"

#if LCD

#define LCD_MAX_POSITION 10


enum { cr = 0, fcr, sr, clr, ram = 5 };


enum { COM1 = 2, COM2 = 0, COM3 = 4, COM4 = 6 };


struct {
	volatile unsigned int *base;

	char str[10];
	char str_small[3];
	unsigned int sym_mask;
	unsigned char on;

	handle_t lock;
	handle_t cond;
} lcd_common;


/* Numbers LUT from '0' to '9' */
static const unsigned char numbers[10] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };


/* Characters LUT for lower case letters from 'a' to 'z' */
static const unsigned char lowercase_letters[] = { 0x5F, 0x7C, 0x58, 0x5E, 0x7B, 0x71, 0x6F, 0x74, 0x04,
	0x0E, 0x76, 0x38, 0x54, 0x54, 0x5C, 0x73, 0x67, 0x50, 0x6D, 0x78, 0x1C, 0x1C, 0x1C, 0x76, 0x6E, 0x6B };


/* Characters LUT for upper case letters from 'A' to 'Z' */
static const unsigned char uppercase_letters[] = { 0x77, 0x7F, 0x39, 0x3F, 0x79, 0x71, 0x7D, 0x76, 0x06,
	0x0E, 0x76, 0x38, 0x37, 0x37, 0x3F, 0x73, 0x3F, 0x77, 0x6D, 0x78, 0x3E, 0x3E, 0x3E, 0x76, 0x6E, 0x6B };


static const unsigned char char_dash = 0x40;
static const unsigned char char_underscore = 0x08;
static const unsigned char char_questionmark = 0x53;
static const unsigned char char_degree = 0x63;


static const unsigned char symbols[LCDSYM_TOTAL][2] = {
	{ COM1, 27 },    /* LCDSYM_SMALL_ONE */
	{ COM1, 23 },    /* LCDSYM_CLOCK */
	{ COM1, 19 },    /* LCDSYM_DATE */
	{ COM1, 17 },    /* LCDSYM_WARN */
	{ COM2, 16 },    /* LCDSYM_BATT */
	{ COM4, 16 },    /* LCDSYM_LOWBATT */
	{ COM3, 16 },    /* LCDSYM_NOBATT */
	{ COM1, 16 },    /* LCDSYM_LOCK */
	{ COM1, 25 },    /* LCDYSM_FACTORY */
	{ COM1, 9 },     /* LCDSYM_FIRE */
	{ COM1, 7 },     /* LCDSYM_COM */
	{ COM1, 6 },     /* LCDSYM_POUND */
	{ COM1, 5 },     /* LCDSYM_EUR */
	{ COM2, 6 },     /* LCDSYM_M3 */
	{ COM2, 5 },     /* LCDSYM_SLASH_TOP */
	{ COM3, 5 },     /* LCDSYM_H_TOP */
	{ COM4, 5 },     /* LCDSYM_H_BOT */
	{ COM3, 6 },     /* LCDSYM_K */
	{ COM4, 6 },     /* LCDSYM_W */
	{ COM2, 7 },     /* LCDSYM_FRAME */
	{ COM4, 29 },    /* LCDSYM_FRAME_SMALL */
	{ COM1, 29 },    /* LCDSYM_BAR0 */
	{ COM2, 29 },    /* LCDSYM_BAR1 */
	{ COM3, 29 },    /* LCDSYM_BAR2 */
	{ COM1, 21 },    /* LCDSYM_DOT1_BOT */
	{ COM4, 15 },    /* LCDSYM_DOT2_TOP */
	{ COM4, 7 },     /* LCDSYM_DOT2_BOT */
	{ COM4, 13 },    /* LCDSYM_DOT3_BOT */
	{ COM4, 11 },    /* LCDSYM_DOT4_TOP */
	{ COM3, 7 },     /* LCDSYM_DOT4_BOT */
};


static const unsigned char pin_to_ram[30] = {
	0, 0, 0, 0, 0, 37, 43, 42, 27, 26, 25, 24, 35, 34, 33, 32,
	31, 30, 28, 15, 14, 13, 12, 4, 38, 39, 18, 19, 20, 21
};


static const unsigned char pin_map[11][7] = {
	{ 0,  0,  0,  0,  0,  0,  0 },
	{ 28, 27, 27, 28, 28, 28, 27 },
	{ 26, 25, 25, 26, 26, 26, 25 },
	{ 24, 23, 23, 24, 24, 24, 23 },
	{ 22, 21, 21, 22, 22, 22, 21 },
	{ 20, 19, 19, 20, 20, 20, 19 },
	{ 18, 17, 17, 18, 18, 18, 17 },
	{ 14, 14, 14, 14, 15, 15, 15 },
	{ 12, 12, 12, 12, 13, 13, 13 },
	{ 10, 10, 10, 10, 11, 11, 11 },
	{ 8,  8,  8,  8,  9,  9,  9 }
};


static const unsigned char com_map[11][7] = {
	{ 0,    0,    0,    0,    0,    0,    0 },
	{ COM1, COM2, COM4, COM4, COM3, COM2, COM3 },
	{ COM1, COM2, COM4, COM4, COM3, COM2, COM3 },
	{ COM1, COM2, COM4, COM4, COM3, COM2, COM3 },
	{ COM1, COM2, COM4, COM4, COM3, COM2, COM3 },
	{ COM1, COM2, COM4, COM4, COM3, COM2, COM3 },
	{ COM1, COM2, COM4, COM4, COM3, COM2, COM3 },
	{ COM1, COM2, COM3, COM4, COM3, COM1, COM2 },
	{ COM1, COM2, COM3, COM4, COM3, COM1, COM2 },
	{ COM1, COM2, COM3, COM4, COM3, COM1, COM2 },
	{ COM1, COM2, COM3, COM4, COM4, COM2, COM3 }
};


static const signed char gpio_pins[5][9] = {
	{ 7, 8,  9,  10, -1, -1, -1, -1, -1 },
	{ 9, 12, 13, 14, 15, -1, -1, -1, -1 },
	{ 0, 1,  2,  3,  6,  7,  8,  9,  12 },
	{ 2, 8,  10, 11, 12, 13, 14, 15, -1 },
	{ 1, 2,  3,  -1, -1, -1, -1, -1, -1 }
};


static int lcd_irqHandler(unsigned int n, void *arg)
{
	/* Turn off interrupt */
	*(lcd_common.base + fcr) &= ~(1 << 3);

	return 1;
}


static inline void _lcd_setRamSegment(unsigned int com, unsigned int pin, unsigned int on)
{
	if (pin > 31)
		++com;

	(lcd_common.base + ram)[com] &= ~(!on << pin % 32);
	(lcd_common.base + ram)[com] |= (!!on << pin % 32);
}


static void _lcd_showChar(char ch, unsigned int pos)
{
	unsigned char segment = 0;
	unsigned int i, on;

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

	for (i = 0; i < 7; i++) {
		on = segment & (unsigned char)(1 << i);
		_lcd_setRamSegment(com_map[pos][i], pin_to_ram[pin_map[pos][i]], on);
	}
}


static void _lcd_update(void)
{
	if (lcd_common.on) {
		/* Request update */
		*(lcd_common.base + fcr) |= 1 << 3;
		*(lcd_common.base + sr) |= 0x04;
		/* Wait for update done */
		while (!(*(lcd_common.base + sr) & 0x08))
			condWait(lcd_common.cond, lcd_common.lock, 0);

		*(lcd_common.base + clr) |= 0x08;
	}
}


static void _lcd_showString(const char *text)
{
	unsigned int i, start, len = 0;

	/* Clear string */
	for (i = 2; i <= LCD_MAX_POSITION; i++)
		_lcd_showChar(' ', i);

	if ((len = strlen(text)) >= LCD_MAX_POSITION - 1)
		len = LCD_MAX_POSITION - 1;

	start = LCD_MAX_POSITION - len + 1;
	for (i = start; i <= LCD_MAX_POSITION; i++)
		_lcd_showChar(text[i - start], i);

	memcpy(lcd_common.str, text, len);
	lcd_common.str[len] = '\0';
}


static void _lcd_showSymbols(unsigned int sym_mask)
{
	unsigned int i, symbol;

	if (lcd_common.sym_mask & LCDSYM_SMALL_ONE)
		sym_mask |= LCDSYM_SMALL_ONE;

	for (i = 0; i < LCDSYM_TOTAL; i++) {
		symbol = sym_mask & (1 << i);

		_lcd_setRamSegment(symbols[i][0], pin_to_ram[symbols[i][1]], !!symbol);
	}

	lcd_common.sym_mask = sym_mask;
}


static void _lcd_showSmallString(const char *text)
{
	if (text[0] == '1')
		lcd_common.sym_mask |= LCDSYM_SMALL_ONE;
	else
		lcd_common.sym_mask &= ~LCDSYM_SMALL_ONE;

	_lcd_showSymbols(lcd_common.sym_mask);

	_lcd_showChar(text[1], 1);

	memcpy(lcd_common.str_small, text, sizeof(lcd_common.str_small) - 1);
	lcd_common.str_small[sizeof(lcd_common.str_small) - 1] = '\0';
}


static void _lcd_enable(int on)
{
	on = !on;

	lcd_common.on = !on;
	*(lcd_common.base + cr) &= ~on;
	*(lcd_common.base + cr) |= !on;
}


void lcd_getDisplay(lcdmsg_t *disp)
{
	mutexLock(lcd_common.lock);

	memcpy(disp->str, lcd_common.str, sizeof(lcd_common.str));
	memcpy(disp->str_small, lcd_common.str_small, sizeof(disp->str_small));
	disp->sym_mask = lcd_common.sym_mask;
	disp->on = lcd_common.on;

	mutexUnlock(lcd_common.lock);
}


void lcd_setDisplay(lcdmsg_t *disp)
{
	mutexLock(lcd_common.lock);

	if (disp->str[0] != '\0')
		_lcd_showString(disp->str);

	_lcd_showSymbols(disp->sym_mask);

	if (disp->str_small[0] != '\0')
		_lcd_showSmallString(disp->str_small);

	_lcd_enable(disp->on);
	_lcd_update();

	mutexUnlock(lcd_common.lock);
}
#endif


int lcd_init(void)
{
#if LCD
	int port, pin;

	lcd_common.base = (void *)0x40002400;
	lcd_common.str[0] = 0;
	lcd_common.sym_mask = 0;
	lcd_common.on = 0;

	/* mask all needed registers */
	*(lcd_common.base + cr) &= 0xffffff81;
	*(lcd_common.base + fcr) &= 0xfc03ffff;

	/* enable LCD clock */
	rcc_devClk(pctl_lcd, 1);

	*(lcd_common.base + cr) |= 0x03 << 2;   /* DUTY = 1/4 */
	*(lcd_common.base + fcr) |= 0x0a << 18; /* DIV  = 16  */
	*(lcd_common.base + fcr) |= 0x02 << 22; /* PS   = 4   */
	*(lcd_common.base + fcr) |= 0x06 << 10; /* CC   = 6   */
	*(lcd_common.base + fcr) |= 0x02 << 7;  /* DEAD = 2   */
	*(lcd_common.base + fcr) |= 0x03 << 4;  /* PON  = 3   */

	/* wait for FCR register sync */
	while (!(*(lcd_common.base + sr) & 0x20));

	/* clear RAM */
	memset((void *)(lcd_common.base + ram), 0, 8);

	/* Init gpio pins */
	for (port = 0; port < 5; port++) {
		for (pin = 0; pin < 9; pin++) {
			if (gpio_pins[port][pin] == -1)
				break;

			gpio_configPin(port + gpioa, gpio_pins[port][pin], 2, 0xb, 0, 1, 0);
		}
	}

	mutexCreate(&lcd_common.lock);
	condCreate(&lcd_common.cond);

	interrupt(lcd_irq, lcd_irqHandler, NULL, lcd_common.cond, NULL);
#endif

	return EOK;
}
