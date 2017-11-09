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
#ifndef _LCDDRV_H_
#define _LCDDRV_H_

#define LCDDRV_CHAR_DEGREE 0xf8

extern unsigned int lcddrv_port;

typedef enum _lcd_symbols_t {
	LCDSYM_SMALL_ONE	= 1 << 0,
	LCDSYM_CLOCK		= 1 << 1,
	LCDSYM_DATE			= 1 << 2,
	LCDSYM_WARN			= 1 << 3,
	LCDSYM_BATT			= 1 << 4,
	LCDSYM_LOWBATT		= 1 << 5,
	LCDSYM_NOBATT		= 1 << 6,
	LCDSYM_LOCK			= 1 << 7,
	LCDYSM_FACTORY		= 1 << 8,
	LCDSYM_FIRE			= 1 << 9,
	LCDSYM_COM			= 1 << 10,
	LCDSYM_POUND		= 1 << 11,
	LCDSYM_EUR			= 1 << 12,
	LCDSYM_M3			= 1 << 13,
	LCDSYM_SLASH_TOP	= 1 << 14,
	LCDSYM_H_TOP		= 1 << 15,
	LCDSYM_H_BOT		= 1 << 16,
	LCDSYM_K			= 1 << 17,
	LCDSYM_W			= 1 << 18,
	LCDSYM_FRAME 		= 1 << 19,
	LCDSYM_FRAME_SMALL	= 1 << 20,
	LCDSYM_BAR0 		= 1 << 21,
	LCDSYM_BAR1			= 1 << 22,
	LCDSYM_BAR2			= 1 << 23,
	LCDSYM_DOT1_BOT		= 1 << 24,
	LCDSYM_DOT2_TOP		= 1 << 25,
	LCDSYM_DOT2_BOT 	= 1 << 26,
	LCDSYM_DOT3_BOT		= 1 << 27,
	LCDSYM_DOT4_TOP		= 1 << 28,
	LCDSYM_DOT4_BOT		= 1 << 29,
	LCDSYM_TOTAL 		= 30
} lcd_symbols_t;

typedef struct _lcddrv_msg_t {
	enum { GET, SET, CONF } type;

	char str[10];
	char str_small[2];
	unsigned int sym_mask;
	unsigned char state;
	int backlight;
	int on;
} __attribute__((packed)) lcddrv_msg_t;

void lcddrv_init(void);


#endif

