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
#include HAL
#include "lcddrv.h"
#include "proc/proc.h"
#include "gpiodrv.h"

#define LCD_MAX_POSITION 10

enum stm32_registers {
	ahbenr = 7, apb1enr = 9
};

struct {
	volatile unsigned int *rcc;
	volatile unsigned int *base;

	char str[10];
	char str_small[3];
	unsigned int sym_mask;
	unsigned char backlight;
	unsigned char on;
} lcd_common;

enum lcd_registers {
	cr = 0, fcr, sr, clr, ram = 5
};

unsigned int lcddrv_port;

typedef enum _lcd_coms_t {
	COM1 = 2,
	COM2 = 0,
	COM3 = 4,
	COM4 = 6
} lcd_coms_t;

/*
 * Tiramisu LCD Segments are mapped like this:
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
static const unsigned char numbers[10] = {
		0x3F, /* 0 */
		0x06, /* 1 */
		0x5B, /* 2 */
		0x4F, /* 3 */
		0x66, /* 4 */
		0x6D, /* 5 */
		0x7D, /* 6 */
		0x07, /* 7 */
		0x7F, /* 8 */
		0x6F  /* 9 */
};


static const unsigned char lowercase_letters[] = {
		0x5F,   /**< a */
		0x7C,   /**< b */
		0x58,   /**< c */
		0x5E,   /**< d */
		0x7B,   /**< e */
		0x71,   /**< f */
		0x6F,   /**< g */
		0x74,   /**< h */
		0x04,   /**< i */
		0x0E,   /**< j */
		0x76,   /**< k - not possible */
		0x38,   /**< l */
		0x54,   /**< m - not possible */
		0x54,   /**< n */
		0x5C,   /**< o */
		0x73,   /**< p */
		0x67,   /**< q */
		0x50,   /**< r */
		0x6D,   /**< s */
		0x78,   /**< t */
		0x1C,   /**< u */
		0x1C,   /**< v - not possible */
		0x1C,   /**< w - not possible */
		0x76,   /**< x - not possible */
		0x6E,   /**< y */
		0x6B    /**< z */
};


static const unsigned char uppercase_letters[] = {
		0x77,   /**< A */
		0x7F,   /**< B */
		0x39,   /**< C */
		0x3F,   /**< D */
		0x79,   /**< E */
		0x71,   /**< F */
		0x7D,   /**< G */
		0x76,   /**< H */
		0x06,   /**< I */
		0x0E,   /**< J */
		0x76,   /**< K - not possible */
		0x38,   /**< L */
		0x37,   /**< M - not possible */
		0x37,   /**< N */
		0x3F,   /**< O */
		0x73,   /**< P */
		0x3F,   /**< Q - not possible */
		0x77,   /**< R */
		0x6D,   /**< S */
		0x78,   /**< T - not possible*/
		0x3E,   /**< U */
		0x3E,   /**< V - not possible */
		0x3E,   /**< W - not possible */
		0x76,   /**< X - not possible */
		0x6E,   /**< Y */
		0x6B    /**< Z */
};


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
		{ COM1, 9 },    /* LCDSYM_FIRE */
		{ COM1, 7 },    /* LCDSYM_COM */
		{ COM1, 6 },    /* LCDSYM_POUND */
		{ COM1, 5 },    /* LCDSYM_EUR */
		{ COM2, 6 },    /* LCDSYM_M3 */
		{ COM2, 5 },    /* LCDSYM_SLASH_TOP */
		{ COM3, 5 },    /* LCDSYM_H_TOP */
		{ COM4, 5 },    /* LCDSYM_H_BOT */
		{ COM3, 6 },    /* LCDSYM_K */
		{ COM4, 6 },    /* LCDSYM_W */
		{ COM2, 7 },    /* LCDSYM_FRAME */
		{ COM4, 29 },    /* LCDSYM_FRAME_SMALL */
		{ COM1, 29 },    /* LCDSYM_BAR0 */
		{ COM2, 29 },    /* LCDSYM_BAR1 */
		{ COM3, 29 },    /* LCDSYM_BAR2 */
		{ COM1, 21 },    /* LCDSYM_DOT1_BOT */
		{ COM4, 15 },    /* LCDSYM_DOT2_TOP */
		{ COM4, 7 },    /* LCDSYM_DOT2_BOT */
		{ COM4, 13 },    /* LCDSYM_DOT3_BOT */
		{ COM4, 11 },    /* LCDSYM_DOT4_TOP */
		{ COM3, 7 },    /* LCDSYM_DOT4_BOT */
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


void lcddrv_update(void)
{
	if(lcd_common.on == 0) {
		return;
	}
	/* request update */
	*(lcd_common.base + sr) |= 0x04;
	/* wait for sync */
	while (!(*(lcd_common.base + sr) & 0x08));

	*(lcd_common.base + clr) |= 0x08;
}


void lcddrv_setRamSegment(unsigned int com, unsigned int pin, unsigned int on)
{
	if (pin > 31) {
		com++;
	}

	if (on) {
		(lcd_common.base + ram)[com] |= (1 << pin % 32);
	} else {
		(lcd_common.base + ram)[com] &= ~(1 << pin % 32);
	}
}


void lcddrv_showChar(char ch, unsigned int pos)
{
	unsigned char segment = 0;
	unsigned int i, on, com, pin, rampin;

	if (ch >= '0' && ch <= '9') {
		segment = numbers[ch - '0'];
	} else if (ch >= 'a' && ch <= 'z') {
		segment = lowercase_letters[ch - 'a'];
	} else if (ch >= 'A' && ch <= 'Z') {
		segment = uppercase_letters[ch - 'A'];
	} else if (ch == '-') {
		segment = char_dash;
	} else if (ch == '_') {
		segment = char_underscore;
	} else if (ch == '?') {
		segment = char_questionmark;
	} else if (ch == LCDDRV_CHAR_DEGREE) {
		segment = char_degree;
	}

	for (i = 0; i < 7; i++) {
		on = segment & (unsigned char)(1 << i);

		com = com_map[pos][i];
		pin = pin_map[pos][i];
		rampin = pin_to_ram[pin];

		lcddrv_setRamSegment(com, rampin, on);
	}
}


void lcddrv_showString(const char *text)
{
	unsigned int i, start, strlen = 0;

	/* clear numbers */
	for (i = 2; i <= LCD_MAX_POSITION; i++) {
		lcddrv_showChar(' ', i);
	}

	while (text[strlen] != '\0' && strlen < LCD_MAX_POSITION - 1) {
		strlen++;
	};

	start = LCD_MAX_POSITION - strlen + 1;
	for (i = start; i <= LCD_MAX_POSITION; i++) {
		lcddrv_showChar(text[i - start], i);
	}

	hal_memcpy(lcd_common.str, text, strlen);
	lcd_common.str[strlen] = '\0';
}


void lcddrv_showSymbol(unsigned int sym_mask, unsigned int state)
{
	unsigned int com, pin, rampin, symbol;
	int i;

	for (i = 0; i < LCDSYM_TOTAL; i++) {
		symbol = sym_mask & (1 << i);
		if (!symbol) {
			continue;
		}

		com = symbols[i][0];
		pin = symbols[i][1];
		rampin = pin_to_ram[pin];

		lcddrv_setRamSegment(com, rampin, state);

		if (state) {
			lcd_common.sym_mask |= symbol;
		} else {
			lcd_common.sym_mask &= ~symbol;
		}
	}
}


void lcddrv_setSymbol(unsigned int sym_mask)
{
	unsigned int com, pin, rampin, symbol;
	int i;

	for (i = 0; i < LCDSYM_TOTAL; i++) {
		symbol = sym_mask & (1 << i);

		com = symbols[i][0];
		pin = symbols[i][1];
		rampin = pin_to_ram[pin];

		lcddrv_setRamSegment(com, rampin, symbol);
		if (symbol) {
			lcd_common.sym_mask |= symbol;
		} else {
			lcd_common.sym_mask &= ~symbol;
		}
	}

	lcddrv_update();
}


void lcddrv_showSmallString(const char *text)
{
	/* clear small digits */
	lcddrv_showSymbol(LCDSYM_SMALL_ONE, 0);
	lcddrv_showChar(' ', 1);

	if (text[0] == '1') {
		lcddrv_showSymbol(LCDSYM_SMALL_ONE, 1);
	}

	lcddrv_showChar(text[1], 1);

	hal_memcpy(lcd_common.str_small, text, sizeof(lcd_common.str_small));
	lcd_common.str[sizeof(lcd_common.str_small)] = '\0';
}


void lcddrv_enable(int on)
{
	lcd_common.on = on;
	if(on) {
		*(lcd_common.base + cr) |= !!on;
	} else {
		*(lcd_common.base + cr) &= ~!on;
	}
}


void lcddrv_setBacklight(unsigned char val)
{
	gpiomsg_t gpio;
	int gpio_res;

	gpio.type = GPIO_SET;
	gpio.port = GPIOD;
	gpio.set.state = !!val;
	gpio.set.mask = 1;
	proc_send(gpiodrv_id, MSG_DEVCTL, &gpio, sizeof(gpio), MSG_NORMAL, &gpio_res, sizeof(gpio_res));

	lcd_common.backlight = val;
}


void lcddrv_initGpioPins(void)
{
	unsigned char port, pin;
	gpiomsg_t gpio;
	int gpio_res;
	static const char gpio_pins[5][9] = {
			{ 7, 8,  9,  10, -1, -1, -1, -1, -1 },
			{ 9, 12, 13, 14, 15, -1, -1, -1, -1 },
			{ 0, 1,  2,  3,  6,  7,  8,  9,  12 },
			{ 2, 8,  10, 11, 12, 13, 14, 15, -1 },
			{ 1, 2,  3,  -1, -1, -1, -1, -1, -1 }
	};

	gpio.type = GPIO_CONFIG;
	gpio.config.otype = 0;
	gpio.config.pupd = 0;
	gpio.config.mode = 2;
	gpio.config.af = 0x0b;
	gpio.config.ospeed = 1;

	for (port = 0; port < 5; port++) {
		for (pin = 0; pin < 9; pin++) {
			if (gpio_pins[port][pin] == -1)
				continue;

			gpio.port = port;
			gpio.config.pin = gpio_pins[port][pin];

			proc_send(gpiodrv_id, MSG_DEVCTL, &gpio, sizeof(gpio), MSG_NORMAL, &gpio_res, sizeof(gpio_res));
		}
	}
}


void lcddrv_mainThread(void *arg)
{
	msghdr_t hdr;
	lcddrv_msg_t msg;
	gpiomsg_t gpio;
	int gpio_res;

	lcddrv_initGpioPins();

	/* init backlight */
	gpio.type = GPIO_CONFIG;
	gpio.port = GPIOD;
	gpio.config.pin = 0;
	gpio.config.mode = 1;
	gpio.config.af = 0;
	gpio.config.ospeed = 0;
	gpio.config.otype = 0;
	gpio.config.pupd = 0;
	proc_send(gpiodrv_id, MSG_DEVCTL, &gpio, sizeof(gpio), MSG_NORMAL, &gpio_res, sizeof(gpio_res));

	for (;;) {
		proc_recv(lcddrv_port, &msg, sizeof(msg), &hdr);

		if (hdr.op != MSG_DEVCTL) {
			continue;
		}

		switch (msg.type) {
			case SET:
				/* Small 1 symbol cannot be set by symbols mask */
				msg.sym_mask &= ~LCDSYM_SMALL_ONE;

				if (hal_strlen(msg.str) > 0) {
					lcddrv_showString(msg.str);
				}
				if (msg.state == 2) {
					lcddrv_setSymbol(msg.sym_mask);
				} else {
					lcddrv_showSymbol(msg.sym_mask, msg.state);
				}
				if (hal_strlen(msg.str_small) > 0) {
					lcddrv_showSmallString(msg.str_small);
				}
				if (msg.backlight >= 0) {
					lcddrv_setBacklight(msg.backlight);
				}

				lcddrv_enable(msg.on);
				lcddrv_update();
				/* fall through, cause we always return actual LCD state */
			case GET:
				hal_memcpy(msg.str, lcd_common.str, sizeof(lcd_common.str));
				hal_memcpy(msg.str_small, lcd_common.str_small, sizeof(lcd_common.str_small));
				msg.sym_mask = lcd_common.sym_mask;
				msg.backlight = lcd_common.backlight;
				msg.on = lcd_common.on;
				proc_respond(lcddrv_port, EOK, &msg, sizeof(msg));
				break;
			case CONF:
				break;
		}
	}
}


void lcddrv_init(void)
{
	lcd_common.rcc = (void *)0x40023800;

	lcd_common.base = (void *)0x40002400;
	lcd_common.str[0] = 0;
	lcd_common.sym_mask = 0;
	lcd_common.backlight = 0;
	lcd_common.on = 0;

	/* mask all needed registers */
	*(lcd_common.base + cr) &= 0xFFFFFF81;
	*(lcd_common.base + fcr) &= 0xFC03FFFF;

	/* enable LCD clock */
	*(lcd_common.rcc + apb1enr) |= 0x0200;

	hal_cpuDataBarrier();

	*(lcd_common.base + cr) |= 0x03 << 2U;   /* DUTY = 1/4 */
	*(lcd_common.base + fcr) |= 0x0A << 18U; /* DIV  = 16  */
	*(lcd_common.base + fcr) |= 0x02 << 22U; /* PS   = 4   */
	*(lcd_common.base + fcr) |= 0x06 << 10U; /* CC   = 6   */
	*(lcd_common.base + fcr) |= 0x02 << 7U;  /* DEAD = 2   */
	*(lcd_common.base + fcr) |= 0x03 << 4U;  /* PON  = 3   */

	/* wait for FCR register sync */
	while (!(*(lcd_common.base + sr) & 0x20));

	/* clear RAM */
	hal_memset((void *)(lcd_common.base + ram), 0, 8);

	proc_portCreate(&lcddrv_port);
	proc_portRegister(lcddrv_port, "/lcddrv");

	proc_threadCreate(NULL, lcddrv_mainThread, 2, 1024, NULL, NULL);
}

#if 0
/*
 *  ASCII version, unused right now.
 *	There is some bug here, if we send two messages one by another then only
 *	first has effect (but both are processed).
 */
static void lcddrv_mainThread(void* arg)
{
	int i, s = 0, on, sym_mask;
	char buff[128], lcdstr[80], sym[6];
	msghdr_t hdr;

//	lib_printf("lcddrv: LCD driver started.\n");
	for(;;) {
		hal_memset(buff, 0, 128);
		hal_memset(lcdstr, 0, 80);
		hal_memset(sym, 0, 6);
		s = 0;

		proc_recv(lcd_common.port, buff, sizeof(buff), &hdr);

		/* get lcd text */
		for(i = 0; i < 80; i++) {
			if(buff[i] == '\033') {
				break;
			}

			lcdstr[i] = buff[i];
		}
		lcdstr[i] = '\0';
		lcddrv_showString(lcdstr);

		/* get symbols */
		while(buff[i] != '\033')
			i++;

		for(s = 0, i++;;i++, s++) {
			switch(buff[i]) {
				case '\0':
					/* backlight */
					sym[s] = '\0';
					lcddrv_enableBacklight(lib_strtoul(sym, NULL, 10));
					break;
				case ',':
					/* symbol mask */
					sym[s] = '\0';
					sym_mask = lib_strtoul(sym, NULL, 10);
					s = 0;
					break;
				case ';':
					/* on/off */
					sym[s] = '\0';
					on = lib_strtoul(sym, NULL, 10);
					s = 0;
					lcddrv_showSymbol(sym_mask);
					break;
			}

			if(buff[i] == '\0') {
				break;
			}

			sym[s] = buff[i];
		}

		proc_respond(lcd_common.port, "1", 2);
		lib_printf("lcddrv: %s", lcdstr);
	}
}
#endif
