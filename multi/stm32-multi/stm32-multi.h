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

/* ADC */


enum { adc_def = 0, adc_get, adc_set };


typedef struct {
	unsigned int off;
	char buff[];
} __attribute__((packed)) adcdrv_data_t;


typedef struct {
	char type;
	char channel;
} __attribute__((packed)) adcdrv_devctl_t;


/* RTC */


typedef struct {
	unsigned char hours;
	unsigned char minutes;
	unsigned char seconds;

	unsigned char day;
	unsigned char month;
	unsigned char wday;
	unsigned int year;
} __attribute__((packed)) rtctimestamp_t;


/* LCD */


#define LCDDRV_CHAR_DEGREE 0xf8


typedef enum _lcd_symbols_t {
	LCDSYM_SMALL_ONE    = 1 << 0,
	LCDSYM_CLOCK        = 1 << 1,
	LCDSYM_DATE         = 1 << 2,
	LCDSYM_WARN         = 1 << 3,
	LCDSYM_BATT         = 1 << 4,
	LCDSYM_LOWBATT      = 1 << 5,
	LCDSYM_NOBATT       = 1 << 6,
	LCDSYM_LOCK         = 1 << 7,
	LCDYSM_FACTORY      = 1 << 8,
	LCDSYM_FIRE         = 1 << 9,
	LCDSYM_COM          = 1 << 10,
	LCDSYM_POUND        = 1 << 11,
	LCDSYM_EUR          = 1 << 12,
	LCDSYM_M3           = 1 << 13,
	LCDSYM_SLASH_TOP    = 1 << 14,
	LCDSYM_H_TOP        = 1 << 15,
	LCDSYM_H_BOT        = 1 << 16,
	LCDSYM_K            = 1 << 17,
	LCDSYM_W            = 1 << 18,
	LCDSYM_FRAME        = 1 << 19,
	LCDSYM_FRAME_SMALL  = 1 << 20,
	LCDSYM_BAR0         = 1 << 21,
	LCDSYM_BAR1         = 1 << 22,
	LCDSYM_BAR2         = 1 << 23,
	LCDSYM_DOT1_BOT     = 1 << 24,
	LCDSYM_DOT2_TOP     = 1 << 25,
	LCDSYM_DOT2_BOT     = 1 << 26,
	LCDSYM_DOT3_BOT     = 1 << 27,
	LCDSYM_DOT4_TOP     = 1 << 28,
	LCDSYM_DOT4_BOT     = 1 << 29,
	LCDSYM_TOTAL        = 30
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


/* I2C */


enum { i2c_def = 0, i2c_get, i2c_set };


typedef struct {
	unsigned int off;
	char buff[];
} __attribute__((packed)) i2cdrv_data_t;


typedef struct {
	char type;
	char addr;
	char reg;
	char buff[];
} __attribute__((packed)) i2cdrv_devctl_t;


/* GPIO */


enum { gpio_config, gpio_get, gpio_set };


enum { gpioa = 0, gpiob, gpioc, gpiod, gpioe, gpiof, gpiog, gpioh };


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
	char type;
	int port;

	union {
		gpioset_t set;
		gpioconfig_t config;
	};
} __attribute__((packed)) gpiomsg_t;


typedef struct {
	size_t off;
	char buff[];
} __attribute__((packed)) gpiodrv_data_t;


/* UART */


enum { usart1 = 0, usart2, usart3, uart4, uart5 };


enum { uart_def = 0, uart_get, uart_enable };


enum { uart_mnormal = 0, uart_mnblock };


enum { uart_parnone = 0, uart_pareven, uart_parodd };


typedef struct {
	unsigned int off;
	char buff[];
} __attribute__((packed)) uartdrv_data_t;


typedef struct {
	char enable;
	char bits;
	char parity;
	unsigned int baud;
} __attribute__((packed)) uartdrv_config_t;


typedef struct {
	char mode;
	unsigned int timeout;
} __attribute__((packed)) uartdrv_get_t;


typedef struct {
	char type;

	union {
		uartdrv_config_t def;
		uartdrv_get_t get;
		int enable;
	};
} __attribute__((packed)) uartdrv_devctl_t;


#endif
