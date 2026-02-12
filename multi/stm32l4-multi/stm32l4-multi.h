/*
 * Phoenix-RTOS
 *
 * STM32L4 multidriver
 *
 * Copyright 2018, 2022 Phoenix Systems
 * Author: Aleksander Kaminski, Tomasz Korniluk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _STM32_MULTI_H_
#define _STM32_MULTI_H_

#include <sys/interrupt.h>
#include <libmulti/libuart.h>
#include <libmulti/libspi.h>
#include <libmulti/libi2c.h>


#define FLASH_REBOOT_MAGIC 0x88bb77aaUL
#define OTP_WRITE_MAGIC    0x5d1a8712UL
#define RTC_BACKUP_SZ      ((128 / 2) - 4)

#define NODE_ID_TYPE_SHIFT 8
#define NODE_ID_MSK        ((1UL << NODE_ID_TYPE_SHIFT) - 1UL)

/* clang-format off */
enum { adc_get = 0, rtc_setcal, rtc_get, rtc_set, rtc_setalarm, i2c_get, i2c_getwreg,
	i2c_set, i2c_setwreg, gpio_def, gpio_get, gpio_set, uart_def, uart_get, uart_set,
	flash_get, flash_set, flash_info, spi_get, spi_set, spi_rw, spi_def, exti_def,
	exti_map, otp_get, otp_set, rtc_setBackup, rtc_getBackup, flash_setRaw, flash_erase,
	rng_get, pwm_def, pwm_setm, pwm_getm, pwm_getfreq, pwm_distim, pwm_dischn, pwm_bitseq };
/* clang-format on */

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


/* clang-format off */
enum { gpioa = 0, gpiob, gpioc, gpiod, gpioe, gpiof, gpiog, gpioh, gpioi,
	gpioj, gpiok, gpiol, gpiom, gpion, gpioo, gpiop, gpioq };
enum gpio_modes { gpio_mode_gpi = 0, gpio_mode_gpo = 1, gpio_mode_af = 2, gpio_mode_analog = 3};
enum gpio_otypes { gpio_otype_pp = 0, gpio_otype_od = 1 };
enum gpio_ospeeds { gpio_ospeed_low = 0, gpio_ospeed_med = 1, gpio_ospeed_hi = 2, gpio_ospeed_vhi = 3 };
enum gpio_pupds { gpio_pupd_nopull = 0, gpio_pupd_pullup = 1, gpio_pupd_pulldn = 2};
/* clang-format on */


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


/* UART */


typedef struct {
	int uart;
	int mode;
	unsigned int timeout;
} __attribute__((packed)) uartget_t;


typedef struct {
	int uart;
} __attribute__((packed)) uartset_t;


typedef struct {
	int uart;
	unsigned int baud;
	char enable;
	char bits;
	char parity;
} __attribute__((packed)) uartdef_t;


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


/* PWM */

enum pwm_tim_ids {
	pwm_tim1,
	pwm_tim2,
	pwm_tim3,
	pwm_tim4,
	pwm_tim5,
	pwm_tim6,
	pwm_tim7,
	pwm_tim8,
	pwm_tim9,
	pwm_tim10,
	pwm_tim11,
	pwm_tim12,
	pwm_tim13,
	pwm_tim14,
	pwm_tim15,
	pwm_tim16,
	pwm_tim17,
	pwm_tim18,
	pwm_tim_count
};

enum pwm_chn_ids {
	pwm_ch1,
	pwm_ch2,
	pwm_ch3,
	pwm_ch4,
	pwm_ch1n,
	pwm_ch2n,
	pwm_ch3n,
	pwm_ch4n,
};

typedef enum pwm_tim_ids pwm_tim_id_t;

typedef enum pwm_chn_ids pwm_ch_id_t;

typedef struct {
	pwm_tim_id_t timer;
	uint16_t prescaler;
	uint32_t top;
} __attribute__((packed)) pwmdef_t;


typedef struct {
	pwm_tim_id_t timer;
	pwm_ch_id_t chn;
	uint32_t compare;
} __attribute__((packed)) pwmset_t;


typedef struct {
	pwm_tim_id_t timer;
	pwm_ch_id_t chn;
} __attribute__((packed)) pwmgeti_t;


typedef struct {
	uint32_t top;
	uint32_t compare;
} __attribute__((packed)) pwmgeto_t;


typedef struct {
	pwm_tim_id_t timer;
} __attribute__((packed)) pwmfreq_t;


typedef struct {
	pwm_tim_id_t timer;
} __attribute__((packed)) pwmdistim_t;


typedef struct {
	pwm_tim_id_t timer;
	pwm_ch_id_t chn;
} __attribute__((packed)) pwmdischn_t;


typedef struct {
	pwm_tim_id_t timer;
	pwm_ch_id_t chn;
	void *data;
	uint32_t nbits;
	uint8_t datasize;
	int flags;
} __attribute__((packed)) pwmbitseq_t;


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


/* FLASH */


typedef struct {
	unsigned char dualbank;
	unsigned char dualboot;
	unsigned char remap;
	unsigned char activebank;
} __attribute__((packed)) flashinfo_t;


/* OTP */


typedef struct {
	unsigned int offset;
	unsigned int magic;
} otprw_t;


/* MULTI */


typedef struct {
	int type;

	union {
		adcget_t adc_get;
		int rtc_calib;
		rtctimestamp_t rtc_timestamp;
		i2cmsg_t i2c_msg;
		uartget_t uart_get;
		uartset_t uart_set;
		uartdef_t uart_def;
		gpiodef_t gpio_def;
		gpioget_t gpio_get;
		gpioset_t gpio_set;
		spirw_t spi_rw;
		spidef_t spi_def;
		extidef_t exti_def;
		extimap_t exti_map;
		otprw_t otp_rw;
		unsigned int flash_addr;
		pwmdef_t pwm_def;
		pwmset_t pwm_set;
		pwmgeti_t pwm_get;
		pwmfreq_t pwm_freq;
		pwmdistim_t pwm_distim;
		pwmdischn_t pwm_dischn;
		pwmbitseq_t pwm_bitseq;
	};
} __attribute__((packed)) multi_i_t;


typedef struct {
	union {
		unsigned short adc_valmv;
		rtctimestamp_t rtc_timestamp;
		unsigned int gpio_get;
		flashinfo_t flash_info;
		uint64_t pwm_basefreq;
		pwmgeto_t pwm_get;
	};
} __attribute__((packed)) multi_o_t;


enum {
	node_id_multi = (0 << NODE_ID_TYPE_SHIFT),
	node_id_i2c = (1 << NODE_ID_TYPE_SHIFT)
};


#endif
