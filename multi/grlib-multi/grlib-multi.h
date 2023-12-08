/*
 * Phoenix-RTOS
 *
 * GRLIB multi driver main
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _GRLIB_MULTI_H_
#define _GRLIB_MULTI_H_


#include <sys/types.h>


/* clang-format off */
/* IDs of special files OIDs */
enum { id_gpio0 = 0, id_gpio1, id_spi0, id_spi1, id_uart0, id_uart1, id_uart2, id_uart3, id_uart4,
	id_uart5, id_console, id_adc0, id_adc1, id_adc2, id_adc3, id_adc4, id_adc5, id_adc6, id_adc7,
	id_pseudoNull, id_pseudoZero, id_pseudoFull, id_pseudoRandom, id_kmsgctrl };
/* clang-format on */


#pragma pack(push, 8)


/* GPIO */


typedef struct {
	/* clang-format off */
	enum { gpio_setPort = 0, gpio_getPort, gpio_setDir, gpio_getDir } type;
	/* clang-format on */
	union {
		struct {
			uint32_t mask;
			uint32_t val;
		} port;

		struct {
			uint32_t mask;
			uint32_t val;
		} dir;
	};
} gpio_t;


/* SPI */


typedef struct {
	uint8_t sck;
	uint8_t mosi;
	uint8_t miso;
	uint8_t cs;
} spi_pins_t;


/* clang-format off */
enum { spi_lsb = 0, spi_msb };

enum { spi_mode0 = 0, spi_mode1, spi_mode2, spi_mode3 };
/* clang-format on */


/* SPI_FREQ = SYSCLK_FREQ / ((4 - (2 * prescFactor)) * (prescaler + 1))
 * Setting div16 further divides the frequency by 16.
 */


typedef struct {
	uint8_t byteOrder;
	uint8_t mode;
	uint8_t prescFactor;
	uint8_t prescaler; /* 0x0 - 0xf */
	uint8_t div16;
} spi_config_t;


typedef struct {
	/* clang-format off */
	enum { spi_setPins = 0, spi_config, spi_transaction } type;
	/* clang-format on */

	union {
		spi_pins_t pins;
		spi_config_t config;

		struct {
			uint8_t slaveMsk; /* 0x1 - 0xf */
			size_t len;
		} transaction;
	};
} spi_t;


/* ADC */

/* clang-format off */
enum { adc_modeDiff = 0, adc_modeSingle };
enum { adc_sampleCnt1 = 0, adc_sampleCnt2, adc_sampleCnt3, adc_sampleCnt4 };
/* clang-format on */


typedef struct {
	uint32_t sampleRate;
	uint8_t mode;
	uint8_t sampleCnt;
} adc_config_t;


typedef struct {
	/* clang-format off */
	enum { adc_config = 0 } type;
	/* clang-format on */

	union {
		adc_config_t config;
	};
} adc_t;


/* MULTI */


typedef struct {
	id_t id;

	union {
		gpio_t gpio;
		spi_t spi;
		adc_t adc;
	};
} multi_i_t;


typedef struct {
	int err;
	unsigned int val;
} multi_o_t;


#pragma pack(pop)


#endif
