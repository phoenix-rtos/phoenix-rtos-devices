/*
 * Phoenix-RTOS
 *
 * i.MX RT multidriver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMXRT_MULTI_H_
#define _IMXRT_MULTI_H_

#include <stdlib.h>


/* IDs of special files OIDs */
/* clang-format off */
enum { id_console = 0, id_uart1, id_uart2, id_uart3, id_uart4, id_uart5, id_uart6, id_uart7, id_uart8,
	id_uart9, id_uart10, id_uart11, id_uart12, id_gpio1, id_gpio2, id_gpio3, id_gpio4, id_gpio5,
	id_gpio6, id_gpio7, id_gpio8, id_gpio9, id_gpio10, id_gpio11, id_gpio12, id_gpio13, id_spi1,
	id_spi2, id_spi3, id_spi4,
#ifdef __CPU_IMXRT117X
	id_spi5, id_spi6,
#endif
	id_i2c1, id_i2c2, id_i2c3, id_i2c4, id_trng,
#ifdef __CPU_IMXRT117X
	id_cm4_0, id_cm4_1, id_cm4_2, id_cm4_3,
#endif
	id_pseudoNull, id_pseudoZero, id_pseudoFull, id_pseudoRandom, id_kmsgctrl
};
/* clang-format on */


#pragma pack(push, 8)


/* GPIO */


typedef struct {
	enum { gpio_set_port = 0, gpio_get_port, gpio_set_dir, gpio_get_dir } type;

	union {
		struct {
			unsigned int mask;
			unsigned int val;
		} port;

		struct {
			unsigned int mask;
			unsigned int val;
		} dir;
	};
} gpio_t;



/* SPI */

/* clang-format off */
enum { spi_msb = 0, spi_lsb = 1};


enum { spi_mode_0 = 0, spi_mode_1, spi_mode_2, spi_mode_3 };


typedef struct {
	enum { spi_config = 0, spi_transaction } type;

	union {
		struct {
			unsigned char cs;
			unsigned char mode;
			unsigned char endian;
			unsigned int sckDiv;
			unsigned int prescaler;
		} config;

		struct {
			unsigned int frameSize;
			unsigned char cs;
		} transaction;
	};

} spi_t;
/* clang-format on */


/* Cortex M4 */


#define CM4_LOAD_FILE 0
#define CM4_LOAD_BUFF 1
#define CM4_RUN_CORE  2


/* MULTI */


typedef struct {
	id_t id;

	union {
		gpio_t gpio;
		spi_t spi;
		int cm4_type;
	};

} multi_i_t;


typedef struct {
	int err;
	unsigned int val;
} multi_o_t;


#pragma pack(pop)


#endif
