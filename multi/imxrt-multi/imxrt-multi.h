/*
 * Phoenix-RTOS
 *
 * i.MX RT multidriver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMXRT_MULTI_H_
#define _IMXRT_MULTI_H_

/* IDs of special files OIDs */
enum { id_console = 0, id_uart1, id_uart2, id_uart3, id_uart4, id_uart5, id_uart6, id_uart7, id_uart8,
	id_gpio1, id_gpio2, id_gpio3, id_gpio4, id_gpio5, id_gpio6, id_gpio7, id_gpio8, id_gpio9};


typedef struct {
	enum { gpio_port = 0, gpio_dir } type;
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
} gpio_i_msg_t;


typedef struct {
	int err;
	unsigned int val;
} gpio_o_msg_t;

#endif
