/*
 * Phoenix-RTOS
 *
 * GRLIB GPIO message interface
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef GRGPIO_MSG_H_
#define GRGPIO_MSG_H_

#include <stdint.h>

#include <sys/msg.h>
#include <sys/types.h>


/* clang-format off */
enum { gpio_setPort = 0, gpio_getPort, gpio_setDir, gpio_getDir };
/* clang-format on */

#pragma pack(push, 8)

typedef struct {
	uint32_t type;
	uint32_t mask;
	uint32_t val;
} gpio_i_t;


_Static_assert(sizeof(gpio_i_t) <= sizeof(((msg_t *)0)->i.raw), "gpio_i_t exceeds size of msg.i.raw");


typedef struct {
	unsigned int val;
} gpio_o_t;


_Static_assert(sizeof(gpio_o_t) <= sizeof(((msg_t *)0)->o.raw), "gpio_o_t exceeds size of msg.o.raw");


#pragma pack(pop)


/* Returns GPIO port value */
int gpiomsg_readPort(const oid_t *oid, uint32_t *val);


/* Sets GPIO port value with mask */
int gpiomsg_writePort(const oid_t *oid, uint32_t val, uint32_t mask);


/* Returns GPIO port direction (0 - input, 1 - output) */
int gpiomsg_readDir(const oid_t *oid, uint32_t *val);


/* Sets GPIO port direction with mask (0 - input, 1 - output) */
int gpiomsg_writeDir(const oid_t *oid, uint32_t val, uint32_t mask);


/* Returns GPIO pin state (0 - low, 1 - high) */
int gpiomsg_readPin(const oid_t *oid, uint32_t pin, uint32_t *val);


/* Sets GPIO pin state (0 - low, 1 - high) */
int gpiomsg_writePin(const oid_t *oid, uint32_t pin, uint32_t val);


/* Returns GPIO pin direction (0 - input, 1 - output) */
int gpiomsg_readPinDir(const oid_t *oid, uint32_t pin, uint32_t *val);


/* Sets GPIO pin direction (0 - input, 1 - output) */
int gpiomsg_writePinDir(const oid_t *oid, uint32_t pin, uint32_t val);


int gpiomsg_open(uint32_t port, oid_t *oid);


#endif
