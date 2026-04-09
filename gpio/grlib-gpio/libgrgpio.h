/*
 * Phoenix-RTOS
 *
 * GRLIB GPIO driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LIBGRGPIO_H_
#define LIBGRGPIO_H_


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


typedef struct {
	volatile uint32_t *vbase;
} grgpio_ctx_t;


void grgpio_handleMsg(grgpio_ctx_t *ctx, msg_t *msg);


int grgpio_createDev(oid_t *oid, unsigned int portNum);


int grgpio_init(grgpio_ctx_t *ctx, unsigned int instance);


#endif
