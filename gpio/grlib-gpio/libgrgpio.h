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
#include <sys/types.h>


typedef struct {
	volatile uint32_t *vbase;
} grgpio_ctx_t;


void grgpio_setPortVal(const grgpio_ctx_t *ctx, uint32_t mask, uint32_t val);


uint32_t grgpio_getPortVal(const grgpio_ctx_t *ctx);


void grgpio_setPortDir(const grgpio_ctx_t *ctx, uint32_t mask, uint32_t val);


uint32_t grgpio_getPortDir(const grgpio_ctx_t *ctx);


int grgpio_init(grgpio_ctx_t *ctx, unsigned int instance);


#endif
