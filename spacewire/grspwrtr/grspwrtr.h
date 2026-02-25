/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Andrzej Tlomak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef SPWRTR_H
#define SPWRTR_H

#include <sys/msg.h>


typedef struct {
	/* clang-format off */
	enum { spwrtr_pmap_set = 0, spwrtr_pmap_get, spwrtr_clkdiv_set, spwrtr_clkdiv_get, spwrtr_reset } type;
	/* clang-format on */
	union {
		struct {
			uint8_t port;
			uint32_t enPorts;
		} mapping;
		struct {
			uint8_t port;
			uint8_t div;
		} clkdiv;
	} task;
} spwrtr_t;


_Static_assert(sizeof(spwrtr_t) <= sizeof(((msg_t *)0)->i.raw), "spwrtr_t exceeds size of msg.i.raw");


typedef struct {
	unsigned int val;
} spwrtr_o_t;


_Static_assert(sizeof(spwrtr_o_t) <= sizeof(((msg_t *)0)->i.raw), "spwrtr_o_t exceeds size of msg.i.raw");

#endif
