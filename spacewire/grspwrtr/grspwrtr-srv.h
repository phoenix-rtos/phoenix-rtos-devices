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
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef GRSPWRTR_SRV_H_
#define GRSPWRTR_SRV_H_

#include <sys/msg.h>


typedef struct {
	/* clang-format off */
	enum { spwrtr_pmap_set = 0, spwrtr_pmap_get, spwrtr_clkdiv_set, spwrtr_clkdiv_get, spwrtr_reset,
		spwrtr_route_set, spwrtr_route_get, spwrtr_route_clear, spwrtr_port_status, spwrtr_port_errclr } type;
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
		struct {
			uint8_t addr;     /* physical (1..31) or logical (32..255) address */
			uint32_t enPorts; /* router port bitmask */
			uint32_t ctrl;    /* SPWRTR_RTACTRL_* (EN is forced on by route_set) */
		} route;
		struct {
			uint8_t port;
		} port;
	} task;
} spwrtr_i_t;


_Static_assert(sizeof(spwrtr_i_t) <= sizeof(((msg_t *)0)->i.raw), "spwrtr_i_t exceeds size of msg.i.raw");


typedef struct {
	unsigned int val;  /* pmap/clkdiv/route port map, port status, cleared port errors */
	unsigned int val2; /* route address control */
} spwrtr_o_t;


_Static_assert(sizeof(spwrtr_o_t) <= sizeof(((msg_t *)0)->i.raw), "spwrtr_o_t exceeds size of msg.i.raw");

#endif
