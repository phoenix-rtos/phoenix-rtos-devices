/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef LIBGRSPW_SRV_H_
#define LIBGRSPW_SRV_H_


#include <sys/msg.h>
#include <phoenix/msg.h>
#include <string.h>
#include <stdbool.h>

#include <libgrspw.h>


enum {
	id_spw0 = 0U,
	id_spw1,
	id_spw2,
	id_spw3,
	id_spw4,
	id_spw5,
};


typedef struct {
	/* clang-format off */
	enum { spw_config = 0, spw_rxConfig, spw_rx, spw_tx, spw_xfer } type;
	/* clang-format on */
	union {
		spw_config_t config;
		spw_rxConfig_t rxConfig;
		spw_rx_t rx;
		spw_tx_t tx;
		spw_xfer_t xfer;
	} task;

} spw_i_t;


_Static_assert(sizeof(spw_i_t) <= sizeof(((msg_t *)0)->i.raw), "spw_i_t exceeds size of msg.i.raw");


typedef struct {
	size_t val;
} spw_o_t;


_Static_assert(sizeof(spw_o_t) <= sizeof(((msg_t *)0)->o.raw), "spw_o_t exceeds size of msg.o.raw");


void spwsrv_handleMsg(msg_t *msg, int dev);


int spwsrv_createDevs(oid_t *oid);


int spwsrv_init(void);


#endif
