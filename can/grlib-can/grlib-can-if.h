/*
 * Phoenix-RTOS
 *
 * GRCANFD driver
 *
 * GRLIB CANFD driver interface library
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mikolaj Matalowski
 *
 * %LICENSE%
 */

#ifndef GRLIB_CAN_IF_H
#define GRLIB_CAN_IF_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/msg.h>

#include "grlib-can-shared.h"

int grlibCan_open(oid_t port);

void grlibCan_close(oid_t port);

int grlibCan_setConfig(oid_t port, grlibCan_config_t *config);

int grlibCan_getConfig(oid_t port, grlibCan_config_t *config);

int grlibCan_Send(oid_t port, grlibCan_msg_t *buffer, size_t size, bool block);

int grlibCan_Recv(oid_t port, grlibCan_msg_t *buffer, size_t size, bool block);

int grlibCan_getStatus(oid_t port, uint32_t *status);

#endif
