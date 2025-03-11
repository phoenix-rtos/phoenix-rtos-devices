/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ZynqMP CAN driver interface library
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef ZYNQMP_CAN_IF_H
#define ZYNQMP_CAN_IF_H

#include <stddef.h>
#include <sys/types.h>
#include <phoenix/msg.h>


int zynqmp_can_send(oid_t *oid, uint16_t id, uint8_t len, uint8_t *payload);


int zynqmp_can_recv(oid_t *oid, uint16_t *id, uint8_t *len, uint8_t *payload);


#endif /* ZYNQMP_CAN_IF_H */
