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

#ifndef ZYNQMP_CAN_PRIV_H
#define ZYNQMP_CAN_PRIV_H

#include <stdint.h>
#include <sys/msg.h>

/* Marker used in messages between app and server to distringuish operations */
enum {
    zynqmp_can_opSend = 0,
    zynqmp_can_opRecv = 1,
};

#endif /* ZYNQMP_CAN_PRIV_H */
