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


typedef enum {
    zynqmp_can_op_send = 0,
    zynqmp_can_op_recv = 1,
} zynqmp_can_operation_e;


typedef struct {
    zynqmp_can_operation_e operation;
    uint16_t id;
    uint8_t len;
    union {
        uint8_t payload[8];
        uint32_t payload_word[2];
    };
} zynqmp_can_frame_t;


#endif /* ZYNQMP_CAN_PRIV_H */
