/*
 * Phoenix-RTOS
 *
 * ZynqMP CAN driver interface library
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala
 *
 * %LICENSE%
 */

#ifndef ZYNQMP_CAN_IF_H
#define ZYNQMP_CAN_IF_H

#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>
#include <phoenix/msg.h>


/* Structure used for passing CAN frames to and from CAN driver */
typedef struct {
    uint16_t id;  /**< CAN 2.0 Standard ID [0x000 ... 0x7FF] */
    uint16_t len; /**< CAN 2.0 Payload len [0 ... 8] */
    union {
        uint8_t bytes[8];
        uint32_t words[2];
    } payload; /**< CAN 2.0 Payload (up to 8 bytes) */
} zynqmp_canFrame;


/**
 * Send up to 128 CAN frames
 *
 * It will block until all frames are flushed into CAN HW FIFO
 *
 * Return 0 in case of success, -1 in case of failure
 */
int zynqmp_can_send(oid_t *oid, zynqmp_canFrame *frames, uint32_t count);


/**
 * Receive CAN frames up to bufLen ammount
 *
 * It will block untill at least one frame is received
 *
 * Number of received CAN frames is stored in recvFrames
 *
 * Return 0 in case of success, -1 in case of failure
 */
int zynqmp_can_recv(oid_t *oid, zynqmp_canFrame *buf, uint32_t bufLen, uint32_t *recvFrames);


#endif /* ZYNQMP_CAN_IF_H */
