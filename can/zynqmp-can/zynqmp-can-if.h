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
#include <sys/time.h>
#include <phoenix/msg.h>

/* Maximum possible CAN standard frame ID */
#define ZYNQMP_CAN_ID_MAX (0x7FF)

/* Structure used for passing CAN frames to and from the CAN driver */
typedef struct {
	uint16_t id;  /**< CAN 2.0 Standard ID [0x000 ... 0x7FF] */
	uint16_t len; /**< CAN 2.0 Payload length [0 ... 8] */
	union {
		uint8_t bytes[8];  /**< Payload represented as an array of bytes */
		uint32_t words[2]; /**< Payload represented as an array of words */
	} payload;             /**< CAN 2.0 Payload (up to 8 bytes) */
} zynqmp_canFrame;

/**
 * Send CAN frames
 *
 * Description: This function attempts to flush the provided CAN frames into the 64 frame wide
 *              hardware TX FIFO buffer
 *
 * Parameters:
 * - oid: Object identifier of the related device file (either /dev/can0 or /dev/can1)
 * - frames: Pointer to an array of CAN frames to be sent
 * - count: Number of CAN frames in the provided array
 * - block: Determines whether the function will block if there is insufficient space in the TX FIFO
 * - sendFrames: Pointer to store the number of frames sent (useful for non-blocking calls; can be NULL for blocking calls)
 *
 * Return values:
 * - EOK: Operation successful
 * - EINVAL: Invalid argument
 * - ENOMEM: Blocking is disabled, and there is no space in the TX buffer
 */
int zynqmp_can_send(oid_t *oid, zynqmp_canFrame *frames, uint32_t count, bool block, uint32_t *sendFrames);

/**
 * Receive CAN frames
 *
 * Description: This function attempts to retrieve received CAN frames from the 64 frame wide
 *              hardware RX FIFO buffer. If the buffer is empty, it will block for the specified timeout
 *
 * Parameters:
 * - oid: Object identifier of the related device file (either /dev/can0 or /dev/can1)
 * - buf: Pointer to the buffer where received frames will be stored
 * - bufLen: Length of the buffer (maximum number of CAN frames it can hold)
 * - timeoutUs: Timeout in microseconds to wait for frames
 * - recvFrames: Pointer to store the number of frames received
 *
 * Return values:
 * - EOK: Operation successful
 * - ETIME: Timeout expired
 * - EINVAL: Invalid argument
 */
int zynqmp_can_recv(oid_t *oid, zynqmp_canFrame *buf, uint32_t bufLen, time_t timeoutUs, uint32_t *recvFrames);

#endif /* ZYNQMP_CAN_IF_H */
