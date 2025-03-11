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

#include <stdint.h>
#include <stdbool.h>

#include <sys/types.h>

/* Maximum possible CAN standard frame ID */
#define ZYNQMP_CAN_ID_MAX (0x7ff)

/* Structure used for passing CAN frames to and from the CAN driver */
typedef struct {
	uint16_t id;  /**< CAN 2.0 Standard ID [0x000 ... 0x7ff] */
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
 * - count: Number of CAN frames in the provided array (accepts up to 64 frames)
 * - block: Determines whether the function will block if there is insufficient space in the TX FIFO
 * - framesSent: Pointer to store the number of frames sent (useful for non-blocking calls; can be NULL for blocking calls)
 *
 * Return values:
 * - 0: Operation successful
 * - -EINVAL: Invalid argument
 * - -EAGAIN: Blocking is disabled, and there is no space in the TX buffer
 */
int zynqmp_canSend(oid_t *oid, const zynqmp_canFrame *frames, uint8_t count, bool block, uint32_t *framesSent);

/**
 * Receive CAN frames
 *
 * Description: This function attempts to retrieve received CAN frames from the 64 frame wide
 *              hardware RX FIFO buffer. If the buffer is empty, it can block for the specified timeout,
 * 				or return immediately
 *
 * Parameters:
 * - oid: Object identifier of the related device file (either /dev/can0 or /dev/can1)
 * - buf: Pointer to the buffer where received frames will be stored
 * - bufLen: Length of the buffer (maximum number of CAN frames it can hold)
 * - block: Determines whether the function will block if there are no frames in RX buffer
 * - timeoutUs: Timeout in microseconds to wait for frames
 * - recvFrames: Pointer to store the number of frames received
 *
 * Return values:
 * - 0: Operation successful
 * - -ETIME: Timeout expired
 * - -EINVAL: Invalid argument
 */
int zynqmp_canRecv(oid_t *oid, zynqmp_canFrame *buf, uint32_t bufLen, bool block, uint32_t timeoutUs, uint32_t *recvFrames);

#endif /* ZYNQMP_CAN_IF_H */
