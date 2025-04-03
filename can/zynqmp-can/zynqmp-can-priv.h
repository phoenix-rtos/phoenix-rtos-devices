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

#ifndef ZYNQMP_CAN_PRIV_H
#define ZYNQMP_CAN_PRIV_H

#include <stdint.h>

/* Marker used in messages between the application and the server to distinguish operations */
typedef enum {
	zynqmp_canOpSend = 0, /**< Send CAN frames */
	zynqmp_canOpRecv = 1, /**< Receive CAN frames */
} zynqmp_canDriverOpCode;

/* Information passed to the driver as a request */
typedef struct {
	uint8_t operation;  /**< Operation code (see zynqmp_canDriverOpCode) */
	uint8_t block;      /**< Boolean switch to enable or disable blocking behaviour */
	uint32_t timeoutUs; /**< Timeout for the operation [microseconds] */
} zynqmp_canDriverReq;

/* Information returned by the driver */
typedef struct {
	uint32_t framesReceivedSend; /**< Number of frames received or sent */
} zynqmp_canDriverResp;

#endif /* ZYNQMP_CAN_PRIV_H */
