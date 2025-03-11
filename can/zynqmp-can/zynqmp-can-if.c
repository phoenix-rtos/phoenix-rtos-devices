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

#include <stdio.h>
#include <errno.h>
#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/msg.h>
#include <string.h>

#include "zynqmp-can-if.h"
#include "zynqmp-can-priv.h"

int zynqmp_canSend(oid_t *oid, const zynqmp_canFrame *frames, uint8_t count, bool block, uint32_t *framesSent)
{
	/* Allocate message on the stack */
	msg_t msg = { 0 };
	/* Use custom request type */
	msg.type = mtDevCtl;
	/* Pass extra information about the request in the "raw" block */
	zynqmp_canDriverReq *req = (zynqmp_canDriverReq *)msg.i.raw;
	req->operation = zynqmp_canOpSend;
	req->block = (uint8_t)block;
	/* Pass frames using the data/size pair */
	msg.i.data = (const void *)frames;
	msg.i.size = count * sizeof(zynqmp_canFrame);

	/* Try flushing messages to the CAN TX FIFO */
	int ret = msgSend(oid->port, &msg);
	/* If msgSend responded with no error, then return the error code from the driver */
	if (ret == 0) {
		ret = msg.o.err;
	}

	/* Copy the number of sent frames */
	if (framesSent != NULL) {
		zynqmp_canDriverResp *response = (zynqmp_canDriverResp *)msg.o.raw;
		*framesSent = response->framesReceivedSend;
	}

	return ret;
}

int zynqmp_canRecv(oid_t *oid, zynqmp_canFrame *buf, uint32_t bufLen, bool block, uint32_t timeoutUs, uint32_t *recvFrames)
{
	/* Allocate message on the stack */
	msg_t msg = { 0 };
	/* Use custom request type */
	msg.type = mtDevCtl;
	/* Pass extra information about the request in the "raw" block */
	zynqmp_canDriverReq *req = (zynqmp_canDriverReq *)msg.i.raw;
	req->operation = zynqmp_canOpRecv;
	req->block = (uint8_t)block;
	req->timeoutUs = timeoutUs;
	/* Pass received frames using the data/size pair */
	msg.o.data = (void *)buf;
	msg.o.size = bufLen * sizeof(zynqmp_canFrame);

	/* Try to read messages from the CAN RX FIFO */
	int ret = msgSend(oid->port, &msg);
	/* If msgSend responded with no error, then return the error code from the driver */
	if (ret == 0) {
		ret = msg.o.err;
	}

	/* Copy the number of received frames */
	zynqmp_canDriverResp *response = (zynqmp_canDriverResp *)msg.o.raw;
	*recvFrames = response->framesReceivedSend;

	return ret;
}
