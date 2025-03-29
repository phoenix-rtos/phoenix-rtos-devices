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
#include <phoenix/msg.h>
#include <sys/msg.h>
#include <string.h>

#include "zynqmp-can-if.h"
#include "zynqmp-can-priv.h"

int zynqmp_can_send(oid_t *oid, zynqmp_canFrame *frames, uint32_t count, bool block, uint32_t *sendFrames)
{
	/* Validate arguments */
	if ((oid == NULL) || (frames == NULL)) {
		printf("zynqmp-can: null pointer argument\n");
		return EINVAL;
	}
	if (count == 0) {
		printf("zynqmp-can: zero messages to send\n");
		return EINVAL;
	}
	if (count > 64) {
		printf("zynqmp-can: too many messages to send\n");
		return EINVAL;
	}

	/* Allocate message on the stack */
	msg_t msg = { 0 };
	/* Use custom request type */
	msg.type = mtDevCtl;
	/* Pass extra information about the request in the "raw" block */
	zynqmp_canDriverReq *req = (zynqmp_canDriverReq *)msg.i.raw;
	req->operation = zynqmp_can_opSend;
	req->blockTxOperation = block;
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
	if (sendFrames != NULL) {
		zynqmp_canDriverResp *response = (zynqmp_canDriverResp *)msg.o.raw;
		*sendFrames = response->framesReceivedSend;
	}

	return ret;
}

int zynqmp_can_recv(oid_t *oid, zynqmp_canFrame *buf, uint32_t bufLen, time_t timeoutUs, uint32_t *recvFrames)
{
	/* Validate arguments */
	if ((oid == NULL) || (buf == NULL) || (recvFrames == NULL)) {
		printf("zynqmp-can: null pointer argument\n");
		return EINVAL;
	}
	if (bufLen == 0) {
		printf("zynqmp-can: zero-length buffer passed\n");
		return EINVAL;
	}

	/* Allocate message on the stack */
	msg_t msg = { 0 };
	/* Use custom request type */
	msg.type = mtDevCtl;
	/* Pass extra information about the request in the "raw" block */
	zynqmp_canDriverReq *req = (zynqmp_canDriverReq *)msg.i.raw;
	req->operation = zynqmp_can_opRecv;
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
