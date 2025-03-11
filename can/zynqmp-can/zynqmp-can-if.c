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

#include <stddef.h>
#include <sys/types.h>
#include <phoenix/msg.h>
#include <sys/msg.h>
#include <string.h>

#include "zynqmp-can-if.h"
#include "zynqmp-can-priv.h"


int zynqmp_can_send(oid_t *oid, uint16_t id, uint8_t len, uint8_t *payload)
{
    /* Validate arguments */
    if (len > 8) {
        return -1;
    }
    if ((len != 0) && (payload == NULL)) {
        return -1;
    }

    /* Allocate message */
	msg_t msg = {0};

    /* All message to this driver are labelled as device controll */
    msg.type = mtDevCtl;

    /* Use only small 64 byte buffer to pass single frame to be send */
    zynqmp_can_frame_t *frame = (zynqmp_can_frame_t *)msg.i.raw;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

    /* Mark meaning of this message */
    frame->operation = zynqmp_can_op_send;

    /* Copy data */
    frame->id = id;
    frame->len = len;
    memcpy(frame->payload, payload, len);

    /* Send message to CAN driver server */
	return msgSend(oid->port, &msg);
}


int zynqmp_can_recv(oid_t *oid, uint16_t *id, uint8_t *len, uint8_t *payload)
{
    /* Validate arguments */
    if ((oid == NULL) || (id == NULL) || (len == NULL) || (payload == NULL)) {
        return -1;
    }

    /* Allocate message */
	msg_t msg = {0};

    /* All message to this driver are labelled as device controll */
    msg.type = mtDevCtl;

    /* Use only small 64 byte buffer to pass single frame to be send */
    zynqmp_can_frame_t *frame = (zynqmp_can_frame_t *)msg.i.raw;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

    /* Mark meaning of this message */
    frame->operation = zynqmp_can_op_recv;

    /* Read message from CAN FIFO */
    int ret = msgSend(oid->port, &msg);

    /* If message was received, copy it to passed buffer */
    if (ret == 0) {
        zynqmp_can_frame_t *frame_out = (zynqmp_can_frame_t *)msg.o.raw;
        *id = frame_out->id;
        *len = frame_out->len;
        memcpy(payload, frame_out->payload, frame_out->len);
    }

    return ret;
}
