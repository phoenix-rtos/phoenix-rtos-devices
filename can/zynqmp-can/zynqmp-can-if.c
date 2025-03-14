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
#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>
#include <phoenix/msg.h>
#include <sys/msg.h>
#include <string.h>

#include "zynqmp-can-if.h"
#include "zynqmp-can-priv.h"

int zynqmp_can_send(oid_t *oid, zynqmp_canFrame *frames, uint32_t count)
{
    /* Validate arguments */
    if ((oid == NULL) || (frames == NULL)) {
        printf("zynqmp-can: nullptr argument\n");
        return -1;
    }
    if (count == 0) {
        printf("zynqmp-can: fail zero msg send\n");
        return -1;
    }
    if (count > 128) {
        printf("zynqmp-can: fail to many messages\n");
        return -1;
    }

    /* Allocate message on stack, pass frames using data pointer */
	msg_t msg = {0};
    msg.type = mtDevCtl;
    msg.i.io.mode = zynqmp_can_opSend;
	msg.i.data = (const void *)frames;
	msg.i.size = count * sizeof(zynqmp_canFrame);

    /* Send message to CAN driver server */
	return msgSend(oid->port, &msg);
}

int zynqmp_can_recv(oid_t *oid, zynqmp_canFrame *buf, uint32_t bufLen, uint32_t *recvFrames)
{
    /* Validate arguments */
    if ((oid == NULL) || (buf == NULL) || (recvFrames == NULL)) {
        printf("zynqmp-can: nullptr argument\n");
        return -1;
    }
    if (bufLen == 0) {
        printf("zynqmp-can: fail zero len buffer passed\n");
        return -1;
    }

    /* Allocate message on stack, pass frames using data pointer */
	msg_t msg = {0};
    msg.type = mtDevCtl;
    msg.i.io.mode = zynqmp_can_opRecv;
	msg.o.data = (void *)buf;
	msg.o.size = bufLen * sizeof(zynqmp_canFrame);

    /* Read message from CAN FIFO */
    int ret = msgSend(oid->port, &msg);

    /* If message was received, pass number of received messages */
    if (ret == 0) {
        *recvFrames = (uint32_t)msg.o.attr.val;
    }
    else {
        *recvFrames = 0;
    }

    return ret;
}
