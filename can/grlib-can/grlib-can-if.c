/*
 * Phoenix-RTOS
 *
 * GRCANFD driver
 *
 * GRLIB CANFD driver interface library
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mikolaj Matalowski
 *
 * %LICENSE%
 */
#include "grlib-can-if.h"

int grlibCan_open(oid_t port)
{
	msg_t msg = { 0 };
	msg.type = mtOpen;
	msg.oid.id = port.id;

	if (msgSend(port.port, &msg) < 0) {
		return -1;
	}

	return msg.o.err;
}

void grlibCan_close(oid_t port)
{
	msg_t msg = { 0 };
	msg.type = mtClose;
	msg.oid.id = port.id;

	msgSend(port.port, &msg);
}

int grlibCan_setConfig(oid_t port, grlibCan_config_t *config)
{
	msg_t msg = { 0 };
	msg.type = mtDevCtl;
	msg.oid.id = port.id;

	((grlibCan_devCtrl_t *)msg.i.raw)->type = can_setConfig;
	msg.i.data = (void *)config;
	msg.i.size = sizeof(grlibCan_config_t);

	if (msgSend(port.port, &msg) < 0) {
		return -1;
	}

	return msg.o.err;
}

int grlibCan_getConfig(oid_t port, grlibCan_config_t *config)
{
	msg_t msg = { 0 };
	msg.type = mtDevCtl;
	msg.oid.id = port.id;

	((grlibCan_devCtrl_t *)msg.i.raw)->type = can_getConfig;
	msg.o.data = (void *)config;
	msg.o.size = sizeof(grlibCan_config_t);

	if (msgSend(port.port, &msg) < 0) {
		return -1;
	}

	return msg.o.err;
}

int grlibCan_Send(oid_t port, grlibCan_msg_t *buffer, size_t size, bool block)
{
	msg_t msg = { 0 };
	msg.type = mtDevCtl;
	msg.oid.id = port.id;

	((grlibCan_devCtrl_t *)msg.i.raw)->type = block ? can_writeSync : can_writeAsync;

	msg.i.data = buffer;
	msg.i.size = size * sizeof(grlibCan_msg_t);

	if (msgSend(port.port, &msg) < 0) {
		return -1;
	}

	return msg.o.err;
}

int grlibCan_Recv(oid_t port, grlibCan_msg_t *buffer, size_t size, bool block)
{
	msg_t msg = { 0 };
	msg.type = mtDevCtl;
	msg.oid.id = port.id;

	((grlibCan_devCtrl_t *)msg.i.raw)->type = block ? can_readSync : can_readAsync;

	msg.o.data = buffer;
	msg.o.size = size * sizeof(grlibCan_msg_t);

	if (msgSend(port.port, &msg) < 0) {
		return -1;
	}

	return msg.o.err;
}

int grlibCan_getStatus(oid_t port, uint32_t *status)
{
	msg_t msg = { 0 };
	msg.type = mtDevCtl;
	msg.oid.id = port.id;

	((grlibCan_devCtrl_t *)msg.i.raw)->type = can_getStatus;

	msg.o.data = status;
	msg.o.size = sizeof(uint32_t);

	if (msgSend(port.port, &msg) < 0) {
		return -1;
	}

	return msg.o.err;
}
