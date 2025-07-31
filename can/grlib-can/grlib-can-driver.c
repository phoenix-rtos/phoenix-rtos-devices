/*
 * Phoenix-RTOS
 *
 * GRCANFD driver
 *
 * GRLIB CANFD driver file
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mikolaj Matalowski
 *
 * %LICENSE%
 */

/* System includes */
#include <errno.h>
#include <paths.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libklog.h>

#include <sys/debug.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/interrupt.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <posix/utils.h>

/* Platform specific includes */
#include "grlib-can-core.h"

static int grlibCan_handleDevCtl(msg_t *msg, grlibCan_dev_t *device)
{
	grlibCan_devCtrl_t *idevctl = (grlibCan_devCtrl_t *)(msg->i.raw);
	int ret;
	uint32_t pending;

	switch (idevctl->type) {
		case can_setConfig:
			if (msg->i.size < sizeof(grlibCan_config_t)) {
				msg->o.err = -1;
				return -1;
			}
			return grlibCan_applyConfig(device, (grlibCan_config_t *)msg->i.data);
		case can_getConfig:
			if (msg->o.size != sizeof(grlibCan_config_t)) {
				msg->o.err = -1;
				return -1;
			}
			grlibCan_copyConfig(device, msg->o.data);
			msg->o.err = 0;
			return 0;
		case can_getStatus:
			if (msg->i.size < sizeof(uint32_t)) {
				msg->o.err = -EINVAL;
				return -1;
			}
			msg->o.err = 0;
			*(uint32_t *)(msg->o.data) = device->device->statReg;
			return 0;
		case can_reset:
			grlibCan_resetDevice(device);
			msg->o.err = EOK;
			return 0;
		case can_writeSync:
			ret = grlibCan_transmitSync(device, (grlibCan_msg_t *)msg->i.data, (uint32_t)msg->i.size / sizeof(grlibCan_msg_t));
			msg->o.err = ret;
			return ret;
		case can_readSync:
			/* Places number of pending frames in RX circular buffer, if buffer cannot fit whole CAN frame */
			ret = grlibCan_recvSync(device, (grlibCan_msg_t *)msg->o.data, (uint32_t)msg->o.size / sizeof(grlibCan_msg_t), &pending);
			*(uint32_t *)((grlibCan_msg_t *)msg->o.data + ret) = pending;
			msg->o.err = ret;
			return ret;
		case can_writeAsync:
			ret = grlibCan_transmitAsync(device, (grlibCan_msg_t *)msg->i.data, (uint32_t)msg->i.size / sizeof(grlibCan_msg_t));
			msg->o.err = ret;
			return ret;
		case can_readAsync:
			ret = grlibCan_recvAsync(device, (grlibCan_msg_t *)msg->o.data, (uint32_t)msg->o.size / sizeof(grlibCan_msg_t));
			msg->o.err = ret;
			return ret;
		default:
			return -1;
	}
}

static int grlibCan_dispatchMsg(msg_t *msg, grlibCan_dev_t *devices, int num)
{
	id_t id = msg->oid.id;
	if (id < num && devices[id].canId != -1) {
		return grlibCan_handleDevCtl(msg, &devices[id]);
	}
	return -1;
}

static void grlibCan_messageThread(void *args)
{
#ifdef VERBOSE
	debug("grlib-can: Entering main message thread\n");
#endif

	grlibCan_driver_t *driverInstance = (grlibCan_driver_t *)args;
	grlibCan_dev_t *devices = driverInstance->devices;
	int num = driverInstance->num;

	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		while (msgRecv(driverInstance->port, &msg, &rid) < 0) { }
		switch (msg.type) {
			case mtDevCtl:
				if (msg.oid.id < num && devices[msg.oid.id].canId != -1 && devices[msg.oid.id].ownerPid == msg.pid) {
					grlibCan_dispatchMsg(&msg, devices, num);
				}
				else {
					msg.o.err = -EBUSY;
				}
				break;
			case mtOpen:
				if (msg.oid.id < num && devices[msg.oid.id].canId != -1 && devices[msg.oid.id].ownerPid == 0) {
					devices[msg.oid.id].ownerPid = msg.pid;
					msg.o.err = EOK;
				}
				else {
					msg.o.err = -EBUSY;
				}
				break;
			case mtClose:
				if (msg.oid.id < num && devices[msg.oid.id].canId != -1 && devices[msg.oid.id].ownerPid == msg.pid) {
					devices[msg.oid.id].ownerPid = 0;
					msg.o.err = EOK;
				}
				else {
					msg.o.err = -EBUSY;
				}
				break;
			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(driverInstance->port, &msg, rid);
	}
}

int main(int argc, char **argv)
{
#ifdef VERBOSE
	debug("grlib-can: Driver process started\n");
#endif

	grlibCan_dev_t devices[GRLIB_MAX_CAN_DEVICES];
	oid_t oid;

	int detectedDevices = grlibCan_queryForDevices(devices);


	if (detectedDevices == 0) {
#ifdef VERBOSE
		debug("grlib-can: There are no GRCANFD devices\n");
#endif
		return EXIT_FAILURE;
	}

	/* Create driver port */
	if (portCreate(&oid.port) < 0) {
#ifdef VERBOSE
		debug("grlib-can: Failed to create port\n");
#endif
		return EXIT_FAILURE;
	}

	grlibCan_driver_t driverInstance = {
		.devices = devices,
		.num = detectedDevices,
		.port = oid.port
	};

	if (grlibCan_initDevices(driverInstance.devices, driverInstance.num) < 0) {
#ifdef VERBOSE
		debug("grlib-can: Failed to initialise devices\n");
#endif
		portDestroy(oid.port);
		return EXIT_FAILURE;
	}

	if (grlibCan_registerDevices(&oid, driverInstance.devices, driverInstance.num) < 0) {
#ifdef VERBOSE
		debug("grlib-can: Failed to register devices in file system\n");
#endif
		for (int i = 0; i < driverInstance.num; i++)
			grlibCan_cleanupResources(&driverInstance.devices[i]);
		portDestroy(oid.port);
		return EXIT_FAILURE;
	}

	grlibCan_messageThread(&driverInstance);

	grlibCan_cleanupResources(driverInstance.devices);
	portDestroy(driverInstance.port);
	grlibCan_unregisterDevices(&oid, driverInstance.devices, driverInstance.num);

	return EXIT_SUCCESS;
}
