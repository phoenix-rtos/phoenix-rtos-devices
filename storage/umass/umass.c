/*
 * Phoenix-RTOS
 *
 * USB Mass Storage class driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/list.h>
#include <sys/msg.h>
#include <stdio.h>
#include <stdlib.h>

#include <usb.h>
#include <usbdriver.h>


#define UMASS_HANDLE "/dev/umass"

static struct {
	usb_device_instance_t *instances;
	unsigned port, hostport;
} umass_common;


int main(int argc, char *argv[])
{
	oid_t oid;
	int ret;
	usb_insertion_t insertion;
	usb_deletion_t deletion;
	usb_device_instance_t *instance;
	usb_device_id_t id = { USB_CONNECT_WILDCARD,
	                       USB_CONNECT_WILDCARD,
	                       USB_CLASS_MASS_STORAGE,
						   USB_CONNECT_WILDCARD,
						   USB_CONNECT_WILDCARD };

	if (portCreate(&umass_common.port) != 0) {
		fprintf(stderr, "umass: Can't create port!\n");
		return -EINVAL;
	}
	fprintf(stderr, "umass: Got port %d\n", umass_common.port);

	if ((umass_common.hostport = usb_connect(&id, umass_common.port)) < 0) {
		fprintf(stderr, "umass: Fail to connect to usb host!\n");
		return -EINVAL;
	}

	for (;;) {
		ret = usb_eventsWait(umass_common.port, &insertion, &deletion);
		switch (ret) {
			case usb_msg_insertion:
				instance = malloc(sizeof(usb_device_instance_t));
				instance->descriptor = insertion.descriptor;
				instance->bus = insertion.bus;
				instance->interface = insertion.interface;
				instance->dev = insertion.dev;
				LIST_ADD(&umass_common.instances, instance);
				printf("New device instance: addres: %d.%d id: %4x:%4x\n",
				       instance->bus, instance->dev, instance->descriptor.idVendor, instance->descriptor.idProduct);
				break;
			case usb_msg_deletion:
				break;
			default:
				fprintf(stderr, "umass: Error when receiving event from host\n");
				break;
		}
	}


	return 0;
}