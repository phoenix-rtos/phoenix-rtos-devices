/*
 * Phoenix-RTOS
 *
 * Telit LE910 driver
 *
 * telit-le910.c
 *
 * Copyright 2018 Phoenix Systems
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/platform.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <phoenix/arch/imx6ull.h>

#include <libusb.h>


typedef union {
	unsigned int val;
	struct {
		unsigned int val;
		unsigned int mask;
	} __attribute__((packed)) w;
} gpiodata_t;

#define TRACE(x, ...) fprintf(stderr, "telit: " x "\n", ##__VA_ARGS__)

#define TELIT_ID_VENDOR 0x1BC7
#define TELIT_ID_PRODUCT 0x36


enum { TELIT_STATE_INSERTED, TELIT_STATE_REMOVED };


static struct {
	handle_t cond;
	handle_t lock;
	int state;
	int device_id;
	u32 port;
} telit_common;


void event_cb(usb_event_t *usb_event, void *data, size_t size)
{

	if (!size) {
		TRACE("no callback data");
		return;
	}

	switch (usb_event->type) {

	case usb_event_insertion:
		TRACE("insert %u", size);
		libusb_dumpConfiguration(stdout, data);
		mutexLock(telit_common.lock);

		telit_common.device_id = usb_event->insertion.device_id;
		
		/* config and interface */
		telit_common.state = TELIT_STATE_INSERTED;
		condSignal(telit_common.cond);

		mutexUnlock(telit_common.lock);
		break;

	case usb_event_removal:
		break;	   

	case usb_event_interrupt:
		break;

	default:
		break;	
	}
}

void telit_init_powerkey(void)
{
	platformctl_t set_mux = {
		.action = pctl_set,
		.type = pctl_iomux,
		.iomux = { .mux = pctl_mux_tamper9, .sion = 0, .mode = 5 },
	};

	platformctl_t set_pad = {
		.action = pctl_set,
		.type = pctl_iopad,
		.iopad = { .pad = pctl_pad_tamper9, .hys = 0, .pus = 0, .pue = 0,
			.pke = 0, .ode = 0, .speed = 2, .dse = 4, .sre = 0 },
	};

	oid_t gpio5_port, gpio5_dir;
	gpiodata_t set_dir = {
		.w = { .val = 1 << 9, .mask = 1 << 9 }
	};
	
	gpiodata_t set_value = {
		.w = { .val = 0, .mask = 1 << 9 }
	};

	msg_t msg = { 0 };

	platformctl(&set_mux);
	platformctl(&set_pad);

	lookup("/dev/gpio5/port", NULL, &gpio5_port);
	lookup("/dev/gpio5/dir", NULL, &gpio5_dir);

	msg.type = mtWrite;
	msg.i.io.oid = gpio5_dir;
	msg.i.data = &set_dir;
	msg.i.size = sizeof(set_dir);
	msgSend(gpio5_dir.port, &msg);

	msg.type = mtWrite;
	msg.i.io.oid = gpio5_port;
	msg.i.data = &set_value;
	msg.i.size = sizeof(set_value);
	msgSend(gpio5_port.port, &msg);
}

int telit_init(void)
{
	int ret = 0;
	
	telit_init_powerkey();

	ret |= condCreate(&telit_common.cond);
	ret |= mutexCreate(&telit_common.lock);
	telit_common.state = 0;

	return ret;
}


int telit_exit(void)
{
	resourceDestroy(telit_common.cond);
	resourceDestroy(telit_common.lock);
	telit_common.state = TELIT_STATE_REMOVED;

	return 0;
}

int main(void)
{
	usb_device_id_t usb_device_id;

	if (telit_init()) {
		TRACE("telit init error");
		return -1;
	}

	if (libusb_init()) {
		TRACE("libusb init error");
		telit_exit();
		return -1;
	}

	usb_device_id.idVendor = TELIT_ID_VENDOR; 
	usb_device_id.idProduct = TELIT_ID_PRODUCT; 
	usb_device_id.bcdDevice = USB_CONNECT_WILDCARD; 
	usb_device_id.bDeviceClass = USB_CONNECT_WILDCARD; 
	usb_device_id.bDeviceSubClass = USB_CONNECT_WILDCARD; 
	usb_device_id.bDeviceProtocol = USB_CONNECT_WILDCARD; 

	if (libusb_connect(&usb_device_id, event_cb)) {
		TRACE("unable to connect with usbd"); 
		libusb_exit();
		telit_exit();
		return -1;
	}

	mutexLock(telit_common.lock);
	while (telit_common.state != TELIT_STATE_INSERTED)
		condWait(telit_common.cond, telit_common.lock, 0);
	mutexUnlock(telit_common.lock);
	
	usb_urb_t urb = { 0 };
	usb_open_t open = { 0 };

	urb.type = usb_transfer_control;
	urb.device_id = telit_common.device_id;
	urb.pipe = 0;

	urb.setup.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_STANDARD | REQUEST_RECIPIENT_DEVICE;
	urb.setup.bRequest = SET_CONFIGURATION;
	urb.setup.wValue =  1;
	urb.setup.wIndex = 0;
	urb.setup.wLength = 0;

	libusb_write(&urb, NULL, 0);

	TRACE("looping");
	while(1)
		sleep(3);

	libusb_exit();
	telit_exit();
	return 0;
}
