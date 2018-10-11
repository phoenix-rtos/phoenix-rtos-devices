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
#include <string.h>
#include <sys/platform.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/mman.h>
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
	void *conf_descriptor;
	device_desc_t dev_descriptor;
} telit_common;


void event_cb(usb_event_t *usb_event, void *data, size_t size)
{

	if (!size) {
		TRACE("no callback data");
		return;
	}

	switch (usb_event->type) {

	case usb_event_insertion:
//		libusb_dumpConfiguration(stdout, data);
		mutexLock(telit_common.lock);

		telit_common.device_id = usb_event->insertion.device_id;

		/* copy device and config descriptors */
		memcpy(&telit_common.dev_descriptor, &usb_event->insertion.descriptor, sizeof(device_desc_t));
		telit_common.conf_descriptor = malloc(size);
		memcpy(telit_common.conf_descriptor, data, size);

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

	TRACE("pwrkey set 0");

	msg.type = mtWrite;
	msg.i.io.oid = gpio5_port;
	msg.i.data = &set_value;
	msg.i.size = sizeof(set_value);
	msgSend(gpio5_port.port, &msg);

	sleep(7);

	TRACE("pwrkey set 1");

	set_value.w.val = 1 << 9;

	msgSend(gpio5_port.port, &msg);
}


void telit_setmux(int pin, int mode)
{
	platformctl_t set_mux = {
		.action = pctl_set,
		.type = pctl_iomux,
		.iomux = { .mux = pin, .sion = 0, .mode = mode },
	};

	platformctl(&set_mux);
}


void telit_init_usbpwr(void)
{
	platformctl_t set_mux = {
		.action = pctl_set,
		.type = pctl_iomux,
		.iomux = { .mux = pctl_mux_sd1_d1, .sion = 0, .mode = 5 },
	};

	platformctl_t set_pad = {
		.action = pctl_set,
		.type = pctl_iopad,
		.iopad = { .pad = pctl_pad_sd1_d1, .hys = 0, .pus = 0, .pue = 0,
			.pke = 0, .ode = 0, .speed = 2, .dse = 4, .sre = 0 },
	};

	oid_t gpio2_port, gpio2_dir;
	gpiodata_t set_dir = {
		.w = { .val = 1 << 19, .mask = 1 << 19 }
	};

	gpiodata_t set_value = {
		.w = { .val = 0 << 19, .mask = 1 << 19 }
	};

	msg_t msg = { 0 };

	platformctl(&set_mux);
	platformctl(&set_pad);

	lookup("/dev/gpio2/port", NULL, &gpio2_port);
	lookup("/dev/gpio2/dir", NULL, &gpio2_dir);

	msg.type = mtWrite;
	msg.i.io.oid = gpio2_dir;
	msg.i.data = &set_dir;
	msg.i.size = sizeof(set_dir);
	msgSend(gpio2_dir.port, &msg);

	msg.type = mtWrite;
	msg.i.io.oid = gpio2_port;
	msg.i.data = &set_value;
	msg.i.size = sizeof(set_value);
	msgSend(gpio2_port.port, &msg);
}


int telit_init(void)
{
	int ret = 0;

	telit_init_powerkey();
	//telit_init_usbpwr();

	telit_setmux(pctl_mux_sd1_d1, 8);
	telit_setmux(pctl_mux_sd1_d2, 8);
	telit_setmux(pctl_mux_sd1_d3, 8);

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

int main(int argc, char **argv)
{
	usb_device_id_t usb_device_id;
	usb_cdc_line_coding_t *line_coding;
	int in = 0, out = 0;
	char *data;
	int sz = 0;

	if (argc < 2) {
		printf("Pass phone number in argument\n");
		return 0;
	}

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
	usb_device_id.bcdDevice = -2;
	usb_device_id.bDeviceClass = -2;
	usb_device_id.bDeviceSubClass = -2;
	usb_device_id.bDeviceProtocol = -2;

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

	data = mmap(NULL, 4096, PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_UNCACHED, OID_NULL, 0);

	usb_urb_t urb = { 0 };
	usb_open_t open = { 0 };

	urb.type = usb_transfer_control;
	urb.device_id = telit_common.device_id;
	urb.pipe = 0;

	memset(data, 0, 4096);

	urb.setup.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_STANDARD | REQUEST_RECIPIENT_DEVICE;
	urb.setup.bRequest = SET_CONFIGURATION;
	urb.setup.wValue =  1;
	urb.setup.wIndex = 0;
	urb.setup.wLength = 0;

	libusb_write(&urb, NULL, 0);

	urb.setup.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE;
	urb.setup.bRequest = 0x20;
	urb.setup.wValue =  0;
	urb.setup.wIndex = 0x0;
	urb.setup.wLength = 7;

	line_coding = (usb_cdc_line_coding_t *)data;
	line_coding->dwDTERate = 0xc1200;
	line_coding->bCharFormat = 0;
	line_coding->bParityType = 0;
	line_coding->bDataBits = 8;

	libusb_write(&urb, line_coding, 7);

	memset(data, 0, 4096);

	urb.setup.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE;
	urb.setup.bRequest = 0x22;
	urb.setup.wValue =  3;
	urb.setup.wIndex = 0x0;
	urb.setup.wLength = 0;

	libusb_write(&urb, NULL, 0);

	open.device_id = telit_common.device_id;

	open.endpoint.bLength = 0x7;
	open.endpoint.bDescriptorType = 0x5;
	open.endpoint.bEndpointAddress = 0x82;
	open.endpoint.bmAttributes = 0x2;
	open.endpoint.wMaxPacketSize = 0x200;
	open.endpoint.bInterval = 0x0;

	in = libusb_open(&open);

	open.endpoint.bLength = 0x7;
	open.endpoint.bDescriptorType = 0x5;
	open.endpoint.bEndpointAddress = 0x2;
	open.endpoint.bmAttributes = 0x2;
	open.endpoint.wMaxPacketSize = 0x200;
	open.endpoint.bInterval = 0x0;

	out = libusb_open(&open);

	TRACE("enpoints in: %d out: %d", in, out);

	memcpy(data, "ATD", 3);
	memcpy(data + 3, argv[1], strlen(argv[1]));
	sz = strlen(data);
	data[sz++] = ';';
	data[sz++] = '\r';
	data[sz] = '\n';

	urb.pipe = out;

	printf("DIAL : %s\n", data);

	urb.type = usb_transfer_control;
	urb.device_id = telit_common.device_id;
	urb.pipe = 0;
	urb.setup.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE;
	urb.setup.bRequest = 0x22;
	urb.setup.wValue =  3;
	urb.setup.wIndex = 0x0;
	urb.setup.wLength = 0;

	libusb_write(&urb, NULL, 0);

	memset(&urb.setup, 0, sizeof(setup_packet_t));
	urb.device_id = telit_common.device_id;
	urb.type = usb_transfer_bulk;
	urb.pipe = out;
	sz = strlen(data);
	libusb_write(&urb, data, sz);

	memset(data, 0, 4096);
	urb.device_id = telit_common.device_id;
	urb.type = usb_transfer_bulk;
	urb.pipe = in;

	libusb_read(&urb, data, sz);
	TRACE("READ: %s", data);

	TRACE("exiting");
	libusb_exit();
	telit_exit();
	return 0;
}
