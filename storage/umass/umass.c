/*
 * Phoenix-RTOS
 *
 * umass driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Jan Sikorski
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
#include <sys/file.h>
#include <sys/msg.h>
#include <poll.h>
#include <errno.h>
#include <posix/utils.h>
#include <fcntl.h>
#include <phoenix/arch/imx6ull.h>
#include <arpa/inet.h>

#include <libusb.h>

#define UMASS_SECSZ  512

#define BULK_WRITE 0
#define BULK_READ  0x80

#define STATE_READY    0
#define STATE_REMOVED -1
#define STATE_CHANGED -2
#define STATE_ERROR   -4
#define STATE_HALT    -5

#define CBW_SIG 0x43425355
#define CSW_SIG 0x53425355

#define MAX_SECS          32
#define MAX_USB_RETRIES   2
#define MAX_SCSI_RETRIES  4

typedef struct _bulk_cbw_t {
	u32 sig;
	u32 tag;
	u32 dlen;
	u8  flags;
	u8  lun;
	u8  clen;
	u8  cmd[16];
} __attribute__((packed)) bulk_cbw_t;


typedef struct _bulk_csw_t {
	u32 sig;
	u32 tag;
	u32 dr;
	u8  status;
} __attribute__((packed)) bulk_csw_t;

typedef union {
	unsigned int val;
	struct {
		unsigned int val;
		unsigned int mask;
	} __attribute__((packed)) w;
} gpiodata_t;

#define TRACE(x, ...) fprintf(stderr, "umass: " x "\n", ##__VA_ARGS__)
#define FUN_TRACE  fprintf(stderr, "umass trace: %s\n", __PRETTY_FUNCTION__)

static struct {
	handle_t cond;
	handle_t lock;
	int device_id, in_pipe, out_pipe, cfg_pipe;
	void *conf_descriptor;
	device_desc_t dev_descriptor;

	int tag;
} umass_common;


void event_cb(usb_event_t *usb_event, char *data, size_t size)
{
	switch (usb_event->type) {
	case usb_event_insertion:
		mutexLock(umass_common.lock);
		umass_common.device_id = usb_event->insertion.device_id;
		memcpy(&umass_common.dev_descriptor, &usb_event->insertion.descriptor, sizeof(device_desc_t));
		umass_common.conf_descriptor = malloc(size);
		memcpy(umass_common.conf_descriptor, data, size);
		mutexUnlock(umass_common.lock);
		libusb_dumpConfiguration(stdout, umass_common.conf_descriptor);
		condSignal(umass_common.cond);
		break;

	case usb_event_removal:
		TRACE("********** REMOVAL!!! **********\n");
		break;

	case usb_event_completion:
		TRACE("COMPLETION");
		break;

	default:
		TRACE("wat");
		break;
	}
}


void umass_setmux(int pin, int mode)
{
	platformctl_t set_mux = {
		.action = pctl_set,
		.type = pctl_iomux,
		.iomux = { .mux = pin, .sion = 0, .mode = mode },
	};

	platformctl(&set_mux);
}


void umass_init_usbpwr(void)
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


int umass_init(void)
{
	int ret = 0;

	//umass_init_usbpwr();

	umass_setmux(pctl_mux_sd1_d1, 8);
	umass_setmux(pctl_mux_sd1_d2, 8);
	umass_setmux(pctl_mux_sd1_d3, 8);

	ret |= condCreate(&umass_common.cond);
	ret |= mutexCreate(&umass_common.lock);

	umass_common.conf_descriptor = NULL;

	return ret;
}


int umass_connect(void)
{
	usb_device_id_t usb_device_id;

	usb_device_id.idVendor = USB_CONNECT_WILDCARD;
	usb_device_id.idProduct = USB_CONNECT_WILDCARD;
	usb_device_id.bcdDevice = USB_CONNECT_NONE;
	usb_device_id.bDeviceClass = USB_CONNECT_NONE;
	usb_device_id.bDeviceSubClass = USB_CONNECT_NONE;
	usb_device_id.bDeviceProtocol = USB_CONNECT_NONE;

	if (libusb_connect(&usb_device_id, (void *)event_cb))
		return -1;

	mutexLock(umass_common.lock);
	while (umass_common.conf_descriptor == NULL)
		condWait(umass_common.cond, umass_common.lock, 0);
	mutexUnlock(umass_common.lock);

	return 0;
}


int umass_open_endpoints(void)
{
	usb_open_t open;

	open.device_id = umass_common.device_id;

	open.endpoint.bLength = 0x7;
	open.endpoint.bDescriptorType = 0x5;
	open.endpoint.bEndpointAddress = 0x81;
	open.endpoint.bmAttributes = 0x2;
	open.endpoint.wMaxPacketSize = 0x40;
	open.endpoint.bInterval = 0x0;

	umass_common.in_pipe = libusb_open(&open);

	open.endpoint.bLength = 0x7;
	open.endpoint.bDescriptorType = 0x5;
	open.endpoint.bEndpointAddress = 0x2;
	open.endpoint.bmAttributes = 0x2;
	open.endpoint.wMaxPacketSize = 0x40;
	open.endpoint.bInterval = 0x0;

	umass_common.out_pipe = libusb_open(&open);

	umass_common.cfg_pipe = 0;

	TRACE("enpoints in: %d out: %d", umass_common.in_pipe, umass_common.out_pipe);
	return 0;
}


int umass_transmit_out(void *buffer, int size)
{
	usb_urb_t urb = {
		.type = usb_transfer_bulk,
		.direction = usb_transfer_out,
		.device_id = umass_common.device_id,
		.pipe = umass_common.out_pipe,
		.transfer_size = size,
		.async = 0,
	};

	return libusb_write(&urb, buffer, size);
}


int umass_transmit_in(void *buffer, int size)
{
	usb_urb_t urb = {
		.type = usb_transfer_bulk,
		.direction = usb_transfer_in,
		.device_id = umass_common.device_id,
		.pipe = umass_common.in_pipe,
		.transfer_size = size,
		.async = 0,
	};

	return libusb_read(&urb, buffer, size);
}


int umass_transmit_cfg(setup_packet_t setup, void *buffer, int size, int direction)
{
	usb_urb_t urb = {
		.type = usb_transfer_control,
		.direction = direction,
		.device_id = umass_common.device_id,
		.pipe = umass_common.cfg_pipe,
		.transfer_size = size,
		.setup = setup,
		.async = 0,
	};

	return direction == usb_transfer_in ? libusb_read(&urb, buffer, size) : libusb_write(&urb, buffer, size);
}


int umass_set_configuration(void)
{
	setup_packet_t setup = (setup_packet_t) {
		.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_STANDARD | REQUEST_RECIPIENT_DEVICE,
		.bRequest = SET_CONFIGURATION,
		.wValue = 1,
		.wIndex = 0,
		.wLength = 0,
	};

	return umass_transmit_cfg(setup, NULL, 0, usb_transfer_out);
}


int umass_reset(void)
{
	setup_packet_t setup = (setup_packet_t) {
		.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE,
		.bRequest = 0xff,
		.wValue = 0,
		.wIndex = 0,
		.wLength = 0,
	};

	return umass_transmit_cfg(setup, NULL, 0, usb_transfer_out);
}


void umass_print_csw(bulk_csw_t *csw)
{
	TRACE("sig:%x tag:%x dr:%x status:%x", csw->sig, csw->tag, csw->dr, csw->status);
}


int bulk_transport(char *cmd, int clen, void *data, int dlen, int dir)
{
	int retry;
	bulk_cbw_t cbw = {0};
	bulk_csw_t csw = {0};

	if (clen > 16)
		return -1;

	cbw.sig = CBW_SIG;
	cbw.tag = umass_common.tag++;
	cbw.dlen = dlen;
	cbw.flags = dir;
	cbw.lun = 0;
	cbw.clen = clen;
	memcpy(cbw.cmd, cmd, clen);

	for (retry = MAX_USB_RETRIES; retry; --retry) {
		TRACE("CBW");
		if (umass_transmit_out(&cbw, sizeof(cbw)))
			continue;

		if (dlen) {
			if (dir == BULK_READ) {
				TRACE("READ");
				if (umass_transmit_in(data, dlen))
					continue;
			}
			else {
				TRACE("WRITE");
				if (umass_transmit_out(data, dlen))
					continue;
			}
		}

		TRACE("CSW");
		if (umass_transmit_in(&csw, sizeof(csw)))
			continue;

		umass_print_csw(&csw);

		TRACE("SUCCESS");
		break;
	}

	if (!retry) {
		TRACE("FAILED");
		return -1;
	}

	return 0;
}


char mbr[2 * UMASS_SECSZ];
int umass_check(void)
{
	int n, res;
	static char scsi_test[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	static char scsi_mbr[16] = { 0x28, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0 };

	for (n = 0; n < MAX_SCSI_RETRIES; n++) {
		TRACE("TEST");
		res = bulk_transport(scsi_test, 16, NULL, 0, BULK_READ);
		if (res == 0)
			break;

		usleep(500 * 1000);
	}

	if (res < 0)
		return res;

	return 0;

	TRACE("MBR");
	if ((res = bulk_transport(scsi_mbr, 16, mbr, UMASS_SECSZ * 2, BULK_READ)) < 0)
		return res;

	return 0;
}


char buff[UMASS_SECSZ * 2];
int umass_read(char *dest, unsigned lbn, unsigned short lbc)
{
	int res;
	int len;
	static char scsi_read[16] = { 0x28, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	while (lbc) {
		len = (lbc > MAX_SECS) ? MAX_SECS : lbc;
		*(unsigned *)(scsi_read + 2) = htonl(lbn);

		if (lbc > 1) {
			*(unsigned short *)(scsi_read + 7) = htons(len);
			if ((res = bulk_transport(scsi_read, 16, dest, UMASS_SECSZ * len, BULK_READ)) < 0)
				return res;
		} else {
			*(unsigned short *)(scsi_read + 7) = htons(2);
			if ((res = bulk_transport(scsi_read, 16, buff, UMASS_SECSZ * 2, BULK_READ)) < 0)
				return res;
			memcpy(dest, buff, UMASS_SECSZ);
		}

		lbn += MAX_SECS;
		lbc -= len;
		dest += UMASS_SECSZ * len;
	}
	return 0;
}


int main(int argc, char **argv)
{
	int readsz = 100;

	if (umass_init()) {
		TRACE("umass init error");
		return -1;
	}

	if (libusb_init()) {
		TRACE("libusb init error");
		return -1;
	}

	if (umass_connect()) {
		TRACE("unable to connect with usbd");
		return -1;
	}

	TRACE("open endpoints");
	umass_open_endpoints();

	TRACE("set configuration");
	umass_set_configuration();

	char *buffer = calloc(readsz, UMASS_SECSZ);

	if (buffer == NULL) {
		TRACE("buffer fail");
		return -1;
	}

	umass_check();
	umass_reset();
	for (int j = 0; 1 /*j < 10000*/; ++j) {
		unsigned hash = 5381;

		memset(buffer, 0, readsz * UMASS_SECSZ);
		int err = umass_read(buffer, 0, readsz);
		TRACE("read %x\n", err);

		for (int i = 0; i < readsz * UMASS_SECSZ; ++i)
			hash = ((hash << 5) + hash) + buffer[i];

		printf("HASH = %x\n", hash);
	}

	TRACE("ok");
	return 0;
}

