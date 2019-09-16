/*
 * Phoenix-RTOS
 *
 * Telit LE910 driver
 *
 * telit-le910.c
 *
 * Copyright 2019 Phoenix Systems
 * Author: Jan Sikorski, Kamil Amanowicz
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
#include <sys/list.h>
#include <syslog.h>
#include <poll.h>
#include <errno.h>
#include <posix/utils.h>
#include <fcntl.h>
#include <phoenix/arch/imx6ull.h>

#include <libusb.h>

#include "fifo.h"

#define USB_CDC_SEND_ENCAPSULATED_COMMAND 0x00
#define USB_CDC_GET_ENCAPSULATED_RESPONSE 0x01
#define USB_CDC_REQ_SET_LINE_CODING  0x20
#define USB_CDC_REQ_GET_LINE_CODING  0x21
#define USB_CDC_REQ_SET_CONTROL_LINE_STATE 0x22
#define USB_CDC_REQ_SEND_BREAK   0x23
#define USB_CDC_SET_ETHERNET_MULTICAST_FILTERS 0x40
#define USB_CDC_SET_ETHERNET_PM_PATTERN_FILTER 0x41
#define USB_CDC_GET_ETHERNET_PM_PATTERN_FILTER 0x42
#define USB_CDC_SET_ETHERNET_PACKET_FILTER 0x43
#define USB_CDC_GET_ETHERNET_STATISTIC  0x44
#define USB_CDC_GET_NTB_PARAMETERS  0x80
#define USB_CDC_GET_NET_ADDRESS   0x81
#define USB_CDC_SET_NET_ADDRESS   0x82
#define USB_CDC_GET_NTB_FORMAT   0x83
#define USB_CDC_SET_NTB_FORMAT   0x84
#define USB_CDC_GET_NTB_INPUT_SIZE  0x85
#define USB_CDC_SET_NTB_INPUT_SIZE  0x86
#define USB_CDC_GET_MAX_DATAGRAM_SIZE  0x87
#define USB_CDC_SET_MAX_DATAGRAM_SIZE  0x88
#define USB_CDC_GET_CRC_MODE   0x89
#define USB_CDC_SET_CRC_MODE   0x8a

#define RX_FIFO_SIZE (4 * SIZE_PAGE)


typedef union {
	unsigned int val;
	struct {
		unsigned int val;
		unsigned int mask;
	} __attribute__((packed)) w;
} gpiodata_t;

#define TRACE(x, ...) //fprintf(stderr, "telit: " x "\n", ##__VA_ARGS__)
#define TRACE_FAIL(x, ...) syslog(LOG_WARNING, x, ##__VA_ARGS__)
#define FUN_TRACE  //fprintf(stderr, "telit trace: %s\n", __PRETTY_FUNCTION__)

#define TELIT_ID_VENDOR 0x1BC7
#define TELIT_ID_PRODUCT 0x36

enum { TELIT_STATE_INSERTED, TELIT_STATE_REMOVED };


typedef struct {
	void *next, *prev;

	msg_t msg;
	unsigned rid;

	struct _ttyacm_t *acm;
	usb_urb_t urb;
} request_t;


typedef struct _ttyacm_t {
	u32 port;
	int id;

	int bulk_interface, intr_interface;
	int pipe_in, pipe_out, pipe_intr;
	fifo_t *fifo;
	int read_buffers, intr_buffers;

	endpoint_desc_t ep_in, ep_out, ep_intr;
	int error;

	handle_t cond;
	handle_t lock;

	request_t *readers;
} ttyacm_t;


static struct {
	handle_t cond;
	handle_t lock;

	int state;
	int device_id;
	u32 port, monitor_port;
	configuration_desc_t *conf_descriptor;
	device_desc_t dev_descriptor;

	int resetting;

	request_t *queue;

	ttyacm_t data[3];
} telit_common;


int _telit_reset(void);


int _telit_write(ttyacm_t *acm, char *data, size_t size)
{
	FUN_TRACE;
	int err;
	usb_urb_t urb;

	memset(&urb.setup, 0, sizeof(setup_packet_t));
	urb.device_id = telit_common.device_id;
	urb.type = usb_transfer_bulk;
	urb.pipe = acm->pipe_out;
	urb.direction = usb_transfer_out;
	urb.async = 0;

	if (acm->error || libusb_write(&urb, data, size) < 0) {
		TRACE_FAIL("write");
		_telit_reset();
		err = -EIO;
	}
	else {
		err = size;
	}

	return err;
}


int _telit_read(ttyacm_t *acm, char *data, size_t size)
{
	FUN_TRACE;
	int i = 0;

	for (i = 0; i < size && !fifo_is_empty(acm->fifo); ++i)
		data[i] = fifo_pop_back(acm->fifo);

	if (acm->error) {
		_telit_reset();
		i = -EIO;
	}

	return i;
}


static void telit_reqRespond(request_t *req)
{
	FUN_TRACE;
	msgRespond(req->acm->port, &req->msg, req->rid);
	free(req);
}


static void telit_msgThread(void *arg)
{
	FUN_TRACE;
	ttyacm_t *acm;
	unsigned port = (unsigned)arg;
	request_t *req = NULL;
	msg_t *msg;
	int id = 0;

	for (;;) {
		if (req == NULL) {
			req = calloc(1, sizeof(request_t));

			if (req == NULL) {
				TRACE_FAIL("allocate request\n");
				sleep(10);
				continue;
			}

			msg = &req->msg;
		}

		memset(msg, 0, sizeof(msg_t));

		if (msgRecv(port, msg, &req->rid) < 0)
			continue;

		switch (msg->type) {
		case mtOpen:
		case mtClose:
			id = msg->i.openclose.oid.id;
			break;

		case mtRead:
		case mtWrite:
			id = msg->i.io.oid.id;
			break;

		case mtGetAttr:
			id = msg->i.attr.oid.id;
			break;
		}

		req->acm = acm = telit_common.data + id;

		switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.io.err = EOK;
			break;

		case mtWrite:
			if (id < 0 || id > sizeof(telit_common.data) / sizeof(telit_common.data[0])) {
				msg->o.io.err = -EINVAL;
				break;
			}

			mutexLock(acm->lock);
			msg->o.io.err = _telit_write(acm, msg->i.data, msg->i.size);
			mutexUnlock(acm->lock);
			break;

		case mtRead:
			if (id < 0 || id > sizeof(telit_common.data) / sizeof(telit_common.data[0])) {
				msg->o.io.err = -EINVAL;
				break;
			}

			if (!msg->o.size)
				break;

			mutexLock(acm->lock);
			if ((msg->o.io.err = _telit_read(acm, msg->o.data, msg->o.size)) == 0) {
				if (msg->i.io.mode & O_NONBLOCK) {
					msg->o.io.err = -EWOULDBLOCK;
				}
				else {
					LIST_ADD(&acm->readers, req);
					req = NULL;
				}
			}
			mutexUnlock(acm->lock);
			break;

		case mtGetAttr:
			if (id < 0 || id > sizeof(telit_common.data) / sizeof(telit_common.data[0])) {
				msg->o.attr.val = -EINVAL;
				break;
			}

			mutexLock(acm->lock);
			if (msg->i.attr.type == atPollStatus)
				msg->o.attr.val = fifo_is_empty(acm->fifo) ? POLLOUT : POLLOUT | POLLIN;
			else
				msg->o.attr.val = -EINVAL;
			mutexUnlock(acm->lock);

			break;

		default:
			break;
		}

		if (req != NULL)
			msgRespond(port, msg, req->rid);
	}
}


int open_pipe(endpoint_desc_t *desc)
{
	usb_open_t open = { 0 };

	open.device_id = telit_common.device_id;
	open.endpoint = *desc;

	return libusb_open(&open);
}


int acm_control(int type, int request, int value, int index, void *buffer, int length)
{
	usb_urb_t urb = { 0 };

	urb.type = usb_transfer_control;
	urb.device_id = telit_common.device_id;
	urb.pipe = 0;
	urb.direction = ((type & REQUEST_DIR_MASK) == REQUEST_DIR_HOST2DEV) ? usb_transfer_out : usb_transfer_in;
	urb.async = 0;

	urb.setup.bmRequestType = type;
	urb.setup.bRequest = request;
	urb.setup.wValue = value;
	urb.setup.wIndex = index;
	urb.setup.wLength = length;

	if (urb.direction == usb_transfer_out)
		return libusb_write(&urb, buffer, length);
	else
		return libusb_read(&urb, buffer, length);
}


int open_ttyacm(ttyacm_t *acm, int bulk_iface, int intr_iface, endpoint_desc_t *inep, endpoint_desc_t *outep, endpoint_desc_t *intrep)
{
	acm->fifo = malloc(sizeof(fifo_t) + RX_FIFO_SIZE * sizeof(acm->fifo->data[0]));
	if (acm->fifo == NULL) {
		TRACE_FAIL("failed to allocate pipe");
		return -ENOMEM;
	}

	fifo_init(acm->fifo, RX_FIFO_SIZE);

	acm->ep_in = *inep;
	acm->ep_out = *outep;
	acm->ep_intr = *intrep;

	acm->pipe_in = open_pipe(inep);
	if (acm->pipe_in < 0) {
		TRACE_FAIL("failed to open input pipe");
		return -EIO;
	}

	acm->pipe_out = open_pipe(outep);
	if (acm->pipe_out < 0) {
		TRACE_FAIL("failed to output pipe");
		return -EIO;
	}

	acm->pipe_intr = open_pipe(intrep);
	if (acm->pipe_intr < 0) {
		TRACE_FAIL("failed to output interrupt pipe");
		return -EIO;
	}

	acm->bulk_interface = bulk_iface;
	acm->intr_interface = intr_iface;

	if (mutexCreate(&acm->lock) < 0) {
		TRACE_FAIL("failed to create mutex");
		return -ENOMEM;
	}

	if (condCreate(&acm->cond) < 0) {
		TRACE_FAIL("failed to create cond");
		return -ENOMEM;
	}

	acm->read_buffers = 0;
	acm->intr_buffers = 0;
	acm->error = 0;

	return EOK;
}


int _telit_init_read_buffers(ttyacm_t *acm)
{
	FUN_TRACE;
	int err;

	usb_urb_t urb = { 0 };

	urb.type = usb_transfer_bulk;
	urb.device_id = telit_common.device_id;
	urb.pipe = acm->pipe_in;
	urb.transfer_size = 0x1000;
	urb.direction = usb_transfer_in;
	urb.async = 1;

	while (acm->read_buffers < 8) {
		if (libusb_read(&urb, NULL, 0) < 0) {
			err = -EIO;
			break;
		}
		acm->read_buffers++;
	}

	return err;
}


int _telit_init_intr_buffers(ttyacm_t *acm)
{
	FUN_TRACE;

	int err;
	usb_urb_t urb = { 0 };

	urb.type = usb_transfer_interrupt;
	urb.device_id = telit_common.device_id;
	urb.pipe = acm->pipe_intr;
	urb.transfer_size = 0x1000;
	urb.direction = usb_transfer_in;
	urb.async = 1;

	while (acm->intr_buffers < 2) {
		if (libusb_read(&urb, NULL, 0) < 0) {
			err = -EIO;
			break;
		}
		acm->intr_buffers++;
	}

	return err;
}


endpoint_desc_t *find_endpoint(int address)
{
	struct desc_header *d = (void *)telit_common.conf_descriptor;
	void *end = (char *)d + telit_common.conf_descriptor->wTotalLength;
	endpoint_desc_t *e;

	while ((void *)d < end) {
		if (d->bDescriptorType == DESC_ENDPOINT) {
			e = (endpoint_desc_t *)d;

			if (e->bEndpointAddress == address)
				return e;
		}

		d = (void *)((char *)d + d->bLength);
	}

	TRACE_FAIL("could not find enpoint");
	return NULL;
}


int telit_init_device(void)
{
	int type;

	type = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_STANDARD | REQUEST_RECIPIENT_DEVICE;
	if (acm_control(type, SET_CONFIGURATION, 1, 0, NULL, 0) < 0) {
		TRACE_FAIL("set configuration");
		return -EIO;
	}

	return EOK;
}


int telit_init_interface(int interface)
{
	usb_cdc_line_coding_t line_coding;
	int type;

	type = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE;

	line_coding.dwDTERate = 0xc1200;
	line_coding.bCharFormat = 0;
	line_coding.bParityType = 0;
	line_coding.bDataBits = 8;

	if (acm_control(type, USB_CDC_REQ_SET_LINE_CODING, 0, interface, &line_coding, sizeof(line_coding)) < 0) {
		TRACE_FAIL("set line coding");
		return -EIO;
	}

	if (acm_control(type, USB_CDC_REQ_SET_CONTROL_LINE_STATE, 3, interface, NULL, 0) < 0) {
		TRACE_FAIL("set control line state");
		return -EIO;
	}

	/* again? */

	return EOK;
}


int telit_init_all(void)
{
	FUN_TRACE;

	int err;
	endpoint_desc_t *inep, *outep, *intrep;
	int bulk_interface, intr_interface;

	telit_init_device();


	telit_init_interface(0);

	inep = find_endpoint(0x82);
	outep = find_endpoint(0x2);
	intrep = find_endpoint(0x81);

	if (inep == NULL || outep == NULL || intrep == NULL)
		return -1;

	intr_interface = 0;
	bulk_interface = 1;

	if ((err = open_ttyacm(&telit_common.data[0], bulk_interface, intr_interface, inep, outep, intrep)) < 0)
		return err;


	telit_init_interface(6);

	inep = find_endpoint(0x88);
	outep = find_endpoint(0x8);
	intrep = find_endpoint(0x87);

	if (inep == NULL || outep == NULL || intrep == NULL)
		return -1;

	intr_interface = 6;
	bulk_interface = 7;

	if ((err = open_ttyacm(&telit_common.data[1], bulk_interface, intr_interface, inep, outep, intrep)) < 0)
		return err;


	telit_init_interface(8);

	inep = find_endpoint(0x8a);
	outep = find_endpoint(0xa);
	intrep = find_endpoint(0x89);

	if (inep == NULL || outep == NULL || intrep == NULL)
		return -1;

	intr_interface = 8;
	bulk_interface = 9;

	if ((err = open_ttyacm(&telit_common.data[2], bulk_interface, intr_interface, inep, outep, intrep)) < 0)
		return err;

	return EOK;
}


int _telit_reset(void)
{
	TRACE_FAIL("resetting");
	telit_common.resetting = 1;

	if (libusb_reset(telit_common.device_id) < 0)
		return -1;

	telit_common.data[0].intr_buffers = 0;
	telit_common.data[0].read_buffers = 0;

	telit_common.data[1].intr_buffers = 0;
	telit_common.data[1].read_buffers = 0;

	telit_common.data[2].intr_buffers = 0;
	telit_common.data[2].read_buffers = 0;

	if (telit_init_device() < 0)
		return -1;

	if (telit_init_interface(0) < 0)
		return -1;

	telit_common.resetting = 0;
	telit_common.data[0].error = 0;
	telit_common.data[1].error = 0;
	telit_common.data[2].error = 0;

	TRACE_FAIL("reset done");

	condBroadcast(telit_common.data[0].cond);
	condBroadcast(telit_common.data[1].cond);
	condBroadcast(telit_common.data[2].cond);

	return EOK;
}


static void clear_halt(endpoint_desc_t ep)
{
	acm_control(REQUEST_TYPE_STANDARD | REQUEST_DIR_HOST2DEV | REQUEST_RECIPIENT_ENDPOINT,
		CLEAR_FEATURE, ENDPOINT_HALT, ep.bEndpointAddress, NULL, 0);
}


void telit_readThread(void *_acm)
{
	FUN_TRACE;
	ttyacm_t *acm = _acm;
	request_t *req;

	mutexLock(acm->lock);
	for (;;) {
		while ((req = acm->readers) == NULL || fifo_is_empty(acm->fifo))
			condWait(acm->cond, acm->lock, 0);

		if ((req->msg.o.io.err = _telit_read(acm, req->msg.o.data, req->msg.o.size))) {
			LIST_REMOVE(&acm->readers, req);
			telit_reqRespond(req);
		}
		else {
			TRACE_FAIL("no read in read thread");
		}
	}
}


void _telit_input(ttyacm_t *acm, char *data, size_t size, int err)
{
	FUN_TRACE;
	int i;

	acm->read_buffers--;

	if (err < 0) {
		TRACE_FAIL("read error in input");
		acm->error = 1;
	}
	else {
		TRACE("input %lu bytes to acm%d", size, acm->id);
		for (i = 0; i < size && !fifo_is_full(acm->fifo); ++i)
			fifo_push(acm->fifo, data[i]);

		if (fifo_is_full(acm->fifo) && i < size)
			TRACE_FAIL("RX fifo overflow");
	}
	condBroadcast(acm->cond);
}


void telit_resubmitThread(void *arg)
{
	FUN_TRACE;
	ttyacm_t *acm = arg;

	mutexLock(acm->lock);
	for (;;) {
		while (acm->read_buffers >= 8 || acm->error)
			condWait(acm->cond, acm->lock, 0);

		_telit_init_read_buffers(acm);
	}
}


void telit_intrresubmitThread(void *arg)
{
	FUN_TRACE;
	ttyacm_t *acm = arg;

	mutexLock(acm->lock);
	for (;;) {
		while (acm->intr_buffers >= 2)
			condWait(acm->cond, acm->lock, 0);

		_telit_init_intr_buffers(acm);
	}
}


void event_cb(usb_event_t *usb_event, char *data, size_t size)
{
	FUN_TRACE;
	ttyacm_t *acm;

	switch (usb_event->type) {
	case usb_event_insertion:
		// libusb_dumpConfiguration(stdout, data);
		telit_common.device_id = usb_event->device_id;
		memcpy(&telit_common.dev_descriptor, &usb_event->insertion.descriptor, sizeof(device_desc_t));
		telit_common.conf_descriptor = malloc(size);
		memcpy(telit_common.conf_descriptor, data, size);

		mutexLock(telit_common.lock);
		telit_common.state = TELIT_STATE_INSERTED;
		mutexUnlock(telit_common.lock);

		condBroadcast(telit_common.cond);
		break;

	case usb_event_removal:
		TRACE_FAIL("device removed");
		mutexLock(telit_common.lock);
		telit_common.state = TELIT_STATE_REMOVED;
		mutexUnlock(telit_common.lock);

		condBroadcast(telit_common.cond);
		break;

	case usb_event_completion:

		if (usb_event->completion.pipe < 4)
			acm = telit_common.data;
		else if (usb_event->completion.pipe < 7)
			acm = telit_common.data + 1;
		else
			acm = telit_common.data + 2;

		/* Ignore aborted transfers */
		if (usb_event->completion.error > 0)
			break;

		mutexLock(acm->lock);
		if (usb_event->completion.pipe == acm->pipe_intr) {
			TRACE("GOT INTERRUPT");
			acm->intr_buffers--;
		}
		else if (usb_event->completion.pipe == acm->pipe_in) {
			_telit_input(acm, data, size, usb_event->completion.error);
		}
		else if (usb_event->completion.pipe == acm->pipe_out) {
			if (usb_event->completion.error < 0) {
				acm->error = 1;
				condBroadcast(acm->cond);
			}
		}
		else {
			TRACE_FAIL("completion on pipe %d unexpected", usb_event->completion.pipe);
		}
		mutexUnlock(acm->lock);
		break;

	default:
		break;
	}
}


void telit_init_powerkey(void)
{
	FUN_TRACE;
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

	TRACE("pwrkey set 0");
	set_value.w.val = 0;
	msgSend(gpio5_port.port, &msg);

	/* Sleep a minimum of 10 seconds. */
	sleep(14);

	TRACE("ppwrkey set 1");
	set_value.w.val = 1 << 9;
	msgSend(gpio5_port.port, &msg);

	/* Sleep a minimum of 200 ms. */
	sleep(1);
}


void telit_monitorthr(void *arg)
{
	ttyacm_t *acm = arg;
	int type = REQUEST_TYPE_STANDARD | REQUEST_DIR_DEV2HOST | REQUEST_RECIPIENT_ENDPOINT;
	short in_status = 0, out_status = 0;

	mutexLock(telit_common.lock);
	for (;;) {
		condWait(telit_common.cond, telit_common.lock, 0);

		if (telit_common.state == TELIT_STATE_REMOVED) {
			TRACE_FAIL("reinitializing");
			telit_init_powerkey();
			telit_init_device();
			telit_init_interface(0);
		}

		acm_control(type, GET_STATUS, 0, acm->ep_in.bEndpointAddress, &in_status, 2);
		acm_control(type, GET_STATUS, 0, acm->ep_out.bEndpointAddress, &out_status, 2);

		if (in_status & 1) {
			TRACE_FAIL("in halted");
			clear_halt(acm->ep_in);
		}

		if (out_status & 1) {
			TRACE_FAIL("out halted");
			clear_halt(acm->ep_out);
		}
	}
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


int telit_init(void)
{
	FUN_TRACE;
	int ret = 0;

	telit_setmux(pctl_mux_sd1_d1, 8);
	telit_setmux(pctl_mux_sd1_d2, 8);
	telit_setmux(pctl_mux_sd1_d3, 8);

	ret |= condCreate(&telit_common.cond);
	ret |= mutexCreate(&telit_common.lock);
	ret |= portCreate(&telit_common.data[0].port);
	telit_common.data[2].port = telit_common.data[1].port = telit_common.data[0].port;
	telit_common.state = TELIT_STATE_REMOVED;

	telit_common.resetting = 0;

	telit_common.data[0].id = 0;
	telit_common.data[1].id = 1;
	telit_common.data[2].id = 2;
	return ret;
}


void telit_start(void)
{
	FUN_TRACE;
	int ret;

	mutexLock(telit_common.lock);
	while (telit_common.state != TELIT_STATE_INSERTED) {
		ret = condWait(telit_common.cond, telit_common.lock, 5000000);

		if (ret == -ETIME)
			telit_init_powerkey();
	}
	mutexUnlock(telit_common.lock);
}


int usb_connect(void)
{
	FUN_TRACE;
	usb_device_id_t id;

	id.idVendor = TELIT_ID_VENDOR;
	id.idProduct = TELIT_ID_PRODUCT;
	id.bcdDevice = USB_CONNECT_NONE;
	id.bDeviceClass = USB_CONNECT_NONE;
	id.bDeviceSubClass = USB_CONNECT_NONE;
	id.bDeviceProtocol = USB_CONNECT_NONE;

	if (libusb_connect(&id, (void *)event_cb)) {
		TRACE_FAIL("connect with usbd");
		return -1;
	}

	return EOK;
}


int main(int argc, char **argv)
{
	oid_t oid;

	openlog("gsm", LOG_CONS, LOG_DAEMON);

	if (libusb_init() < 0) {
		TRACE_FAIL("init libusb");
		return -1;
	}

	if (telit_init() > 0) {
		TRACE_FAIL("module init");
		return -1;
	}

	if (usb_connect() < 0)
		return -1;

	telit_start();

	if (telit_init_all() < 0) {
		TRACE_FAIL("interface init");
		return -1;
	}

	beginthread(telit_resubmitThread, 4, malloc(0x4000), 0x4000, telit_common.data);
	beginthread(telit_resubmitThread, 4, malloc(0x4000), 0x4000, telit_common.data + 1);
	beginthread(telit_resubmitThread, 4, malloc(0x4000), 0x4000, telit_common.data + 2);

	beginthread(telit_readThread, 4, malloc(0x4000), 0x4000, telit_common.data);
	beginthread(telit_readThread, 4, malloc(0x4000), 0x4000, telit_common.data + 1);
	beginthread(telit_readThread, 4, malloc(0x4000), 0x4000, telit_common.data + 2);

	oid = (oid_t){ .port = telit_common.data[0].port, .id = 0 };
	create_dev(&oid, "ttyacm0");

	oid = (oid_t){ .port = telit_common.data[0].port, .id = 1 };
	create_dev(&oid, "ttyacm1");

	oid = (oid_t){ .port = telit_common.data[0].port, .id = 2 };
	create_dev(&oid, "ttyacm2");

	printf("telit-le910: initialized\n");
	telit_msgThread((void *)telit_common.data[0].port);
}
