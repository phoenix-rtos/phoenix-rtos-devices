/*
 * Phoenix-RTOS
 *
 * USB CDC ACM driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Maciej Purski
 *
 * %LICENSE%
 */

#include <errno.h>
#include <fcntl.h>
#include <sys/list.h>
#include <sys/msg.h>
#include <sys/types.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <sys/minmax.h>
#include <posix/utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <usb.h>
#include <usbdriver.h>
#include <cdc.h>

#include "../libtty/fifo.h"

#define RX_FIFO_SIZE (8 * _PAGE_SIZE)

typedef struct usbacm_dev {
	char stack[2][1024] __attribute__ ((aligned(8)));
	int tid[2];
	struct usbacm_dev *prev, *next;
	char path[32];
	usb_insertion_t instance;
	int pipeCtrl;
	int pipeIntIN;
	int pipeBulkIN;
	int pipeBulkOUT;
	int id;
	char rxbuf[512];
	char txbuf[1024];
	unsigned port;
	handle_t readLock;
	fifo_t *fifo;
} usbacm_dev_t;


static struct {
	usbacm_dev_t *devices;
	unsigned drvport;
} usbacm_common;

static const usb_device_id_t filters[] = {
	/* Huawei E3372 - Mass Storage mode for modeswitch */
	{ 0x12d1, 0x1f01, USBDRV_ANY, USBDRV_ANY, USBDRV_ANY },
	/* Huawei E3372 - PPP mode */
	{ 0x12d1, 0x1001, USBDRV_ANY, USBDRV_ANY, USBDRV_ANY },
	/* Telit FN980 */
	{ 0x1bC7, 0x0036, USBDRV_ANY, USBDRV_ANY, USBDRV_ANY },
	/* USB CDC ACM class */
	{ USBDRV_ANY, USBDRV_ANY, USB_CLASS_CDC, USB_SUBCLASS_ACM, USBDRV_ANY },
};

static const usb_modeswitch_t modeswitch[] = {
	{
		/* Huawei E3372 ACM mode */
		.vid = 0x12d1,
		.pid = 0x1f01,
		.msg = { 0x55, 0x53, 0x42, 0x43,
		         0x12, 0x34, 0x56, 0x78,
				 0x00, 0x00, 0x00, 0x00,
				 0x00, 0x00, 0x00, 0x11,
				 0x06, 0x30, 0x00, 0x00,
				 0x01, 0x00, 0x00, 0x00,
				 0x00, 0x00, 0x00, 0x00,
				 0x00, 0x00, 0x00},
		.scsiresp = 1
	},
};


static int usbacm_init(usbacm_dev_t *dev)
{
	usb_cdc_line_coding_t line_coding;
	usb_setup_packet_t setup;

	type = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE;

	line_coding.dwDTERate = 0xc1200;
	line_coding.bCharFormat = 0;
	line_coding.bParityType = 0;
	line_coding.bDataBits = 8;

	setup.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE;
	setup.bRequest = USB_CDC_REQ_SET_LINE_CODING;
	setup.wIndex = dev->instance.interface;
	setup.wValue = 0;
	setup.wLength = sizeof(line_coding);

	if (usb_transferControl(dev->pipeCtrl, &setup, &line_coding, sizeof(line_coding), usb_dir_out) < 0) {
		fprintf(stderr, "usbacm: Control transfer failed\n");
		return -EIO;
	}

	setup.bRequest = USB_CDC_REQ_SET_CONTROL_LINE_STATE;
	setup.wValue = 3;
	setup.wLength = 0;
	if (usb_transferControl(dev->pipeCtrl, &setup, NULL, 0, usb_dir_out) < 0) {
		fprintf(stderr, "usbacm: Control transfer failed\n");
		return -EIO;
	}

	dev->fifo = malloc(sizeof(fifo_t) + RX_FIFO_SIZE * sizeof(dev->fifo->data[0]));
	if (dev->fifo == NULL) {
		fprintf(stderr, "usbacm: Fail to allocate pipe\n");
		return -ENOMEM;
	}

	fifo_init(dev->fifo, RX_FIFO_SIZE);

	return EOK;
}


static int usbacm_read(usbacm_dev_t *dev, char *buf, size_t len)
{
	int i = 0;

	mutexLock(dev->readLock);
	if (dev->fifo) {
		for (; i < len && !fifo_is_empty(dev->fifo); i++)
			buf[i] = fifo_pop_back(dev->fifo);
	}
	mutexUnlock(dev->readLock);

	return i;
}


static int usbacm_write(usbacm_dev_t *dev, char *data, size_t len)
{
	size_t written = 0, tmp;
	int ret = 0;

	while (written < len) {
		tmp = min(sizeof(dev->txbuf), len);
		memcpy(dev->txbuf, data + written, tmp);
		if ((ret = usb_transferBulk(dev->pipeBulkOUT, dev->txbuf, tmp, usb_dir_out)) < 0) {
			fprintf(stderr, "usbacm: write failed\n");
			return -EIO;
		}
		written += tmp;
	}

	return ret;
}


static void usbacm_readthr(void *arg)
{
	usbacm_dev_t *dev = (usbacm_dev_t *)arg;
	int ret, i;

	dev->tid[1] = gettid();
	for (;;) {
		if ((ret = usb_transferBulk(dev->pipeBulkIN, dev->rxbuf, sizeof(dev->rxbuf), usb_dir_in)) < 0) {
			fprintf(stderr, "usbacm%d: read failed\n", dev->id);
			break;
		}

		mutexLock(dev->readLock);
		for (i = 0; i < ret && !fifo_is_full(dev->fifo); i++)
			fifo_push(dev->fifo, dev->rxbuf[i]);
		mutexUnlock(dev->readLock);
	}

	mutexLock(dev->readLock);
	free(dev->fifo);
	dev->fifo = NULL;
	mutexUnlock(dev->readLock);

	endthread();
}


static void usbacm_msgthr(void *arg)
{
	usbacm_dev_t *dev = (usbacm_dev_t *)arg;
	unsigned long rid;
	msg_t msg;

	dev->tid[0] = gettid();
	for (;;) {
		if (msgRecv(dev->port, &msg, &rid) < 0) {
			fprintf(stderr, "usbacm: msgRecv returned with err\n");
			break;
		}

		switch (msg.type) {
		case mtOpen:
		case mtClose:
			msg.o.io.err = EOK;
			break;

		case mtRead:
			if ((msg.o.io.err = usbacm_read(dev, msg.o.data, msg.o.size)) == 0) {
				if (msg.i.io.mode & O_NONBLOCK) {
					msg.o.io.err = -EWOULDBLOCK;
				}
				else {
					// LIST_ADD(&dev->readers, req);
					// req = NULL;
				}
			}
			break;

		case mtWrite:
			msg.o.io.err = usbacm_write(dev, msg.i.data, msg.i.size);
			break;

		case mtGetAttr:
			break;

		default:
			msg.o.io.err = -EINVAL;
		}
		msgRespond(dev->port, &msg, rid);
	}

	endthread();
}


static int usbacm_handleInsertion(usb_insertion_t *insertion)
{
	usbacm_dev_t *dev;
	usb_modeswitch_t *mode;
	oid_t oid;
	int ret;

	if ((mode = usb_modeswitchFind(insertion->descriptor.idVendor,
	                               insertion->descriptor.idProduct,
								   modeswitch, sizeof(modeswitch) / sizeof(modeswitch[0]))) != NULL) {
		return usb_modeswitchHandle(insertion, mode);
	}

	/* TODO: temporary in order to reduce memory usage */
	if (usbacm_common.devices != NULL)
		return 0;

	if ((dev = malloc(sizeof(usbacm_dev_t))) == NULL) {
		fprintf(stderr, "usbacm: Not enough memory\n");
		return -ENOMEM;
	}
	dev->instance = *insertion;

	if ((dev->pipeCtrl = usb_open(insertion, usb_transfer_control, usb_dir_bi)) < 0) {
		free(dev);
		return -EINVAL;
	}

	if (usb_setConfiguration(dev->pipeCtrl, 1) != 0) {
		free(dev);
		return -EINVAL;
	}

	if ((dev->pipeBulkIN = usb_open(insertion, usb_transfer_bulk, usb_dir_in)) < 0) {
		free(dev);
		return -EINVAL;
	}

	if ((dev->pipeBulkOUT = usb_open(insertion, usb_transfer_bulk, usb_dir_out)) < 0) {
		free(dev);
		return -EINVAL;
	}

	dev->pipeIntIN = usb_open(insertion, usb_transfer_interrupt, usb_dir_in);

	if ((ret = portCreate(&dev->port)) != 0) {
		fprintf(stderr, "usbacm: Can't create port ret: %d!\n", ret);
		return -EINVAL;
	}

	/* Get next device number */
	if (usbacm_common.devices == NULL)
		dev->id = 0;
	else
		dev->id = usbacm_common.devices->prev->id + 1;

	if (usbacm_init(dev) != EOK) {
		fprintf(stderr, "usbacm: Init failed\n");
		free(dev);
		return -EINVAL;
	}

	snprintf(dev->path, sizeof(dev->path), "/dev/usbacm%d", dev->id);
	oid.port = dev->port;
	oid.id = dev->id;
	if (create_dev(&oid, dev->path) != 0) {
		fprintf(stderr, "usbacm: Can't create dev!\n");
		return -EINVAL;
	}

	if (mutexCreate(&dev->readLock) != 0) {
		fprintf(stderr, "usbacm: Can't create mutex!\n");
		return -EINVAL;
	}

	LIST_ADD(&usbacm_common.devices, dev);
	dev->tid[0] = 0;
	dev->tid[1] = 0;

	fprintf(stderr, "usbacm: New device: %s\n", dev->path);
	if ((ret = beginthread(usbacm_msgthr, 4, dev->stack[0], sizeof(dev->stack[0]), dev)) != 0) {
		fprintf(stderr, "usbacm: fail to beginthread ret: %d\n", ret);
		return -1;
	}


	if ((ret = beginthread(usbacm_readthr, 4, dev->stack[1], sizeof(dev->stack[1]), dev)) != 0) {
		fprintf(stderr, "usbacm: fail to beginthread ret: %d\n", ret);
		return -1;
	}

	return 0;
}


static int usbacm_handleDeletion(usb_deletion_t *del)
{
	usbacm_dev_t *dev = usbacm_common.devices;
	int err, cnt = 0;

	if (dev == NULL)
		return 0;

	do {
		if (dev->instance.bus == del->bus && dev->instance.dev == del->dev &&
		    dev->instance.interface == del->interface) {
			LIST_REMOVE(&usbacm_common.devices, dev);
			portDestroy(dev->port);
			remove(dev->path);
 			while (cnt < 2) {
				err = threadJoin(0);
				if (err == dev->tid[0] || err == dev->tid[1])
					cnt++;
			}
			resourceDestroy(dev->readLock);
			free(dev);
			break;
		}
	} while ((dev = dev->next) != usbacm_common.devices);

	return 0;
}


int main(int argc, char *argv[])
{
	int ret;
	msg_t msg;
	usb_msg_t *umsg = (usb_msg_t *)msg.i.raw;

	if (portCreate(&usbacm_common.drvport) != 0) {
		fprintf(stderr, "usbacm: Can't create port!\n");
		return -EINVAL;
	}

	if ((usb_connect(filters, sizeof(filters) / sizeof(filters[0]), usbacm_common.drvport)) < 0) {
		fprintf(stderr, "usbacm: Fail to connect to usb host!\n");
		return -EINVAL;
	}

	for (;;) {
		ret = usb_eventsWait(usbacm_common.drvport, &msg);
		if (ret != 0)
			continue;
		switch (umsg->type) {
			case usb_msg_insertion:
				usbacm_handleInsertion(&umsg->insertion);
				break;
			case usb_msg_deletion:
				usbacm_handleDeletion(&umsg->deletion);
				break;
			default:
				fprintf(stderr, "usbacm: Error when receiving event from host\n");
				break;
		}
	}

	return 0;
}
