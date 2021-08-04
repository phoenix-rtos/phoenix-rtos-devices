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

#define USBACM_N_MSG_THREADS 2


typedef struct usbacm_dev {
	char rxbuf[1024] __attribute__ ((aligned(32)));
	char txbuf[1024] __attribute__ ((aligned(32)));

	struct usbacm_dev *prev, *next;
	char path[32];
	usb_devinfo_t instance;
	int pipeCtrl;
	int pipeIntIN;
	int pipeBulkIN;
	int pipeBulkOUT;
	int id;
	unsigned port;
	oid_t oid;
	handle_t readLock;
	handle_t writeLock;
	int valid;
} usbacm_dev_t;


static struct {
	char stack[USBACM_N_MSG_THREADS][1024] __attribute__ ((aligned(8)));
	usbacm_dev_t *devices;
	unsigned drvport;
	unsigned msgport;
	handle_t lock;
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

	return EOK;
}


static int usbacm_read(usbacm_dev_t *dev, char *data, size_t len)
{
	size_t toread = 0;
	int ret = 0;

	/* NOTE: rxbuf and txbuf are needed, because the USB stack requires buffer
	 * to reside in an uncached memory. For optimal utilization they should be
	 * 32-byte alligned */
	toread = min(sizeof(dev->rxbuf), len);

	mutexLock(dev->readLock);
	if ((ret = usb_transferBulk(dev->pipeBulkIN, dev->rxbuf, toread, usb_dir_in)) >= 0) {
		memcpy(data, dev->rxbuf, ret);
	}
	else {
		fprintf(stderr, "usbacm: read failed\n");
		ret = -EIO;
	}
	mutexUnlock(dev->readLock);

	return ret;
}


static int usbacm_write(usbacm_dev_t *dev, char *data, size_t len)
{
	size_t written = 0, tmp;
	int ret = 0;

	mutexLock(dev->writeLock);
	while (written < len) {
		tmp = min(sizeof(dev->txbuf), len);
		memcpy(dev->txbuf, data + written, tmp);
		if ((ret = usb_transferBulk(dev->pipeBulkOUT, dev->txbuf, tmp, usb_dir_out)) < 0) {
			fprintf(stderr, "usbacm: write failed\n");
			ret = -EIO;
			break;
		}
		written += tmp;
	}
	mutexUnlock(dev->writeLock);

	return ret;
}


static usbacm_dev_t *usbacm_devFind(int id)
{
	usbacm_dev_t *tmp = usbacm_common.devices;

	if (tmp != NULL) {
		do {
			if (tmp->id == id - 1 && tmp->valid)
				return tmp;
			tmp = tmp->next;
		} while (tmp != usbacm_common.devices);
	}

	return NULL;
}


static usbacm_dev_t *usbacm_devAlloc(void)
{
	usbacm_dev_t *dev, *tmp;

	/* Try to find a free device entry, otherwise allocate it */
	dev = NULL;
	tmp = usbacm_common.devices;
	if (tmp != NULL) {
		do {
			if (!tmp->valid) {
				dev = tmp;
				break;
			}
		} while ((tmp = tmp->next) != usbacm_common.devices);
	}

	if (dev == NULL) {
		if ((dev = malloc(sizeof(usbacm_dev_t))) == NULL) {
			fprintf(stderr, "usbacm: Not enough memory\n");
			return NULL;
		}

		/* Get next device number */
		if (usbacm_common.devices == NULL)
			dev->id = 0;
		else
			dev->id = usbacm_common.devices->prev->id + 1;

		if (mutexCreate(&dev->readLock)) {
			free(dev);
			return NULL;
		}

		if (mutexCreate(&dev->writeLock)) {
			free(dev);
			return NULL;
		}

		snprintf(dev->path, sizeof(dev->path), "/dev/usbacm%d", dev->id);
		dev->valid = 0;
		LIST_ADD(&usbacm_common.devices, dev);
	}

	return dev;
}


static void usbacm_msgthr(void *arg)
{
	usbacm_dev_t *dev;
	unsigned long rid;
	msg_t msg;

	for (;;) {
		if (msgRecv(usbacm_common.msgport, &msg, &rid) < 0) {
			fprintf(stderr, "usbacm: msgRecv returned with err\n");
			break;
		}

		/* Ignore this msg, as it might have been sent by us after deletion event */
		if (msg.type == mtUnlink) {
			msg.o.io.err = EOK;
			msgRespond(usbacm_common.msgport, &msg, rid);
			continue;
		}

		mutexLock(usbacm_common.lock);
		dev = usbacm_devFind(msg.i.io.oid.id);
		mutexUnlock(usbacm_common.lock);

		if (dev == NULL) {
			msg.o.io.err = -ENOENT;
			msgRespond(usbacm_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;
			case mtRead:
				msg.o.io.err = usbacm_read(dev, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.io.err = usbacm_write(dev, msg.i.data, msg.i.size);
				break;

			case mtGetAttr:
				msg.o.attr.val = -EINVAL;
				break;

			default:
				msg.o.io.err = -EINVAL;
		}

		msgRespond(usbacm_common.msgport, &msg, rid);
	}

	endthread();
}


static int usbacm_handleInsertion(usb_devinfo_t *insertion)
{
	usbacm_dev_t *dev;
	const usb_modeswitch_t *mode;
	oid_t oid;

	if ((mode = usb_modeswitchFind(insertion->descriptor.idVendor,
	                               insertion->descriptor.idProduct,
								   modeswitch, sizeof(modeswitch) / sizeof(modeswitch[0]))) != NULL) {
		return usb_modeswitchHandle(insertion, mode);
	}

	if ((dev = usbacm_devAlloc()) == NULL)
		return -ENOMEM;

	dev->instance = *insertion;

	if ((dev->pipeCtrl = usb_open(insertion, usb_transfer_control, 0)) < 0)
		return -EINVAL;

	if (usb_setConfiguration(dev->pipeCtrl, 1) != 0)
		return -EINVAL;

	if ((dev->pipeBulkIN = usb_open(insertion, usb_transfer_bulk, usb_dir_in)) < 0)
		return -EINVAL;

	if ((dev->pipeBulkOUT = usb_open(insertion, usb_transfer_bulk, usb_dir_out)) < 0)
		return -EINVAL;

	dev->pipeIntIN = usb_open(insertion, usb_transfer_interrupt, usb_dir_in);

	if (usbacm_init(dev) != EOK) {
		fprintf(stderr, "usbacm: Init failed\n");
		return -EINVAL;
	}

	oid.port = usbacm_common.msgport;
	oid.id = dev->id + 1;
	if (create_dev(&oid, dev->path) != 0) {
		fprintf(stderr, "usbacm: Can't create dev!\n");
		return -EINVAL;
	}

	dev->valid = 1;
	fprintf(stderr, "usbacm: New device: %s\n", dev->path);

	return 0;
}


static int usbacm_handleDeletion(usb_deletion_t *del)
{
	usbacm_dev_t *dev = usbacm_common.devices;

	if (dev == NULL)
		return 0;

	do {
		if (dev->valid && dev->instance.bus == del->bus && dev->instance.dev == del->dev &&
		    dev->instance.interface == del->interface) {
			dev->valid = 0;
			remove(dev->path);
			fprintf(stderr, "usbacm: Device removed: %s\n", dev->path);
		}
		dev = dev->next;
	} while (dev != usbacm_common.devices);

	return 0;
}


int main(int argc, char *argv[])
{
	int ret;
	msg_t msg;
	usb_msg_t *umsg = (usb_msg_t *)msg.i.raw;
	int i;

	/* Port for communication with the USB stack */
	if (portCreate(&usbacm_common.drvport) != 0) {
		fprintf(stderr, "usbacm: Can't create port!\n");
		return -ENOMEM;
	}

	/* Port for communication with driver clients */
	if (portCreate(&usbacm_common.msgport) != 0) {
		fprintf(stderr, "usbacm: Can't create port!\n");
		return -ENOMEM;
	}

	if (mutexCreate(&usbacm_common.lock)) {
		fprintf(stderr, "usbacm: Can't create mutex!\n");
		return -ENOMEM;
	}

	if ((usb_connect(filters, sizeof(filters) / sizeof(filters[0]), usbacm_common.drvport)) < 0) {
		fprintf(stderr, "usbacm: Fail to connect to usb host!\n");
		return -EINVAL;
	}

	for (i = 0; i < USBACM_N_MSG_THREADS; i++) {
		if ((ret = beginthread(usbacm_msgthr, 4, usbacm_common.stack[i], sizeof(usbacm_common.stack[i]), NULL)) != 0) {
			fprintf(stderr, "usbacm: fail to beginthread ret: %d\n", ret);
			return -1;
		}
	}

	for (;;) {
		ret = usb_eventsWait(usbacm_common.drvport, &msg);
		if (ret != 0)
			continue;
		mutexLock(usbacm_common.lock);
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
		mutexUnlock(usbacm_common.lock);
	}

	return 0;
}
