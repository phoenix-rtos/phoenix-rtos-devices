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
#include <poll.h>
#include <posix/utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <usb.h>
#include <usbdriver.h>
#include <cdc.h>

#include "../libtty/fifo.h"

#define USBACM_N_MSG_THREADS 2

#define RX_FIFO_SIZE  4096
#define RX_STACK_SIZE 2048

typedef struct _usbacm_dev {
	struct _usbacm_dev *prev, *next;
	char *stack;
	int rxtid;
	char path[32];
	usb_devinfo_t instance;
	int pipeCtrl;
	int pipeIntIN;
	int pipeBulkIN;
	int pipeBulkOUT;
	int id;
	int fileId;
	int flags;
	volatile int rxRunning;
	volatile int thrFinished;
	unsigned port;
	fifo_t *fifo;
	oid_t oid;
	handle_t fifoLock;
	handle_t fifoFull;
} usbacm_dev_t;


static struct {
	char stack[USBACM_N_MSG_THREADS][1024] __attribute__((aligned(8)));
	usbacm_dev_t *devices;
	unsigned drvport;
	unsigned msgport;
	handle_t lock;
	int lastId;
} usbacm_common;


static const usb_device_id_t filters[] = {
	/* Huawei E3372 - Mass Storage mode for modeswitch */
	{ 0x12d1, 0x1f01, USBDRV_ANY, USBDRV_ANY, USBDRV_ANY },
	/* Huawei E3372 - PPP mode */
	{ 0x12d1, 0x1001, USBDRV_ANY, USBDRV_ANY, USBDRV_ANY },
	/* Telit FN980 */
	{ 0x1bc7, 0x1050, 0xff, 0, 0 },
	/* USB CDC ACM class */
	{ USBDRV_ANY, USBDRV_ANY, USB_CLASS_CDC, USB_SUBCLASS_ACM, USBDRV_ANY },
};


static const usb_modeswitch_t modeswitch[] = {
	/* Huawei E3372 ACM mode */
	{
		.vid = 0x12d1,
		.pid = 0x1f01,
		.msg = { 0x55, 0x53, 0x42, 0x43,
			0x12, 0x34, 0x56, 0x78,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x11,
			0x06, 0x30, 0x00, 0x00,
			0x01, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00 },
		.scsiresp = 1 },
};


static int usbacm_init(usbacm_dev_t *dev)
{
	usb_setup_packet_t setup;

	setup.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE;
	setup.bRequest = USB_CDC_REQ_SET_CONTROL_LINE_STATE;
	setup.wIndex = dev->instance.interface;
	setup.wValue = 3;
	setup.wLength = 0;

	if (usb_transferControl(dev->pipeCtrl, &setup, NULL, 0, usb_dir_out) < 0) {
		fprintf(stderr, "usbacm: Control transfer failed\n");
		return -EIO;
	}

	return EOK;
}


static void usbacm_rxthr(void *arg)
{
	usbacm_dev_t *dev = (usbacm_dev_t *)arg;
	char buf[512];
	int i, ret;

	mutexLock(dev->fifoLock);
	dev->rxRunning = 1;
	dev->rxtid = gettid();
	mutexUnlock(dev->fifoLock);

	for (;;) {
		if ((ret = usb_transferBulk(dev->pipeBulkIN, buf, sizeof(buf), usb_dir_in)) < 0) {
			fprintf(stderr, "usbacm%d: read failed\n", dev->id);
			break;
		}

		mutexLock(dev->fifoLock);
		for (i = 0; i < ret; i++) {
			while (fifo_is_full(dev->fifo))
				condWait(dev->fifoFull, dev->fifoLock, 0);
			fifo_push(dev->fifo, buf[i]);
		}
		mutexUnlock(dev->fifoLock);
	}

	mutexLock(dev->fifoLock);
	dev->rxRunning = 0;
	mutexUnlock(dev->fifoLock);

	endthread();
}


static int _usbacm_readNonblock(usbacm_dev_t *dev, char *data, size_t len)
{
	int full, i;

	if (!dev->rxRunning)
		return -EIO;

	full = fifo_is_full(dev->fifo);
	for (i = 0; i < len && !fifo_is_empty(dev->fifo); i++)
		data[i] = fifo_pop_back(dev->fifo);
	if (full)
		condSignal(dev->fifoFull);

	return i;
}


static int usbacm_read(usbacm_dev_t *dev, char *data, size_t len)
{
	int ret = 0;
	int flags;

	mutexLock(usbacm_common.lock);
	flags = dev->flags;
	mutexUnlock(usbacm_common.lock);

	if (flags & O_NONBLOCK) {
		mutexLock(dev->fifoLock);
		ret = _usbacm_readNonblock(dev, data, len);
		mutexUnlock(dev->fifoLock);
	}
	else {
		if ((ret = usb_transferBulk(dev->pipeBulkIN, data, len, usb_dir_in)) < 0) {
			fprintf(stderr, "usbacm: read failed\n");
			ret = -EIO;
		}
	}

	return ret;
}


static int usbacm_write(usbacm_dev_t *dev, char *data, size_t len)
{
	int ret = 0;

	if ((ret = usb_transferBulk(dev->pipeBulkOUT, data, len, usb_dir_out)) < 0) {
		fprintf(stderr, "usbacm: write failed\n");
		ret = -EIO;
	}

	return ret;
}


static usbacm_dev_t *usbacm_devFind(int id)
{
	usbacm_dev_t *tmp, *dev = NULL;

	mutexLock(usbacm_common.lock);
	tmp = usbacm_common.devices;
	if (tmp != NULL) {
		do {
			if (tmp->fileId == id) {
				dev = tmp;
				break;
			}
			tmp = tmp->next;
		} while (tmp != usbacm_common.devices);
	}
	mutexUnlock(usbacm_common.lock);

	return dev;
}


static usbacm_dev_t *usbacm_devAlloc(void)
{
	usbacm_dev_t *dev;

	if ((dev = calloc(1, sizeof(usbacm_dev_t))) == NULL) {
		fprintf(stderr, "usbacm: Not enough memory\n");
		return NULL;
	}

	/* Get next device number */
	if (usbacm_common.devices == NULL)
		dev->id = 0;
	else
		dev->id = usbacm_common.devices->prev->id + 1;

	dev->fileId = usbacm_common.lastId++;

	snprintf(dev->path, sizeof(dev->path), "/dev/usbacm%d", dev->id);

	return dev;
}


static int _usbacm_open(usbacm_dev_t *dev, int flags)
{
	fifo_t *fifo;

	if (dev->flags != 0)
		return -EIO;

	if (flags & O_NONBLOCK && dev->fifo == NULL) {
		if ((fifo = malloc(sizeof(fifo_t) + RX_FIFO_SIZE)) == NULL)
			return -ENOMEM;

		fifo_init(fifo, RX_FIFO_SIZE);

		if (mutexCreate(&dev->fifoLock) != 0) {
			free(fifo);
			return -ENOMEM;
		}

		if (condCreate(&dev->fifoFull) != 0) {
			resourceDestroy(dev->fifoLock);
			free(fifo);
			return -ENOMEM;
		}

		if ((dev->stack = malloc(RX_STACK_SIZE)) == NULL) {
			free(fifo);
			resourceDestroy(dev->fifoLock);
			resourceDestroy(dev->fifoFull);
			return -ENOMEM;
		}

		if (beginthread(usbacm_rxthr, 4, dev->stack, RX_STACK_SIZE, dev) != 0) {
			resourceDestroy(dev->fifoLock);
			resourceDestroy(dev->fifoFull);
			free(fifo);
			free(dev->stack);
			return -ENOMEM;
		}
		dev->fifo = fifo;
	}

	dev->flags = flags;

	return EOK;
}


static void _usbacm_close(usbacm_dev_t *dev)
{
	dev->flags = 0;
}


static void usbacm_msgthr(void *arg)
{
	usbacm_dev_t *dev;
	unsigned long rid;
	msg_t msg;
	int id;

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

		if (msg.type == mtGetAttr)
			id = msg.i.attr.oid.id;
		else if (msg.type == mtOpen)
			id = msg.i.openclose.oid.id;
		else
			id = msg.i.io.oid.id;

		if ((dev = usbacm_devFind(id)) == NULL) {
			msg.o.io.err = -ENOENT;
			msgRespond(usbacm_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				mutexLock(usbacm_common.lock);
				msg.o.io.err = _usbacm_open(dev, msg.i.openclose.flags);
				mutexUnlock(usbacm_common.lock);
				break;

			case mtClose:
				mutexLock(usbacm_common.lock);
				_usbacm_close(dev);
				mutexUnlock(usbacm_common.lock);
				msg.o.io.err = EOK;
				break;

			case mtRead:
				if ((msg.o.io.err = usbacm_read(dev, msg.o.data, msg.o.size)) == 0) {
					if (msg.i.io.mode & O_NONBLOCK)
						msg.o.io.err = -EWOULDBLOCK;
				}
				break;

			case mtWrite:
				msg.o.io.err = usbacm_write(dev, msg.i.data, msg.i.size);
				break;

			case mtGetAttr:
				if ((msg.i.attr.type == atPollStatus)) {
					msg.o.attr.val = fifo_is_empty(dev->fifo) ? POLLOUT : POLLOUT | POLLIN;
					msg.o.attr.err = EOK;
					break;
				}
				else {
					msg.o.attr.err = -EINVAL;
				}
			default:
				msg.o.io.err = -EINVAL;
		}

		msgRespond(usbacm_common.msgport, &msg, rid);
	}

	endthread();
}


static int _usbacm_handleInsertion(usb_devinfo_t *insertion)
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

	if ((dev->pipeCtrl = usb_open(insertion, usb_transfer_control, 0)) < 0) {
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

	/* Interrupt pipe is optional */
	dev->pipeIntIN = usb_open(insertion, usb_transfer_interrupt, usb_dir_in);

	if (usbacm_init(dev) != EOK) {
		free(dev);
		fprintf(stderr, "usbacm: Init failed\n");
		return -EINVAL;
	}

	oid.port = usbacm_common.msgport;
	oid.id = dev->fileId;
	if (create_dev(&oid, dev->path) != 0) {
		free(dev);
		fprintf(stderr, "usbacm: Can't create dev!\n");
		return -EINVAL;
	}

	LIST_ADD(&usbacm_common.devices, dev);

	fprintf(stderr, "usbacm: New device: %s\n", dev->path);

	return 0;
}


static int _usbacm_threadJoin(usbacm_dev_t *dev)
{
	usbacm_dev_t *tmp;
	int err;

	while (!dev->thrFinished) {
		if ((err = threadJoin(0)) < 0)
			return err;

		/* Find a device, whose thread has ended */
		tmp = usbacm_common.devices;
		do {
			if (tmp->rxtid == err) {
				tmp->thrFinished = 1;
				break;
			}
			tmp = tmp->next;
		} while (tmp != usbacm_common.devices);
	}

	return 0;
}


static int _usbacm_handleDeletion(usb_deletion_t *del)
{
	usbacm_dev_t *next, *dev = usbacm_common.devices;

	if (dev == NULL)
		return 0;

	do {
		next = dev->next;
		if (dev->instance.bus == del->bus && dev->instance.dev == del->dev &&
				dev->instance.interface == del->interface) {
			fprintf(stderr, "usbacm: Device removed: %s\n", dev->path);
			if (dev->fifo != NULL) {
				_usbacm_threadJoin(dev);
				/* Non-blocking device */
				free(dev->fifo);
				dev->fifo = NULL;
				resourceDestroy(dev->fifoLock);
				resourceDestroy(dev->fifoFull);
				free(dev->stack);
			}
			remove(dev->path);
			LIST_REMOVE(&usbacm_common.devices, dev);
			free(dev);
			if (dev == next)
				break;
		}
		dev = next;
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
		return 1;
	}

	/* Port for communication with driver clients */
	if (portCreate(&usbacm_common.msgport) != 0) {
		fprintf(stderr, "usbacm: Can't create port!\n");
		return 1;
	}

	if (mutexCreate(&usbacm_common.lock)) {
		fprintf(stderr, "usbacm: Can't create mutex!\n");
		return 1;
	}

	if ((usb_connect(filters, sizeof(filters) / sizeof(filters[0]), usbacm_common.drvport)) < 0) {
		fprintf(stderr, "usbacm: Fail to connect to usb host!\n");
		return 1;
	}

	usbacm_common.lastId = 1;

	for (i = 0; i < USBACM_N_MSG_THREADS; i++) {
		if ((ret = beginthread(usbacm_msgthr, 4, usbacm_common.stack[i], sizeof(usbacm_common.stack[i]), NULL)) != 0) {
			fprintf(stderr, "usbacm: fail to beginthread ret: %d\n", ret);
			return 1;
		}
	}

	for (;;) {
		ret = usb_eventsWait(usbacm_common.drvport, &msg);
		if (ret != 0)
			return 1;
		mutexLock(usbacm_common.lock);
		switch (umsg->type) {
			case usb_msg_insertion:
				_usbacm_handleInsertion(&umsg->insertion);
				break;
			case usb_msg_deletion:
				_usbacm_handleDeletion(&umsg->deletion);
				break;
			default:
				fprintf(stderr, "usbacm: Error when receiving event from host\n");
				break;
		}
		mutexUnlock(usbacm_common.lock);
	}

	return 0;
}
