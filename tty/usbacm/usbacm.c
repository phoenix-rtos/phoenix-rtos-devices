/*
 * Phoenix-RTOS
 *
 * USB CDC ACM driver
 *
 * Copyright 2021, 2022 Phoenix Systems
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
#include <board_config.h>

#include "../libtty/fifo.h"


#ifndef USBACM_N_MSG_THREADS
#define USBACM_N_MSG_THREADS 2
#endif

#ifndef USBACM_N_UMSG_THREADS
#define USBACM_N_UMSG_THREADS 2
#endif

#ifndef RX_FIFO_SIZE
#define RX_FIFO_SIZE  8192
#endif

#ifndef USBACM_N_URBS
#define USBACM_N_URBS 2
#endif

#ifndef USBACM_MSG_PRIO
#define USBACM_MSG_PRIO 3
#endif

#ifndef USBACM_UMSG_PRIO
#define USBACM_UMSG_PRIO 3
#endif

#define USBACM_BULK_SZ 2048


typedef struct _usbacm_dev {
	struct _usbacm_dev *prev, *next;
	int rxtid;
	char path[32];
	usbdrv_pipe_t *pipeCtrl;
	usbdrv_pipe_t *pipeIntIN;
	usbdrv_pipe_t *pipeBulkIN;
	usbdrv_pipe_t *pipeBulkOUT;
	int flags;
	pid_t clientpid;
	volatile int thrFinished;
	unsigned port;
	fifo_t *fifo;
	char *rxptr;
	int rxrunning;
	size_t rxbuflen;
	oid_t oid;
	handle_t rxLock;
	handle_t rxCond;
	usbdrv_urb_t *urbs[USBACM_N_URBS];
	char *bufs[USBACM_N_URBS];
	usbdrv_urb_t *urbctrl;
} usbacm_dev_t;


static struct {
	char msgstack[USBACM_N_MSG_THREADS][1024] __attribute__((aligned(8)));
	unsigned msgport;
	handle_t lock;
} usbacm_common;


static const usbdrv_devid_t filters[] = {
	/* Huawei E3372 - Mass Storage mode for modeswitch */
	{ 0x12d1, 0x1f01, USBDRV_ANY, USBDRV_ANY, USBDRV_ANY },
	/* Huawei E3372 - PPP mode */
	{ 0x12d1, 0x1001, USBDRV_ANY, USBDRV_ANY, USBDRV_ANY },
	/* Telit FN980 */
	{ 0x1bc7, 0x1050, 0xff, 0, 0 },
	/* USB CDC ACM class */
	{ USBDRV_ANY, USBDRV_ANY, USB_CLASS_CDC, USB_SUBCLASS_ACM, USBDRV_ANY },
};


static const usbdrv_modeswitch_t modeswitch[] = {
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


static int usbacm_rxStop(usbacm_dev_t *dev)
{
	usb_setup_packet_t setup;

	setup.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE;
	setup.bRequest = USB_CDC_REQ_SET_CONTROL_LINE_STATE;
	setup.wIndex = dev->instance.interface;
	setup.wValue = 0;
	setup.wLength = 0;

	return usbdrv_transferControlAsync(dev->urbctrl, &setup, NULL, 0, 1000);
}


static void usbacm_fifoPush(usbacm_dev_t *dev, char *data, size_t size)
{
	unsigned i;

	mutexLock(dev->rxLock);
	for (i = 0; i < size; i++)
		fifo_push(dev->fifo, data[i]);
	mutexUnlock(dev->rxLock);
}


static void usbacm_handleCompletion(usbdrv_dev_t *dev, usbdrv_urb_t *urb, char *data, size_t transferred, int status)
{
	usbacm_dev_t *dev;
	size_t bytes;
	int retransfer = 0;
	int sig = 0;

	// dev = usbacm_getByPipe(c->pipeid);
	// if (dev == NULL) {
	// 	return;
	// }

	// if (c->err != 0) {
	// 	/* Error, stop receiving */
	// 	mutexLock(dev->rxLock);
	// 	dev->rxrunning = 0;
	// 	if ((dev->flags & O_NONBLOCK) == 0) {
	// 		/* Stop the blocking read in progress */
	// 		sig = 1;
	// 	}
	// 	mutexUnlock(dev->rxLock);
	// 	if (sig == 1) {
	// 		condSignal(dev->rxCond);
	// 	}
	// 	usbacm_put(dev);
	// 	return;
	// }

	// if (c->pipeid != dev->pipeBulkIN->id) {
	// 	usbacm_put(dev);
	// 	return;
	// }

	// if (dev->flags & O_NONBLOCK) {
	// 	usbacm_fifoPush(dev, data, len);
	// 	retransfer = 1;
	// }
	// else if (dev->rxptr != NULL) {
	// 	mutexLock(dev->rxLock);
	// 	if (c->err == 0) {
	// 		bytes = min(len, dev->rxbuflen);
	// 		memcpy(dev->rxptr, data, bytes);
	// 		dev->rxptr += bytes;
	// 		dev->rxbuflen -= bytes;

	// 		if (dev->rxbuflen > 0) {
	// 			retransfer = 1;
	// 		}
	// 		else {
	// 			dev->rxrunning = 0;
	// 			sig = 1;
	// 		}
	// 	}
	// 	else {
	// 		dev->rxrunning = 0;
	// 		sig = 1;
	// 	}
	// 	mutexUnlock(dev->rxLock);
	// }

	// if (retransfer != 0) {
	// 	usbdrv_transferBulkAsync(c, USBACM_BULK_SZ, NULL);
	// }
	// else if (sig == 1) {
	// 	condSignal(dev->rxCond);
	// }

	// usbacm_put(dev);
}


static int usbacm_rxStart(usbacm_dev_t *dev)
{
	usb_setup_packet_t setup;
	int i;

	setup.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE;
	setup.bRequest = USB_CDC_REQ_SET_CONTROL_LINE_STATE;
	setup.wIndex = dev->instance.interface;
	setup.wValue = 3;
	setup.wLength = 0;

	if (usbdrv_transferControlAsync(dev->urbctrl, &setup, NULL, 1000) < 0) {
		return -EIO;
	}

	for (i = 0; i < USBACM_N_URBS; i++) {
		if (usbdrv_transferBulkAsync(dev->urbs[i], dev->bufs[i], USBACM_BULK_SZ, 1000) < 0) {
			return -EIO;
		}
	}

	return 0;
}


static int usbacm_read(usbacm_dev_t *dev, char *data, size_t len)
{
	int ret;

	if ((dev->flags & (O_RDONLY | O_RDWR)) == 0) {
		return -EPERM;
	}

	mutexLock(dev->rxLock);
	if ((dev->flags & O_NONBLOCK) && dev->rxrunning) {
		for (ret = 0; ret < len && !fifo_is_empty(dev->fifo); ret++) {
			data[ret] = fifo_pop_back(dev->fifo);
		}
	}
	else if ((dev->flags & O_NONBLOCK) == 0 && !dev->rxrunning) {
		dev->rxptr = data;
		dev->rxbuflen = len;
		dev->rxrunning = 1;

		usbacm_rxStart(dev);
		/* Block until len bytes is received or an error occurs */
		while (dev->rxbuflen > 0 && dev->rxrunning) {
			condWait(dev->rxCond, dev->rxLock, 0);
		}
		usbacm_rxStop(dev);

		ret = len - dev->rxbuflen;
		dev->rxbuflen = 0;
		dev->rxptr = NULL;
	}
	else {
		ret = -EIO;
	}

	mutexUnlock(dev->rxLock);

	return ret;
}


static int usbacm_write(usbacm_dev_t *dev, char *data, size_t len)
{
	int ret = 0;

	if ((ret = usbdrv_transferBulk(dev->pipeBulkOUT, data, len, 1000)) <= 0) {
		fprintf(stderr, "usbacm: write failed\n");
		ret = -EIO;
	}

	return ret;
}


static usbacm_dev_t *usbacm_devAlloc(usbdrv_dev_t *dev)
{
	usbacm_dev_t *acm;

	if ((acm = calloc(1, sizeof(usbacm_dev_t))) == NULL) {
		fprintf(stderr, "usbacm: Not enough memory\n");
		return NULL;
	}

	snprintf(acm->path, sizeof(acm->path), "/dev/usbacm%d", usbdrv_devID(dev));

	return acm;
}


static int _usbacm_urbsAlloc(usbacm_dev_t *dev)
{
	int i, j;

	dev->urbctrl = usbdrv_urbAlloc(dev->pipeCtrl);
	if (dev->urbctrl < 0) {
		return -1;
	}

	for (i = 0; i < USBACM_N_URBS; i++) {
		dev->urbs[i] = usbdrv_urbAlloc(dev->pipeBulkIN);
		if (dev->urbs[i] < 0) {
			for (j = i - 1; j >= 0; j--) {
				usbdrv_urbFree(dev->pipeBulkIN, dev->urbs[j]);
				dev->urbs[j] = 0;
			}

			usbdrv_urbFree(dev->pipeCtrl, dev->urbctrl);
			return -1;
		}
	}

	return 0;
}


static int _usbacm_openNonblock(usbacm_dev_t *dev)
{
	fifo_t *fifo;
	int ret = 0;

	if ((fifo = malloc(sizeof(fifo_t) + RX_FIFO_SIZE)) == NULL) {
		return -ENOMEM;
	}

	fifo_init(fifo, RX_FIFO_SIZE);

	mutexLock(dev->rxLock);
	dev->fifo = fifo;
	dev->rxrunning = 1;

	if (usbacm_rxStart(dev) != 0) {
		free(fifo);
		dev->fifo = 0;
		dev->rxrunning = 0;
		ret = -EIO;
	}
	mutexUnlock(dev->rxLock);

	return ret;
}


static void _usbacm_close(usbacm_dev_t *dev)
{
	int i;

	for (i = 0; i < USBACM_N_URBS; i++) {
		usbdrv_urbFree(dev->pipeBulkIN, dev->urbs[i]);
	}

	if (dev->flags & O_NONBLOCK) {
		usbacm_rxStop(dev);
	}

	usbdrv_urbFree(dev->pipeCtrl, dev->urbctrl);

	dev->flags = 0;
	dev->clientpid = 0;
}


static int _usbacm_open(usbacm_dev_t *dev, int flags, pid_t pid)
{
	/* Already opened */
	if (dev->flags != 0) {
		return -EBUSY;
	}

	if ((flags & O_RDONLY) || (flags & O_RDWR)) {
		if (mutexCreate(&dev->rxLock) != 0) {
			return -ENOMEM;
		}

		if (condCreate(&dev->rxCond) != 0) {
			resourceDestroy(dev->rxLock);
			return -ENOMEM;
		}

		if (_usbacm_urbsAlloc(dev) < 0) {
			resourceDestroy(dev->rxCond);
			resourceDestroy(dev->rxLock);
			return -EPERM;
		}

		if (flags & O_NONBLOCK) {
			/* Start receiving data */
			if (_usbacm_openNonblock(dev) < 0) {
				resourceDestroy(dev->rxCond);
				resourceDestroy(dev->rxLock);
				_usbacm_close(dev);
				return -EPERM;
			}
		}
	}

	dev->clientpid = pid;
	dev->flags = flags;

	return EOK;
}


static void usbacm_msgthr(void *arg)
{
	usbdrv_dev_t *dev;
	usbacm_dev_t *acm;
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

		dev = usbdrv_devGet(id);
		if (dev == NULL) {
			msg.o.io.err = -ENOENT;
			msgRespond(usbacm_common.msgport, &msg, rid);
			continue;
		}

		acm = (usbacm_dev_t *)dev->ctx;

		/* A device can be opened only by one process */
		if ((msg.type != mtOpen) && (msg.pid != acm->clientpid)) {
			msg.o.io.err = -EBUSY;
			msgRespond(usbacm_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				mutexLock(usbacm_common.lock);
				msg.o.io.err = _usbacm_open(acm, msg.i.openclose.flags, msg.pid);
				mutexUnlock(usbacm_common.lock);
				break;

			case mtClose:
				mutexLock(usbacm_common.lock);
				_usbacm_close(acm);
				mutexUnlock(usbacm_common.lock);
				msg.o.io.err = EOK;
				break;

			case mtRead:
				if ((msg.o.io.err = usbacm_read(acm, msg.o.data, msg.o.size)) == 0) {
					if (msg.i.io.mode & O_NONBLOCK)
						msg.o.io.err = -EWOULDBLOCK;
				}
				break;

			case mtWrite:
				msg.o.io.err = usbacm_write(acm, msg.i.data, msg.i.size);
				break;

			case mtGetAttr:
				if ((msg.i.attr.type == atPollStatus)) {
					msg.o.attr.val = fifo_is_empty(acm->fifo) ? POLLOUT : POLLOUT | POLLIN;
					msg.o.attr.err = EOK;
					break;
				}
				else {
					msg.o.attr.err = -EINVAL;
				}
			default:
				msg.o.io.err = -EINVAL;
		}
		usbacm_put(acm);
		msgRespond(usbacm_common.msgport, &msg, rid);
	}

	endthread();
}


static int usbacm_handleInsertion(usbdrv_dev_t *dev)
{
	usbacm_dev_t *acmdev;
	const usbdrv_modeswitch_t *mode;
	oid_t oid;

	/* TODO: modeswitch handle by quering server for a descriptor */
	// if ((mode = usbdrv_modeswitchFind(dev->descriptor.idVendor,
	// 		dev->descriptor.idProduct,
	// 		modeswitch, sizeof(modeswitch) / sizeof(modeswitch[0]))) != NULL) {
	// 	return usbdrv_modeswitchHandle(dev, mode);
	// }

	if ((acmdev = usbacm_devAlloc(dev)) == NULL)
		return -ENOMEM;

	dev->ctx = acmdev;

	acmdev->pipeCtrl = usbdrv_pipeOpen(dev, usb_transfer_control, 0);
	if (acmdev->pipeCtrl == NULL) {
		fprintf(stderr, "usbacm: Fail to open control pipe\n");
		free(acmdev);
		return -EINVAL;
	}

	if (usbdrv_setConfiguration(acmdev->pipeCtrl, 1) != 0) {
		fprintf(stderr, "usbacm: Fail to set configuration\n");
		free(acmdev);
		return -EINVAL;
	}

	acmdev->pipeBulkIN = usbdrv_pipeOpen(dev, usb_transfer_bulk, usb_dir_in);
	if (acmdev->pipeBulkIN == NULL) {
		free(acmdev);
		return -EINVAL;
	}

	acmdev->pipeBulkOUT = usbdrv_pipeOpen(dev, usb_transfer_bulk, usb_dir_out);
	if (acmdev->pipeBulkOUT == NULL) {
		free(acmdev);
		return -EINVAL;
	}

	/* Interrupt pipe is optional */
	acmdev->pipeIntIN = usbdrv_pipeOpen(dev, usb_transfer_interrupt, usb_dir_in);

	oid.port = usbacm_common.msgport;
	oid.id = usbdrv_devID(dev);
	if (create_dev(&oid, acmdev->path) != 0) {
		fprintf(stderr, "usbacm: Can't create dev!\n");
		free(acmdev);
		return -EINVAL;
	}

	fprintf(stdout, "usbacm: New device: %s\n", acmdev->path);

	return 0;
}


static int usbacm_handleDeletion(usbdrv_dev_t *dev)
{
	usbacm_dev_t *acm = dev->ctx;

	mutexLock(usbacm_common.lock);

	remove(acm->path);
	fprintf(stdout, "usbacm: Device removed: %s\n", acm->path);
	free(acm);

	mutexUnlock(usbacm_common.lock);

	return 0;
}


int main(int argc, char *argv[])
{
	static const usbdrv_handlers_t handlers[] = {
		usbacm_handleInsertion,
		usbacm_handleDeletion,
		usbacm_handleCompletion
	};
	int ret, i;

	/* Port for communication with driver clients */
	if (portCreate(&usbacm_common.msgport) != 0) {
		fprintf(stderr, "usbacm: Can't create port!\n");
		return 1;
	}

	if (mutexCreate(&usbacm_common.lock)) {
		fprintf(stderr, "usbacm: Can't create mutex!\n");
		return 1;
	}

	if (usbdrv_connect(handlers, filters, sizeof(filters) / sizeof(filters[0])) < 0) {
		fprintf(stderr, "usbacm: Fail to connect to usb host!\n");
		return 1;
	}

	for (i = 0; i < USBACM_N_MSG_THREADS - 1; i++) {
		ret = beginthread(usbacm_msgthr, USBACM_MSG_PRIO, usbacm_common.msgstack[i], sizeof(usbacm_common.msgstack[i]), NULL);
		if (ret < 0) {
			fprintf(stderr, "usbacm: fail to beginthread ret: %d\n", ret);
			return 1;
		}
	}

	priority(USBACM_MSG_PRIO);
	usbacm_msgthr(NULL);

	return 0;
}
