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

/* clang-format off */
#define TRACE(fmt, ...) if (0) printf("usbacm: " fmt "\n", ##__VA_ARGS__)

enum { RxStopped = 0, RxRunning = 1, RxDisconnected = -1 };
/* clang-format on */

typedef struct _usbacm_dev {
	struct _usbacm_dev *prev, *next;
	usb_devinfo_t instance;

	/* USB HOST related */
	int pipeCtrl;
	int pipeIntIN;
	int pipeBulkIN;
	int pipeBulkOUT;
	int urbs[USBACM_N_URBS];
	int urbctrl;

	char path[32];
	unsigned int id; /* ACM id (/dev/usbacm[ID]) */
	int fileId;      /* oid.id */

	/* opened file state */
	int flags;
	pid_t clientpid;

	volatile int rfcnt; /* protected by usbacm_common.lock */

	/* blocking/nonblocking READ state - protected by rxLock */
	char *rxptr;
	size_t rxbuflen;
	fifo_t *fifo;
	handle_t rxLock;
	handle_t rxCond;
	int rxState;

} usbacm_dev_t;


static struct {
	char msgstack[USBACM_N_MSG_THREADS][1024] __attribute__((aligned(8)));
	char ustack[USBACM_N_UMSG_THREADS - 1][2048] __attribute__((aligned(8)));
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


/* called always under dev->rxLock */
static int _usbacm_rxStop(usbacm_dev_t *dev)
{
	TRACE("rxStop");
	usb_setup_packet_t setup;

	setup.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE;
	setup.bRequest = USB_CDC_REQ_SET_CONTROL_LINE_STATE;
	setup.wIndex = dev->instance.interface;
	setup.wValue = 0;
	setup.wLength = 0;

	dev->rxState = RxStopped;
	return usb_transferAsync(dev->pipeCtrl, dev->urbctrl, 0, &setup);
}


static void usbacm_fifoPush(usbacm_dev_t *dev, char *data, size_t size)
{
	unsigned i;

	mutexLock(dev->rxLock);
	for (i = 0; i < size; i++)
		fifo_push(dev->fifo, data[i]);
	mutexUnlock(dev->rxLock);
}


static usbacm_dev_t *usbacm_getByPipe(int pipe)
{
	usbacm_dev_t *tmp, *dev = NULL;

	mutexLock(usbacm_common.lock);
	tmp = usbacm_common.devices;
	if (tmp != NULL) {
		do {
			if (tmp->pipeBulkIN == pipe || tmp->pipeBulkOUT == pipe || tmp->pipeIntIN == pipe) {
				dev = tmp;
				dev->rfcnt++;
				break;
			}
			tmp = tmp->next;
		} while (tmp != usbacm_common.devices);
	}
	mutexUnlock(usbacm_common.lock);

	return dev;
}


static usbacm_dev_t *usbacm_get(int id)
{
	usbacm_dev_t *tmp, *dev = NULL;

	mutexLock(usbacm_common.lock);
	tmp = usbacm_common.devices;
	if (tmp != NULL) {
		do {
			if (tmp->fileId == id) {
				dev = tmp;
				dev->rfcnt++;
				break;
			}
			tmp = tmp->next;
		} while (tmp != usbacm_common.devices);
	}
	mutexUnlock(usbacm_common.lock);

	return dev;
}


static void usbacm_free(usbacm_dev_t *dev)
{
	fprintf(stdout, "usbacm: Device removed: %s\n", dev->path);
	remove(dev->path);

	free(dev->fifo);
	resourceDestroy(dev->rxCond);
	resourceDestroy(dev->rxLock);
	free(dev);
}


static void usbacm_freeAll(usbacm_dev_t **devices)
{
	usbacm_dev_t *next, *dev = *devices;

	if (dev != NULL) {
		do {
			next = dev->next;
			usbacm_free(dev);
			dev = next;
		} while (dev != *devices);
	}

	*devices = NULL;
}


static int _usbacm_put(usbacm_dev_t *dev)
{
	if (--dev->rfcnt == 0) {
		LIST_REMOVE(&usbacm_common.devices, dev);
	}
	return dev->rfcnt;
}


static void usbacm_put(usbacm_dev_t *dev)
{
	int rfcnt;

	mutexLock(usbacm_common.lock);
	rfcnt = _usbacm_put(dev);
	mutexUnlock(usbacm_common.lock);

	if (rfcnt == 0) {
		usbacm_free(dev);
	}
}


static void usbacm_handleCompletion(usb_completion_t *c, char *data, size_t len)
{
	usbacm_dev_t *dev;
	size_t bytes;

	dev = usbacm_getByPipe(c->pipeid);
	if (dev == NULL) {
		return;
	}
	TRACE("handleCompletion: c->err=%d, len=%u, rxbuflen=%u", c->err, len, dev->rxbuflen);

	if (c->err != 0) {
		/* Error, stop receiving */
		mutexLock(dev->rxLock);
		dev->rxState = RxStopped;
		if ((dev->flags & O_NONBLOCK) == 0) {
			/* Stop the blocking read in progress */
			condSignal(dev->rxCond);
		}
		mutexUnlock(dev->rxLock);
		usbacm_put(dev);
		return;
	}

	if (c->pipeid != dev->pipeBulkIN) {
		usbacm_put(dev);
		return;
	}

	if (dev->flags & O_NONBLOCK) {
		usbacm_fifoPush(dev, data, len);
		/* assume rxState == RxRunning unless the device was ejected */
	}
	else if (dev->rxptr != NULL) {
		mutexLock(dev->rxLock);

		bytes = min(len, dev->rxbuflen);
		memcpy(dev->rxptr, data, bytes);
		dev->rxptr += bytes;
		dev->rxbuflen -= bytes;

		if (dev->rxbuflen == 0) {
			dev->rxState = RxStopped;
			condSignal(dev->rxCond);
		}
		mutexUnlock(dev->rxLock);
	}

	if (dev->rxState == RxRunning) {
		usb_transferAsync(dev->pipeBulkIN, c->urbid, USBACM_BULK_SZ, NULL);
	}

	usbacm_put(dev);
}


/* called always under dev->rxLock */
static int _usbacm_rxStart(usbacm_dev_t *dev)
{
	usb_setup_packet_t setup;
	int i;

	setup.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE;
	setup.bRequest = USB_CDC_REQ_SET_CONTROL_LINE_STATE;
	setup.wIndex = dev->instance.interface;
	setup.wValue = 3;
	setup.wLength = 0;

	if (usb_transferAsync(dev->pipeCtrl, dev->urbctrl, 0, &setup) < 0) {
		return -EIO;
	}

	for (i = 0; i < USBACM_N_URBS; i++) {
		if (usb_transferAsync(dev->pipeBulkIN, dev->urbs[i], USBACM_BULK_SZ, NULL) < 0) {
			return -EIO;
		}
	}

	dev->rxState = RxRunning;
	return 0;
}


static int usbacm_read(usbacm_dev_t *dev, char *data, size_t len)
{
	int ret;

	if ((dev->flags & (O_RDONLY | O_RDWR)) == 0) {
		return -EPERM;
	}
	TRACE("read: len=%u, flags=%u, rxState=%d", len, dev->flags, dev->rxState);

	mutexLock(dev->rxLock);
	do {
		if ((dev->flags & O_NONBLOCK) && (dev->rxState == RxRunning)) {
			for (ret = 0; ret < len && !fifo_is_empty(dev->fifo); ret++) {
				data[ret] = fifo_pop_back(dev->fifo);
			}
		}
		else if ((dev->flags & O_NONBLOCK) == 0 && (dev->rxState == RxStopped)) {
			dev->rxptr = data;
			dev->rxbuflen = len;

			ret = _usbacm_rxStart(dev);
			if (ret != 0) {
				TRACE("rxStart error: %d", ret);
				break;
			}

			/* Block until len bytes is received or an error occurs */
			while ((dev->rxbuflen > 0) && (dev->rxState == RxRunning)) {
				condWait(dev->rxCond, dev->rxLock, 0);
			}

			ret = _usbacm_rxStop(dev);
			if (ret != 0) {
				TRACE("rxStop error: %d", ret);
				break;
			}

			ret = len - dev->rxbuflen;
		}
		else {
			ret = -EIO;
		}
	} while (0);

	dev->rxbuflen = 0;
	dev->rxptr = NULL;
	mutexUnlock(dev->rxLock);

	return ret;
}


static int usbacm_write(usbacm_dev_t *dev, char *data, size_t len)
{
	int ret = 0;

	TRACE("write: len=%u, flags=%u", len, dev->flags);
	if ((ret = usb_transferBulk(dev->pipeBulkOUT, data, len, usb_dir_out)) <= 0) {
		fprintf(stderr, "usbacm: write failed\n");
		ret = -EIO;
	}

	return ret;
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

	snprintf(dev->path, sizeof(dev->path), "/dev/usbacm%u", dev->id);

	return dev;
}


static int _usbacm_urbsAlloc(usbacm_dev_t *dev)
{
	int i, j;

	dev->urbctrl = usb_urbAlloc(dev->pipeCtrl, NULL, usb_dir_out, 0, usb_transfer_control);
	if (dev->urbctrl < 0) {
		return -1;
	}

	for (i = 0; i < USBACM_N_URBS; i++) {
		dev->urbs[i] = usb_urbAlloc(dev->pipeBulkIN, NULL, usb_dir_in, USBACM_BULK_SZ, usb_transfer_bulk);
		if (dev->urbs[i] < 0) {
			for (j = i - 1; j >= 0; j--) {
				usb_urbFree(dev->pipeBulkIN, dev->urbs[j]);
				dev->urbs[j] = 0;
			}

			usb_urbFree(dev->pipeCtrl, dev->urbctrl);
			return -1;
		}
	}

	return 0;
}


static int _usbacm_openNonblock(usbacm_dev_t *dev)
{
	/* TODO: switch to lf-fifo */
	fifo_t *fifo;
	int ret = 0;
	TRACE("openNonblock");

	if (dev->fifo != NULL) {
		fifo = dev->fifo;
	}
	else {
		fifo = malloc(sizeof(fifo_t) + RX_FIFO_SIZE);
		if (fifo == NULL) {
			return -ENOMEM;
		}
	}

	fifo_init(fifo, RX_FIFO_SIZE);

	mutexLock(dev->rxLock);
	dev->fifo = fifo;

	if (_usbacm_rxStart(dev) != 0) {
		dev->fifo = NULL;
		free(fifo);
		ret = -EIO;
	}
	mutexUnlock(dev->rxLock);

	return ret;
}


static void _usbacm_close(usbacm_dev_t *dev)
{
	int i;
	TRACE("close: flags=%u", dev->flags);

	for (i = 0; i < USBACM_N_URBS; i++) {
		usb_urbFree(dev->pipeBulkIN, dev->urbs[i]);
	}

	if (dev->flags & O_NONBLOCK) {
		mutexLock(dev->rxLock);
		_usbacm_rxStop(dev);
		mutexUnlock(dev->rxLock);
	}

	usb_urbFree(dev->pipeCtrl, dev->urbctrl);

	dev->flags = 0;
	dev->clientpid = 0;
}


static int _usbacm_open(usbacm_dev_t *dev, int flags, pid_t pid)
{
	/* Already opened */
	if (dev->flags != 0) {
		return -EBUSY;
	}
	TRACE("open: flags=%u", flags);

	if ((flags & O_RDONLY) || (flags & O_RDWR)) {
		if (_usbacm_urbsAlloc(dev) < 0) {
			return -EPERM;
		}

		if (flags & O_NONBLOCK) {
			/* Start receiving data */
			if (_usbacm_openNonblock(dev) < 0) {
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
	usbacm_dev_t *dev;
	msg_rid_t rid;
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

		if (msg.type == mtGetAttr) {
			id = msg.i.attr.oid.id;
		}
		else if ((msg.type == mtOpen) || (msg.type == mtClose)) {
			id = msg.i.openclose.oid.id;
		}
		else {
			id = msg.i.io.oid.id;
		}

		if ((dev = usbacm_get(id)) == NULL) {
			msg.o.io.err = -ENOENT;
			msgRespond(usbacm_common.msgport, &msg, rid);
			continue;
		}

		/* A device can be opened only by one process */
		/* FIXME: allow mtClose with different PID (files closing on process exit have invalid PID value as of 2023-10-23) */
		if (((msg.type != mtOpen) && (msg.type != mtClose)) && (msg.pid != dev->clientpid)) {
			msg.o.io.err = -EBUSY;
			msgRespond(usbacm_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				mutexLock(usbacm_common.lock);
				msg.o.io.err = _usbacm_open(dev, msg.i.openclose.flags, msg.pid);
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
				if (msg.i.attr.type == atPollStatus) {
					msg.o.attr.val = POLLOUT;
					if (((dev->flags & O_NONBLOCK) != 0) && fifo_is_empty(dev->fifo)) {
						msg.o.attr.val |= POLLIN;
					}
					msg.o.attr.err = EOK;
					break;
				}
				else {
					msg.o.attr.err = -EINVAL;
				}
			default:
				msg.o.io.err = -EINVAL;
		}
		usbacm_put(dev);
		msgRespond(usbacm_common.msgport, &msg, rid);
	}

	endthread();
}


static int usbacm_handleInsertion(usb_devinfo_t *insertion)
{
	usbacm_dev_t *dev;
	const usb_modeswitch_t *mode;
	oid_t oid;

	TRACE("handleInsertion");

	if ((mode = usb_modeswitchFind(insertion->descriptor.idVendor,
			insertion->descriptor.idProduct,
			modeswitch, sizeof(modeswitch) / sizeof(modeswitch[0]))) != NULL) {
		return usb_modeswitchHandle(insertion, mode);
	}

	if ((dev = usbacm_devAlloc()) == NULL)
		return -ENOMEM;

	dev->instance = *insertion;

	if ((dev->pipeCtrl = usb_open(insertion, usb_transfer_control, 0)) < 0) {
		fprintf(stderr, "usbacm: Fail to open control pipe\n");
		free(dev);
		return -EINVAL;
	}

	if (usb_setConfiguration(dev->pipeCtrl, 1) != 0) {
		fprintf(stderr, "usbacm: Fail to set configuration\n");
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

	if (mutexCreate(&dev->rxLock) != 0) {
		free(dev);
		return -ENOMEM;
	}

	if (condCreate(&dev->rxCond) != 0) {
		resourceDestroy(dev->rxLock);
		free(dev);
		return -ENOMEM;
	}

	oid.port = usbacm_common.msgport;
	oid.id = dev->fileId;
	if (create_dev(&oid, dev->path) != 0) {
		resourceDestroy(dev->rxCond);
		resourceDestroy(dev->rxLock);
		free(dev);
		fprintf(stderr, "usbacm: Can't create dev!\n");
		return -EINVAL;
	}

	dev->rfcnt = 1;
	mutexLock(usbacm_common.lock);
	LIST_ADD(&usbacm_common.devices, dev);
	mutexUnlock(usbacm_common.lock);

	fprintf(stdout, "usbacm: New device: %s\n", dev->path);

	return 0;
}


static int _usbacm_handleDeletion(usb_deletion_t *del, usbacm_dev_t **devicesToFree)
{
	usbacm_dev_t *next, *dev = usbacm_common.devices;
	int cont = 1;

	if (dev == NULL)
		return 0;

	TRACE("handleDeletion");

	do {
		next = dev->next;
		if (dev->instance.bus == del->bus && dev->instance.dev == del->dev &&
				dev->instance.interface == del->interface) {
			if (dev == next)
				cont = 0;

			/* close pending transfers, reject new ones */
			mutexLock(dev->rxLock);
			dev->rxState = RxDisconnected;
			condSignal(dev->rxCond);
			mutexUnlock(dev->rxLock);

			if (_usbacm_put(dev) == 0) {
				LIST_ADD(devicesToFree, dev);
			}
			if (!cont)
				break;
		}
		dev = next;
	} while (dev != usbacm_common.devices);

	return 0;
}


static void usbthr(void *arg)
{
	msg_t msg;
	usb_msg_t *umsg = (usb_msg_t *)msg.i.raw;
	msg_rid_t rid;
	usbacm_dev_t *devicesToFree = NULL;

	for (;;) {
		if (msgRecv(usbacm_common.drvport, &msg, &rid) < 0)
			continue;

		switch (umsg->type) {
			case usb_msg_insertion:
				usbacm_handleInsertion(&umsg->insertion);
				break;
			case usb_msg_deletion:
				mutexLock(usbacm_common.lock);
				_usbacm_handleDeletion(&umsg->deletion, &devicesToFree);
				mutexUnlock(usbacm_common.lock);
				usbacm_freeAll(&devicesToFree);
				break;
			case usb_msg_completion:
				usbacm_handleCompletion(&umsg->completion, msg.i.data, msg.i.size);
				break;
			default:
				fprintf(stderr, "usbacm: Error when receiving event from host\n");
				break;
		}

		msgRespond(usbacm_common.drvport, &msg, rid);
	}
}

int main(int argc, char *argv[])
{
	int ret;
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
		ret = beginthread(usbacm_msgthr, USBACM_MSG_PRIO, usbacm_common.msgstack[i], sizeof(usbacm_common.msgstack[i]), NULL);
		if (ret < 0) {
			fprintf(stderr, "usbacm: fail to beginthread ret: %d\n", ret);
			return 1;
		}
	}

	for (i = 0; i < USBACM_N_UMSG_THREADS - 1; i++) {
		ret = beginthread(usbthr, USBACM_UMSG_PRIO, usbacm_common.ustack[i], sizeof(usbacm_common.ustack[i]), NULL);
		if (ret < 0) {
			fprintf(stderr, "usbacm: fail to beginthread ret: %d\n", ret);
			return 1;
		}
	}

	priority(USBACM_UMSG_PRIO);
	usbthr(NULL);

	return 0;
}
