/*
 * Phoenix-RTOS
 *
 * USB CDC ACM driver
 *
 * Copyright 2021, 2022, 2024 Phoenix Systems
 * Author: Maciej Purski, Adam Greloch
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
#include <posix/idtree.h>
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

#ifndef RX_FIFO_SIZE
#define RX_FIFO_SIZE 8192
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

#ifndef USBACM_SET_LINE_DEFAULT_RATE
#define USBACM_SET_LINE_DEFAULT_RATE 57600
#endif

#define USBACM_BULK_SZ 2048

/* clang-format off */
#define TRACE(fmt, ...) if (0) printf("usbacm: " fmt "\n", ##__VA_ARGS__)

enum { RxStopped = 0, RxRunning = 1, RxDisconnected = -1 };
/* clang-format on */

typedef struct _usbacm_dev {
	idnode_t node; /* node.id is oid.id and ACM id (/dev/usbacm[ID]) */

	usb_devinfo_t instance;

	/* USB HOST related */
	int pipeCtrl;
	int pipeIntIN;
	int pipeBulkIN;
	int pipeBulkOUT;
	int urbs[USBACM_N_URBS];
	int urbctrl;

	char path[32];

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

	usb_driver_t *drv;

	usb_cdc_line_coding_t line;
} usbacm_dev_t;


static struct {
	char msgstack[USBACM_N_MSG_THREADS][1024] __attribute__((aligned(8)));
	idtree_t devices;
	unsigned msgport;
	handle_t lock;
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
	return usb_transferAsync(dev->drv, dev->pipeCtrl, dev->urbctrl, 0, &setup);
}


static void usbacm_fifoPush(usbacm_dev_t *dev, const char *data, size_t size)
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
	rbnode_t *node;

	mutexLock(usbacm_common.lock);
	for (node = lib_rbMinimum(usbacm_common.devices.root); node != NULL; node = lib_rbNext(node)) {
		tmp = lib_treeof(usbacm_dev_t, node, lib_treeof(idnode_t, linkage, node));
		if (tmp->pipeBulkIN == pipe || tmp->pipeBulkOUT == pipe || tmp->pipeIntIN == pipe) {
			dev = tmp;
			dev->rfcnt++;
			break;
		}
	}
	mutexUnlock(usbacm_common.lock);

	return dev;
}


static usbacm_dev_t *usbacm_get(int id)
{
	usbacm_dev_t *dev;

	mutexLock(usbacm_common.lock);
	dev = lib_treeof(usbacm_dev_t, node, idtree_find(&usbacm_common.devices, id));
	if (dev != NULL) {
		dev->rfcnt++;
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


static int _usbacm_put(usbacm_dev_t *dev)
{
	if (--dev->rfcnt == 0) {
		idtree_remove(&usbacm_common.devices, &dev->node);
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


static int usbacm_setLine(usbacm_dev_t *dev)
{
	usb_setup_packet_t setup = {
		.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE,
		.bRequest = 0x20, /* SET_LINE_CODING */
		.wValue = 0,
		.wIndex = 0,
		.wLength = sizeof(usb_cdc_line_coding_t),
	};

	return usb_transferControl(dev->drv, dev->pipeCtrl, &setup, &dev->line, sizeof(usb_cdc_line_coding_t), usb_dir_out);
}


static int usbacm_handleCompletion(usb_driver_t *drv, usb_completion_t *c, const char *data, size_t len)
{
	usbacm_dev_t *dev;
	size_t bytes;

	dev = usbacm_getByPipe(c->pipeid);
	if (dev == NULL) {
		return -1;
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
		return -1;
	}

	if (c->pipeid != dev->pipeBulkIN) {
		usbacm_put(dev);
		return -1;
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
		usb_transferAsync(drv, dev->pipeBulkIN, c->urbid, USBACM_BULK_SZ, NULL);
	}

	usbacm_put(dev);

	return 0;
}


/* called always under dev->rxLock */
static int _usbacm_rxStart(usbacm_dev_t *dev)
{
	usb_setup_packet_t setup;
	int i, ret;

	setup.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE;
	setup.bRequest = USB_CDC_REQ_SET_CONTROL_LINE_STATE;
	setup.wIndex = dev->instance.interface;
	setup.wValue = 3;
	setup.wLength = 0;

	ret = usb_transferAsync(dev->drv, dev->pipeCtrl, dev->urbctrl, 0, &setup);
	if (ret < 0) {
		return -EIO;
	}

	for (i = 0; i < USBACM_N_URBS; i++) {
		ret = usb_transferAsync(dev->drv, dev->pipeBulkIN, dev->urbs[i], USBACM_BULK_SZ, NULL);
		if (ret < 0) {
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


static int usbacm_write(usbacm_dev_t *dev, const char *data, size_t len)
{
	int ret = 0;

	TRACE("write: len=%u, flags=%u", len, dev->flags);

	ret = usb_transferBulk(dev->drv, dev->pipeBulkOUT, (void *)data, len, usb_dir_out);
	if (ret <= 0) {
		fprintf(stderr, "usbacm: write failed\n");
		ret = -EIO;
	}

	return ret;
}


static usbacm_dev_t *_usbacm_devAlloc(void)
{
	usbacm_dev_t *dev;

	dev = calloc(1, sizeof(usbacm_dev_t));
	if (dev == NULL) {
		fprintf(stderr, "usbacm: Not enough memory\n");
		return NULL;
	}

	if (idtree_alloc(&usbacm_common.devices, &dev->node) < 0) {
		free(dev);
		return NULL;
	}

	dev->rfcnt = 1;

	return dev;
}


static int _usbacm_urbsAlloc(usbacm_dev_t *dev)
{
	int i, j;

	dev->urbctrl = usb_urbAlloc(dev->drv, dev->pipeCtrl, NULL, usb_dir_out, 0, usb_transfer_control);
	if (dev->urbctrl < 0) {
		return -1;
	}

	for (i = 0; i < USBACM_N_URBS; i++) {
		dev->urbs[i] = usb_urbAlloc(dev->drv, dev->pipeBulkIN, NULL, usb_dir_in, USBACM_BULK_SZ, usb_transfer_bulk);
		if (dev->urbs[i] < 0) {
			for (j = i - 1; j >= 0; j--) {
				usb_urbFree(dev->drv, dev->pipeBulkIN, dev->urbs[j]);
				dev->urbs[j] = 0;
			}

			usb_urbFree(dev->drv, dev->pipeCtrl, dev->urbctrl);
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
		usb_urbFree(dev->drv, dev->pipeBulkIN, dev->urbs[i]);
	}

	if (dev->flags & O_NONBLOCK) {
		mutexLock(dev->rxLock);
		_usbacm_rxStop(dev);
		mutexUnlock(dev->rxLock);
	}

	usb_urbFree(dev->drv, dev->pipeCtrl, dev->urbctrl);

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
			msg.o.err = EOK;
			msgRespond(usbacm_common.msgport, &msg, rid);
			continue;
		}

		id = msg.oid.id;

		if ((dev = usbacm_get(id)) == NULL) {
			msg.o.err = -ENOENT;
			msgRespond(usbacm_common.msgport, &msg, rid);
			continue;
		}

		/* A device can be opened only by one process */
		/* FIXME: allow mtClose with different PID (files closing on process exit have invalid PID value as of 2023-10-23) */
		if (((msg.type != mtOpen) && (msg.type != mtClose)) && (msg.pid != dev->clientpid)) {
			usbacm_put(dev);
			msg.o.err = -EBUSY;
			msgRespond(usbacm_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				mutexLock(usbacm_common.lock);
				msg.o.err = _usbacm_open(dev, msg.i.openclose.flags, msg.pid);
				mutexUnlock(usbacm_common.lock);
				break;

			case mtClose:
				mutexLock(usbacm_common.lock);
				_usbacm_close(dev);
				mutexUnlock(usbacm_common.lock);
				msg.o.err = EOK;
				break;

			case mtRead:
				msg.o.err = usbacm_read(dev, msg.o.data, msg.o.size);
				if (msg.o.err == 0) {
					if (msg.i.io.mode & O_NONBLOCK) {
						msg.o.err = -EWOULDBLOCK;
					}
				}
				break;

			case mtWrite:
				msg.o.err = usbacm_write(dev, msg.i.data, msg.i.size);
				break;

			case mtGetAttr:
				if (msg.i.attr.type == atPollStatus) {
					msg.o.attr.val = POLLOUT;
					if (((dev->flags & O_NONBLOCK) != 0) && fifo_is_empty(dev->fifo)) {
						msg.o.attr.val |= POLLIN;
					}
					msg.o.err = EOK;
				}
				else {
					msg.o.err = -EINVAL;
				}
				break;
			default:
				msg.o.err = -ENOSYS;
				break;
		}
		usbacm_put(dev);
		msgRespond(usbacm_common.msgport, &msg, rid);
	}

	endthread();
}


static int usbacm_handleInsertion(usb_driver_t *drv, usb_devinfo_t *insertion, usb_event_insertion_t *event)
{
	usbacm_dev_t *dev;
	const usb_modeswitch_t *mode;
	oid_t oid;
	int err;

	TRACE("handleInsertion");

	mode = usb_modeswitchFind(insertion->descriptor.idVendor,
			insertion->descriptor.idProduct,
			modeswitch, sizeof(modeswitch) / sizeof(modeswitch[0]));
	if (mode != NULL) {
		return usb_modeswitchHandle(drv, insertion, mode);
	}

	mutexLock(usbacm_common.lock);

	dev = _usbacm_devAlloc();
	if (dev == NULL) {
		mutexUnlock(usbacm_common.lock);
		return -ENOMEM;
	}

	dev->instance = *insertion;
	dev->drv = drv;

	do {
		dev->pipeCtrl = usb_open(drv, insertion, usb_transfer_control, 0);
		if (dev->pipeCtrl < 0) {
			fprintf(stderr, "usbacm: Fail to open control pipe\n");
			err = -EINVAL;
			break;
		}

		err = usb_setConfiguration(drv, dev->pipeCtrl, 1);
		if (err != 0) {
			fprintf(stderr, "usbacm: Fail to set configuration\n");
			err = -EINVAL;
			break;
		}

		dev->pipeBulkIN = usb_open(drv, insertion, usb_transfer_bulk, usb_dir_in);
		if (dev->pipeBulkIN < 0) {
			err = -EINVAL;
			break;
		}

		dev->pipeBulkOUT = usb_open(drv, insertion, usb_transfer_bulk, usb_dir_out);
		if (dev->pipeBulkOUT < 0) {
			err = -EINVAL;
			break;
		}

		/* Interrupt pipe is optional */
		dev->pipeIntIN = usb_open(drv, insertion, usb_transfer_interrupt, usb_dir_in);

		err = mutexCreate(&dev->rxLock);
		if (err != 0) {
			err = -ENOMEM;
			break;
		}

		err = condCreate(&dev->rxCond);
		if (err != 0) {
			resourceDestroy(dev->rxLock);
			err = -ENOMEM;
			break;
		}

		dev->line.dwDTERate = USBACM_SET_LINE_DEFAULT_RATE;
		dev->line.bCharFormat = 0;
		dev->line.bParityType = 0;
		dev->line.bDataBits = 8;

		/* Some broken devices won't function without doing the set line cmd first */
		err = usbacm_setLine(dev);
		if (err < 0) {
			fprintf(stderr, "usbacm: Set line to speed %d failed/unsupported: %d\n", dev->line.dwDTERate, err);
			/* Continue as this is optional: device may reject the setting/not support the set line cmd */
		}

		oid.port = usbacm_common.msgport;
		oid.id = idtree_id(&dev->node);

		snprintf(dev->path, sizeof(dev->path), "/dev/usbacm%d", idtree_id(&dev->node));

		err = create_dev(&oid, dev->path);
		if (err != 0) {
			resourceDestroy(dev->rxCond);
			resourceDestroy(dev->rxLock);
			fprintf(stderr, "usbacm: Can't create dev: %s\n", dev->path);
			err = -EINVAL;
			break;
		}
	} while (0);

	if (err < 0) {
		_usbacm_put(dev);
	}

	mutexUnlock(usbacm_common.lock);

	if (err < 0) {
		free(dev);
		return err;
	}

	fprintf(stdout, "usbacm: New device: %s\n", dev->path);

	event->deviceCreated = true;
	event->dev = oid;
	strncpy(event->devPath, dev->path, sizeof(event->devPath));

	return 0;
}


static int usbacm_handleDeletion(usb_driver_t *drv, usb_deletion_t *del)
{
	rbnode_t *node, *next;
	usbacm_dev_t *dev;

	TRACE("handleDeletion");

	mutexLock(usbacm_common.lock);

	node = lib_rbMinimum(usbacm_common.devices.root);
	while (node != NULL) {
		next = lib_rbNext(node);
		dev = lib_treeof(usbacm_dev_t, node, lib_treeof(idnode_t, linkage, node));

		if (dev->instance.bus == del->bus && dev->instance.dev == del->dev &&
				dev->instance.interface == del->interface) {

			/* close pending transfers, reject new ones */
			mutexLock(dev->rxLock);
			dev->rxState = RxDisconnected;
			condSignal(dev->rxCond);
			mutexUnlock(dev->rxLock);

			if (_usbacm_put(dev) == 0) {
				usbacm_free(dev);
			}
		}

		node = next;
	}

	mutexUnlock(usbacm_common.lock);

	return 0;
}


static int usbacm_init(usb_driver_t *drv, void *args)
{
	int ret;
	int i;

	/* Port for communication with driver clients */
	if (portCreate(&usbacm_common.msgport) != 0) {
		fprintf(stderr, "usbacm: Can't create port!\n");
		return 1;
	}

	if (mutexCreate(&usbacm_common.lock)) {
		fprintf(stderr, "usbacm: Can't create mutex!\n");
		return 1;
	}

	idtree_init(&usbacm_common.devices);

	for (i = 0; i < USBACM_N_MSG_THREADS; i++) {
		ret = beginthread(usbacm_msgthr, USBACM_MSG_PRIO, usbacm_common.msgstack[i], sizeof(usbacm_common.msgstack[i]), NULL);
		if (ret < 0) {
			fprintf(stderr, "usbacm: fail to beginthread ret: %d\n", ret);
			return 1;
		}
	}

	return 0;
}


static int usbacm_destroy(usb_driver_t *drv)
{
	/* TODO */
	return EOK;
}


static usb_driver_t usbacm_driver = {
	.name = "usbacm",
	.handlers = {
		.insertion = usbacm_handleInsertion,
		.deletion = usbacm_handleDeletion,
		.completion = usbacm_handleCompletion,
	},
	.ops = {
		.init = usbacm_init,
		.destroy = usbacm_destroy,
	},
	.filters = filters,
	.nfilters = sizeof(filters) / sizeof(filters[0]),
	.priv = (void *)&usbacm_common,
};


__attribute__((constructor)) static void usbacm_register(void)
{
	usb_driverRegister(&usbacm_driver);
}
