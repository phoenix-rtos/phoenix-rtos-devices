/*
 * Phoenix-RTOS
 *
 * USB WLAN driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Julian Uziembło
 *
 * %LICENSE%
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <posix/utils.h>
#include <stdlib.h>
#include <string.h>
#include <sys/minmax.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <unistd.h>

#include <usbdriver.h>
#include <usbhost.h>
#include <usb.h>

#include <board_config.h>

#include "usbwlan.h"

#ifndef USBWLAN_MSGTHREAD_PRIO
#define USBWLAN_MSGTHREAD_PRIO 3
#endif

#ifndef USBWLAN_MSGTHREAD_STACK_SIZE
#define USBWLAN_MSGTHREAD_STACK_SIZE 1024
#endif

#define USBWLAN_N_MSGTHREADS 1
#define USBWLAN_BULK_SIZE    2048

#define usbwlan_log(fmt, ...) \
	do { \
		fprintf(stderr, "usbwlan: " fmt "\n", ##__VA_ARGS__); \
	} while (0)


typedef struct {
	idnode_t node;
	usb_devinfo_t dev;

	/* USB HOST */
	int pipeCtrl;
	int pipeIntIn;
	int pipeBulkIn;
	int pipeBulkOut;

	char path[32];

	int flags;
	pid_t clientpid;
	volatile int refcnt; /* protected by usbwlan_common.lock */

	usb_driver_t *drv;

	struct {
		handle_t lock;
		msg_t msg;
		msg_rid_t rid;
		int urbid;
		bool isPending;
	} rx;
} usbwlan_dev_t;


static struct {
	idtree_t devices;
	uint32_t msgport;
	handle_t lock;
	char stacks[USBWLAN_N_MSGTHREADS][USBWLAN_MSGTHREAD_STACK_SIZE] __attribute__((aligned(8)));
} usbwlan_common;


static const usb_device_id_t filters[] = {
	/*
	 * Sterling LWB5+ USB Dongle
	 * Enumerates as Remote Download for the host to download firmware onto it.
	 * After downloading the firmware, it enumerates as the 802.11 device.
	 */
	/* Remote Download Wireless Adapter */
	{ 0x04b4, 0xbd29, USBDRV_ANY, USBDRV_ANY, USBDRV_ANY },
	/* Cypress USB 802.11 Wireless Adapter */
	{ 0x04b4, 0x0bdc, USBDRV_ANY, USBDRV_ANY, USBDRV_ANY },
};


static usbwlan_dev_t *_usbwlan_devAlloc(void)
{
	usbwlan_dev_t *dev;

	dev = calloc(1, sizeof(*dev));
	if (dev == NULL) {
		usbwlan_log("Not enough memory for new device");
		return NULL;
	}

	dev->rx.urbid = -1;

	if (mutexCreate(&dev->rx.lock) < 0) {
		usbwlan_log("Could not create RX mutex");
		free(dev);
		return NULL;
	}

	if (idtree_alloc(&usbwlan_common.devices, &dev->node) < 0) {
		usbwlan_log("Could not alloc idtree for new device");
		resourceDestroy(dev->rx.lock);
		free(dev);
		return NULL;
	}

	dev->refcnt = 1;

	return dev;
}


static void usbwlan_free(usbwlan_dev_t *dev)
{
	usbwlan_log("Device %s removed", dev->path);

	mutexLock(dev->rx.lock);
	if (dev->rx.isPending) {
		dev->rx.msg.o.err = -EIO;
		(void)msgRespond(usbwlan_common.msgport, &dev->rx.msg, dev->rx.rid);
		dev->rx.isPending = false;
	}
	mutexUnlock(dev->rx.lock);

	if (dev->rx.urbid != -1) {
		usb_urbFree(dev->drv, dev->pipeBulkIn, dev->rx.urbid);
	}
	resourceDestroy(dev->rx.lock);
	remove(dev->path);
	free(dev);
}


static usbwlan_dev_t *usbwlan_get(int id)
{
	usbwlan_dev_t *dev;

	mutexLock(usbwlan_common.lock);
	idnode_t *idnode = idtree_find(&usbwlan_common.devices, id);
	dev = lib_treeof(usbwlan_dev_t, node, idnode);
	if (dev != NULL) {
		dev->refcnt++;
	}
	mutexUnlock(usbwlan_common.lock);

	return dev;
}


static usbwlan_dev_t *usbwlan_getByPipe(int pipe)
{
	usbwlan_dev_t *tmp, *dev = NULL;
	rbnode_t *node;

	mutexLock(usbwlan_common.lock);
	for (node = lib_rbMinimum(usbwlan_common.devices.root); node != NULL; node = lib_rbNext(node)) {
		tmp = lib_treeof(usbwlan_dev_t, node, lib_treeof(idnode_t, linkage, node));
		assert(tmp != NULL);
		if (tmp->pipeBulkIn == pipe || tmp->pipeBulkOut == pipe || tmp->pipeIntIn == pipe || pipe == tmp->pipeCtrl) {
			dev = tmp;
			dev->refcnt++;
			break;
		}
	}
	mutexUnlock(usbwlan_common.lock);

	return dev;
}


static int _usbwlan_put(usbwlan_dev_t *dev)
{
	dev->refcnt--;
	if (dev->refcnt == 0) {
		idtree_remove(&usbwlan_common.devices, &dev->node);
	}
	return dev->refcnt;
}


static void usbwlan_put(usbwlan_dev_t *dev)
{
	int refcnt;

	mutexLock(usbwlan_common.lock);
	refcnt = _usbwlan_put(dev);
	mutexUnlock(usbwlan_common.lock);

	if (refcnt == 0) {
		usbwlan_free(dev);
	}
}


static int usbwlan_handleInsertion(usb_driver_t *drv, usb_devinfo_t *ins, usb_event_insertion_t *event_out)
{
	usbwlan_dev_t *dev;
	oid_t oid;
	int err = 0;

	mutexLock(usbwlan_common.lock);

	dev = _usbwlan_devAlloc();
	if (dev == NULL) {
		mutexUnlock(usbwlan_common.lock);
		return -ENOMEM;
	}

	dev->dev = *ins;
	dev->drv = drv;

	do {
		dev->pipeCtrl = usb_open(drv, ins, usb_transfer_control, 0);
		if (dev->pipeCtrl < 0) {
			usbwlan_log("Failed to open ctrl pipe");
			err = -EINVAL;
			break;
		}

		dev->pipeIntIn = usb_open(drv, ins, usb_transfer_interrupt, usb_dir_in);
		if (dev->pipeIntIn < 0) {
			usbwlan_log("Failed to open interrupt IN pipe");
			err = -EINVAL;
			break;
		}

		dev->pipeBulkIn = usb_open(drv, ins, usb_transfer_bulk, usb_dir_in);
		if (dev->pipeBulkIn < 0) {
			usbwlan_log("Failed to open bulk IN pipe");
			err = -EINVAL;
			break;
		}

		dev->pipeBulkOut = usb_open(drv, ins, usb_transfer_bulk, usb_dir_out);
		if (dev->pipeBulkOut < 0) {
			usbwlan_log("Failed to open bulk OUT pipe");
			err = -EINVAL;
			break;
		}

		dev->rx.urbid = usb_urbAlloc(dev->drv, dev->pipeBulkIn, NULL, usb_dir_in, USBWLAN_BULK_SIZE, usb_transfer_bulk);
		if (dev->rx.urbid < 0) {
			usbwlan_log("Fail to allocate URB");
			err = -ENOMEM;
			break;
		}

		oid.port = usbwlan_common.msgport;
		oid.id = idtree_id(&dev->node);
		snprintf(dev->path, sizeof(dev->path), "/dev/wlan%ju", (uintmax_t)oid.id);
		if (create_dev(&oid, dev->path) < 0) {
			usbwlan_log("Failed to create device file");
			err = -EINVAL;
			break;
		}
	} while (0);

	if (err < 0) {
		_usbwlan_put(dev);
	}

	mutexUnlock(usbwlan_common.lock);

	if (err < 0) {
		usbwlan_free(dev);
		return err;
	}

	event_out->deviceCreated = true;
	event_out->dev = oid;
	(void)snprintf(event_out->devPath, sizeof(event_out->devPath), "%s", dev->path);

	usb_setConfiguration(dev->drv, dev->pipeCtrl, 1);

	return 0;
}


static int usbwlan_handleDeletion(usb_driver_t *drv, usb_deletion_t *del)
{
	rbnode_t *node, *next;
	usbwlan_dev_t *dev;

	mutexLock(usbwlan_common.lock);

	node = lib_rbMinimum(usbwlan_common.devices.root);
	while (node != NULL) {
		next = lib_rbNext(node);
		dev = lib_treeof(usbwlan_dev_t, node, lib_treeof(idnode_t, linkage, node));

		assert(dev != NULL);
		if (dev->dev.bus == del->bus && dev->dev.dev == del->dev && dev->dev.interface == del->interface) {
			if (_usbwlan_put(dev) == 0) {
				usbwlan_free(dev);
			}
		}

		node = next;
	}

	mutexUnlock(usbwlan_common.lock);

	return 0;
}


static int usbwlan_handleCompletion(struct usb_driver *drv, usb_completion_t *completion, const char *data, size_t len)
{
	int err;
	usbwlan_dev_t *dev;

	dev = usbwlan_getByPipe(completion->pipeid);
	if (dev == NULL) {
		return -1;
	}

	if (completion->pipeid != dev->pipeBulkIn) {
		usbwlan_put(dev);
		return -1;
	}

	mutexLock(dev->rx.lock);
	do {
		if (!dev->rx.isPending) {
			err = -1;
			break;
		}

		if (completion->err != 0) {
			dev->rx.msg.o.err = -abs(completion->err);
		}
		else {
			memcpy(dev->rx.msg.o.data, data, len);
			dev->rx.msg.o.err = len;
		}

		err = msgRespond(usbwlan_common.msgport, &dev->rx.msg, dev->rx.rid);
		dev->rx.isPending = false;
	} while (0);
	mutexUnlock(dev->rx.lock);

	usbwlan_put(dev);

	return err;
}


static int _usbwlan_close(usbwlan_dev_t *dev, pid_t pid)
{
	/* FIXME: On process exit, we get mtClose with pid=1 (as of 21.10.2025) */
	if (dev->clientpid == 0 || (dev->clientpid != pid && pid != 1)) {
		return -EBADF;
	}

	mutexLock(dev->rx.lock);
	if (dev->rx.isPending) {
		dev->rx.msg.o.err = -EBADF;
		(void)msgRespond(usbwlan_common.msgport, &dev->rx.msg, dev->rx.rid);
		dev->rx.isPending = false;
	}
	mutexUnlock(dev->rx.lock);

	dev->flags = 0;
	dev->clientpid = 0;

	return EOK;
}


static int _usbwlan_open(usbwlan_dev_t *dev, int flags, pid_t pid)
{
	if (dev->clientpid != 0 || dev->flags != 0) {
		return -EBUSY;
	}

	dev->clientpid = pid;
	dev->flags = flags;

	return EOK;
}


static int usbwlan_dlCmd(usbwlan_dev_t *dev, uint8_t cmd, uint16_t wIndex, void *data, size_t size)
{
	usb_setup_packet_t setup = {
		.bmRequestType = REQUEST_DIR_DEV2HOST | REQUEST_TYPE_VENDOR | REQUEST_RECIPIENT_INTERFACE,
		.bRequest = cmd,
		.wValue = 0,
		.wIndex = wIndex,
		.wLength = size,
	};
	return usb_transferControl(dev->drv, dev->pipeCtrl, &setup, data, size, usb_dir_in);
}


static int usbwlan_ctrlSend(usbwlan_dev_t *dev, const void *data, size_t size)
{
	usb_setup_packet_t setup = {
		.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE,
		.bRequest = 0,
		.wValue = 0,
		.wIndex = 0,
		.wLength = size,
	};
	return usb_transferControl(dev->drv, dev->pipeCtrl, &setup, (void *)data, size, usb_dir_out);
}


static int usbwlan_ctrlReceive(usbwlan_dev_t *dev, void *data, size_t size)
{
	usb_setup_packet_t setup = {
		.bmRequestType = REQUEST_DIR_DEV2HOST | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE,
		.bRequest = 1,
		.wValue = 0,
		.wIndex = 0,
		.wLength = size,
	};
	return usb_transferControl(dev->drv, dev->pipeCtrl, &setup, data, size, usb_dir_in);
}


static int usbwlan_hwRegWrite(usbwlan_dev_t *dev, uint8_t cmd, const void *data, size_t size)
{
	usb_setup_packet_t setup = {
		.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_VENDOR | REQUEST_RECIPIENT_INTERFACE,
		.bRequest = cmd,
		.wValue = 1,
		.wIndex = 0,
		.wLength = size,
	};
	return usb_transferControl(dev->drv, dev->pipeCtrl, &setup, (void *)data, size, usb_dir_out);
}


static int usbwlan_hwRegRead(usbwlan_dev_t *dev, uint8_t cmd, uint32_t regaddr, void *data, size_t size)
{
	usb_setup_packet_t setup = {
		.bmRequestType = REQUEST_DIR_DEV2HOST | REQUEST_TYPE_VENDOR | REQUEST_RECIPIENT_INTERFACE,
		.bRequest = cmd,
		.wValue = (uint16_t)(regaddr & 0xffff),
		.wIndex = (uint16_t)((regaddr >> 16) & 0xffff),
		.wLength = size,
	};
	return usb_transferControl(dev->drv, dev->pipeCtrl, &setup, data, size, usb_dir_in);
}


static int usbwlan_bulkSend(usbwlan_dev_t *dev, const void *data, size_t size)
{
	return usb_transferBulk(dev->drv, dev->pipeBulkOut, (void *)data, size, usb_dir_out);
}


/*
 * if EWOULDBLOCK returned - caller shall not respond to msg
 * calls must be synchronized by dev->rx.lock
 */
static int _usbwlan_bulkReceive(usbwlan_dev_t *dev, msg_t *msg, msg_rid_t rid)
{
	if (dev->rx.isPending) {
		msg->o.err = -EBUSY;
	}
	else {
		dev->rx.isPending = true;
		dev->rx.msg = *msg;
		dev->rx.rid = rid;
		msg->o.err = usb_transferAsync(dev->drv, dev->pipeBulkIn, dev->rx.urbid, USBWLAN_BULK_SIZE, NULL);
		if (msg->o.err >= 0) {
			return EWOULDBLOCK;
		}
	}
	return 0;
}


static int usbwlan_handleDevCtl(msg_t *msg, usbwlan_dev_t *dev)
{
	usbwlan_i_t *imsg = (usbwlan_i_t *)msg->i.raw;

	switch (imsg->type) {
		case usbwlan_dl:
			return usbwlan_dlCmd(dev, imsg->dl.cmd, imsg->dl.wIndex, msg->o.data, msg->o.size);

		case usbwlan_ctrlIn:
			return usbwlan_ctrlReceive(dev, msg->o.data, msg->o.size);

		case usbwlan_ctrlOut:
			return usbwlan_ctrlSend(dev, msg->i.data, msg->i.size);

		case usbwlan_regRead:
			return usbwlan_hwRegRead(dev, imsg->reg.cmd, imsg->reg.regaddr, msg->o.data, msg->o.size);

		case usbwlan_regWrite:
			return usbwlan_hwRegWrite(dev, imsg->reg.cmd, msg->i.data, msg->i.size);

		case usbwlan_abort:
			/* forced abort */
			mutexLock(usbwlan_common.lock);
			_usbwlan_close(dev, msg->pid);
			mutexUnlock(usbwlan_common.lock);
			return EOK;

		default:
			return -EINVAL;
	}
}


static void usbwlan_msgthread(void *arg)
{
	usbwlan_dev_t *dev;
	msg_t msg;
	msg_rid_t rid;
	int err;

	for (;;) {
		err = msgRecv(usbwlan_common.msgport, &msg, &rid);
		if (err < 0) {
			if (err == EINTR) {
				continue;
			}
			usbwlan_log("msgRecv returned with err: %s (%d)", strerror(-err), err);
			break;
		}

		/* Ignore this msg, as it might have been sent by us after deletion event */
		if (msg.type == mtUnlink) {
			msg.o.err = EOK;
			msgRespond(usbwlan_common.msgport, &msg, rid);
			continue;
		}

		dev = usbwlan_get(msg.oid.id);
		if (dev == NULL) {
			msg.o.err = -ENOENT;
			msgRespond(usbwlan_common.msgport, &msg, rid);
			continue;
		}

		/* A device can be opened only by one process */
		/* FIXME: On process exit, we get mtClose with pid=1 (as of 21.10.2025) */
		if (((msg.type != mtOpen) && (msg.type != mtClose)) && (msg.pid != dev->clientpid && msg.pid != 1)) {
			usbwlan_put(dev);
			msg.o.err = -EBADF;
			msgRespond(usbwlan_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				mutexLock(usbwlan_common.lock);
				msg.o.err = _usbwlan_open(dev, msg.i.openclose.flags, msg.pid);
				mutexUnlock(usbwlan_common.lock);
				break;

			case mtClose:
				mutexLock(usbwlan_common.lock);
				msg.o.err = _usbwlan_close(dev, msg.pid);
				mutexUnlock(usbwlan_common.lock);
				break;

			case mtRead:
				mutexLock(dev->rx.lock);
				err = _usbwlan_bulkReceive(dev, &msg, rid);
				mutexUnlock(dev->rx.lock);
				if (err == EWOULDBLOCK) {
					usbwlan_put(dev);
					continue;
				}
				break;

			case mtWrite:
				msg.o.err = usbwlan_bulkSend(dev, msg.i.data, msg.i.size);
				break;

			case mtDevCtl:
				msg.o.err = usbwlan_handleDevCtl(&msg, dev);
				break;

			case mtDestroy:
				if (msg.pid != getpid()) {
					/* assuming mtDestroy can only be called from usbwlan_destroy */
					msg.o.err = -EPERM;
				}
				else {
					usbwlan_put(dev);
					msgRespond(usbwlan_common.msgport, &msg, rid);
					endthread();
					__builtin_unreachable();
				}
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}
		usbwlan_put(dev);
		msgRespond(usbwlan_common.msgport, &msg, rid);
	}

	endthread();
}


static int usbwlan_destroy(usb_driver_t *drv)
{
	usbwlan_dev_t *dev;
	rbnode_t *node;

	if (usbwlan_common.msgport != 0) {
		for (size_t i = 0; i < USBWLAN_N_MSGTHREADS; i++) {
			msg_t msg = { 0 };
			msg.type = mtDestroy;
			(void)msgSend(usbwlan_common.msgport, &msg);
		}
		portDestroy(usbwlan_common.msgport);
	}

	node = lib_rbMinimum(usbwlan_common.devices.root);
	while (node != NULL) {
		dev = lib_treeof(usbwlan_dev_t, node, lib_treeof(idnode_t, linkage, node));
		node = lib_rbNext(node);
		usbwlan_free(dev);
	}
	memset(&usbwlan_common.devices, 0, sizeof(usbwlan_common.devices));

	if (usbwlan_common.lock != 0) {
		resourceDestroy(usbwlan_common.lock);
		usbwlan_common.lock = 0;
	}

	return 0;
}


static int usbwlan_init(usb_driver_t *drv, void *arg)
{
	int ret = 0;

	do {
		if (portCreate(&usbwlan_common.msgport) != 0) {
			usbwlan_log("Failed to create port");
			ret = -ENOMEM;
			break;
		}

		if (mutexCreate(&usbwlan_common.lock) != 0) {
			usbwlan_log("Failed to create lock");
			ret = -ENOMEM;
			break;
		}

		idtree_init(&usbwlan_common.devices);

		for (size_t i = 0; i < USBWLAN_N_MSGTHREADS; i++) {
			ret = beginthread(usbwlan_msgthread, USBWLAN_MSGTHREAD_PRIO, usbwlan_common.stacks[i], sizeof(usbwlan_common.stacks[i]), NULL);
			if (ret < 0) {
				usbwlan_log("Failed to start msgthread #%zu/%u", i + 1, USBWLAN_N_MSGTHREADS);
				break;
			}
		}
	} while (0);

	if (ret != 0) {
		(void)usbwlan_destroy(drv);
		return ret;
	}

	return EOK;
}


static usb_driver_t usbwlan_drv = {
	.name = "usbwlan",
	.handlers = {
		.insertion = usbwlan_handleInsertion,
		.deletion = usbwlan_handleDeletion,
		.completion = usbwlan_handleCompletion,
	},
	.ops = {
		.init = usbwlan_init,
		.destroy = usbwlan_destroy,
	},
	.filters = filters,
	.nfilters = sizeof(filters) / sizeof(filters[0]),
	.priv = &usbwlan_common,
};


__attribute__((constructor)) static void usbwlan_register(void)
{
	usb_driverRegister(&usbwlan_drv);
}
