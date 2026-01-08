/*
 * Phoenix-RTOS
 *
 * USB CDC ECM driver
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
#include <sys/rb.h>
#include <sys/minmax.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <unistd.h>

#include <cdc.h>
#include <usbdriver.h>
#include <usbhost.h>
#include <usb.h>

#include "usbecm.h"

#include <board_config.h>

#ifndef USBECM_MSGTHREAD_PRIO
#define USBECM_MSGTHREAD_PRIO 3
#endif

#ifndef USBECM_MSGTHREAD_STACK_SIZE
#define USBECM_MSGTHREAD_STACK_SIZE 1024
#endif

#define USBECM_N_MSGTHREADS 1
#define USBECM_BULK_SIZE    1536

/* FIXME: probably should be read from the union descriptor */
#define USBECM_CTRL_INTERFACE 0
#define USBECM_DATA_INTERFACE 1

#define usbecm_log(fmt, ...) \
	do { \
		fprintf(stderr, "usbecm: " fmt "\n", ##__VA_ARGS__); \
	} while (0)


typedef struct {
	rbnode_t node;
	unsigned int locationID;

	/* USB HOST */
	int pipeCtrl;
	int pipeIntIn;
	int pipeBulkIn;
	int pipeBulkOut;

	char path[32];

	int flags;
	pid_t clientpid;
	volatile int refcnt; /* protected by usbecm_common.lock */

	usb_driver_t *drv;

	struct {
		handle_t lock;
		msg_t msg;
		msg_rid_t rid;
		int urbid;
		bool isPending;
	} rx, irq;
} usbecm_dev_t;

static struct {
	int n;
	rbtree_t devices;
	uint32_t msgport;
	handle_t lock;
	char stacks[USBECM_N_MSGTHREADS][USBECM_MSGTHREAD_STACK_SIZE] __attribute__((aligned(8)));
} usbecm_common;


static const usb_device_id_t filters[] = {
	{ USBDRV_ANY, USBDRV_ANY, USB_INTERFACE_COMMUNICATIONS, USB_SUBCLASS_ECM, USBDRV_ANY },
	{ USBDRV_ANY, USBDRV_ANY, USB_INTERFACE_DATA, 0, USBDRV_ANY }
};


static int usbecm_cmp(rbnode_t *node1, rbnode_t *node2)
{
	usbecm_dev_t *dev1 = lib_treeof(usbecm_dev_t, node, node1);
	usbecm_dev_t *dev2 = lib_treeof(usbecm_dev_t, node, node2);

	if (dev1->locationID > dev2->locationID) {
		return 1;
	}
	else if (dev2->locationID > dev1->locationID) {
		return -1;
	}
	else {
		return 0;
	}
}


static usbecm_dev_t *_usbecm_devAlloc(void)
{
	usbecm_dev_t *dev;

	dev = calloc(1, sizeof(*dev));
	if (dev == NULL) {
		usbecm_log("Not enough memory for new device");
		return NULL;
	}

	dev->rx.urbid = -1;

	if (mutexCreate(&dev->rx.lock) < 0) {
		usbecm_log("Could not create RX mutex");
		free(dev);
		return NULL;
	}

	if (mutexCreate(&dev->irq.lock) < 0) {
		usbecm_log("Could not create IRQ mutex");
		resourceDestroy(dev->rx.lock);
		free(dev);
		return NULL;
	}

	dev->refcnt = 1;

	return dev;
}


static void usbecm_free(usbecm_dev_t *dev)
{
	usbecm_log("Device %s removed", dev->path);
	if (dev->rx.urbid != -1) {
		usb_urbFree(dev->drv, dev->pipeBulkIn, dev->rx.urbid);
	}
	resourceDestroy(dev->rx.lock);
	resourceDestroy(dev->irq.lock);
	destroy_dev(dev->path);
	free(dev);
}


static usbecm_dev_t *_usbecm_get(unsigned int id)
{
	static usbecm_dev_t tmp;
	usbecm_dev_t *dev;

	tmp.locationID = id;
	rbnode_t *node = lib_rbFind(&usbecm_common.devices, &tmp.node);
	dev = lib_treeof(usbecm_dev_t, node, node);
	if (dev != NULL) {
		dev->refcnt++;
	}

	return dev;
}


static usbecm_dev_t *usbecm_get(unsigned int id)
{
	usbecm_dev_t *dev;

	mutexLock(usbecm_common.lock);
	dev = _usbecm_get(id);
	mutexUnlock(usbecm_common.lock);

	return dev;
}


static usbecm_dev_t *usbecm_getByPipe(int pipe)
{
	usbecm_dev_t *tmp, *dev = NULL;
	rbnode_t *node;

	mutexLock(usbecm_common.lock);
	for (node = lib_rbMinimum(usbecm_common.devices.root); node != NULL; node = lib_rbNext(node)) {
		tmp = lib_treeof(usbecm_dev_t, node, lib_treeof(idnode_t, linkage, node));
		assert(tmp != NULL);
		if (tmp->pipeBulkIn == pipe || tmp->pipeBulkOut == pipe || tmp->pipeIntIn == pipe || pipe == tmp->pipeCtrl) {
			dev = tmp;
			dev->refcnt++;
			break;
		}
	}
	mutexUnlock(usbecm_common.lock);

	return dev;
}


static int _usbecm_put(usbecm_dev_t *dev)
{
	dev->refcnt--;
	if (dev->refcnt == 0) {
		lib_rbRemove(&usbecm_common.devices, &dev->node);
	}
	return dev->refcnt;
}


static void usbecm_put(usbecm_dev_t *dev)
{
	int refcnt;

	mutexLock(usbecm_common.lock);
	refcnt = _usbecm_put(dev);
	mutexUnlock(usbecm_common.lock);

	if (refcnt == 0) {
		usbecm_free(dev);
	}
}


static int usbecm_handleInsertion(usb_driver_t *drv, usb_devinfo_t *ins, usb_event_insertion_t *event_out)
{
	usbecm_dev_t *dev;
	oid_t oid;
	int err = 0;
	bool created = false;

	mutexLock(usbecm_common.lock);

	// TODO: if we already have "half" the device:
	// TODO: 1) check, which half we have and which half was "inserted"
	// TODO: 2) if match: open relevant pipes
	// TODO:
	// TODO: if we don't have the other half: alloc and insert device, open relevant pipes and return
	dev = _usbecm_get(ins->locationID);

	do {
		if (dev != NULL) {
			oid.port = usbecm_common.msgport;
			oid.id = dev->locationID;
			snprintf(dev->path, sizeof(dev->path), "/dev/usbecm%u", usbecm_common.n++);
			if (create_dev(&oid, dev->path) < 0) {
				usbecm_log("Failed to create device file");
				err = -EINVAL;
				break;
			}
			created = true;
		}
		else {
			dev = _usbecm_devAlloc();
			if (dev == NULL) {
				usbecm_log("Failed to allocate memory for device");
				err = -ENOMEM;
				break;
			}
			dev->drv = drv;
			dev->locationID = ins->locationID;
			if (lib_rbInsert(&usbecm_common.devices, &dev->node) != NULL) {
				// TODO: should this be an error?
				usbecm_log("Device is already inserted");
				err = -EINVAL;
				break;
			}
		}

		// TODO: check what we already have and what we should have
		if (ins->interface == USBECM_CTRL_INTERFACE) {
			dev->pipeCtrl = usb_open(drv, ins, usb_transfer_control, 0);
			if (dev->pipeCtrl < 0) {
				usbecm_log("Failed to open ctrl pipe");
				err = -EINVAL;
				break;
			}

			dev->pipeIntIn = usb_open(drv, ins, usb_transfer_interrupt, usb_dir_in);
			if (dev->pipeIntIn < 0) {
				usbecm_log("Failed to open interrupt IN pipe");
				err = -EINVAL;
				break;
			}
		}
		else if (ins->interface == USBECM_DATA_INTERFACE) {
			if (usb_setInterface(dev->drv, dev->pipeCtrl, ins->interface, 1) < 0) {
				usbecm_log("Failed to set alternate setting to 1");
				err = -EINVAL;
				break;
			}

			dev->pipeBulkIn = usb_open(drv, ins, usb_transfer_bulk, usb_dir_in);
			if (dev->pipeBulkIn < 0) {
				usbecm_log("Failed to open bulk IN pipe");
				err = -EINVAL;
				break;
			}

			dev->pipeBulkOut = usb_open(drv, ins, usb_transfer_bulk, usb_dir_out);
			if (dev->pipeBulkOut < 0) {
				usbecm_log("Failed to open bulk OUT pipe");
				err = -EINVAL;
				break;
			}

			dev->rx.urbid = usb_urbAlloc(dev->drv, dev->pipeBulkIn, NULL, usb_dir_in, USBECM_BULK_SIZE, usb_transfer_bulk);
			if (dev->rx.urbid < 0) {
				usbecm_log("Fail to allocate URB");
				err = -ENOMEM;
				break;
			}
		}
		else {
			usbecm_log("Inserted interface %d is invalid", ins->interface);
			err = -EINVAL;
			break;
		}

	} while (0);

	if (err < 0) {
		_usbecm_put(dev);
	}

	mutexUnlock(usbecm_common.lock);

	/* report device created only once we've "assembled" the whole device */
	if (created) {
		if (err < 0) {
			usbecm_free(dev);
			usbecm_log("returning err %d", err);
			return err;
		}
		event_out->deviceCreated = true;
		event_out->dev = oid;
		(void)snprintf(event_out->devPath, sizeof(event_out->devPath), "%s", dev->path);
	}

	usbecm_log("claimed interface %d", ins->interface);

	return 0;
}


static int usbecm_handleDeletion(usb_driver_t *drv, usb_deletion_t *del)
{
	rbnode_t *node, *next;
	usbecm_dev_t *dev;

	mutexLock(usbecm_common.lock);

	node = lib_rbMinimum(usbecm_common.devices.root);
	while (node != NULL) {
		next = lib_rbNext(node);
		dev = lib_treeof(usbecm_dev_t, node, lib_treeof(idnode_t, linkage, node));

		assert(dev != NULL);
		// TODO: dev->dev maybe needed to handle deletion of the device - it's a composit key with dev->dev, dev->bus and dev->interface
		if (dev->locationID == del->interface) {
			if (_usbecm_put(dev) == 0) {
				usbecm_free(dev);
			}
		}

		node = next;
	}

	mutexUnlock(usbecm_common.lock);

	return 0;
}


static int _usbecm_irqRespond(usbecm_dev_t *dev, int linkstatus)
{
	int err;
	usbecm_msg_t *omsg = (usbecm_msg_t *)dev->irq.msg.o.raw;

	if (!dev->irq.isPending) {
		return -1;
	}

	dev->irq.msg.o.err = EOK;
	omsg->type = usbecm_getLinkStatusChange;
	omsg->irq.linkstatus = linkstatus;

	err = msgRespond(usbecm_common.msgport, &dev->irq.msg, dev->irq.rid);
	dev->irq.isPending = false;
	return err;
}


static int _usbecm_rxRespond(usbecm_dev_t *dev, int completionErr, const char *data, size_t len)
{
	int err;
	if (!dev->rx.isPending) {
		return -1;
	}

	if (completionErr != 0) {
		dev->rx.msg.o.err = -abs(completionErr);
	}
	else {
		memcpy(dev->rx.msg.o.data, data, len);
		dev->rx.msg.o.size = len;
		dev->rx.msg.o.err = len;
	}

	err = msgRespond(usbecm_common.msgport, &dev->rx.msg, dev->rx.rid);
	dev->rx.isPending = false;
	return err;
}


static int usbecm_handleCompletion(struct usb_driver *drv, usb_completion_t *completion, const char *data, size_t len)
{
	int err;
	usbecm_dev_t *dev;

	dev = usbecm_getByPipe(completion->pipeid);
	if (dev == NULL) {
		return -1;
	}

	if (completion->pipeid == dev->pipeIntIn) {
		mutexLock(dev->irq.lock);
		// TODO: read linkstatus
		err = _usbecm_irqRespond(dev, -1);
		mutexUnlock(dev->irq.lock);
	}
	else if (completion->pipeid == dev->pipeBulkIn) {
		mutexLock(dev->rx.lock);
		err = _usbecm_rxRespond(dev, completion->err, data, len);
		mutexUnlock(dev->rx.lock);
	}
	else {
		err = -1;
	}

	usbecm_put(dev);

	return err;
}


static int _usbecm_close(usbecm_dev_t *dev, pid_t pid)
{
	/* FIXME: On process exit, we get mtClose with pid=1 (as of 21.10.2025) */
	if (dev->clientpid == 0 || (dev->clientpid != pid && pid != 1)) {
		return -EINVAL;
	}

	mutexLock(dev->rx.lock);
	dev->rx.isPending = false;
	mutexUnlock(dev->rx.lock);

	dev->flags = 0;
	dev->clientpid = 0;

	return EOK;
}


static int _usbecm_open(usbecm_dev_t *dev, int flags, pid_t pid)
{
	if (dev->clientpid != 0 || dev->flags != 0) {
		return -EBUSY;
	}

	dev->clientpid = pid;
	dev->flags = flags;

	return EOK;
}


static int usbecm_bulkSend(usbecm_dev_t *dev, const void *data, size_t size)
{
	return usb_transferBulk(dev->drv, dev->pipeBulkOut, (void *)data, size, usb_dir_out);
}


static void usbecm_msgthread(void *arg)
{
	usbecm_dev_t *dev;
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		if (msgRecv(usbecm_common.msgport, &msg, &rid) < 0) {
			usbecm_log("msgRecv returned with err");
			continue;
		}

		/* Ignore this msg, as it might have been sent by us after deletion event */
		if (msg.type == mtUnlink) {
			msg.o.err = EOK;
			msgRespond(usbecm_common.msgport, &msg, rid);
			continue;
		}

		dev = usbecm_get(msg.oid.id);
		if (dev == NULL) {
			msg.o.err = -ENOENT;
			msgRespond(usbecm_common.msgport, &msg, rid);
			continue;
		}

		/* A device can be opened only by one process */
		/* FIXME: On process exit, we get mtClose with pid=1 (as of 21.10.2025) */
		if (((msg.type != mtOpen) && (msg.type != mtClose)) && (msg.pid != dev->clientpid && msg.pid != 1)) {
			usbecm_put(dev);
			msg.o.err = -EBUSY;
			msgRespond(usbecm_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				mutexLock(usbecm_common.lock);
				msg.o.err = _usbecm_open(dev, msg.i.openclose.flags, msg.pid);
				mutexUnlock(usbecm_common.lock);
				break;

			case mtClose:
				mutexLock(usbecm_common.lock);
				msg.o.err = _usbecm_close(dev, msg.pid);
				mutexUnlock(usbecm_common.lock);
				break;

			case mtRead:
				mutexLock(dev->rx.lock);
				if (dev->rx.isPending) {
					msg.o.err = -EINTR;
				}
				else {
					dev->rx.isPending = true;
					dev->rx.msg = msg;
					dev->rx.rid = rid;
					msg.o.err = usb_transferAsync(dev->drv, dev->pipeBulkIn, dev->rx.urbid, USBECM_BULK_SIZE, NULL);
					if (msg.o.err >= 0) {
						mutexUnlock(dev->rx.lock);
						usbecm_put(dev);
						continue;
					}
				}
				mutexUnlock(dev->rx.lock);
				break;

			case mtWrite:
				msg.o.err = usbecm_bulkSend(dev, msg.i.data, msg.i.size);
				break;

			case mtDestroy:
				if (msg.pid != getpid()) {
					/* assume mtDestroy can only be called from usbecm_destroy */
					msg.o.err = -EPERM;
				}
				else {
					usbecm_put(dev);
					msgRespond(usbecm_common.msgport, &msg, rid);
					endthread();
					__builtin_unreachable();
				}
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}
		usbecm_put(dev);
		msgRespond(usbecm_common.msgport, &msg, rid);
	}

	endthread();
}


static int usbecm_destroy(usb_driver_t *drv)
{
	usbecm_dev_t *dev;
	rbnode_t *node;

	for (size_t i = 0; i < USBECM_N_MSGTHREADS; i++) {
		msg_t msg = { 0 };
		msg.type = mtDestroy;
		(void)msgSend(usbecm_common.msgport, &msg);
	}

	node = lib_rbMinimum(usbecm_common.devices.root);
	while (node != NULL) {
		dev = lib_treeof(usbecm_dev_t, node, lib_treeof(idnode_t, linkage, node));
		node = lib_rbNext(node);
		usbecm_free(dev);
	}

	resourceDestroy(usbecm_common.lock);
	portDestroy(usbecm_common.msgport);

	return 0;
}


static int usbecm_init(usb_driver_t *drv, void *arg)
{
	int ret = 0;

	do {
		if (portCreate(&usbecm_common.msgport) != 0) {
			usbecm_log("Failed to create port");
			ret = -ENOMEM;
			break;
		}

		if (mutexCreate(&usbecm_common.lock) != 0) {
			usbecm_log("Failed to create lock");
			ret = -ENOMEM;
			break;
		}

		lib_rbInit(&usbecm_common.devices, usbecm_cmp, NULL);

		for (size_t i = 0; i < USBECM_N_MSGTHREADS; i++) {
			ret = beginthread(usbecm_msgthread, USBECM_MSGTHREAD_PRIO, usbecm_common.stacks[i], sizeof(usbecm_common.stacks[i]), NULL);
			if (ret < 0) {
				usbecm_log("Failed to start msgthread #%zu/%u", i + 1, USBECM_N_MSGTHREADS);
				break;
			}
		}
	} while (0);

	if (ret != 0) {
		(void)usbecm_destroy(drv);
		return ret;
	}

	return EOK;
}


static usb_driver_t usbecm_drv = {
	.name = "usbecm",
	.handlers = {
		.insertion = usbecm_handleInsertion,
		.deletion = usbecm_handleDeletion,
		.completion = usbecm_handleCompletion,
	},
	.ops = {
		.init = usbecm_init,
		.destroy = usbecm_destroy,
	},
	.filters = filters,
	.nfilters = sizeof(filters) / sizeof(filters[0]),
	.priv = &usbecm_common,
};


__attribute__((constructor)) static void usbecm_register(void)
{
	usb_driverRegister(&usbecm_drv);
}
