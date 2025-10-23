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

#ifndef USBWLAN_N_MSGTHREADS
#define USBWLAN_N_MSGTHREADS 2
#endif

#ifndef USBWLAN_MSGTHREAD_STACK_SIZE
#define USBWLAN_MSGTHREAD_STACK_SIZE 1024
#endif

#ifndef USBWLAN_MSGTHREAD_PRIO
#define USBWLAN_MSGTHREAD_PRIO 2
#endif

#ifndef USBWLAN_BULK_SIZE
#define USBWLAN_BULK_SIZE 2048
#endif

#define log_err(fmt, ...) \
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

	struct {
		handle_t lock;
		handle_t cond;
		bool ongoing;

		int urbid;
		int err;
		char *data;
		size_t len;
	} bulkIn;

	char path[32];

	int flags;
	pid_t clientpid;
	volatile int refcnt; /* protected by usbwlan_common.lock */

	usb_driver_t *drv;
} usbwlan_dev_t;


static struct {
	idtree_t devices;
	unsigned msgport;
	handle_t lock;
	char stacks[USBWLAN_N_MSGTHREADS][USBWLAN_MSGTHREAD_STACK_SIZE] __attribute__((aligned(8)));
} usbwlan_common;


static const usb_device_id_t filters[] = {
	/*
	 * Sterling LWB5+ USB Dongle
	 * Enumerates as Remote Download for the host to download firmware onto it
	 * Then, it enumerates as the 802.11 device
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
		log_err("Not enough memory for new device");
		return NULL;
	}

	if (idtree_alloc(&usbwlan_common.devices, &dev->node) < 0) {
		log_err("Could not alloc idtree for new device");
		free(dev);
		return NULL;
	}

	if (mutexCreate(&dev->bulkIn.lock) < 0) {
		log_err("Could not create lock");
		idtree_remove(&usbwlan_common.devices, &dev->node);
		free(dev);
		return NULL;
	}

	if (condCreate(&dev->bulkIn.cond) < 0) {
		log_err("Could not create cond");
		resourceDestroy(dev->bulkIn.lock);
		idtree_remove(&usbwlan_common.devices, &dev->node);
		free(dev);
		return NULL;
	}

	dev->refcnt = 1;

	return dev;
}


static void usbwlan_free(usbwlan_dev_t *dev)
{
	log_err("Device %s removed", dev->path);
	remove(dev->path);
	resourceDestroy(dev->bulkIn.lock);
	resourceDestroy(dev->bulkIn.cond);
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
	int rfcnt;

	mutexLock(usbwlan_common.lock);
	rfcnt = _usbwlan_put(dev);
	mutexUnlock(usbwlan_common.lock);

	if (rfcnt == 0) {
		usbwlan_free(dev);
	}
}


static usbwlan_dev_t *usbwlan_getByPipe(int pipe)
{
	usbwlan_dev_t *tmp, *dev = NULL;
	rbnode_t *node;

	mutexLock(usbwlan_common.lock);
	for (node = lib_rbMinimum(usbwlan_common.devices.root); node != NULL; node = lib_rbNext(node)) {
		tmp = lib_treeof(usbwlan_dev_t, node, lib_treeof(idnode_t, linkage, node));
		if (tmp->pipeCtrl == pipe || tmp->pipeBulkIn == pipe || tmp->pipeBulkOut == pipe || tmp->pipeIntIn == pipe) {
			dev = tmp;
			dev->refcnt++;
			break;
		}
	}
	mutexUnlock(usbwlan_common.lock);

	return dev;
}


static int usbwlan_urbAlloc(usbwlan_dev_t *dev)
{
	dev->bulkIn.urbid = usb_urbAlloc(dev->drv, dev->pipeBulkIn, NULL, usb_dir_in, USBWLAN_BULK_SIZE, usb_transfer_bulk);
	if (dev->bulkIn.urbid < 0) {
		return -ENOMEM;
	}

	return 0;
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
	dev->bulkIn.ongoing = false;

	do {
		dev->pipeCtrl = usb_open(drv, ins, usb_transfer_control, 0);
		if (dev->pipeCtrl < 0) {
			log_err("Fail to open ctrl pipe");
			err = -EINVAL;
			break;
		}

		dev->pipeIntIn = usb_open(drv, ins, usb_transfer_interrupt, usb_dir_in);
		if (dev->pipeIntIn < 0) {
			log_err("Fail to open interrupt IN pipe");
			err = -EINVAL;
			break;
		}

		dev->pipeBulkIn = usb_open(drv, ins, usb_transfer_bulk, usb_dir_in);
		if (dev->pipeBulkIn < 0) {
			log_err("Fail to open bulk IN pipe");
			err = -EINVAL;
			break;
		}

		dev->pipeBulkOut = usb_open(drv, ins, usb_transfer_bulk, usb_dir_out);
		if (dev->pipeBulkOut < 0) {
			log_err("Fail to open bulk OUT pipe");
			err = -EINVAL;
			break;
		}

		oid.port = usbwlan_common.msgport;
		oid.id = idtree_id(&dev->node);
		snprintf(dev->path, sizeof(dev->path), "/dev/wlan%u", oid.id);
		if (create_dev(&oid, dev->path) < 0) {
			log_err("Fail to create device file");
			err = -EINVAL;
			break;
		}
	} while (0);

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

		if (dev != NULL && dev->dev.bus == del->bus && dev->dev.dev == del->dev && dev->dev.interface == del->interface) {
			if (_usbwlan_put(dev) == 0) {
				usbwlan_free(dev);
			}
		}

		node = next;
	}

	mutexUnlock(usbwlan_common.lock);

	return 0;
}


static int usbwlan_handleCompletion(usb_driver_t *drv, usb_completion_t *completion, const char *data, size_t len)
{
	int ret = 0;
	usbwlan_dev_t *dev = usbwlan_getByPipe(completion->pipeid);
	if (dev == NULL) {
		return -ENODEV;
	}

	mutexLock(dev->bulkIn.lock);

	do {
		if (completion->err != 0) {
			ret = -abs(completion->err);
			break;
		}

		if (completion->pipeid != dev->pipeBulkIn || completion->urbid != dev->bulkIn.urbid) {
			ret = -EINVAL;
			break;
		}

		dev->bulkIn.data = (void *)data;
		dev->bulkIn.len = len;
	} while (0);

	if (ret != 0) {
		dev->bulkIn.err = ret;
	}

	condBroadcast(dev->bulkIn.cond);
	mutexUnlock(dev->bulkIn.lock);

	usbwlan_put(dev);

	return ret;
}


static int usbwlan_open(usbwlan_dev_t *dev, int flags, pid_t pid)
{
	if (dev->clientpid != 0 || dev->flags != 0) {
		return -EBUSY;
	}

	if (usbwlan_urbAlloc(dev) < 0) {
		return -ENOMEM;
	}

	dev->clientpid = pid;
	dev->flags = flags;

	return EOK;
}


static void usbwlan_close(usbwlan_dev_t *dev)
{
	dev->flags = 0;
	dev->clientpid = 0;

	usb_urbFree(dev->drv, dev->pipeBulkIn, dev->bulkIn.urbid);

	mutexLock(dev->bulkIn.lock);
	dev->bulkIn.err = -EINTR;
	condBroadcast(dev->bulkIn.cond);
	mutexUnlock(dev->bulkIn.lock);
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


static int usbwlan_regWrite(usbwlan_dev_t *dev, uint8_t cmd, const void *data, size_t size)
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


static int usbwlan_regRead(usbwlan_dev_t *dev, uint8_t cmd, uint32_t regaddr, void *data, size_t size)
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
 * This transfer is responsible for reading incoming packets, so it's important
 * to let it be interrupted if the device needs to close
 */
static int usbwlan_bulkReceive(usbwlan_dev_t *dev, void *data, size_t size)
{
	int ret = 0;

	mutexLock(dev->bulkIn.lock);

	if (dev->bulkIn.ongoing) {
		condWait(dev->bulkIn.cond, dev->bulkIn.lock, 0);
	}

	/* file was closed - return */
	if (dev->clientpid == 0) {
		mutexUnlock(dev->bulkIn.lock);
		return -EBADF;
	}

	dev->bulkIn.data = NULL;
	dev->bulkIn.len = 0;
	dev->bulkIn.err = 0;

	if (usb_transferAsync(dev->drv, dev->pipeBulkIn, dev->bulkIn.urbid, size, NULL) < 0) {
		mutexUnlock(dev->bulkIn.lock);
		return -ENOMEM;
	}

	dev->bulkIn.ongoing = true;

	condWait(dev->bulkIn.cond, dev->bulkIn.lock, 0);

	if (dev->bulkIn.err != 0) {
		ret = dev->bulkIn.err;
	}
	else if (data == NULL && size == 0) {
		ret = 0;
	}
	else if (dev->bulkIn.data != NULL && dev->bulkIn.len != 0) {
		size = min(size, dev->bulkIn.len);
		memcpy(data, dev->bulkIn.data, size);
		ret = size;
	}
	else {
		ret = -EFAULT;
	}

	dev->bulkIn.ongoing = false;
	mutexUnlock(dev->bulkIn.lock);

	return ret;
}


static int usbwlan_handleDevCtl(msg_t *msg, usbwlan_dev_t *dev)
{
	usbwlan_i_t *imsg = (usbwlan_i_t *)msg->i.raw;

	switch (imsg->type) {
		case usbwlan_dl:
			return usbwlan_dlCmd(dev, imsg->dl.cmd, imsg->dl.wIndex, msg->o.data, msg->o.size);

		case usbwlan_ctrl_in:
			return usbwlan_ctrlReceive(dev, msg->o.data, msg->o.size);

		case usbwlan_ctrl_out:
			return usbwlan_ctrlSend(dev, msg->i.data, msg->i.size);

		case usbwlan_reg_read:
			return usbwlan_regRead(dev, imsg->reg.cmd, imsg->reg.regaddr, msg->o.data, msg->o.size);

		case usbwlan_reg_write:
			return usbwlan_regWrite(dev, imsg->reg.cmd, msg->i.data, msg->i.size);

		/* custom abort, so that we can somehow escape from blocking read hell... */
		case usbwlan_abort:
			usbwlan_close(dev);
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

	for (;;) {
		if (msgRecv(usbwlan_common.msgport, &msg, &rid) < 0) {
			log_err("msgRecv returned with err");
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
			msg.o.err = -EBUSY;
			msgRespond(usbwlan_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				if (dev->clientpid != 0) {
					msg.o.err = -EBUSY;
					break;
				}
				msg.o.err = usbwlan_open(dev, msg.i.openclose.flags, msg.pid);
				break;

			case mtClose:
				usbwlan_close(dev);
				msg.o.err = EOK;
				break;

			/*
			 * NOTE: Reads are sync, but can be interrupted by close.
			 * The assumption is that only one read call is performed
			 * at a time. If two reads are performed in two threads
			 * at the same time, the msgthread will be unresponsive
			 * until one of them returns.
			 */
			case mtRead:
				msg.o.err = usbwlan_bulkReceive(dev, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.err = usbwlan_bulkSend(dev, msg.i.data, msg.i.size);
				break;

			case mtDevCtl:
				msg.o.err = usbwlan_handleDevCtl(&msg, dev);
				break;

			case mtDestroy:
				if (dev->clientpid != 0) {
					usbwlan_close(dev);
				}
				usbwlan_put(dev);
				msgRespond(usbwlan_common.msgport, &msg, rid);
				endthread();
				__builtin_unreachable();
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
	for (size_t i = 0; i < USBWLAN_N_MSGTHREADS; i++) {
		msg_t msg = { 0 };
		msg.type = mtDestroy;
		(void)msgSend(usbwlan_common.msgport, &msg);
	}

	if (usbwlan_common.lock != -1) {
		(void)resourceDestroy(usbwlan_common.lock);
	}
	if (usbwlan_common.msgport != -1) {
		portDestroy(usbwlan_common.msgport);
	}

	return 0;
}


static int usbwlan_init(usb_driver_t *drv, void *arg)
{
	int ret = 0;

	usbwlan_common.msgport = -1;
	usbwlan_common.lock = -1;

	do {
		if (portCreate(&usbwlan_common.msgport) != 0) {
			log_err("Fail to create port");
			ret = -ENOMEM;
			break;
		}

		if (mutexCreate(&usbwlan_common.lock) != 0) {
			log_err("Fail to create mutex");
			ret = -ENOMEM;
			break;
		}

		idtree_init(&usbwlan_common.devices);

		for (size_t i = 0; i < USBWLAN_N_MSGTHREADS; i++) {
			ret = beginthread(usbwlan_msgthread, USBWLAN_MSGTHREAD_PRIO, usbwlan_common.stacks[i], sizeof(usbwlan_common.stacks[i]), NULL);
			if (ret < 0) {
				log_err("Fail to start msgthread %zu/%u", i, USBWLAN_N_MSGTHREADS);
				break;
			}
		}
		if (ret != 0) {
			break;
		}
	} while (0);

	if (ret != 0) {
		(void)usbwlan_destroy(drv);
		return ret;
	}

	return 0;
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
