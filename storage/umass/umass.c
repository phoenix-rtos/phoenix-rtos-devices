/*
 * Phoenix-RTOS
 *
 * USB Mass Storage class driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <arpa/inet.h>
#include <errno.h>
#include <limits.h>
#include <sys/list.h>
#include <sys/msg.h>
#include <sys/minmax.h>
#include <sys/types.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <posix/utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <usb.h>
#include <usbdriver.h>

#include "../pc-ata/mbr.h"

#define UMASS_N_MSG_THREADS 1

#define UMASS_SECTOR_SIZE 512

#define UMASS_WRITE 0
#define UMASS_READ  0x80

#define CBW_SIG 0x43425355
#define CSW_SIG 0x53425355


typedef struct {
	uint32_t sig;
	uint32_t tag;
	uint32_t dlen;
	uint8_t flags;
	uint8_t lun;
	uint8_t clen;
	uint8_t cmd[16];
} __attribute__((packed)) umass_cbw_t;


typedef struct {
	uint32_t sig;
	uint32_t tag;
	uint32_t dr;
	uint8_t status;
} __attribute__((packed)) umass_csw_t;


typedef struct {
	uint8_t opcode;
	uint8_t action : 5;
	uint8_t misc0 : 3;
	uint32_t lba;
	uint8_t misc1;
	uint16_t length;
	uint8_t control;
} __attribute__((packed)) scsi_cdb10_t;


typedef struct umass_dev {
	char buffer[8 * UMASS_SECTOR_SIZE];
	struct umass_dev *prev, *next;
	usb_devinfo_t instance;
	char path[32];
	int pipeCtrl;
	int pipeIn;
	int pipeOut;
	int id;
	int fileId;
	int tag;
	unsigned port;
	uint32_t partOffset;
	uint32_t partSize;
	handle_t lock;
} umass_dev_t;


static struct {
	umass_dev_t *devices;
	char stack[UMASS_N_MSG_THREADS][1024] __attribute__((aligned(8)));
	unsigned drvport;
	unsigned msgport;
	handle_t lock;
	int lastId;
} umass_common;


static const usb_device_id_t filters[] = {
	/* USB Mass Storage class */
	{ USBDRV_ANY, USBDRV_ANY, USB_CLASS_MASS_STORAGE, USBDRV_ANY, USBDRV_ANY },
};


static int umass_transmit(umass_dev_t *dev, void *cmd, size_t clen, char *data, size_t dlen, int dir)
{
	umass_cbw_t cbw = { 0 };
	umass_csw_t csw = { 0 };
	int ret = 0, bytes = 0;
	int dataPipe;

	if (clen > 16)
		return -1;

	dataPipe = (dir == usb_dir_out) ? dev->pipeOut : dev->pipeIn;
	cbw.sig = CBW_SIG;
	cbw.tag = dev->tag++;
	cbw.dlen = dlen;
	cbw.flags = (dir == usb_dir_out) ? UMASS_WRITE : UMASS_READ;
	cbw.lun = 0;
	cbw.clen = clen;
	memcpy(cbw.cmd, cmd, clen);

	if ((ret = usb_transferBulk(dev->pipeOut, &cbw, sizeof(cbw), usb_dir_out)) != sizeof(cbw)) {
		fprintf(stderr, "umass_transmit: usb_transferBulk OUT failed\n");
		return -EIO;
	}

	/* Optional data transfer */
	if (dlen > 0) {
		if ((ret = usb_transferBulk(dataPipe, data, dlen, dir)) < 0) {
			fprintf(stderr, "umass_transmit: umass_transmit data transfer failed\n");
			return ret;
		}
		bytes = ret;
	}

	if ((ret = usb_transferBulk(dev->pipeIn, &csw, sizeof(csw), usb_dir_in)) != sizeof(csw)) {
		fprintf(stderr, "umass_transmit: usb_transferBulk IN transfer failed\n");
		return -EIO;
	}

	/* Transferred finished, check transfer correctness */
	if (csw.sig != CSW_SIG || csw.tag != cbw.tag || csw.status != 0) {
		fprintf(stderr, "umass_transmit: transfer incorrect\n");
		return -EIO;
	}

	return bytes;
}


static int umass_check(umass_dev_t *dev)
{
	scsi_cdb10_t readcmd = {
		.opcode = 0x28,
		.length = htons(0x1)
	};
	char testcmd[6] = { 0 };
	mbr_t *mbr;

	if (umass_transmit(dev, testcmd, sizeof(testcmd), NULL, 0, usb_dir_in) < 0) {
		fprintf(stderr, "umass_transmit failed\n");
		return -1;
	}

	/* Read MBR */
	if (umass_transmit(dev, &readcmd, sizeof(readcmd), dev->buffer, UMASS_SECTOR_SIZE, usb_dir_in) < 0) {
		fprintf(stderr, "umass_transmit 2 failed\n");
		return -1;
	}

	mbr = (mbr_t *)dev->buffer;
	if (mbr->magic != MBR_MAGIC)
		return -1;

	/* Read only the first partition */
	dev->partOffset = mbr->pent[0].start;
	dev->partSize = mbr->pent[0].sectors;

	return 0;
}


static int umass_read(umass_dev_t *dev, offs_t offs, char *buf, size_t len)
{
	scsi_cdb10_t readcmd = { .opcode = 0x28 };
	int ret;

	if ((offs % UMASS_SECTOR_SIZE) || (len % UMASS_SECTOR_SIZE))
		return -EINVAL;

	if (offs + len > dev->partSize * UMASS_SECTOR_SIZE)
		return -EINVAL;

	len = min(len, sizeof(dev->buffer));

	readcmd.lba = htonl(offs / UMASS_SECTOR_SIZE + dev->partOffset);
	readcmd.length = htons((uint16_t)(len / UMASS_SECTOR_SIZE));

	mutexLock(dev->lock);
	if ((ret = umass_transmit(dev, &readcmd, sizeof(readcmd), dev->buffer, len, usb_dir_in)) > 0)
		memcpy(buf, dev->buffer, ret);
	mutexUnlock(dev->lock);

	return ret;
}


static int umass_write(umass_dev_t *dev, offs_t offs, char *buf, size_t len)
{
	scsi_cdb10_t writecmd = { .opcode = 0x2a };
	int ret;

	if ((offs % UMASS_SECTOR_SIZE) || (len % UMASS_SECTOR_SIZE))
		return -EINVAL;

	if (offs + len > dev->partSize * UMASS_SECTOR_SIZE)
		return -EINVAL;

	len = min(len, sizeof(dev->buffer));

	writecmd.lba = htonl(offs / UMASS_SECTOR_SIZE + dev->partOffset);
	writecmd.length = htons((uint16_t)(len / UMASS_SECTOR_SIZE));

	mutexLock(dev->lock);
	memcpy(dev->buffer, buf, len);
	ret = umass_transmit(dev, &writecmd, sizeof(writecmd), dev->buffer, len, usb_dir_out);
	mutexUnlock(dev->lock);

	return ret;
}


static int umass_getattr(umass_dev_t *dev, int type, long long int *attr)
{
	if (type != atSize)
		return -EINVAL;

	*attr = dev->partSize * UMASS_SECTOR_SIZE;

	return EOK;
}


static umass_dev_t *umass_devFind(int id)
{
	umass_dev_t *tmp = umass_common.devices;

	if (tmp != NULL) {
		do {
			if (tmp->fileId == id)
				return tmp;
			tmp = tmp->next;
		} while (tmp != umass_common.devices);
	}

	return NULL;
}


static void umass_msgthr(void *arg)
{
	umass_dev_t *dev;
	msg_rid_t rid;
	int id;
	msg_t msg;

	for (;;) {
		if (msgRecv(umass_common.msgport, &msg, &rid) < 0)
			break;

		/* Ignore this msg, as it might have been sent by us after deletion event */
		if (msg.type == mtUnlink) {
			msg.o.io.err = EOK;
			msgRespond(umass_common.msgport, &msg, rid);
			continue;
		}

		if (msg.type == mtGetAttr)
			id = msg.i.attr.oid.id;
		else
			id = msg.i.io.oid.id;

		mutexLock(umass_common.lock);
		dev = umass_devFind(id);
		mutexUnlock(umass_common.lock);

		if (dev == NULL) {
			msg.o.io.err = -ENOENT;
			msgRespond(umass_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;

			case mtRead:
				msg.o.io.err = umass_read(dev, msg.i.io.offs, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.io.err = umass_write(dev, msg.i.io.offs, msg.i.data, msg.i.size);
				break;

			case mtGetAttr:
				msg.o.attr.err = umass_getattr(dev, msg.i.attr.type, &msg.o.attr.val);
				break;

			default:
				msg.o.io.err = -EINVAL;
		}

		msgRespond(umass_common.msgport, &msg, rid);
	}

	endthread();
}


static umass_dev_t *umass_devAlloc(void)
{
	umass_dev_t *dev;

	if ((dev = malloc(sizeof(umass_dev_t))) == NULL) {
		fprintf(stderr, "umass: Not enough memory\n");
		return NULL;
	}

	/* Get next device number */
	if (umass_common.devices == NULL)
		dev->id = 0;
	else
		dev->id = umass_common.devices->prev->id + 1;

	dev->fileId = umass_common.lastId++;

	if (mutexCreate(&dev->lock)) {
		free(dev);
		return NULL;
	}

	snprintf(dev->path, sizeof(dev->path), "/dev/umass%d", dev->id);

	return dev;
}


static int umass_handleInsertion(usb_devinfo_t *insertion)
{
	umass_dev_t *dev;
	oid_t oid;

	if ((dev = umass_devAlloc()) == NULL) {
		fprintf(stderr, "umass: devAlloc failed\n");
		return -ENOMEM;
	}

	dev->instance = *insertion;
	if ((dev->pipeCtrl = usb_open(insertion, usb_transfer_control, 0)) < 0) {
		free(dev);
		fprintf(stderr, "umass: usb_open failed\n");
		return -EINVAL;
	}

	if (usb_setConfiguration(dev->pipeCtrl, 1) != 0) {
		free(dev);
		fprintf(stderr, "umass: setConfiguration failed\n");
		return -EINVAL;
	}

	if ((dev->pipeIn = usb_open(insertion, usb_transfer_bulk, usb_dir_in)) < 0) {
		fprintf(stderr, "umass: pipe open failed \n");
		free(dev);
		return -EINVAL;
	}

	if ((dev->pipeOut = usb_open(insertion, usb_transfer_bulk, usb_dir_out)) < 0) {
		fprintf(stderr, "umass: pipe open failed\n");
		free(dev);
		return -EINVAL;
	}
	dev->tag = 0;

	if (umass_check(dev)) {
		fprintf(stderr, "umass: umass_check failed\n");
		free(dev);
		return -EINVAL;
	}

	oid.port = umass_common.msgport;
	oid.id = dev->fileId;
	if (create_dev(&oid, dev->path) != 0) {
		free(dev);
		fprintf(stderr, "usb: Can't create dev!\n");
		return -EINVAL;
	}

	LIST_ADD(&umass_common.devices, dev);
	fprintf(stderr, "umass: New USB Mass Storage device: %s sectors: %d\n", dev->path, dev->partSize);

	return 0;
}


static int umass_handleDeletion(usb_deletion_t *del)
{
	umass_dev_t *next, *dev = umass_common.devices;
	int cont = 1;

	if (dev == NULL)
		return 0;

	do {
		next = dev->next;
		if (dev->instance.bus == del->bus && dev->instance.dev == del->dev &&
				dev->instance.interface == del->interface) {
			if (dev == next)
				cont = 0;
			resourceDestroy(dev->lock);
			remove(dev->path);
			LIST_REMOVE(&umass_common.devices, dev);
			fprintf(stderr, "umass: Device removed: %s\n", dev->path);
			free(dev);
			if (!cont)
				break;
		}
		dev = next;
	} while (dev != umass_common.devices);

	return 0;
}


int main(int argc, char *argv[])
{
	int ret, i;
	msg_t msg;
	usb_msg_t *umsg = (usb_msg_t *)msg.i.raw;

	/* Port for communication with the USB stack */
	if (portCreate(&umass_common.drvport) != 0) {
		fprintf(stderr, "umass: Can't create port!\n");
		return 1;
	}

	/* Port for communication with driver clients */
	if (portCreate(&umass_common.msgport) != 0) {
		fprintf(stderr, "umass: Can't create port!\n");
		return 1;
	}

	if ((usb_connect(filters, sizeof(filters) / sizeof(filters[0]), umass_common.drvport)) < 0) {
		fprintf(stderr, "umass: Fail to connect to usb host!\n");
		return 1;
	}

	if (mutexCreate(&umass_common.lock)) {
		fprintf(stderr, "umass: Can't create mutex!\n");
		return 1;
	}

	umass_common.lastId = 1;

	for (i = 0; i < UMASS_N_MSG_THREADS; i++) {
		if ((ret = beginthread(umass_msgthr, 4, umass_common.stack[i], sizeof(umass_common.stack[i]), NULL)) != 0) {
			fprintf(stderr, "umass: fail to beginthread ret: %d\n", ret);
			return 1;
		}
	}

	for (;;) {
		ret = usb_eventsWait(umass_common.drvport, &msg);
		if (ret != 0)
			return 1;
		mutexLock(umass_common.lock);
		switch (umsg->type) {
			case usb_msg_insertion:
				if (umass_handleInsertion(&umsg->insertion) != 0)
					fprintf(stderr, "umass: Failed to initialize device!\n");
				break;
			case usb_msg_deletion:
				umass_handleDeletion(&umsg->deletion);
				break;
			default:
				fprintf(stderr, "umass: Error when receiving event from host\n");
				break;
		}
		mutexUnlock(umass_common.lock);
	}

	return 0;
}
