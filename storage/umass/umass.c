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
#include <sys/list.h>
#include <sys/msg.h>
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

#define UMASS_SECTOR_SIZE 512
#define UMASS_MAX_SECTORS 8
#define UMASS_PART_OFFSET 32

#define UMASS_CACHE_SIZE 8

#define UMASS_WRITE 0
#define UMASS_READ  0x80

#define STATE_READY    0
#define STATE_REMOVED -1
#define STATE_CHANGED -2
#define STATE_ERROR   -4
#define STATE_HALT    -5

#define CBW_SIG 0x43425355
#define CSW_SIG 0x53425355

#define MAX_SECS          32
#define MAX_USB_RETRIES   2
#define MAX_SCSI_RETRIES  4

typedef struct umass_cbw {
	uint32_t sig;
	uint32_t tag;
	uint32_t dlen;
	uint8_t  flags;
	uint8_t  lun;
	uint8_t  clen;
	uint8_t  cmd[16];
} __attribute__((packed)) umass_cbw_t;


typedef struct umass_csw {
	uint32_t sig;
	uint32_t tag;
	uint32_t dr;
	uint8_t  status;
} __attribute__((packed)) umass_csw_t;


typedef struct scsi_cdb10 {
	uint8_t opcode;
	uint8_t action : 5;
	uint8_t misc0 : 3;
	uint32_t lba;
	uint8_t misc1;
	uint16_t length;
	uint8_t control;
} __attribute__((packed)) scsi_cdb10_t;


typedef struct umass_dev {
	char buffer[UMASS_SECTOR_SIZE * 2];
	char stack[1024] __attribute__ ((aligned(8)));
	struct umass_dev *prev, *next;
	usb_insertion_t instance;
	int pipeCtrl;
	int pipeIn;
	int pipeOut;
	int id;
	int tag;
	unsigned port;
	uint32_t partOffset;
	uint32_t partSize;
} umass_dev_t;


static struct {
	umass_dev_t *devices;
	unsigned drvport;
} umass_common;


static void umass_cswDump(umass_csw_t *csw)
{
	fprintf(stderr, "sig:%x tag:%x dr:%x status:%x\n", csw->sig, csw->tag, csw->dr, csw->status);
}


static int umass_transmit(umass_dev_t *dev, void *cmd, size_t clen, char *data, size_t dlen, int dir)
{
	umass_cbw_t cbw = { 0 };
	umass_csw_t csw = { 0 };

	if (clen > 16)
		return -1;

	cbw.sig = CBW_SIG;
	cbw.tag = dev->tag++;
	cbw.dlen = dlen;
	cbw.flags = (dir == usb_dir_out) ? UMASS_WRITE : UMASS_READ;
	cbw.lun = 0;
	cbw.clen = clen;
	memcpy(cbw.cmd, cmd, clen);

	if (usb_transferBulk(dev->pipeOut, &cbw, sizeof(cbw), usb_dir_out) != 0)
		return -1;

	if (dlen > 0) {
		if (usb_transferBulk((dir == usb_dir_in) ? dev->pipeIn : dev->pipeOut, data, dlen, dir) != 0)
			return -1;
	}

	if (usb_transferBulk(dev->pipeIn, &csw, sizeof(csw), usb_dir_in) != 0)
		return -1;

	if (csw.sig != CSW_SIG || csw.tag != cbw.tag || csw.status != 0) {
		fprintf(stderr, "umass: transmit fail\n");
		return -1;
	}

	return dlen;
}


static int umass_check(umass_dev_t *dev)
{
	scsi_cdb10_t readcmd = {
		.opcode = 0x28,
		.length = htons(0x1)
	};
	char testcmd[6] = { 0 };
	mbr_t *mbr;

	if (umass_transmit(dev, testcmd, sizeof(testcmd), NULL, 0, usb_dir_in) < 0)
		return -1;

	/* Read MBR */
	if (umass_transmit(dev, &readcmd, sizeof(readcmd), dev->buffer, UMASS_SECTOR_SIZE, usb_dir_in) < 0)
		return -1;

	mbr = (mbr_t *)dev->buffer;
	if (mbr->magic != MBR_MAGIC)
		return -1;

	/* Read only the first partition */
	dev->partOffset = mbr->pent[0].start;
	dev->partSize = mbr->pent[0].sectors;

	return 0;
}


static int umass_reset(umass_dev_t *dev)
{
	usb_setup_packet_t setup = (usb_setup_packet_t) {
		.bmRequestType = REQUEST_DIR_HOST2DEV | REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE,
		.bRequest = 0xff,
		.wValue = 0,
		.wIndex = dev->instance.interface,
		.wLength = 0,
	};

	return usb_transferControl(dev->pipeCtrl, &setup, NULL, 0, usb_dir_out);
}


static int umass_read(umass_dev_t *dev, offs_t offs, char *buf, size_t len)
{
	scsi_cdb10_t readcmd = { .opcode = 0x28 };

	if (offs + len > dev->partSize * UMASS_SECTOR_SIZE)
		return -EINVAL;

	if ((offs % UMASS_SECTOR_SIZE) || (len % UMASS_SECTOR_SIZE))
		return -EINVAL;

	readcmd.lba = htonl(offs / UMASS_SECTOR_SIZE + dev->partOffset);
	readcmd.length = htons((uint16_t)(len / UMASS_SECTOR_SIZE));

	return umass_transmit(dev, &readcmd, sizeof(readcmd), buf, len, usb_dir_in);
}


static int umass_write(umass_dev_t *dev, offs_t offs, char *buf, size_t len)
{
	scsi_cdb10_t writecmd = { .opcode = 0x2a };

	if (offs + len > dev->partSize * UMASS_SECTOR_SIZE)
		return -EINVAL;

	if ((offs % UMASS_SECTOR_SIZE) || (len % UMASS_SECTOR_SIZE))
		return -EINVAL;

	writecmd.lba = htonl(offs / UMASS_SECTOR_SIZE + dev->partOffset);
	writecmd.length = htons((uint16_t)(len / UMASS_SECTOR_SIZE));

	return umass_transmit(dev, &writecmd, sizeof(writecmd), buf, len, usb_dir_out);
}


static int umass_getattr(umass_dev_t *dev, int type, int *attr)
{
	switch (type) {
	case atSize:
		*attr = dev->partSize * UMASS_SECTOR_SIZE;
		break;

	default:
		*attr = -EINVAL;
	}

	return EOK;
}


static void umass_poolthr(void *arg)
{
	umass_dev_t *dev = (umass_dev_t *)arg;
	unsigned long rid;
	msg_t msg;

	for (;;) {
		if (msgRecv(dev->port, &msg, &rid) < 0)
			continue;

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
			umass_getattr(dev, msg.i.attr.type, &msg.o.attr.val);
			break;

		default:
			msg.o.io.err = -EINVAL;
		}

		msgRespond(dev->port, &msg, rid);
	}

	endthread();
}

static int umass_handleInsertion(usb_insertion_t *insertion)
{
	char path[32];
	umass_dev_t *dev;
	oid_t oid;

	dev = (umass_dev_t *)malloc(sizeof(umass_dev_t));
	dev->instance = *insertion;

	if ((dev->pipeCtrl = usb_open(insertion, usb_transfer_control, usb_dir_bi)) < 0) {
		free(dev);
		return -EINVAL;
	}

	if (usb_setConfiguration(dev->pipeCtrl, 1) != 0) {
		free(dev);
		return -EINVAL;
	}

	if ((dev->pipeIn = usb_open(insertion, usb_transfer_bulk, usb_dir_in)) < 0) {
		free(dev);
		return -EINVAL;
	}

	if ((dev->pipeOut = usb_open(insertion, usb_transfer_bulk, usb_dir_out)) < 0) {
		free(dev);
		return -EINVAL;
	}
	dev->tag = 0;

	if (umass_check(dev)) {
		free(dev);
		return -EINVAL;
	}

	//fprintf(stderr, "umass: Check success, trying to create port\n");

	if (portCreate(&dev->port) != 0) {
		fprintf(stderr, "umass: Can't create port!\n");
		return -EINVAL;
	}

	/* Get next device number */
	if (umass_common.devices == NULL)
		dev->id = 0;
	else
		dev->id = umass_common.devices->prev->id + 1;

	snprintf(path, sizeof(path), "/dev/umass%d", dev->id);
	oid.port = dev->port;
	oid.id = dev->id;
	if (create_dev(&oid, path) != 0) {
		fprintf(stderr, "usb: Can't create dev!\n");
		return -EINVAL;
	}

	LIST_ADD(&umass_common.devices, dev);

	return beginthread(umass_poolthr, 4, dev->stack, sizeof(dev->stack), dev);
}


int main(int argc, char *argv[])
{
	oid_t oid;
	int ret;
	msg_t msg;
	usb_msg_t *umsg = (usb_msg_t *)msg.i.raw;
	usb_device_id_t id = { USB_CONNECT_WILDCARD,
	                       USB_CONNECT_WILDCARD,
	                       USB_CLASS_MASS_STORAGE,
	                       USB_CONNECT_WILDCARD,
	                       USB_CONNECT_WILDCARD };

	if (portCreate(&umass_common.drvport) != 0) {
		fprintf(stderr, "umass: Can't create port!\n");
		return -EINVAL;
	}

	if ((usb_connect(&id, umass_common.drvport)) < 0) {
		fprintf(stderr, "umass: Fail to connect to usb host!\n");
		return -EINVAL;
	}

	for (;;) {
		ret = usb_eventsWait(umass_common.drvport, &msg);
		if (ret != 0)
			continue;
		switch (umsg->type) {
			case usb_msg_insertion:
				umass_handleInsertion(&umsg->insertion);
				break;
			case usb_msg_deletion:
				break;
			default:
				fprintf(stderr, "umass: Error when receiving event from host\n");
				break;
		}
	}

	return 0;
}