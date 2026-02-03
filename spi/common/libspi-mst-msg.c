/*
 * Phoenix-RTOS
 *
 * SPI message interface
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <spi-mst.h>
#include <spi-mst-msg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>


int spimsg_pack(const spimsg_transfer_t *ops, size_t opsCnt, void *ibuf, size_t ibufLen, size_t *packedLen)
{
	const size_t txDataOffset = opsCnt * sizeof(spi_transferDesc_t);
	if (ibufLen < txDataOffset) {
		return -ENOMEM;
	}

	size_t offset = 0;

	/* Pack descriptors */
	for (size_t i = 0; i < opsCnt; ++i) {
		spi_transferDesc_t desc;
		desc.type = ops[i].type;
		if ((desc.type != spi_transfer_tx) && (desc.type != spi_transfer_rx) && (desc.type != spi_transfer_xfer)) {
			return -EINVAL;
		}
		desc.len = ops[i].len;
		memcpy((uint8_t *)ibuf + offset, &desc, sizeof(spi_transferDesc_t));
		offset += sizeof(spi_transferDesc_t);
	}

	/* Pack TX data */
	offset = txDataOffset;
	for (size_t i = 0; i < opsCnt; ++i) {
		if ((ops[i].type == spi_transfer_tx) || (ops[i].type == spi_transfer_xfer)) {
			if ((offset + ops[i].len) > ibufLen) {
				return -ENOMEM;
			}
			memcpy((uint8_t *)ibuf + offset, ops[i].txBuf, ops[i].len);
			offset += ops[i].len;
		}
	}

	*packedLen = offset;

	return 0;
}


int spimsg_unpack(spimsg_transfer_t *ops, size_t opsCnt, const void *ibuf, size_t ibufLen, const void *obuf, size_t obufLen)
{
	if (ibufLen < (opsCnt * sizeof(spi_transferDesc_t))) {
		return -ENOMEM;
	}

	const spi_transferDesc_t *descs = (const spi_transferDesc_t *)ibuf;
	size_t offset = 0;

	/* Unpack descriptors and assign RX buffer pointers */
	for (size_t i = 0; i < opsCnt; ++i) {
		if ((descs[i].type == spi_transfer_rx) || (descs[i].type == spi_transfer_xfer)) {
			size_t rxLen = descs[i].len;
			if ((offset + rxLen) > obufLen) {
				return -ENOMEM;
			}
			ops[i].rxBuf = (uint8_t *)obuf + offset;
			offset += rxLen;
		}
	}

	return 0;
}


int spimsg_transaction(const oid_t *oid, const spimsg_config_t *cfg, const void *ibuf, size_t ibufLen, void *obuf, size_t obufLen)
{
	msg_t msg;
	spi_devctl_t *idevctl = (spi_devctl_t *)msg.i.raw;
	int err, needscopy = 0;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;
	msg.oid = *oid;

	idevctl->cfg = *cfg;

	/* Pack msg to the raw fields */
	if (ibufLen <= (sizeof(msg.i.raw) - sizeof(spi_devctl_t))) {
		memcpy(idevctl->payload, ibuf, ibufLen);
	}
	else {
		msg.i.data = ibuf;
		msg.i.size = ibufLen;
	}

	if (obufLen <= sizeof(msg.o.raw)) {
		needscopy = 1;
	}
	else {
		msg.o.data = obuf;
		msg.o.size = obufLen;
	}

	err = msgSend(msg.oid.port, &msg);
	if (err < 0) {
		return err;
	}

	if (needscopy != 0) {
		memcpy(obuf, msg.o.raw, obufLen);
	}

	return msg.o.err;
}


int spimsg_close(const oid_t *oid)
{
	(void)oid;

	return 0;
}


int spimsg_open(unsigned int dev, unsigned int ss, oid_t *oid)
{
	unsigned int ntries = 10;
	char devName[16];
	int err;

	if (ss == SPI_SS_EXTERNAL) {
		err = snprintf(devName, sizeof(devName), "/dev/spi%u", dev);
	}
	else {
		err = snprintf(devName, sizeof(devName), "/dev/spi%u.%u", dev, ss);
	}

	if (err >= sizeof(devName)) {
		return -EINVAL;
	}

	while (lookup(devName, NULL, oid) < 0) {
		if (--ntries == 0) {
			return -ENOENT;
		}
		usleep(10 * 1000);
	}

	return 0;
}
