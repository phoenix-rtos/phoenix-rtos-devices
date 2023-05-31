/*
 * Phoenix-RTOS
 *
 * i.MX 6ull NOR flash device driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <sys/threads.h>
#include <imx6ull-qspi.h>

#include "imx6ull-nor.h"
#include "imx6ull-flashnor-drv.h"


#define MIN(a, b) ({ \
	__typeof__(a) _a = (a); \
	__typeof__(b) _b = (b); \
	_a > _b ? _b : _a; \
})


static struct {
	handle_t lock; /* Common lock as QSPI buffers are shared among devices. */
	struct nor_device devs[2];
	flashnor_devInfo_t devInfo[2];
	int qspi_init;
} flashnor_common;


static struct nor_device *flashnor_qspiNorDev(int ndev)
{
	return &flashnor_common.devs[ndev - 1];
}


static ssize_t flashnor_qspiRead(int ndev, off_t offs, void *buff, size_t bufflen)
{
	size_t len, size;
	ssize_t res;
	struct nor_device *dev = flashnor_qspiNorDev(ndev);

	(void)mutexLock(flashnor_common.lock);
	for (len = 0; len < bufflen; len += res) {
		/* Limit read size to QSPI_RXBUFSIZE to prevent buffer overflow in case of time quant end. */
		/* TODO rewrite QSPI handler to use DMA. */
		size = MIN(bufflen - len, QSPI_RXBUFSIZE);
		res = nor_readData(&dev->qspi, dev->port, offs + len, ((uint8_t *)buff) + len, size, dev->timeout);
		if (res < 0) {
			(void)mutexUnlock(flashnor_common.lock);
			return res;
		}
	}
	(void)mutexUnlock(flashnor_common.lock);

	return bufflen;
}


static ssize_t flashnor_qspiWrite(int ndev, off_t offs, const void *buff, size_t bufflen)
{
	size_t size, len;
	int err;
	struct nor_device *dev = flashnor_qspiNorDev(ndev);

	(void)mutexLock(flashnor_common.lock);
	for (len = 0; len < bufflen; len += size) {
		/* Limit write size to the page aligned offsets (don't wrap around at the page boundary) */
		size = MIN(bufflen - len, dev->nor->pageSz - ((offs + len) % dev->nor->pageSz));

		err = nor_pageProgram(&dev->qspi, dev->port, offs + len, ((const uint8_t *)buff) + len, size, dev->timeout);
		if (err < 0) {
			(void)mutexUnlock(flashnor_common.lock);
			return err;
		}
	}
	(void)mutexUnlock(flashnor_common.lock);

	return bufflen;
}


static int flashnor_qspiErase(int ndev, off_t offs, size_t size)
{
	int err;
	size_t len;
	struct nor_device *dev = flashnor_qspiNorDev(ndev);

	(void)mutexLock(flashnor_common.lock);
	for (len = 0; len < size; len += dev->nor->blockSz) {
		err = nor_eraseBlock(&dev->qspi, dev->port, offs + len, dev->timeout);
		if (err < 0) {
			(void)mutexUnlock(flashnor_common.lock);
			return err;
		}
	}
	(void)mutexUnlock(flashnor_common.lock);

	return EOK;
}


static const flashnor_ops_t flashnor_qspiOps = {
	.erase = flashnor_qspiErase,
	.write = flashnor_qspiWrite,
	.read = flashnor_qspiRead,
};


int flashnor_qspiInit(int ndev, flashnor_info_t *info)
{
	int err, port;
	const char *vendor;
	struct nor_device *dev;

	if ((ndev != 1) && (ndev != 2)) {
		return -EINVAL;
	}

	port = (ndev == 1) ? 0 : 2;

	dev = &flashnor_common.devs[ndev - 1];

	if (flashnor_common.qspi_init == 0) {
		err = mutexCreate(&flashnor_common.lock);
		if (err < 0) {
			return err;
		}
		err = qspi_init(&dev->qspi);
		if (err < 0) {
			(void)resourceDestroy(flashnor_common.lock);
			return err;
		}
		flashnor_common.qspi_init = 1;
	}

	nor_deviceInit(dev, port, 0, NOR_DEFAULT_TIMEOUT);

	err = nor_probe(&dev->qspi, port, &dev->nor, &vendor);
	if (err < 0) {
		return err;
	}

	dev->active = 1;

	flashnor_common.devInfo[ndev - 1] = (flashnor_devInfo_t) {
		.name = dev->nor->name,
		.size = dev->nor->totalSz,
		.writeBuffsz = dev->nor->pageSz,
		.erasesz = dev->nor->blockSz, /* Erases are in block as this driver is meant for JFFS2 and big blocks are recommended for this FS. */
	};

	info->devInfo = &flashnor_common.devInfo[ndev - 1];
	info->ndev = ndev;
	info->ops = &flashnor_qspiOps;

	(void)printf("imx6ull-flashnor: Configured %s %s %zuMB nor flash\n", vendor, dev->nor->name, dev->nor->totalSz >> 20);

	return EOK;
}
