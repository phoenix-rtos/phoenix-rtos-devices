/*
 * Phoenix-RTOS
 *
 * i.MX RT SPI driver's tests
 *
 * Copyright 2019 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/msg.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>

#include <imxrt-multi.h>


#define MAX_BUFFER_SZ 0x204


struct {
	uint8_t tx[MAX_BUFFER_SZ];
	uint8_t rx[MAX_BUFFER_SZ];
} test_common;


static oid_t test_getOid(void)
{
	oid_t dir;
	while (lookup("/dev/spi1", NULL, &dir) < 0)
		usleep(9000);

	return dir;
}


static int test_spiConfig(oid_t dir)
{
	msg_t msg;
	multi_i_t *idevctl = NULL;
	multi_o_t *odevctl = NULL;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	idevctl = (multi_i_t *)msg.i.raw;
	idevctl->id = dir.id;
	idevctl->spi.type = spi_config;
	idevctl->spi.config.cs = 0;
	idevctl->spi.config.endian = spi_msb;
	idevctl->spi.config.mode = spi_mode_0;
	idevctl->spi.config.prescaler = 2;
	idevctl->spi.config.sckDiv = 64;

	odevctl = (multi_o_t *)msg.o.raw;

	if (msgSend(dir.port, &msg) < 0)
		return -1;

	if (odevctl->err < 0)
		return -1;

	return EOK;
}


static int test_spiTransmit(oid_t dir, uint8_t *tx, uint8_t *rx, int sz)
{
	msg_t msg;
	multi_i_t *idevctl = NULL;
	multi_o_t *odevctl = NULL;

	msg.type = mtDevCtl;
	msg.i.data = tx;
	msg.i.size = sz;
	msg.o.data = rx;
	msg.o.size = 0;

	idevctl = (multi_i_t *)msg.i.raw;
	idevctl->id = dir.id;
	idevctl->spi.type = spi_transaction;
	idevctl->spi.transaction.frameSize = sz;
	idevctl->spi.transaction.cs = 0;

	odevctl = (multi_o_t *)msg.o.raw;

	if (msgSend(dir.port, &msg) < 0)
		return -1;

	if (odevctl->err < 0)
		return -1;

	return odevctl->err;
}


static void test_setData(uint16_t size)
{
	int i;
	memset(test_common.rx, 0, MAX_BUFFER_SZ);
	memset(test_common.tx, 0, MAX_BUFFER_SZ);

	for (i = 0; i < size; i += 4) {
		test_common.tx[i] = 0xaa;
		test_common.tx[i + 1] = 0xbb;
		test_common.tx[i + 2] = 0xcc;
		test_common.tx[i + 3] = 0xdd;
	}
}


int test_spi_transfer_max_frame(void)
{
	oid_t dir;
	int rcvSize;
	const uint16_t buffSz = 0x200;  /* maximum framze size */

	dir = test_getOid();
	test_spiConfig(dir);

	test_setData(buffSz);

	/* Transmit data in a loop - back (MOSI --> MISO) */
	rcvSize = test_spiTransmit(dir, test_common.tx, test_common.rx, buffSz);

	if ((memcmp(test_common.tx, test_common.rx, buffSz) == 0) && (rcvSize == buffSz))
		return EOK;
	else
		return -EINVAL;
}


int test_spi_transfer_overfilled_frame(void)
{
	oid_t dir;
	int rcvSize;
	const uint16_t buffSz = 0x200 + 1;

	dir = test_getOid();
	test_spiConfig(dir);

	/* Transmit data in a loop - back (MOSI --> MISO) */
	rcvSize = test_spiTransmit(dir, test_common.tx, test_common.rx, buffSz);

	if (rcvSize < 0)
		return EOK;
	else
		return -EINVAL;
}


int test_spi_transfer_middle_data_sz(void)
{
	oid_t dir;
	int rcvSize;
	const uint16_t buffSz = 16 * 4 * 2;

	dir = test_getOid();
	test_spiConfig(dir);

	test_setData(buffSz);

	/* Transmit data in a loop - back (MOSI --> MISO) */
	rcvSize = test_spiTransmit(dir, test_common.tx, test_common.rx, buffSz);

	if ((memcmp(test_common.tx, test_common.rx, buffSz) == 0) && (rcvSize == buffSz))
		return EOK;
	else
		return -EINVAL;
}


int test_spi_transfer_partially_filled_fifo(void)
{
	oid_t dir;
	int rcvSize;
	const uint16_t buffSz = 14 * 4;

	dir = test_getOid();
	test_spiConfig(dir);

	test_setData(buffSz);

	/* Transmit data in a loop - back (MOSI --> MISO) */
	rcvSize = test_spiTransmit(dir, test_common.tx, test_common.rx, buffSz);

	if ((memcmp(test_common.tx, test_common.rx, buffSz) == 0) && (rcvSize == buffSz))
		return EOK;
	else
		return -EINVAL;
}


int test_spi_multiple_transmission(void)
{
	oid_t dir;
	int rcvSize;
	u_int16_t size;

	dir = test_getOid();
	test_spiConfig(dir);

	/* Transmit data in a loop - back (MOSI --> MISO) */

	/* First transmission */
	size = 0x100;
	test_setData(size);
	rcvSize = test_spiTransmit(dir, test_common.tx, test_common.rx, size);

	if (!((memcmp(test_common.tx, test_common.rx, size) == 0) && (rcvSize == size)))
		return -EINVAL;

	/* Second transmission */
	size = 0x10;
	test_setData(size);
	rcvSize = test_spiTransmit(dir, test_common.tx, test_common.rx, size);

	if (!((memcmp(test_common.tx, test_common.rx, size) == 0) && (rcvSize == size)))
		return -EINVAL;

	/* Third transmission */
	size = 0x150;
	test_setData(size);
	size = 0x150 - 3;
	rcvSize = test_spiTransmit(dir, test_common.tx, test_common.rx, size);

	if (!((memcmp(test_common.tx, test_common.rx, size) == 0) && (rcvSize == size)))
		return -EINVAL;

	return EOK;
}
