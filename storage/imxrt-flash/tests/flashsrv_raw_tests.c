/*
 * Phoenix-RTOS
 *
 * i.MX RT flash server tests for chip and raw partitions interface
 *
 * Copyright 2020 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>

#include "tests.h"
#include <imxrt-flashsrv.h>


#define LOG_ERROR(str, ...) do { fprintf(stderr, __FILE__  ":%d error: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)



/* Internal functions - communication with imxrt-flash via msg */

static int write2Flash(oid_t oid, uint32_t paddr, void *data, int size)
{
	msg_t msg;
	int err = EOK;

	msg.type = mtWrite;
	msg.i.io.oid = oid;
	msg.i.io.offs = paddr;
	msg.i.data = data;
	msg.i.size = size;
	msg.o.data = NULL;
	msg.o.size = 0;

	if ((err = msgSend(oid.port, &msg)) != 0 )
		LOG_ERROR("Cannot send msg.");

	if ((err = msg.o.io.err) < size)
		LOG_ERROR("Cannot write data to flash, err: %d.");

	return err;
}


static int readFromFlash(oid_t oid, uint32_t paddr, void *data, int size)
{
	msg_t msg;
	int err = EOK;

	msg.type = mtRead;
	msg.i.io.oid = oid;
	msg.i.io.offs = paddr;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = data;
	msg.o.size = size;

	if ((err = msgSend(oid.port, &msg)) != 0 )
		LOG_ERROR("Cannot send msg.");

	if ((err = msg.o.io.err) < size)
		LOG_ERROR("Cannot read data from flash, err: %d.");

	return err;
}


static int syncFlash(oid_t oid)
{
	msg_t msg;
	int err = EOK;
	flash_i_devctl_t *idevctl = NULL;
	flash_o_devctl_t *odevctl = NULL;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	idevctl = (flash_i_devctl_t *)msg.i.raw;
	idevctl->type = flashsrv_devctl_sync;
	idevctl->oid = oid;

	odevctl = (flash_o_devctl_t *)msg.o.raw;

	if ((err = msgSend(oid.port, &msg)) != 0 )
		LOG_ERROR("Cannot send msg.");

	if ((err = odevctl->err) < 0)
		LOG_ERROR("Cannot sync flash, err: (%s).", strerror(err));

	return err;
}


static int eraseSector(oid_t oid, uint32_t addr)
{
	msg_t msg;
	int err = EOK;
	flash_i_devctl_t *idevctl = NULL;
	flash_o_devctl_t *odevctl = NULL;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	idevctl = (flash_i_devctl_t *)msg.i.raw;
	idevctl->type = flashsrv_devctl_eraseSector;
	idevctl->oid = oid;
	idevctl->addr = addr;

	odevctl = (flash_o_devctl_t *)msg.o.raw;

	if ((err = msgSend(oid.port, &msg)) != 0 )
		LOG_ERROR("Cannot send msg.");

	if ((err = odevctl->err) < 0)
		LOG_ERROR("Cannot erase sector, err: (%s).", strerror(err));

	return err;
}


static int erasePartition(oid_t oid)
{
	msg_t msg;
	int err = EOK;
	flash_i_devctl_t *idevctl = NULL;
	flash_o_devctl_t *odevctl = NULL;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	idevctl = (flash_i_devctl_t *)msg.i.raw;
	idevctl->type = flashsrv_devctl_erasePartition;
	idevctl->oid = oid;

	odevctl = (flash_o_devctl_t *)msg.o.raw;

	if ((err = msgSend(oid.port, &msg)) != 0 )
		LOG_ERROR("Cannot send msg.");

	if ((err = odevctl->err) < 0)
		LOG_ERROR("Cannot erase partition, err: (%s).", strerror(err));

	return err;
}


static int getProperties(oid_t oid, flash_o_devctl_t *odevctl)
{
	msg_t msg;
	int err = EOK;
	flash_i_devctl_t *i = NULL;
	flash_o_devctl_t *o = NULL;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	i = (flash_i_devctl_t *)msg.i.raw;
	i->type = flashsrv_devctl_properties;
	i->oid = oid;

	o = (flash_o_devctl_t *)msg.o.raw;

	if ((err = msgSend(oid.port, &msg)) != 0 )
		LOG_ERROR("Cannot send msg.");

	if ((err = o->err) < 0)
		LOG_ERROR("Cannot get properties, err: (%s).", strerror(err));

	odevctl->properties.size = o->properties.size;
	odevctl->properties.psize = o->properties.psize;
	odevctl->properties.ssize = o->properties.ssize;

	return err;
}


/* General functions */
int write_pTable(const char *flashName)
{
	oid_t oid;
	const uint16_t buffSize = 0x100;
	char buff[buffSize];

	flash_o_devctl_t odevctl;

	while (lookup(flashName, NULL, &oid) < 0)
		usleep(10000);

	if (getProperties(oid, &odevctl) < 0)
		return -1;

	/* Write partition table */
	serializepTable(buff, &tHeader, parts);

	if (write2Flash(oid, odevctl.properties.size - odevctl.properties.ssize, buff, buffSize) != buffSize)
		return -1;

	if (syncFlash(oid) < 0)
		return -1;

	return EOK;
}


void serializepTable(char *buff, const ptable_header_t *tHeader, const ptable_partition_t *parts)
{
	int offs = 0;

	memcpy(buff + offs, tHeader, sizeof(ptable_header_t));
	offs += sizeof(ptable_header_t);

	memcpy(buff + offs, parts, tHeader->pCnt * sizeof(ptable_partition_t));
	offs += tHeader->pCnt * sizeof(ptable_partition_t);

	memcpy(buff + offs, pt_magicBytes, sizeof(pt_magicBytes));
}



/* Tests */

int test_flashsrv_getFlashProperties(void)
{
	oid_t oid;
	int res = EOK;
	flash_o_devctl_t odevctl;

	while (lookup(INTERNAL_FLASH_PATH, NULL, &oid) < 0)
		usleep(10000);

	if (getProperties(oid, &odevctl) < 0)
		return -1;

	/* Verification */
	if ((odevctl.properties.size != 0x400000) && (odevctl.properties.psize != 0x100) && (odevctl.properties.ssize != 0x1000))
		res = -1;

	return res;
}


int test_flashsrv_writeAndReadFlashPage(void)
{
	oid_t oid;
	int res = EOK, i;
	const uint32_t addr = 0x1000;
	const uint16_t size = 0x100;
	const uint8_t checkValue = 0x5a;

	uint8_t buff[size];

	memset(buff, checkValue, size);

	while (lookup(INTERNAL_FLASH_PATH, NULL, &oid) < 0)
		usleep(10000);

	if (write2Flash(oid, addr, buff, size) != size)
		return -1;

	if ((res = syncFlash(oid)) < 0)
		return res;

	memset(buff, 0, size);

	if (readFromFlash(oid, addr, buff, size) != size)
		return -1;

	/* Verification */
	for (i = 0; i < size; ++i) {
		if (buff[i] != checkValue) {
			res = -1;
			break;
		}
	}

	return res;
}


int test_flashsrv_writeAndReadFlash(void)
{
	oid_t oid;
	int res = EOK, i;
	const uint32_t addr = 0x4850;
	const uint16_t size = 0x300;
	const uint8_t checkValue = 0xca;

	uint8_t buff[size];

	memset(buff, checkValue, size);

	while (lookup(INTERNAL_FLASH_PATH, NULL, &oid) < 0)
		usleep(10000);

	if (write2Flash(oid, addr, buff, size) != size)
		return -1;

	if ((res = syncFlash(oid)) < 0)
		return res;

	memset(buff, 0, size);

	if (readFromFlash(oid, addr, buff, size) != size)
		return -1;

	/* Verification */
	for (i = 0; i < size; ++i) {
		if (buff[i] != checkValue) {
			res = -1;
			break;
		}
	}

	return res;
}

int test_flashsrv_eraseFlashSector(void)
{
	oid_t oid;
	int res = EOK, i, j;

	const uint16_t size = 0x100;
	const uint32_t addr = 0x5000;
	const uint8_t checkValue = 0xff;
	const uint16_t sectorSz = 0x1000;

	uint8_t buff[size];

	memset(buff, 0xa5, size);

	while (lookup(INTERNAL_FLASH_PATH, NULL, &oid) < 0)
		usleep(10000);

	for (j = 0; j < (sectorSz / size); ++j) {
		if (write2Flash(oid, addr + j * size, buff, size) != size)
			return -1;
	}

	if (eraseSector(oid, addr) < 0)
		return -1;

	/* Verification */
	memset(buff, 0, size);

	for (j = 0; j < (sectorSz / size); ++j) {
		if (readFromFlash(oid, addr + j * size, buff, size) != size) {
			res = -1;
			break;
		}

		for (i = 0; i < size; ++i) {
			if (buff[i] != checkValue)
				return -1;

			buff[i] = 0x0;
		}
	}

	return res;
}


int test_flashsrv_eraseFlash(void)
{
	oid_t oid;
	int res = EOK, i, j;

	uint32_t addr = 0;
	const uint8_t nb = 10;
	const uint16_t size = 0x100;
	const uint16_t chunk = 0x1000;
	const uint8_t checkValue = 0xff;

	uint8_t buff[size];

	memset(buff, 0xa5, size);

	while (lookup(INTERNAL_FLASH_PATH, NULL, &oid) < 0)
		usleep(10000);

	for (i = 0; i < nb; ++i) {
		if (write2Flash(oid, addr + i * chunk, buff, size) != size)
			return -1;
	}

	if (erasePartition(oid) < 0)
		return -1;

	/* Verification */
	memset(buff, 0, size);

	for (j = 0; j < nb; ++j) {
		if (readFromFlash(oid, addr + i * chunk, buff, size) != size)
			return -1;

		for (i = 0; i < size; ++i) {
			if (buff[i] != checkValue)
				return -1;
		}
	}

	return res;
}


int test_flashsrv_verifyPartitionTable(void)
{
	oid_t oid;
	int res = EOK;
	uint16_t ptSize;
	uint8_t buff[0x100];

	flash_o_devctl_t odevctl;

	while (lookup(INTERNAL_FLASH_PATH, NULL, &oid) < 0)
		usleep(10000);

	if (getProperties(oid, &odevctl) < 0)
		return -1;

	/* Write partition table */
	if (write_pTable(INTERNAL_FLASH_PATH) < 0)
		return -1;

	/* Read partition table */
	ptSize = sizeof(ptable_header_t) + tHeader.pCnt * sizeof(ptable_partition_t) + sizeof(pt_magicBytes);
	if (readFromFlash(oid, odevctl.properties.size - odevctl.properties.ssize, buff, ptSize) != ptSize)
		return -1;

	/* Verification */
	if (memcmp(buff, &tHeader, sizeof(ptable_header_t)) != 0)
		res = -1;

	if (memcmp(buff + sizeof(ptable_header_t), parts, tHeader.pCnt * sizeof(ptable_partition_t)) != 0)
		res = -1;

	if (memcmp(buff + sizeof(ptable_header_t) +  tHeader.pCnt * sizeof(ptable_partition_t), pt_magicBytes, sizeof(pt_magicBytes)) != 0)
		res = -1;

	return res;
}


int test_flashsrv_rawPartProperties(void)
{
	oid_t oid;
	int res = EOK;
	flash_o_devctl_t odevctl;
	const char *path = "/dev/flash1.raw2";

	while (lookup(path, NULL, &oid) < 0)
		usleep(10000);

	if (getProperties(oid, &odevctl) < 0)
		return -1;

	/* Verification */
	if ((odevctl.properties.size != 6 * 0x1000) && (odevctl.properties.psize != 0x100) &&
		(odevctl.properties.ssize != 0x1000) && (odevctl.properties.offs != 0x10000))
		res = -1;

	return res;
}


int test_flashsrv_rawdWriteAndRead(void)
{
	oid_t oid;
	int res = EOK, i;
	const uint32_t addr = 0x500;
	const uint16_t size = 0x100;
	const uint8_t checkValue = 0xca;
	const char *path = "/dev/flash1.raw1";

	uint8_t buff[size];

	memset(buff, checkValue, size);

	while (lookup(path, NULL, &oid) < 0)
		usleep(10000);

	if (write2Flash(oid, addr, buff, size) != size)
		return -1;

	if ((res = syncFlash(oid)) < 0)
		return -1;

	memset(buff, 0, size);

	if (readFromFlash(oid, addr, buff, size) != size)
		return -1;

	/* Verification */
	for (i = 0; i < size; ++i) {
		if (buff[i] != checkValue) {
			res = -1;
			break;
		}
	}

	return res;
}


int test_flashsrv_rawSectorErase(void)
{
	oid_t oid;
	int res = EOK, i;
	const uint32_t addr = 0x350;
	const uint16_t size = 0x100;
	const uint8_t checkValue = 0xff;
	const char *path = "/dev/flash1.raw1";

	uint8_t buff[size];

	memset(buff, 0xab, size);

	while (lookup(path, NULL, &oid) < 0)
		usleep(10000);

	if (write2Flash(oid, addr, buff, size) != size)
		return -1;

	if ((res = syncFlash(oid)) < 0)
		return res;

	if (eraseSector(oid, 0) < 0)
		return -1;

	memset(buff, 0, size);

	if (readFromFlash(oid, addr, buff, size) != size)
		return -1;

	/* Verification */
	for (i = 0; i < size; ++i) {
		if (buff[i] != checkValue) {
			res = -1;
			break;
		}
	}

	return res;
}


int test_flashsrv_rawPartitionErase(void)
{
	oid_t oid;
	int res = EOK, i, j;
	uint8_t sNb;
	uint32_t addr = 0x0;
	flash_o_devctl_t odevctl;
	const uint16_t size = 0x100;
	const uint8_t checkValue = 0xff;
	const char *path = "/dev/flash1.raw2";

	uint8_t buff[size];

	while (lookup(path, NULL, &oid) < 0)
		usleep(10000);

	/* Get information about partition properties */
	if (getProperties(oid, &odevctl) < 0)
		return -1;

	/* Write data to each sector of partition */
	memset(buff, 0xec, size);

	sNb = odevctl.properties.size / odevctl.properties.ssize;
	for (i = 0; i < sNb; ++i) {
		if (write2Flash(oid, addr + i * odevctl.properties.ssize, buff, size) != size)
			return -1;
	}

	if ((res = syncFlash(oid)) < 0)
		return res;

	/* Erase the whole partition */
	if (erasePartition(oid) < 0)
		return -1;

	memset(buff, 0, size);

	/* Verification */
	for (i = 0; i < sNb; ++i) {
		if (readFromFlash(oid, addr + i * odevctl.properties.ssize, buff, size) != size)
			return -1;

		for (j = 0; j < size; ++j) {
			if (buff[j] != checkValue)
				return -1;
			buff[j] = 0;
		}
	}

	return res;
}
