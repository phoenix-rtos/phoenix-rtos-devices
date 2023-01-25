/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL NOR flash driver test program
 *
 * Copyright 2021, 2023 Phoenix Systems
 * Author: Jakub Sarzynski, Hubert Badocha
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <meterfs.h>

#include "storage.h"
#include "flashnor-ecspi.h"
#include "flashnor-qspi.h"
#include <qspi.h>


static int (*flashnor_eraseSector)(unsigned int addr);
static int (*flashnor_read)(unsigned int addr, void *buff, size_t bufflen);
static int (*flashnor_write)(unsigned int addr, const void *buff, size_t bufflen);
static int (*flashnor_init)(unsigned int ndev, storage_t *dev);

static unsigned valcmp(uint8_t *buf, size_t len, uint8_t val)
{
	unsigned errors = 0;
	size_t i;

	for (i = 0; i < len; i++) {
		if (buf[i] != val)
			errors++;
	}

	return errors;
}


static int test_read_write(size_t flashsz, size_t sectorsz)
{
	int res = 0;
	unsigned errors = 0, addr;
	uint8_t *pattern, *buf;

	printf("%s: ", __func__);

	pattern = malloc(sectorsz);
	if (pattern == NULL) {
		printf("fail: malloc\n");
		return -1;
	}

	buf = malloc(sectorsz);
	if (buf == NULL) {
		printf("fail: malloc\n");
		free(pattern);
		return -1;
	}

	memset(pattern, 0x55, sectorsz);

	/* Test only every sixteenth block */
	for (addr = 0; addr < flashsz; addr += sectorsz * 16) {
		/* As meterfs is fixed right now in the driver implementation
		   we have to erase sector before write, because of metadata written by meterfs */
		if (flashnor_eraseSector(addr) < 0) {
			printf("fail: erase sector at addr %u\n", addr);
			res = -1;
			break;
		}

		if (flashnor_write(addr, pattern, sectorsz) < 0) {
			printf("fail: write sector at addr %u\n", addr);
			res = -1;
			break;
		}

		if (flashnor_read(addr, buf, sectorsz) < 0) {
			printf("fail: read sector at addr %u\n", addr);
			res = -1;
			break;
		}

		errors += valcmp(buf, sectorsz, pattern[0]);
	}

	if (res == 0) {
		if (errors) {
			printf("fail: read %u errors\n", errors);
			res = -1;
		}
		else {
			printf("ok\n");
		}
	}

	free(pattern);
	free(buf);

	return res;
}


static int test_erase(size_t flashsz, size_t sectorsz)
{
	int res = 0;
	unsigned errors = 0, addr;
	uint8_t *buf;

	printf("%s: ", __func__);

	buf = malloc(sectorsz);
	if (buf == NULL) {
		printf("fail: malloc\n");
		return -1;
	}

	/* Clean sectors written by test_write_read */
	for (addr = 0; addr < flashsz; addr += sectorsz * 16) {
		if (flashnor_eraseSector(addr) < 0) {
			printf("fail: erase sector at addr %u\n", addr);
			res = -1;
			break;
		}

		if (flashnor_read(addr, buf, sectorsz) < 0) {
			printf("fail: erase sector read at addr %u\n", addr);
			res = -1;
			break;
		}

		errors += valcmp(buf, sectorsz, 0xff);
	}

	if (res == 0) {
		if (errors) {
			printf("fail: read %u errors\n", errors);
			res = -1;
		}
		else {
			printf("ok\n");
		}
	}

	free(buf);
	return res;
}


static int run_tests(size_t flashsz, size_t sectorsz)
{
	if (test_read_write(flashsz, sectorsz) < 0)
		return -1;

	if (test_erase(flashsz, sectorsz) < 0)
		return -1;

	return 0;
}

static void flashnor_test_help(const char *prog)
{
	printf("Usage: %s [options]\n", prog);
	printf("\t-e <n> | --ecspi <n>   - initialize ECSPI NOR flash device\n");
	printf("\t\tn:      ECSPI instance number (by default ecspi no. 3 is used)\n");
	printf("\t-q <n> | --qspi <n>   - initialize QSPI NOR flash device\n");
	printf("\t\tn:      QSPI instance number (by default qspi no. 3 is used)\n");
}

#define READ_SIZE 32
static int qspi_test(qspi_dev_t dev)
{
	_qspi_init(dev);
	qspi_setTCSH(0);
	qspi_setTCSS(0);

	// Get device data.
	{
		lut_seq_t seq = { .instrs = {
							  LUT_INSTR(lut_cmd, 0, 0x9f),
							  LUT_INSTR(lut_read, 0, 3),
							  0,
						  } };
		qspi_setLutSeq(&seq, 0);
		char data[3];
		memset(data, 0xFF, 3);

		_qspi_read(dev, 0, 0, data, 3);
		printf("VENDOR\n");
		for (int i = 0; i < 3; i++) {
			printf("%#x ", data[i]);
		}
		printf("\n");
	}
	printf("\n-------------------\n\n");

	// Enter 4 I/O Mode.
	{
		lut_seq_t seq = { .instrs = {
							  LUT_INSTR(lut_cmd, 0, 0x35),
							  0,
						  } };
		qspi_setLutSeq(&seq, 1);

		_qspi_read(dev, 1, 0, NULL, 0);
		printf("ENTERED 4 I/O\n");
	}
	printf("\n-------------------\n\n");

	// Write enable.
	{
		lut_seq_t seq = { .instrs = {
							  LUT_INSTR(lut_cmd, 2, 0x06),  // Write enable
							  0,
						  } };
		qspi_setLutSeq(&seq, 9);

		_qspi_read(dev, 9, 0, NULL, 0);
		printf("ENABLED WRITE\n");
	}
	printf("\n-------------------\n\n");
	// ERASE
	{
		lut_seq_t seq = { .instrs = {
							  LUT_INSTR(lut_cmd, 2, 0x21),
							  LUT_INSTR(lut_addr, 2, 32),
							  0,
						  } };
		qspi_setLutSeq(&seq, 4);

		_qspi_read(dev, 4, 0, NULL, 0);
		printf("ERASED DATA\n");
	}
	printf("\n-------------------\n\n");
	{
		lut_seq_t seq = { .instrs = {
							  LUT_INSTR(lut_cmd, 2, 0x05),
							  LUT_INSTR(lut_read, 2, 1),
							  0,
						  } };
		qspi_setLutSeq(&seq, 10);
		unsigned char status = 1;
		while (status & 1) {
			_qspi_read(dev, 10, 0, &status, 1);
			printf("Status register: %#x\n", status);
		}
	}
	printf("\n-------------------\n\n");
	// WRITE ENABLE
	{
		_qspi_read(dev, 9, 0, NULL, 0);
		printf("ENABLED WRITE\n");
	}
	printf("\n-------------------\n\n");

	// WRITE
	{
		lut_seq_t seq = { .instrs = {
							  LUT_INSTR(lut_cmd, 2, 0x34),
							  LUT_INSTR(lut_addr, 2, 32),
							  LUT_INSTR(lut_write, 2, 0),
							  0,
						  } };
		qspi_setLutSeq(&seq, 2);
		char data[READ_SIZE / 2];
		memset(data, 0x06, READ_SIZE / 2);

		_qspi_write(dev, 2, 0, data, READ_SIZE / 2);
		printf("WRITTEN DATA\n");
	}
	printf("\n-------------------\n\n");

	// Check status register
	{
		unsigned char status = 1;
		while (status & 1) {
			_qspi_read(dev, 10, 0, &status, 1);
			printf("Status register: %#x\n", status);
		}
	}
	printf("\n-------------------\n\n");
	// WRITE ENABLE
	{
		_qspi_read(dev, 9, 0, NULL, 0);
		printf("ENABLED WRITE\n");
	}
	printf("\n-------------------\n\n");
	{
		char data[READ_SIZE / 2];
		memset(data, 0x09, READ_SIZE / 2);

		_qspi_write(dev, 2, 0 + READ_SIZE / 2, data, READ_SIZE / 2);
		printf("WRITTEN DATA\n");
	}
	printf("\n-------------------\n\n");
	// Check status register
	{
		unsigned char status = 1;
		while (status & 1) {
			_qspi_read(dev, 10, 0, &status, 1);
			printf("Status register: %#x\n", status);
		}
	}
	printf("\n-------------------\n\n");

	// READ
	{
		lut_seq_t seq = { .instrs = {
							  LUT_INSTR(lut_cmd, 2, 0xEC),
							  LUT_INSTR(lut_addr, 2, 32),
							  LUT_INSTR(lut_dummy, 2, 10),
							  LUT_INSTR(lut_read, 2, 0),
							  0,
						  } };
		qspi_setLutSeq(&seq, 3);
		char data[READ_SIZE];
		memset(data, 0xFF, READ_SIZE);

		_qspi_read(dev, 3, 0, data, READ_SIZE);
		printf("READ\n");
		for (int i = 0; i < READ_SIZE; i++) {
			printf("%#x ", data[i]);
		}
		printf("\n");
	}
	// Write enable.
	{
		_qspi_read(dev, 9, 0, NULL, 0);
		printf("ENABLED WRITE\n");
	}
	printf("\n-------------------\n\n");
	// ERASE
	{
		lut_seq_t seq = { .instrs = {
							  LUT_INSTR(lut_cmd, 2, 0x21),
							  LUT_INSTR(lut_addr, 2, 32),
							  0,
						  } };
		qspi_setLutSeq(&seq, 4);

		_qspi_read(dev, 4, 0, NULL, 0);
		printf("ERASED DATA\n");
	}
	printf("\n-------------------\n\n");

	// Check status register
	{
		unsigned char status = 1;
		while (status & 1) {
			_qspi_read(dev, 10, 0, &status, 1);
			printf("Status register: %#x\n", status);
		}
	}
	printf("\n-------------------\n\n");

	// READ AGAIN
	{
		char data[READ_SIZE];
		memset(data, 0xFF, READ_SIZE);

		_qspi_read(dev, 3, 0, data, READ_SIZE);
		printf("READ\n");
		for (int i = 0; i < READ_SIZE; i++) {
			printf("%#x ", data[i]);
		}
		printf("\n");
	}
	printf("\n-------------------\n\n");

	return 0;
}

static int qspi_test_read_write(qspi_dev_t dev, size_t flashsz, size_t sectorsz)
{
	int res = 0;
	unsigned errors = 0, addr;
	uint8_t *pattern, *buf;

	printf("%s: ", __func__);

	pattern = malloc(sectorsz);
	if (pattern == NULL) {
		printf("fail: malloc\n");
		return -1;
	}

	buf = malloc(sectorsz);
	if (buf == NULL) {
		printf("fail: malloc\n");
		free(pattern);
		return -1;
	}

	memset(pattern, 0x55, sectorsz);

	/* Test only every sixteenth block */
	for (addr = 0; addr < flashsz; addr += sectorsz * 16) {
		/* As meterfs is fixed right now in the driver implementation
		   we have to erase sector before write, because of metadata written by meterfs */
		if (flashnor_qspiEraseSector(dev, addr) < 0) {
			printf("fail: erase sector at addr %u\n", addr);
			res = -1;
			break;
		}

		if (flashnor_qspiWrite(dev, addr, pattern, sectorsz) < 0) {
			printf("fail: write sector at addr %u\n", addr);
			res = -1;
			break;
		}

		if (flashnor_qspiRead(dev, addr, buf, sectorsz) < 0) {
			printf("fail: read sector at addr %u\n", addr);
			res = -1;
			break;
		}

		errors += valcmp(buf, sectorsz, pattern[0]);
	}

	if (res == 0) {
		if (errors) {
			printf("fail: read %u errors\n", errors);
			res = -1;
		}
		else {
			printf("ok\n");
		}
	}

	free(pattern);
	free(buf);

	return res;
}


static int qspi_test_erase(qspi_dev_t dev, size_t flashsz, size_t sectorsz)
{
	int res = 0;
	unsigned errors = 0, addr;
	uint8_t *buf;

	printf("%s: ", __func__);

	buf = malloc(sectorsz);
	if (buf == NULL) {
		printf("fail: malloc\n");
		return -1;
	}

	/* Clean sectors written by test_write_read */
	for (addr = 0; addr < flashsz; addr += sectorsz * 16) {
		if (flashnor_qspiEraseSector(dev, addr) < 0) {
			printf("fail: erase sector at addr %u\n", addr);
			res = -1;
			break;
		}

		if (flashnor_qspiRead(dev, addr, buf, sectorsz) < 0) {
			printf("fail: erase sector read at addr %u\n", addr);
			res = -1;
			break;
		}

		errors += valcmp(buf, sectorsz, 0xff);
	}

	if (res == 0) {
		if (errors) {
			printf("fail: read %u errors\n", errors);
			res = -1;
		}
		else {
			printf("ok\n");
		}
	}

	free(buf);
	return res;
}


static int qspi_run_tests(qspi_dev_t dev, size_t flashsz, size_t sectorsz)
{
	storage_t storage_dev = { 0 };
	int err;
	if ((err = _flashnor_qspiInit(dev, &storage_dev)) < 0) {
		return err;
	}

	if (qspi_test_read_write(dev, flashsz, sectorsz) < 0)
		return -1;

	if (qspi_test_erase(dev, flashsz, sectorsz) < 0)
		return -1;

	return 0;
}

int main(int argc, char *argv[])
{
	return qspi_run_tests(qspi_flash_a, 256 * 1024 * 1024, 4 * 1024);
	return qspi_test(qspi_flash_a);
	int err, c;
	storage_t dev = { 0 };
	unsigned spi_no = 3;
	meterfs_ctx_t *ctx;

	while ((c = getopt(argc, argv, "e:q")) != -1) {
		switch (c) {
			case 'e':
				spi_no = strtoul(optarg, NULL, 0);
				flashnor_eraseSector = flashnor_ecspiEraseSector;
				flashnor_read = flashnor_ecspiRead;
				flashnor_write = flashnor_ecspiWrite;
				flashnor_init = flashnor_ecspiInit;
				break;
			case 'q':
				return qspi_test(qspi_flash_a);
				// spi_no = strtoul(optarg, NULL, 0);


				// return EXIT_SUCCESS;
				// flashnor_eraseSector = flashnor_qspiEraseSector;
				// flashnor_read = flashnor_qspiRead;
				// flashnor_write = flashnor_qspiWrite;
				// flashnor_init = flashnor_qspiInit;
				break;
			default:
				flashnor_test_help(argv[0]);
				return EXIT_FAILURE;
		}
	}

	err = flashnor_init(spi_no, &dev);
	if (err < 0) {
		printf("fail: failed to initialize device (err: %d)\n", err);
		return EXIT_FAILURE;
	}

	ctx = dev.ctx;

	if (run_tests(ctx->sz, ctx->sectorsz) < 0)
		return EXIT_FAILURE;

	return EXIT_SUCCESS;
}
