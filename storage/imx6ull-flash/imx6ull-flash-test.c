/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND flash driver.
 *
 * Copyright 2018 Phoenix Systems
 * Author: Jan Sikorski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include <sys/file.h>
#include <sys/mman.h>
#include <sys/msg.h>

#include "imx6ull-flashdrv.h"
#include "imx6ull-flashsrv.h"

#define FLASHDRV_PAGESZ (4096 + 224)


void test_1(void)
{
	void *data, *meta;
	flashdrv_dma_t *dma;
	int err;
	unsigned last_block = 0xff << 6;
	int i, b;

	unsigned long long corrected_errors = 0, uncorrectable_blocks = 0, failed_erase = 0, failed_write = 0, error_erased = 0;

	flashdrv_meta_t *m;

	flashdrv_init();

	dma = flashdrv_dmanew();

	data = mmap(NULL, SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);
	m = meta = mmap(NULL, SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);

	for (int i = 0; i < 0x1000; ++i) {
		((char *)data)[i] = (char)i;
		((char *)meta)[i] = 0xba;
	}

	printf("reset\n");
	flashdrv_reset(dma);

	for (i = 0; ; ++i) {
		printf("%d: ", i);

		if ((err = flashdrv_erase(dma, last_block)))
			failed_erase++;

		if ((err = flashdrv_write(dma, last_block, data, meta)))
			failed_write++;

		if ((err = flashdrv_read(dma, last_block, data, meta))) {
			for (b = 0; b < 9; ++b) {
				switch (m->errors[b]) {
					case flash_no_errors:
						break;
					case flash_uncorrectable:
						uncorrectable_blocks++;
						break;
					case flash_erased:
						error_erased++;
						break;
					default:
						corrected_errors += m->errors[b];
						break;
				}
			}
		}

		printf(" corrected: %llu uncorrectable: %llu failed erase: %llu failed write: %llu erased: %llu\n",
			corrected_errors, uncorrectable_blocks, failed_erase, failed_write, error_erased);
	}
}


int test_erase(const char *path, size_t offset, size_t size)
{
	msg_t msg;
	oid_t oid;
	flash_i_devctl_t *idevctl = NULL;
	flash_o_devctl_t *odevctl = NULL;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	idevctl = (flash_i_devctl_t *)msg.i.raw;

	if (lookup(path, NULL, &oid) < 0) {
		printf("Lookup error.");
		return -1;
	}

	idevctl->type = flashsrv_devctl_erase;
	idevctl->erase.oid = oid;
	idevctl->erase.size = size;
	idevctl->erase.offset = offset;

	if (msgSend(oid.port, &msg) < 0) {
		printf("\nSending error to port: %u.", oid.port);
		return -1;
	}

	odevctl = (flash_o_devctl_t *)msg.o.raw;

	if (odevctl->err < 0) {
		printf("Err: %d.\n", odevctl->err);
		return -1;
	}

	printf("\nErase completed.\n");

	return 0;
}


void test_readwrite(const char *path, char content, const size_t SIZE)
{
	char data[SIZE];
	char rcv[SIZE];

	int fd = -1;
	int counter = 0;

	memset(data, content, SIZE - 1);
	data[SIZE-1] = '\0';

	printf("#\tWriting. Data len: %d.\n", sizeof(data));
	fd = open(path, O_WRONLY);
	if (fd > 0) {
		errno = 0;
		counter = write(fd, data, sizeof(data));
		printf("Saved: %d bytes. Errno: %s\n", counter, strerror(errno));
		close(fd);
	}

	printf("\n#\tReading.\n");
	fd = open(path, O_RDONLY);
	if (fd > 0) {
		errno = 0;
		counter = read(fd, rcv, sizeof(rcv));
		printf("Read: %d bytes. Errno: %s\n", counter, strerror(errno));
		close(fd);
	}

	printf("# Compare buffers:\n");
	if (strcmp(data, rcv) == 0)
		printf("- Reading and writing to partition correctly.\n");
	else
		printf("- Reading and writing to partition uncorrectly.\n");
}


void test_2(const char *path)
{
	const size_t DATA_SIZE = 4096;

	printf("\nSTART test for: %s\n\n", path);
	if (test_erase(path, 0, DATA_SIZE) < 0)
		return;

	test_readwrite(path, 0xaa, DATA_SIZE);

	if (test_erase(path, 0, DATA_SIZE) < 0)
		return;

	test_readwrite(path, 0x55, DATA_SIZE);
	printf("\nEND\n");
}


void test_3(void)
{
	void *buffer = mmap(NULL, 16 * SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_PHYSMEM, 0x900000);
	flashdrv_dma_t *dma;
	int err;

	memset(buffer, 0, 16 * SIZE_PAGE);

	for (int i = 0; i < 0x1000; ++i) {
		((char *)buffer)[i] = 0xb2;
		((char *)buffer)[0x1000 + i] = 0x8a;
	}


	flashdrv_init();

	printf("creating\n");

	dma = flashdrv_dmanew();

	printf("reset\n");
	flashdrv_reset(dma);

	printf("erase\n");
	flashdrv_erase(dma, 0);

	printf("write ");
	err = flashdrv_write(dma, 0, buffer, buffer + 0x1000);
	printf("%d\n", err);

	printf("readraw ");
	err = flashdrv_readraw(dma, 0, buffer + 0x5000, FLASHDRV_PAGESZ);
	printf("%d\n", err);


	printf("read ");
	err = flashdrv_read(dma, 0, buffer + 0x2000, buffer + 0x3000);
	printf("%d\n", err);

	printf("read ");
	err = flashdrv_read(dma, 0, buffer + 0xb000, buffer + 0xc000);
	printf("%d\n", err);

	printf("erase\n");
	flashdrv_erase(dma, 0);


	*((char *)buffer + 0x5100) |= 1;

	printf("writeraw EVIL ");
	err = flashdrv_writeraw(dma, 0, buffer + 0x5000, FLASHDRV_PAGESZ);
	printf("%d\n", err);



	printf("readraw ");
	err = flashdrv_readraw(dma, 0, buffer + 0x9000, FLASHDRV_PAGESZ);
	printf("%d\n", err);

	printf("readmeta ");
	err = flashdrv_read(dma, 0, NULL, buffer + 0x4000);
	printf("%d\n", err);


	printf("read ");
	err = flashdrv_read(dma, 0, buffer + 0x7000, buffer + 0x8000);
	printf("%d\n", err);

	printf("read ");
	err = flashdrv_read(dma, 0, buffer + 0xd000, buffer + 0xe000);
	printf("%d\n", err);


	printf("done\n");


usleep(1000000);
__asm__ volatile ("1: b 1b");
}


int main(int argc, char **argv)
{
//	test_1();
//	test_2( "/dev/flash0");
//	test_2( "/dev/flash1");
//	test_2( "/dev/flash2");
	test_2( "/dev/flash3");
//	test_2( "/dev/flashsrv");
//	test_3();

	return 0;
}
