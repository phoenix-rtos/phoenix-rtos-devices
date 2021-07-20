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
#include <time.h>

#include <sys/file.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/reboot.h>

#include "imx6ull-flashdrv.h"
#include "imx6ull-flashsrv.h"

static flashdrv_info_t flashinfo;

#define FLASHDRV_PAGESZ  (flashinfo.writesz + flashinfo.metasz)
#define TOTAL_BLOCKS_CNT (flashinfo.size / flashinfo.erasesz)

#define FAIL(...) \
	do { \
		printf(__VA_ARGS__); \
		printf("FAIL!\n"); \
		for (;;) \
			; \
	} while (0)


#define TIMEPROF_SETUP() struct timespec start, end

#define TIMEPROF_START() clock_gettime(CLOCK_MONOTONIC, &start)

/* returns elasped time in us */
#define TIMEPROF_END() (_timeprof_end(&start, &end))
static inline long _timeprof_end(struct timespec *start, struct timespec *end)
{
	clock_gettime(CLOCK_MONOTONIC, end);
	return ((long)end->tv_sec * 1000 * 1000 + end->tv_nsec / 1000) - (start->tv_sec * 1000 * 1000 + start->tv_nsec / 1000);
}

#define TIMEPROF_END_WARN(_thresh_ms, _text) \
	do { \
		long duration = TIMEPROF_END(); \
		if (duration > _thresh_ms * 1000) \
			printf("\n[%4u] %s took too long: %ld.%03ld ms > %d ms\n", blockno, _text, duration / 1000, duration % 1000, _thresh_ms); \
	} while (0)

/* note: these functions are not static to silence compiler warnings when commenting out some of the procedures */


/* clang-format off */
static unsigned char fcb[_PAGE_SIZE * 2] = {
	0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x8b, 0xfb, 0xff, 0xff, 0x46, 0x43, 0x42, 0x20, 0x00, 0x00, 0x00, 0x01, 0x78, 0x3c, 0x19, 0x06,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
	0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,
	0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
	0xa1, 0xcd, 0xe8, 0xa2, 0x8b, 0x77, 0x96, 0x51, 0x6d, 0xe6, 0xe8, 0x5c, 0x76, 0x15, 0x2c, 0x2e,
	0x80, 0x3f, 0x2c, 0x6c, 0x94, 0x5e, 0x94, 0x25, 0xb9, 0x5e, 0xd3, 0x26, 0x40, 0x6c, 0x8f, 0xb1,
	0xfb, 0xd2, 0x01, 0xbe, 0xed, 0x45, 0x8c, 0x1f, 0x0c, 0xc4, 0x4f, 0x09, 0x4a, 0x59, 0x01, 0x74,
	0xa6, 0x4b, 0xa6, 0x2c, 0xed, 0x6e, 0x93, 0x0f, 0x77, 0x5b, 0xd5, 0x7c, 0x48, 0xad, 0xe4, 0xde,
	0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x28, 0x18, 0xd1, 0x34, 0x77, 0x51, 0x24, 0x53, 0x0b, 0xd3, 0xc3, 0x37, 0x28, 0x47, 0xbb,
	0x0d, 0x14, 0x65, 0x21, 0x2b, 0x11, 0x7e, 0x67, 0x0c, 0x21, 0xa5, 0x3e, 0x74, 0x8f, 0x54, 0x72,
	0xe0, 0x3f, 0xad, 0x3a, 0x02, 0x94, 0xd7, 0x47, 0xf9, 0xe5, 0x7a, 0x7e, 0xc8, 0xe9, 0x5d, 0x28,
	0x29, 0xd8, 0xf9, 0x32, 0xa4, 0x3d, 0xc0, 0xdf, 0x45, 0x20, 0x1f, 0x95, 0xd5, 0xb2, 0x78, 0x6d,
	0x82, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
/* clang-format on */


void print_block(const uint8_t *data, unsigned int len)
{
	unsigned int i, j;

	for (i = 0; i < len; i += 16) {
		printf("[%4u]", i);

		for (j = 0; j < 16 && i + j < len; ++j)
			printf(" 0x%02x", data[i + j]);

		printf("\n");
	}
}


/* write predefined FCB block */
void test_write_fcb(void)
{
	flashdrv_dma_t *dma;
	uint8_t *data;
	int err;

	data = mmap(NULL, _PAGE_SIZE * 2, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);
	if (data == MAP_FAILED)
		FAIL("failed to mmap data buffer\n");

	dma = flashdrv_dmanew();

#if 1
	memcpy(data, fcb, FLASHDRV_PAGESZ);
	flashdrv_erase(dma, 0 * 64);
	if ((err = flashdrv_writeraw(dma, 0, data, FLASHDRV_PAGESZ)) < 0)
		printf("writeraw failed\n");
#endif

#if 0 /* uncomment to erase all FCB blocks */
	flashdrv_erase(dma, 0 * 64);
	flashdrv_erase(dma, 1 * 64);
	flashdrv_erase(dma, 2 * 64);
	flashdrv_erase(dma, 3 * 64);
#endif

	flashdrv_dmadestroy(dma);
	munmap(data, _PAGE_SIZE * 2);
}


void test_meta(void)
{
	flashdrv_dma_t *dma;
	uint8_t *data, *meta;
	flashdrv_meta_t *aux;
	int err, i;

	/* block under testing */
	const unsigned int blockno = 0;
	uint32_t paddr = blockno * 64;

	data = mmap(NULL, _PAGE_SIZE * 2, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);
	meta = data + _PAGE_SIZE;
	aux = (flashdrv_meta_t *)meta;

	if (data == MAP_FAILED)
		FAIL("failed to mmap data buffer\n");

	dma = flashdrv_dmanew();

	if ((err = flashdrv_erase(dma, paddr) < 0))
		printf("erase() failed: %d\n", err);

	/* test writing page data + meta */
	memset(data, 0xff, _PAGE_SIZE * 2);
	for (i = 0; i < sizeof(aux->metadata); ++i) {
		aux->metadata[i] = i;
	}

	if ((err = flashdrv_write(dma, paddr, data, (char *)meta)))
		printf("write() failed: %d\n", err);

	memset(data, 0x0, _PAGE_SIZE * 2);

	if ((err = flashdrv_read(dma, paddr, data, aux) < 0))
		printf("read() failed: %d\n", err);

	for (i = 0; i < sizeof(aux->metadata); ++i) {
		if (aux->metadata[i] != i)
			printf("FAIL: meta[%2u] = 0x%02x\n", i, aux->metadata[i]);
	}

	for (i = 0; i < sizeof(aux->errors); ++i) {
		if (aux->errors[i] > 0)
			printf("WARN: errors[%2u] = 0x%02x\n", i, aux->errors[i]);
	}

	for (i = 0; i < flashinfo.writesz; ++i) {
		if (data[i] != 0xff)
			printf("FAIL: data[%u] = 0x%02x\n", i, data[i]);
	}

#if 0 /* uncomment for debuggging */
	if ((err = flashdrv_readraw(dma, paddr, data, FLASHDRV_PAGESZ) < 0))
		printf("readraw() failed: %d\n",  err);

	print_block(data, FLASHDRV_PAGESZ);
#endif


	if ((err = flashdrv_erase(dma, paddr) < 0))
		printf("erase() failed: %d\n", err);

	/* test writing meta only */
	memset(meta, 0xff, _PAGE_SIZE);
	for (i = 0; i < sizeof(aux->metadata); ++i) {
		aux->metadata[i] = i;
	}

	if ((err = flashdrv_write(dma, paddr, NULL, (char *)meta)))
		printf("write() failed: %d\n", err);

	memset(data, 0x0, _PAGE_SIZE * 2);

	/* read only metadata */
	if ((err = flashdrv_read(dma, paddr, NULL, aux) < 0))
		printf("read() failed: %d\n", err);

	for (i = 0; i < sizeof(aux->metadata); ++i) {
		if (aux->metadata[i] != i)
			printf("FAIL: meta[%2u] = 0x%02x\n", i, aux->metadata[i]);
	}

	if (aux->errors[0] > 0) /* errors[0] is for metadata ECC */
		printf("WARN: errors[0] = 0x%02x\n", aux->errors[0]);

	if ((err = flashdrv_erase(dma, paddr) < 0))
		printf("erase() failed: %d\n", err);

	flashdrv_dmadestroy(dma);
	munmap(data, _PAGE_SIZE * 2);
}


/* should be done on factory-new or completely ereased NAND flash */
void test_badblocks(void)
{
	flashdrv_dma_t *dma;
	uint8_t *data;
	unsigned int blockno;
	int err;
	int total_read_fails = 0, total_bad_blocks = 0;

	data = mmap(NULL, _PAGE_SIZE * 2, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);
	if (data == MAP_FAILED)
		FAIL("failed to mmap data buffer\n");

	dma = flashdrv_dmanew();

#if 0 /* uncomment to create fake badblock */
	flashdrv_markbad(dma, 1337 * 64);
	if (flashdrv_isbad(dma, 1337 * 64)) {
		printf("1337 BADBLOCK\n");
	}
	flashdrv_erase(dma, 1337 * 64);
#endif

	/* check every erease block (read first page metadata as RAW to omit ECC checks) */
	for (blockno = 0; blockno < TOTAL_BLOCKS_CNT; ++blockno) {
		uint32_t addr = blockno * 64;

		printf("\rchecking block %4u (page addr=0x%8p)", blockno, (void *)addr);
		if (flashdrv_isbad(dma, addr)) {
			printf("\n[%4u] BADBLOCK\n", blockno);
		}

		memset(data, 0, FLASHDRV_PAGESZ);
		if ((err = flashdrv_readraw(dma, addr, data, FLASHDRV_PAGESZ) < 0)) {
			printf("\n[%4u] readraw() failed: %d\n", blockno, err);
			total_read_fails += 1;
			continue;
		}

		if (data[0] == 0x00) {
			total_bad_blocks += 1;
			printf("\n[%4u] bad block marker detected\n", blockno);
			printf("[%4u] FIRST BYTES: 0x%02x 0x%02x 0x%02x 0x%02x\n", blockno, data[0], data[1], data[2], data[3]);

#if 0 /* according to datasheet erasing badblocks should not be done, test how flash will handle it */
			if ((err = flashdrv_erase(dma, addr) < 0)) {
				printf("[%4u] erase() failed: %d\n", blockno, err);
			}

			if ((err = flashdrv_readraw(dma, addr, data, FLASHDRV_PAGESZ) < 0)) {
				printf("[%4u] readraw() failed: %d\n", blockno, err);
				continue;
			}
			printf("[%4u] FIRST BYTES: 0x%02x 0x%02x 0x%02x 0x%02x\n", blockno, data[0], data[1], data[2], data[3]);
			if (data[0] == 0x00)
				printf("[%4u] still a badblock\n", blockno);
#endif
		}
	}

	flashdrv_dmadestroy(dma);
	munmap(data, _PAGE_SIZE * 2);

	printf("\n------------------\n");
	printf("total_read_fails = %d; total_bad_blocks = %d\n", total_read_fails, total_bad_blocks);
}


/* continuously erase/write/read single block */
void test_stress_one_block(void)
{
	void *data, *meta;
	flashdrv_dma_t *dma;
	int err;
	unsigned int block_no = TOTAL_BLOCKS_CNT - 1; /* use last block */
	uint32_t addr = block_no * 64;
	int i, b;

	unsigned long long corrected_errors = 0, uncorrectable_blocks = 0, failed_erase = 0, failed_write = 0, error_erased = 0;

	flashdrv_meta_t *m;

	dma = flashdrv_dmanew();

	data = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);
	m = meta = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);

	for (int i = 0; i < 0x1000; ++i) {
		((char *)data)[i] = (char)i;
		((char *)meta)[i] = 0xba;
	}

	printf("reset\n");
	flashdrv_reset(dma);

	for (i = 0; ; ++i) {
		printf("%d: ", i);

		if ((err = flashdrv_erase(dma, addr)))
			failed_erase++;

		if ((err = flashdrv_write(dma, addr, data, meta)))
			failed_write++;

		if ((err = flashdrv_read(dma, addr, data, meta))) {
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


/* erase using imx6ull-flashsrv */
/* TODO: test it */
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
	idevctl->erase.address = offset;

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


/* read/write using imx6ull-flashsrv */
/* TODO: test it */
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


void test_flashsrv(const char *path)
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


/* write, read and check */
void _writeraw_and_check(flashdrv_dma_t *dma, uint32_t blockno, uint8_t *data, uint8_t byte)
{
	uint32_t addr = blockno * 64;
	unsigned int i;
	int err;
	TIMEPROF_SETUP();

	TIMEPROF_START();
	memset(data, byte, FLASHDRV_PAGESZ);
	if ((err = flashdrv_writeraw(dma, addr, data, FLASHDRV_PAGESZ)) < 0)
		printf("\n[%4u] writeraw(0x%02x) failed: %d\n", blockno, byte, err);
	TIMEPROF_END_WARN(30, "writeraw");

	TIMEPROF_START();
	memset(data, 0, FLASHDRV_PAGESZ);
	if ((err = flashdrv_readraw(dma, addr, data, FLASHDRV_PAGESZ)) < 0)
		printf("[%4u] readraw(0x%02x) failed: %d\n", blockno, byte, err);
	TIMEPROF_END_WARN(30, "readraw");

	for (i = 0; i < FLASHDRV_PAGESZ; ++i) {
		if (data[i] != byte)
			printf("[%4u] writeraw(0x%02x)[%u] invalid data: 0x%02x\n", blockno, byte, i, data[i]);
	}
}


void test_single_block_raw(flashdrv_dma_t *dma, uint32_t blockno, uint8_t *data, uint8_t *meta)
{
	int err;
	uint32_t addr = blockno * 64;
	TIMEPROF_SETUP();

	TIMEPROF_START();
	if ((err = flashdrv_erase(dma, addr) < 0)) {
		printf("[%4u] erase(1) failed: %d\n", blockno, err);
		/* assume bad-block, don't test further */
		return;
	}
	TIMEPROF_END_WARN(100, "erase(1)");

	_writeraw_and_check(dma, blockno, data, 0x55);

	TIMEPROF_START();
	if ((err = flashdrv_erase(dma, addr) < 0))
		printf("[%4u] erase(2) failed: %d\n", blockno, err);
	TIMEPROF_END_WARN(100, "erase(2)");

	_writeraw_and_check(dma, blockno, data, 0xAA);

	TIMEPROF_START();
	if ((err = flashdrv_erase(dma, addr) < 0))
		printf("[%4u] erase(3) failed: %d\n", blockno, err);
	TIMEPROF_END_WARN(100, "erase(3)");
}


/* test writes with 0x55 and 0xAA pattarn (+ read, erase) */
void test_write_read_erase(void)
{
	flashdrv_dma_t *dma;
	uint8_t *data, *meta;
	unsigned int blockno;

	data = mmap(NULL, _PAGE_SIZE * 2, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);
	meta = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);
	if (data == MAP_FAILED || meta == MAP_FAILED)
		FAIL("failed to mmap data buffers\n");

	dma = flashdrv_dmanew();

	/* check every erease block (read first page metadata as RAW to omit ECC checks) */
	for (blockno = 0; blockno < TOTAL_BLOCKS_CNT; ++blockno) {
		printf("\rblock %4u", blockno);
		test_single_block_raw(dma, blockno, data, meta);
	}

	flashdrv_dmadestroy(dma);
	munmap(data, _PAGE_SIZE * 2);
	munmap(meta, _PAGE_SIZE);

	printf("\n------------------\n");
}


void test_3(void)
{
	void *buffer = mmap(NULL, 16 * _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_PHYSMEM, 0x900000);
	flashdrv_dma_t *dma;
	int err;

	memset(buffer, 0, 16 * _PAGE_SIZE);

	for (int i = 0; i < 0x1000; ++i) {
		((char *)buffer)[i] = 0xb2;
		((char *)buffer)[0x1000 + i] = 0x8a;
	}


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
}

int main(int argc, char **argv)
{
	printf("%s: starting tests\n", argv[0]);

	flashdrv_init();
	memcpy(&flashinfo, flashdrv_info(), sizeof(flashdrv_info_t));

	test_meta();
	//test_write_fcb();
	//test_badblocks();
//	test_write_read_erase();
//	test_stress_one_block();
//	test_flashsrv("/dev/flash0");
//	test_flashsrv("/dev/flash1");
//	test_flashsrv( "/dev/flash2");
//	test_flashsrv( "/dev/flash3");
//	test_flashsrv( "/dev/flashsrv");
//	test_3();
	printf("%s: tests finished\n", argv[0]);
	usleep(3 * 1000 * 1000);
	reboot(PHOENIX_REBOOT_MAGIC);

	return 0;
}
