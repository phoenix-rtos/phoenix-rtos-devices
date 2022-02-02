/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL NOR flash driver test program
 *
 * Copyright 2021 Phoenix Systems
 * Author: Jakub Sarzynski
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
		if (flashnor_ecspiEraseSector(addr) < 0) {
			printf("fail: erase sector at addr %u\n", addr);
			res = -1;
			break;
		}

		if (flashnor_ecspiWrite(addr, pattern, sectorsz) < 0) {
			printf("fail: write sector at addr %u\n", addr);
			res = -1;
			break;
		}

		if (flashnor_ecspiRead(addr, buf, sectorsz) < 0) {
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
		if (flashnor_ecspiEraseSector(addr) < 0) {
			printf("fail: erase sector at addr %u\n", addr);
			res = -1;
			break;
		}

		if (flashnor_ecspiRead(addr, buf, sectorsz) < 0) {
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
}


int main(int argc, char *argv[])
{
	int err, c;
	storage_t dev = { 0 };
	unsigned ecspi_no = 3;
	meterfs_ctx_t *ctx;

	while ((c = getopt(argc, argv, "e:")) != -1) {
		switch (c) {
			case 'e':
				ecspi_no = strtoul(optarg, NULL, 0);
				break;
			default:
				flashnor_test_help(argv[0]);
				return EXIT_FAILURE;
		}
	}

	err = flashnor_ecspiInit(ecspi_no, &dev);
	if (err < 0) {
		printf("fail: failed to initialize device (err: %d)\n", err);
		return EXIT_FAILURE;
	}

	ctx = dev.ctx;

	if (run_tests(ctx->sz, ctx->sectorsz) < 0)
		return EXIT_FAILURE;

	return EXIT_SUCCESS;
}
