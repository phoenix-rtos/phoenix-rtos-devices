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

#include "imx6ull-flashnor-ecspi.h"
#include "imx6ull-flashnor-qspi.h"
#include "imx6ull-flashnor-drv.h"


static unsigned valcmp(uint8_t *buf, size_t len, uint8_t val)
{
	unsigned errors = 0;
	size_t i;

	for (i = 0; i < len; i++) {
		if (buf[i] != val) {
			errors++;
		}
	}

	return errors;
}


static int test_read_write(const flashnor_info_t *info, unsigned int start)
{
	int res = 0;
	unsigned errors = 0;
	off_t addr;
	uint8_t *pattern, *buf;
	size_t sectorsz = info->devInfo->erasesz;

	(void)printf("%s: ", __func__);

	pattern = malloc(sectorsz);
	if (pattern == NULL) {
		(void)printf("fail: malloc\n");
		return -1;
	}

	buf = malloc(sectorsz);
	if (buf == NULL) {
		(void)printf("fail: malloc\n");
		free(pattern);
		return -1;
	}

	(void)memset(pattern, 0x55, sectorsz);

	/* Test only every sixteenth block */
	for (addr = start; addr < info->devInfo->size; addr += sectorsz * 16u) {
		/* As meterfs is fixed right now in the driver implementation
		   we have to erase sector before write, because of metadata written by meterfs */
		if (info->ops->erase(info->ndev, addr, sectorsz) < 0) {
			(void)printf("fail: erase sector at addr %llu\n", addr);
			res = -1;
			break;
		}
		int err = info->ops->write(info->ndev, addr, pattern, sectorsz);
		if (err < 0) {
			(void)printf("fail: write sector at addr %llu err %d\n", addr, err);
			res = -1;
			break;
		}

		if (info->ops->read(info->ndev, addr, buf, sectorsz) < 0) {
			(void)printf("fail: read sector at addr %llu\n", addr);
			res = -1;
			break;
		}

		errors += valcmp(buf, sectorsz, pattern[0]);
	}

	if (res == 0) {
		if (errors != 0u) {
			(void)printf("fail: read %u errors\n", errors);
			res = -1;
		}
		else {
			(void)printf("ok\n");
		}
	}

	free(pattern);
	free(buf);

	return res;
}


static int test_erase(const flashnor_info_t *info, unsigned int start)
{
	int res = 0;
	unsigned errors = 0;
	off_t addr;
	uint8_t *buf;
	size_t sectorsz = info->devInfo->erasesz;

	(void)printf("%s: ", __func__);

	buf = malloc(sectorsz);
	if (buf == NULL) {
		(void)printf("fail: malloc\n");
		return -1;
	}

	/* Clean sectors written by test_write_read */
	for (addr = start; addr < info->devInfo->size; addr += sectorsz * 16u) {
		if (info->ops->erase(info->ndev, addr, sectorsz) < 0) {
			(void)printf("fail: erase sector at addr %llu\n", addr);
			res = -1;
			break;
		}

		if (info->ops->read(info->ndev, addr, buf, sectorsz) < 0) {
			(void)printf("fail: erase sector read at addr %llu\n", addr);
			res = -1;
			break;
		}

		errors += valcmp(buf, sectorsz, 0xff);
	}

	if (res == 0) {
		if (errors > 0u) {
			(void)printf("fail: read %u errors\n", errors);
			res = -1;
		}
		else {
			(void)printf("ok\n");
		}
	}

	free(buf);
	return res;
}


static int run_tests(const flashnor_info_t *info, unsigned int start)
{
	if (test_read_write(info, start) < 0) {
		return -1;
	}

	if (test_erase(info, start) < 0) {
		return -1;
	}

	return 0;
}


static void flashnor_test_help(const char *prog)
{
	(void)printf("Usage: %s [options]\n", prog);
	(void)printf("\t-e <n> | --ecspi <n>   - initialize ECSPI NOR flash device\n");
	(void)printf("\t\tn:      ECSPI instance number\n");
	(void)printf("\t-q <n> | --qspi <n>   - initialize QSPI NOR flash device\n");
	(void)printf("\t\tn:      QSPI instance number(1 - flash A, 2 - flash B)\n");
	(void)printf("\t-s <n> | --start <n>   - test start offset, must be provided as the first argument\n");
}


int main(int argc, char *argv[])
{
	int c, ndev;
	unsigned int start = 0;
	flashnor_info_t info = { 0 };

	if (argc < 2) {
		flashnor_test_help(argv[0]);
		return EXIT_FAILURE;
	}

	while ((c = getopt(argc, argv, "e:q:s:")) != -1) {
		switch (c) {
			case 'e':
				ndev = strtoul(optarg, NULL, 0);
				flashnor_ecspiInit(ndev, &info);

				if (run_tests(&info, start) < 0) {
					return EXIT_FAILURE;
				}
				break;
			case 'q':
				ndev = strtoul(optarg, NULL, 0);
				flashnor_qspiInit(ndev, &info);

				if (run_tests(&info, start) < 0) {
					return EXIT_FAILURE;
				}
				break;
			case 's':
				start = strtoul(optarg, NULL, 0);
				break;
			default:
				flashnor_test_help(argv[0]);
				return EXIT_FAILURE;
		}
	}

	return EXIT_SUCCESS;
}
