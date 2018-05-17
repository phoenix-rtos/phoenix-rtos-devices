#/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND tool.
 *
 * Writes image file to flash
 *
 * Copyright 2018 Phoenix Systems
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include <sys/msg.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "../imx6ull-flash/flashdrv.h"

#include "test.h"

#define PAGE_SIZE 4096
#define READ_SIZE 4320
#define PAGES_PER_BLOCK 64
#define BLOCKS_CNT 4096

static inline int check_block(char *raw_block)
{
	if (!raw_block[0])
		return 1;
	return 0;
}


int flash_image(char *path, int start)
{
	int ret = 0;
	int imgfd, offs = 0;
	struct stat *stat;
	void *img;
	void *img_buf;
	void *meta_buf;
	flashdrv_dma_t *dma;

	printf("\n------ FLASH ------\n");

	if (start < 0) {
		printf("Start eraseblock must by larger than 0\n");
		printf("\n------------------\n");
		return -1;
	}

	stat = malloc(sizeof(struct stat));

	printf("Flashing %s starting from page %d... \n", path, start);
	imgfd = open(path, 'r');

	if (fstat(imgfd, stat)) {
		printf("File stat failed\n");
		printf("\n------------------\n");
		return -1;
	}

	img = mmap(NULL, (stat->st_size + 0xfff) & ~0xfff, PROT_READ | PROT_WRITE, 0, OID_NULL, 0);

	img_buf = mmap(NULL, 2 * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_PHYSMEM, 0x900000);
	meta_buf = img_buf + PAGE_SIZE;

	memset(meta_buf, 0xff, sizeof(flashdrv_meta_t));

	while ((ret = read(imgfd, img + offs, 1024)) > 0) {
		offs += ret;
		if (offs == stat->st_size)
			break;
	}

	flashdrv_init();
	dma = flashdrv_dmanew();

	flashdrv_reset(dma);

	offs = 0;

	while (offs < stat->st_size) {
		memcpy(img_buf, img + offs, PAGE_SIZE);
		if ((ret = flashdrv_write(dma, offs / PAGE_SIZE, img_buf, meta_buf))) {
			printf("Image wrtie error 0x%x at offset 0x%x\n", ret, offs);
			return -1;
		}
		offs += PAGE_SIZE;
	}

	printf("\n------------------\n");
	return ret;
}


void flash_check(void)
{
	flashdrv_dma_t *dma;
	int i, ret = 0;
	int bad = 0;
	int err = 0;
	void *raw_data = mmap(NULL, 2 * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_PHYSMEM, 0x900000);

	memset(raw_data, 0, 4320);

	if (raw_data == NULL) {
		printf("Failed to map pages from OC RAM\n");
		return;
	}

	flashdrv_init();
	dma = flashdrv_dmanew();

	flashdrv_reset(dma);

	printf("\n------ CHECK ------\n");

	for (i = 0; i < BLOCKS_CNT; i++) {

		memset(raw_data, 0, 4320);
		ret = flashdrv_readraw(dma, (i * PAGES_PER_BLOCK), raw_data, READ_SIZE);

		if (ret != EOK) {
			printf("Reading block %d returned an error\n", i);
			err++;
		}

		if (check_block(raw_data)) {
			printf("Block %d is marked as bad\n", i);
			bad++;
		}
	}

	printf("\nTotal blocks read: %d\n\n", i);
	printf("Number of read errors: %d\n", err);
	printf("Number of bad blocks:  %d\n", bad);
	printf("------------------\n");
}


/* test some things */
void flash_test(int test_no, void * arg)
{
	int ret = 0;

	if (!test_enabled) {
		printf("Tests are not compiled\n");
		return;
	}

	init_tests();

	if (test_no >= test_cnt || test_no >= 15) {
		printf("Invalid test number\n");
		return;
	}

	printf("\n------ TEST %d ------\n", test_no);

	ret = test_func[test_no](arg);

	if (ret)
		printf("------ FAILED ------\n\n");
	else
		printf("------ PASSED ------\n\n");
}

void flash_erase(int start, int end)
{
	flashdrv_dma_t *dma;
	int i;
	int err;

	printf("\n------ ERASE ------\n", start, end);

	if (end < start)
		printf("Invalid range (%d-%d)\n", start, end);

	printf("Erasing blocks from %d to %d\n", start, end);

	flashdrv_init();
	dma = flashdrv_dmanew();

	flashdrv_reset(dma);

	for (i = start; i <= end; i++) {
		err = flashdrv_erase(dma, i);
		if (err)
			printf("Erasing block %d returned error %d\n", i, err);
	}
	printf("--------------------\n");
}

void print_help(void)
{
	printf("Usage:\n" \
			"\t-i (path) - image file path (requires -s option)\n" \
			"\t-s (number) - start flashing from page (requires -i option)\n" \
			"\t-c - search for bad blocks from factory and print summary\n" \
			"\t-h - print this message\n" \
			"\t-t (number) - run test #no\n" \
			"\t-e (start:end) - erase blocks form start to end\n");
}


int main(int argc, char **argv)
{
	int c;
	char *path = NULL;
	int start = -1;
	char *tok;
	while ((c = getopt(argc, argv, "i:s:hct:e:")) != -1) {
		switch (c) {

			case 'i':
				path = optarg;
				break;

			case 'c':
				flash_check();
				return 0;

			case 's':
				start = atoi(optarg);
				break;

			case 't':
				if (optarg != NULL)
					flash_test(atoi(optarg), NULL);
				return 0;
			case 'e':
				if (optarg != NULL) {
					tok = strtok(optarg,":");
					start = atoi(tok);
					tok = strtok(NULL,":");
					if (tok != NULL)
						flash_erase(start, atoi(tok));
					else
						flash_erase(start, start);
				} else
					print_help();
				return 0;
			case 'h':
			default:
				print_help();
				return 0;
		}
	}

	if (path == NULL || start < 0) {
		print_help();
		return 0;
	}

	return flash_image(path, start);
}

