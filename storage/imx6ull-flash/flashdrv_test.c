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
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include <sys/mman.h>
#include "flashdrv.h"

#define FLASHDRV_PAGESZ (4096 + 224)

int main(int argc, char **argv)
{
	/* run some tests */
#if 0
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


#else
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
#endif
	return 0;
}
