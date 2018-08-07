/*
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


#include "bcb.h"
#include "test.h"

test_func_t test_func[16];
int test_cnt;


#define nand_msg(silent, fmt, ...)		\
	do {								\
		if (!silent)					\
		printf(fmt, ##__VA_ARGS__);		\
	} while (0)


static inline int check_block(char *raw_block)
{
	if (raw_block[4096] != 0xff)
		return 1;
	return 0;
}


int flash_image(void *arg, char *path, u32 start, u32 block_offset, int silent, int raw, dbbt_t *dbbt)
{
	int ret = 0;
	int imgfd, pgsz, offs = 0;
	struct stat *stat;
	void *img;
	void *img_buf;
	void *meta_buf;
	u32 page_num, block_num;
	flashdrv_dma_t *dma;

	nand_msg(silent, "\n------ FLASH ------\n");

	if (start < 0) {
		nand_msg(silent, "Start block must by larger than 0\n");
		nand_msg(silent, "\n------------------\n");
		return -1;
	}

	stat = malloc(sizeof(struct stat));

	nand_msg(silent, "Flashing %s starting from block %d... \n", path, start);
	imgfd = open(path, 'r');

	if (fstat(imgfd, stat)) {
		nand_msg(silent, "File stat failed\n");
		nand_msg(silent, "\n------------------\n");
		return -1;
	}

	img = mmap(NULL, (stat->st_size + 0xfff) & ~0xfff, PROT_READ | PROT_WRITE, 0, OID_NULL, 0);

	img_buf = mmap(NULL, 2 * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_PHYSMEM, 0x900000);
	meta_buf = img_buf + PAGE_SIZE;

	memset(meta_buf, 0xff, sizeof(flashdrv_meta_t));

	while ((ret = read(imgfd, img + offs, 1024)) > 0) {
		offs += ret;
		if (offs >= stat->st_size)
			break;
	}

	if (arg == NULL) {
		flashdrv_init();
		dma = flashdrv_dmanew();

		flashdrv_reset(dma);
	} else
		dma = (flashdrv_dma_t *)arg;

	offs = 0;
	pgsz = raw ? RAW_PAGE_SIZE : PAGE_SIZE;
	while (offs < stat->st_size) {
		page_num = (offs / pgsz) + (start * 64);
		block_num = page_num / (PAGES_PER_BLOCK);

		if (!(page_num % PAGES_PER_BLOCK) && dbbt_block_is_bad(dbbt, block_num)) {
			start++;
			continue;
		}
		memset(img_buf, 0x00, pgsz);
		memcpy(img_buf, img + offs, pgsz + offs > stat->st_size ? stat->st_size - offs : pgsz);
		if (!raw) {
			if ((ret = flashdrv_write(dma,  page_num + block_offset, img_buf, meta_buf))) {
				nand_msg(silent, "Image write error 0x%x at offset 0x%x\n", ret, offs);
				return -1;
			}
		} else {
			if ((ret = flashdrv_writeraw(dma, (offs / RAW_PAGE_SIZE) + (start * 64) + block_offset, img_buf, RAW_PAGE_SIZE))) {
				nand_msg(silent, "Image write raw error 0x%x at offset 0x%x\n", ret, offs);
				return -1;
			}
		}
		offs += pgsz;
		block_offset = 0;
	}

	nand_msg(silent, "\n------------------\n");

	if (arg == NULL)
		flashdrv_dmadestroy(dma);

	return ret;
}


int flash_check(void *arg, int silent, dbbt_t **dbbt)
{
	flashdrv_dma_t *dma;
	int i, ret = 0;
	int bad = 0;
	int err = 0;
	void *raw_data = mmap(NULL, 2 * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_PHYSMEM, 0x900000);
	u32 bbt[256] = { 0 };
	u32 bbtn = 0;
	int dbbtsz;

	if (raw_data == MAP_FAILED) {
		nand_msg(silent, "Failed to map pages from OC RAM\n");
		return -1;
	}

	memset(raw_data, 0, 4320);

	if (arg == NULL) {
		flashdrv_init();
		dma = flashdrv_dmanew();
		flashdrv_reset(dma);
	} else
		dma = (flashdrv_dma_t *)arg;

	nand_msg(silent, "\n------ CHECK ------\n");

	for (i = 0; i < BLOCKS_CNT; i++) {

		memset(raw_data, 0, RAW_PAGE_SIZE);
		ret = flashdrv_readraw(dma, (i * PAGES_PER_BLOCK), raw_data, RAW_PAGE_SIZE);

		if (ret != EOK) {
			nand_msg(silent, "Reading block %d returned an error\n", i);
			err++;
			bbt[bbtn++] = i;
		}

		if (check_block(raw_data)) {
			nand_msg(silent, "Block %d is marked as bad\n", i);
			bad++;
			bbt[bbtn++] = i;
		}

		if (bbtn >= BB_MAX) {
			nand_msg(silent, "Too many bad blocks. Flash is not usable\n");
			break;
		}
	}

	nand_msg(silent, "\nTotal blocks read: %d\n\n", i);
	nand_msg(silent, "Number of read errors: %d\n", err);
	nand_msg(silent, "Number of bad blocks:  %d\n", bad);
	nand_msg(silent, "------------------\n");

	if (dbbt != NULL && bbtn < BB_MAX) {
		dbbtsz = (sizeof(dbbt_t) + (sizeof(u32) * bbtn) + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1); 
		*dbbt = malloc(dbbtsz);
		memset(*dbbt, 0, dbbtsz);
		memcpy(&((*dbbt)->bad_block), &bbt, sizeof(u32) * bbtn);
		(*dbbt)->entries_num = bbtn;
	}

	if (arg == NULL)
		flashdrv_dmadestroy(dma);

	munmap(raw_data, 2 * PAGE_SIZE);
	return (bbtn >= BB_MAX);
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

void flash_erase(void *arg, int start, int end, int silent)
{
	flashdrv_dma_t *dma;
	int i;
	int err;

	nand_msg(silent, "\n------ ERASE ------\n", start, end);

	if (end < start)
		nand_msg(silent, "Invalid range (%d-%d)\n", start, end);

	printf("Erasing blocks from %d to %d\n", start, end);
	if (arg == NULL) {
		flashdrv_init();
		dma = flashdrv_dmanew();

		flashdrv_reset(dma);
	} else
		dma = (flashdrv_dma_t *)arg;

	for (i = start; i <= end; i++) {
		err = flashdrv_erase(dma, PAGES_PER_BLOCK * i);
		if (err)
			printf("Erasing block %d returned error %d\n", i, err);
	}
	if (arg == NULL)
		flashdrv_dmadestroy(dma);
	nand_msg(silent, "--------------------\n");
}

void set_nandboot(char *primary, char *secondary, char *rootfs)
{
	int ret = 0, err = 0;
	flashdrv_dma_t *dma;
	fcb_t *fcb = malloc(sizeof(fcb_t));
	dbbt_t *dbbt;

	flashdrv_init();
	dma = flashdrv_dmanew();
	flashdrv_reset(dma);

	printf("\n- NANDBOOT SETUP -\n");

	printf("Flash erase\n");
	flash_erase(dma, 0, 192, 1);

	printf("Performing flash check\n");
	ret = flash_check(dma, 1, &dbbt);

	if (ret) {
		printf("Error while checking flash %d\n", ret);
		return;
	}

	printf("Flashing fcb\n");
	/* we do not check for bad blocks when flashing fcb,
	 * if these blocks are bad we can't do much about it. */
	ret = fcb_flash(dma, fcb);

	if (ret) {
		if (ret >= 4) {
			printf("ERROR: Flashing fcb failed entirely - this device won't boot correctly\n");
			printf("------ FAIL ------\n");
			free(dbbt);
			free(fcb);
			return;
		} else
			printf("WARNING: Flashing fcb failed %d out of 4 times - this may impact device's lifespan\n", ret);
	}

	if(dbbt_flash(dma, dbbt))
		err++;

	printf("Flashing primary image: %s\n", primary);
	if (flash_image(dma, primary, fcb->fw1_start / PAGES_PER_BLOCK, 0, 1, 0, dbbt))
		err++;

	printf("Flashing secondary image: %s\n", secondary);
	if (flash_image(dma, secondary, fcb->fw2_start / PAGES_PER_BLOCK, 0, 1, 0, dbbt))
		err++;

	printf("Flashing rootfs: %s\n", rootfs);
	if (flash_image(dma, rootfs, 64, 0, 1, 0, dbbt))
		err++;

	if (flash_image(dma, rootfs, 128, 0, 1, 0, dbbt))
		err++;

	flashdrv_dmadestroy(dma);
	printf("------------------\n");

	free(dbbt);
	free(fcb);

	if (err)
		printf("%d error(s) occured. Device may not boot correctly after restart.\n", err);
	else
		printf("All done. Restart the device to boot from internal storage.\n");
}

void print_help(void)
{
	printf("Usage:\n" \
			"\t-i (path) - file path (requires -s option)\n" \
			"\t-r (path) - just like -i option but raw\n" \
			"\t-s (number) - start flashing from page (requires -i option)\n" \
			"\t-c - search for bad blocks from factory and print summary\n" \
			"\t-h - print this message\n" \
			"\t-t (number) - run test #no\n" \
			"\t-e (start:end) - erase blocks form start to end\n" \
			"\t-f (fw1) (fw2) (rootfs) - set flash for internal booting\n");
}

int main(int argc, char **argv)
{
	int c;
	char *path = NULL;
	int start = -1;
	char *tok, *primary, *secondary, *rootfs;
	int len, i, raw = 0;

	while ((c = getopt(argc, argv, "i:r:s:hct:e:f")) != -1) {
		switch (c) {

			case 'i':
				path = optarg;
				raw = 0;
				break;

			case 'r':
				path = optarg;
				raw = 1;
				break;

			case 'c':
				flash_check(NULL, 0, NULL);
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
						flash_erase(NULL, start, atoi(tok), 0);
					else
						flash_erase(NULL, start, start, 0);
				} else
					print_help();
				return 0;
			case 'f':
				if (argc == 2) {

					primary = "/init/primary.img";
					secondary = "/init/secondary.img";
					rootfs = "/init/rootfs.img";
					set_nandboot(primary, secondary, rootfs);
				} else if (argc == 4) {

					len = strlen(argv[optind]);
					for (i = len; i >= 0 && argv[optind][i] != '/'; --i);

					primary = malloc(strlen("/init/") + strlen(argv[optind] + i + 1));
					primary[0] = 0;
					strcat(primary, "/init/");
					strcat(primary, argv[optind++] + i + 1);

					len = strlen(argv[optind]);
					for (i = len; i >= 0 && argv[optind][i] != '/'; --i);

					secondary = malloc(strlen("/init/") + strlen(argv[optind] + i + 1));
					secondary[0] = 0;
					strcat(secondary, "/init/");
					strcat(secondary, argv[optind++] + i + 1);

					len = strlen(argv[optind]);
					for (i = len; i >= 0 && argv[optind][i] != '/'; --i);

					rootfs = malloc(strlen("/init/") + strlen(argv[optind] + i + 1));
					rootfs[0] = 0;
					strcat(rootfs, "/init/");
					strcat(rootfs, argv[optind] + i + 1);

					set_nandboot(primary, secondary, rootfs);

					free(primary);
					free(secondary);
					free(rootfs);

					return 0;
				}
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

	return flash_image(NULL, path, start, 0, 0, raw, NULL);
}
