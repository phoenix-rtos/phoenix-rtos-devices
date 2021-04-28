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
#include <stdint.h>
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

#include <imx6ull-flashsrv.h>

#define PAGES_PER_BLOCK 64
#define FLASH_PAGE_SIZE 0x1000

/* jffs2 cleanmarker - write it on clean blocks to mount faster */
struct cleanmarker
{
	uint16_t magic;
	uint16_t type;
	uint32_t len;
};

static const struct cleanmarker oob_cleanmarker =
{
	.magic = 0x1985,
	.type = 0x2003,
	.len = 8
};

/* tests */
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


int flash_image(void *arg, char *path, uint32_t start, uint32_t block_offset, int silent, int raw, dbbt_t *dbbt)
{
	int ret = 0;
	int imgfd, pgsz, offs = 0;
	struct stat *stat;
	void *img;
	void *img_buf;
	void *meta_buf;
	uint32_t page_num, block_num;
	flashdrv_dma_t *dma;

	nand_msg(silent, "\n------ FLASH ------\n");

	if (start < 0) {
		nand_msg(silent, "Start block must by larger than 0\n");
		nand_msg(silent, "\n------------------\n");
		return -1;
	}

	stat = malloc(sizeof(struct stat));

	nand_msg(silent, "Flashing %s starting from block %d... \n", path, start);
	imgfd = open(path, O_RDONLY);

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


int flash_check_range(void *arg, int start, int end, int silent, dbbt_t **dbbt)
{
	flashdrv_dma_t *dma;
	int i, ret = 0;
	int bad = 0;
	int err = 0;
	void *raw_data = mmap(NULL, 2 * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_PHYSMEM, 0x900000);
	uint32_t bbt[256] = { 0 };
	uint32_t bbtn = 0;
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

	for (i = start; i < end; i++) {

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
		dbbtsz = (sizeof(dbbt_t) + (sizeof(uint32_t) * bbtn) + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1);
		*dbbt = malloc(dbbtsz);
		memset(*dbbt, 0, dbbtsz);
		memcpy(&((*dbbt)->bad_block), &bbt, sizeof(uint32_t) * bbtn);
		(*dbbt)->entries_num = bbtn;
	}

	if (arg == NULL)
		flashdrv_dmadestroy(dma);

	munmap(raw_data, 2 * PAGE_SIZE);
	return (bbtn >= BB_MAX);
}


int flash_check(void *arg, int silent, dbbt_t **dbbt)
{
	return flash_check_range(arg, 0, BLOCKS_CNT, silent, dbbt);
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

	nand_msg(silent, "\n------ ERASE ------\n");

	if (end < start)
		nand_msg(silent, "Invalid range (%d-%d)\n", start, end);

	printf("Erasing blocks from %d to %d\n", start, end);
	if (arg == NULL) {
		flashdrv_init();
		dma = flashdrv_dmanew();

		flashdrv_reset(dma);
	} else
		dma = (flashdrv_dma_t *)arg;

	for (i = start; i < end; i++) {
		err = flashdrv_erase(dma, PAGES_PER_BLOCK * i);
		if (err)
			printf("Erasing block %d returned error %d\n", i, err);
	}
	if (arg == NULL)
		flashdrv_dmadestroy(dma);
	nand_msg(silent, "------------------\n");
}


int flash_write_cleanmarkers(void *arg, int start, int end)
{
	flashdrv_dma_t *dma;
	void *metabuf;
	int i, ret = 0;

	if (arg == NULL) {
		flashdrv_init();
		dma = flashdrv_dmanew();
		flashdrv_reset(dma);
	}
	else
		dma = (flashdrv_dma_t *)arg;

	metabuf = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);
	memset(metabuf, 0xff, PAGE_SIZE);
	memcpy(metabuf, &oob_cleanmarker, 8);
	for (i = start; i < end; i++) {
		ret += flashdrv_write(dma, (i * PAGES_PER_BLOCK), NULL, metabuf);
	}

	return ret;
}

void set_nandboot(char *primary, char *secondary, char *rootfs, size_t rootfssz, int rwfs_erase)
{
	int ret = 0, err = 0;
	flashdrv_dma_t *dma;
	fcb_t *fcb = malloc(sizeof(fcb_t));
	dbbt_t *dbbt;

	flashdrv_init();
	dma = flashdrv_dmanew();
	flashdrv_reset(dma);

	printf("\n- NANDBOOT SETUP -\n");
	printf("Root partition size: %u\n", rootfssz);

	if (!rwfs_erase) {
		printf("Erasing rootfs only\n");
		flash_erase(dma, 0, 64 + (2 * rootfssz), 1);
		ret = flash_check_range(dma, 0, 64 + (2 * rootfssz), 1, &dbbt);
	}
	else {
		printf("Erasing rootfs and data partition\n");
		flash_erase(dma, 0, BLOCKS_CNT, 1);
		ret = flash_check(dma, 1, &dbbt);
		flash_write_cleanmarkers(dma, 64 + (2 * rootfssz), BLOCKS_CNT);
	}

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

	if (flash_image(dma, rootfs, 64 + rootfssz, 0, 1, 0, dbbt))
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


static int send_nandErase(unsigned port, unsigned start, unsigned end)
{
	msg_t msg = { 0 };
	flash_i_devctl_t *devctl;
	int err;

	devctl = (flash_i_devctl_t *)&msg.i.raw;

	msg.type = mtDevCtl;
	devctl->type = flashsrv_devctl_erase;
	devctl->erase.offset = start * PAGES_PER_BLOCK	* _PAGE_SIZE;

	if (start > end)
		return -EINVAL;

	devctl->erase.size = (end - start) * PAGES_PER_BLOCK * _PAGE_SIZE;
	devctl->erase.oid.id = -1;
	devctl->erase.oid.port = port;

	err = msgSend(port, &msg);

	if (err)
		return err;

	return msg.o.io.err;
}


static int send_nandFlash(unsigned port, unsigned offset, unsigned end, void *data, size_t size)
{
	msg_t msg = { 0 };
	int err;

	msg.type = mtWrite;

	msg.i.io.oid.id = -1;
	msg.i.io.oid.port = port;
	msg.i.io.offs = offset * _PAGE_SIZE;

	msg.i.data = data;
	msg.i.size = size;

	err = msgSend(port, &msg);

	if (err)
		return err;

	return msg.o.io.err;
}


static int flash_update_tool(char **args)
{
	char *arg;
	void *data;
	unsigned start, end, offset, port;
	size_t size;
	oid_t oid;
	FILE *f;
	int result = 0;

	lookup("/dev/flashsrv", NULL, &oid);
	port = oid.port;

	if ((arg = *(args++)) == NULL)
		return -EINVAL;

	if (!strncmp(arg, "erase", sizeof("erase"))) {
		if ((arg = *(args++)) == NULL)
			return -EINVAL;

		start = strtoul(arg, NULL, 10);

		if ((arg = *(args++)) == NULL)
			return -EINVAL;

		end = strtoul(arg, NULL, 10);

		if (end < start) {
			fprintf(stderr, "flash: start block must be smaller than end block\n");
			return -EINVAL;
		}

		result = send_nandErase(port, start, end);

		if (result < 0)
			fprintf(stderr, "flash: error while erasing\n");

		return result;
	}
	else if (!strncmp(arg, "write", sizeof("write"))) {
		if ((arg = *(args++)) == NULL)
			return -EINVAL;

		offset = strtoul(arg, NULL, 10);

		if ((arg = *(args++)) == NULL)
			return -EINVAL;

		end = strtoul(arg, NULL, 10);

		if ((arg = *(args++)) == NULL)
			return -EINVAL;

		data = malloc(1 << 20);

		if (data == NULL) {
			fprintf(stderr, "flash: could not allocate buffer\n");
			return -ENOMEM;
		}

		f = fopen(arg, "r");

		if (f == NULL) {
			fprintf(stderr, "flash: could not open file %s\n", arg);
			free(data);
			return -EINVAL;
		}

		while (!feof(f)) {
			size = fread(data, 1, 1 << 20, f);

			if (!size)
				break;

			if (size & (_PAGE_SIZE - 1)) {
				memset(data + size, 0xff, _PAGE_SIZE - (size & (_PAGE_SIZE - 1)));
				size = (size + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1);
			}

			result = send_nandFlash(port, offset, end, data, size);
			offset += size / _PAGE_SIZE;

			if (result < 0) {
				fprintf(stderr, "flash: error while writing\n");
				break;
			}
		}

		fclose(f);
		free(data);
		return result;
	}

	fprintf(stderr, "usage: nandtool -U (erase start-block end-block | write start-page [end-page/0] file)\n");
	return -EINVAL;
}


int main(int argc, char **argv)
{
	int c;
	char *path = NULL;
	int start = -1;
	char *tok, *primary, *secondary, *rootfs;
	int len, i, raw = 0;
	size_t rootfssz = 64, erase_data = 0;

	while ((c = getopt(argc, argv, "i:r:s:hct:e:f:U")) != -1) {
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
				if (argc < 5) {
					if (optarg != NULL)
						rootfssz = atoi(optarg);
					if (argv[optind] != NULL)
						erase_data = atoi(argv[optind]);
					primary = "/init/primary.img";
					secondary = "/init/secondary.img";
					rootfs = "/init/rootfs.img";
					set_nandboot(primary, secondary, rootfs, rootfssz, erase_data);
				} else if (argc == 5) {

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

					set_nandboot(primary, secondary, rootfs, rootfssz, erase_data);

					free(primary);
					free(secondary);
					free(rootfs);

					return 0;
				}
				return 0;

			case 'U':
				return flash_update_tool(argv + optind) < 0 ? EXIT_FAILURE : EXIT_SUCCESS;

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

	return flash_image(NULL, path, start, 0, 0, raw, NULL) < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}
