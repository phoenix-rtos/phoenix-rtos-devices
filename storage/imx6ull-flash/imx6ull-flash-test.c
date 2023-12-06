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

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>

#include <sys/file.h>
#include <sys/minmax.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/reboot.h>

#include "imx6ull-flashdrv.h"
#include "imx6ull-flashsrv.h"

static flashdrv_info_t flashinfo;
static size_t pagemapsz;

#define FLASHDRV_PAGESZ  (flashinfo.writesz + flashinfo.metasz)
#define TOTAL_BLOCKS_CNT (flashinfo.size / flashinfo.erasesz)
#define BLOCK_PAGES_CNT  (flashinfo.erasesz / flashinfo.writesz)

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


/* Flips n bits in a raw block (assume n <= end - start) */
/* Please note that calling this function twice with the same parameters and the same rand() seed */
/* results in flipping the same bits, effectively restoring original data */
int flip_bits(unsigned int n, unsigned int nblock, unsigned int start, unsigned int end, int check_ecc)
{
	unsigned int i, j, mapped_cnt, offs, paddr = nblock * BLOCK_PAGES_CNT;
	flashdrv_dma_t *dma;
	flashdrv_meta_t *aux;
	uint8_t **data;
	int err = EOK;

	if ((end > BLOCK_PAGES_CNT * FLASHDRV_PAGESZ) || (end <= start) || (n > end - start))
		return -EINVAL;

	if (!n)
		return EOK;

	if ((dma = flashdrv_dmanew()) == MAP_FAILED) {
		err = -ENOMEM;
		printf("dmanew() failed, err: %d\n", err);
		return err;
	}

	if ((data = malloc(BLOCK_PAGES_CNT * sizeof(uint8_t *))) == NULL) {
		err = -ENOMEM;
		printf("malloc() failed, err: %d\n", err);
		flashdrv_dmadestroy(dma);
		return err;
	}

	do {
		/* Read raw block data into memory and flip bits */
		for (mapped_cnt = 0; mapped_cnt < BLOCK_PAGES_CNT; mapped_cnt++) {
			if ((data[mapped_cnt] = mmap(NULL, pagemapsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0)) == MAP_FAILED) {
				err = -ENOMEM;
				printf("mmap() failed: %d\n", err);
				break;
			}

			if ((err = flashdrv_readraw(dma, paddr + mapped_cnt, data[mapped_cnt], FLASHDRV_PAGESZ)) < 0) {
				printf("readraw() failed: %d\n", err);
				break;
			}

			/* Page start offset within the block */
			offs = mapped_cnt * FLASHDRV_PAGESZ;

			/* Flip bits (one bit flip per byte) */
			while (start < end) {
				/* Skip pages outside the bit flip range */
				if ((start < offs) || (start >= offs + FLASHDRV_PAGESZ))
					break;

				data[mapped_cnt][start - offs] ^= 1 << rand() % 8;

				/* Keep bytes with bit flips evenly distributed */
				start += (end - start) / n--;
			}
		}

		if (err)
			break;

		/* Erase the block */
		if ((err = flashdrv_erase(dma, paddr)) < 0) {
			printf("erase() failed: %d\n", err);
			break;
		}

		/* Write back raw block data with flipped bits */
		for (i = 0; i < BLOCK_PAGES_CNT; i++) {
			if ((err = flashdrv_writeraw(dma, paddr + i, data[i], FLASHDRV_PAGESZ)) < 0) {
				printf("writeraw() failed: %d\n", err);
				break;
			}

			/* Print ECC errors info */
			if (check_ecc) {
				aux = (flashdrv_meta_t *)(data[i] + _PAGE_SIZE);
				if ((err = flashdrv_read(dma, paddr + i, data[i], aux)) < 0) {
					printf("read() failed: %d\n", err);
					break;
				}

				for (j = 0; j < sizeof(aux->errors); j++) {
					switch (aux->errors[j]) {
						case flash_no_errors:
						case flash_erased:
							break;

						case flash_uncorrectable:
							printf("[%4u]: uncorrectable data in chunk %u\n", paddr + i, j);
							break;

						default:
							printf("[%4u]: %u corrections in chunk %u\n", paddr + i, aux->errors[j], j);
					}
				}
			}
		}
	} while (0);

	for (i = 0; i < mapped_cnt; i++)
		munmap(data[i], pagemapsz);
	free(data);
	flashdrv_dmadestroy(dma);

	return err;
}


/* Breaks ECC correction / sets BBM in FCB, DBBT, FW1 and FW2 */
/* After running the test the BootROM shouldn't be able to boot from NAND flash */
/* Run the test again to restore original data (won't remove BBMs) */
void test_bootrom(void)
{
	const unsigned int rawmetasz = 16 + 26;  /* 16B META + 26B ECC16 */
	const unsigned int rawdatasz = 512 + 23; /* 512B DATA + 22.75B ECC14 (adding 2 bits for byte alignment) */
	const unsigned int seed = 0xaa55aa55;    /* Use fixed rand() seed (so we could revert the bit flips by running the test again) */
	flashdrv_dma_t *dma;

	if ((dma = flashdrv_dmanew()) == MAP_FAILED) {
		printf("dmanew() failed, err: %d\n", -ENOMEM);
		return;
	}

	srand(seed);

	do {
		/* FCB structure encoded with BCH40 (as expected by the BootROM) */
		/* [32B]  8x[128B + 65B]   [536B] */
		/* [META] 8x[DATA + ECC40] [SPARE] */

		/* Break FCB0 first data chunk (ECC should correct up to 40 bit flips) */
		/* Passing check_ecc = 0 since our ECC would always fail due to different configuration */
		printf("Breaking FCB0 ECC...\n");
		if (flip_bits(41, 0, 32, 32 + 128 + 65, 0) < 0) {
			printf("failed to break FCB0\n");
			break;
		}

#if 0
		/* Set FCB0 BBM */
		if (flashdrv_markbad(dma, 0 * 64) < 0) {
			printf("failed to set FCB0 BBM\n");
			break;
		}
#endif

		/* Default page structure (only FCB is read with different configuration by the BootROM) */
		/* [16B + 26B]    8x[512B + 22.75B] [32B] */
		/* [META + ECC16] 8x[DATA + ECC14]  [SPARE] */

		/* Break DBBT0 first data chunk (ECC should correct up to 14 bit flips) */
		printf("Breaking DBBT0 ECC...\n");
		if (flip_bits(15, 4, rawmetasz, rawmetasz + rawdatasz, 1) < 0) {
			printf("failed to break DBBT0\n");
			break;
		}

#if 0
		/* Set DBBT0 BBM */
		if (flashdrv_markbad(dma, 4 * 64) < 0) {
			printf("failed to set DBBT0 BBM\n");
			break;
		}
#endif

		/* Break FW1 first data chunk (ECC should correct up to 14 bit flips) */
		printf("Breaking FW1 ECC...\n");
		if (flip_bits(15, 8, rawmetasz, rawmetasz + rawdatasz, 1) < 0) {
			printf("failed to break FW1\n");
			break;
		}

#if 0
		/* Set FW1 BBM */
		if (flashdrv_markbad(dma, 8 * 64) < 0) {
			printf("failed to set FW1 BBM\n");
			break;
		}
#endif

		/* Break FW2 first data chunk (ECC should correct up to 14 bit flips) */
		printf("Breaking FW2 ECC...\n");
		if (flip_bits(15, 24, rawmetasz, rawmetasz + rawdatasz, 1) < 0) {
			printf("failed to break FW2\n");
			break;
		}

#if 0
		/* Set FW2 BBM */
		if (flashdrv_markbad(dma, 24 * 64) < 0) {
			printf("failed to set FW2 BBM\n");
			break;
		}
#endif
	} while (0);

	flashdrv_dmadestroy(dma);
}


static unsigned int flashdrv_checkErased(const void *buff, size_t boffs, size_t blen)
{
	const uint8_t *buff8 = buff;
	const uint32_t *buff32 = buff;
	unsigned int ret = 0;
	uint32_t data32;
	uint8_t data8;

	buff8 += boffs / CHAR_BIT;
	boffs %= CHAR_BIT;

	/* Check first byte */
	if (boffs > 0) {
		data8 = *buff8++;
		data8 |= (uint8_t)(0xff << (CHAR_BIT - boffs));

		/* Is it also last byte? */
		if (boffs + blen < CHAR_BIT) {
			data8 |= (uint8_t)(0xff >> (boffs + blen));
			blen = 0;
		}
		else {
			blen -= CHAR_BIT - boffs;
		}

		ret += CHAR_BIT - __builtin_popcount(data8);
	}

	/* Check bytes until 32-bit aligned address */
	while ((blen > CHAR_BIT) && (((uintptr_t)buff8) % sizeof(data32))) {
		data8 = *buff8++;
		blen -= CHAR_BIT;
		ret += CHAR_BIT - __builtin_popcount(data8);
	}

	/* Check 32-bit words */
	buff32 = (const uint32_t *)buff8;
	while (blen > CHAR_BIT * sizeof(data32)) {
		data32 = *buff32++;
		blen -= CHAR_BIT * sizeof(data32);

		if (data32 == 0xffffffff) {
			continue;
		}
		ret += CHAR_BIT * sizeof(data32) - __builtin_popcount(data32);
	}

	/* Check rest of the bytes */
	buff8 = (const uint8_t *)buff32;
	while (blen > CHAR_BIT) {
		data8 = *buff8++;
		blen -= CHAR_BIT;
		ret += CHAR_BIT - __builtin_popcount(data8);
	}

	/* Check last byte */
	if (blen > 0) {
		data8 = *buff8;
		data8 |= (uint8_t)(0xff >> blen);
		ret += CHAR_BIT - __builtin_popcount(data8);
	}

	return ret;
}


/* Checks ECC corrections reported by the BCH module */
void test_ecc(void)
{
	const unsigned int paddr = rand() % (TOTAL_BLOCKS_CNT * BLOCK_PAGES_CNT); /* Page under testing */
	const unsigned int rawmetasz = 16 + 26;                                   /* 16B META + 26B ECC16 */
	const unsigned int rawdatasz = 512 + 23;                                  /* 512B DATA + 22.75B ECC14 (adding 2 bits for byte alignment) */
	const unsigned int maxmetaecc = 16;                                       /* Max number of bits per meta chunk ECC can correct */
	const unsigned int maxdataecc = 14;                                       /* Max number of bits per data chunk ECC can correct */
	const unsigned int thresecc = 10;                                         /* Min number of bitflips for page rewrite */
	const size_t mlen = rawmetasz * CHAR_BIT;                                 /* Raw metadata chunk size in bits */
	const size_t dlen = rawdatasz * CHAR_BIT - 2;                             /* Raw data chunk size in bits */
	const unsigned int nblock = paddr / BLOCK_PAGES_CNT;
	const unsigned int start = (paddr % BLOCK_PAGES_CNT) * FLASHDRV_PAGESZ;
	unsigned int offs, flips, maxflips = 0;
	uint8_t *data, *raw = NULL;
	flashdrv_meta_t *aux;
	flashdrv_dma_t *dma;
	size_t boffs, blen;
	int i, err;

	if ((dma = flashdrv_dmanew()) == MAP_FAILED) {
		printf("dmanew() failed, err: %d\n", -ENOMEM);
		return;
	}

	if ((data = mmap(NULL, pagemapsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0)) == MAP_FAILED) {
		printf("mmap() failed: %d\n", -ENOMEM);
		flashdrv_dmadestroy(dma);
		return;
	}
	aux = (flashdrv_meta_t *)(data + _PAGE_SIZE);

	do {
		printf("Checking programmed page bitflips handling\n");

		/* Erase the block */
		if ((err = flashdrv_erase(dma, paddr)) < 0) {
			printf("erase() failed: %d\n", err);
			break;
		}

		/* Program the page */
		for (i = 0; i < flashinfo.writesz; i++)
			data[i] = i;
		memset(aux, 0xff, flashinfo.metasz);

		if ((err = flashdrv_write(dma, paddr, data, (char *)aux)) < 0) {
			printf("write() failed: %d\n", err);
			break;
		}

		/* Verify the page */
		if ((err = flashdrv_read(dma, paddr, data, aux)) < 0) {
			printf("read() failed: %d\n", err);
			break;
		}

		for (i = 0; i < flashinfo.writesz; i++) {
			if (data[i] != (uint8_t)i) {
				printf("failed to flash page data\n");
				err = -EFAULT;
				break;
			}
		}

		if (err < 0)
			break;

		for (i = 0; i < sizeof(aux->metadata); i++) {
			if (aux->metadata[i] != 0xff) {
				printf("failed to flash page metadata\n");
				err = -EFAULT;
				break;
			}
		}

		if (err < 0)
			break;

		/* Flip metadata bits (ECC should correct up to 16 bit flips) */
		printf("[%4u]: adding bit flips to metadata (%u bad bits already present)\n", paddr, aux->errors[0]);
		if ((err = flip_bits(maxmetaecc - aux->errors[0], nblock, start, start + rawmetasz, 0)) < 0)
			break;

		/* Flip data bits (ECC should correct up to 14 bit flips) */
		offs = start + rawmetasz;
		for (i = 1; i < sizeof(aux->errors); i++) {
			printf("[%4u]: adding bit flips to data chunk %u (%u bad bits already present)\n", paddr, i, aux->errors[i]);
			if ((err = flip_bits(maxdataecc - aux->errors[i], nblock, offs, offs + rawdatasz, 0)) < 0)
				break;
			offs += rawdatasz;
		}

		if (err < 0)
			break;

		/* Read modified page */
		if ((err = flashdrv_read(dma, paddr, data, aux)) < 0) {
			printf("read() failed: %d\n", err);
			break;
		}

		/* Check metadata */
		if (aux->errors[0] != maxmetaecc)
			printf("WARN: flipped present bad bit in metadata? (expected %u errors, but got %u)\n", maxmetaecc, aux->errors[0]);

		for (i = 0; i < sizeof(aux->metadata); i++) {
			if (aux->metadata[i] != 0xff) {
				printf("[%4u]: ECC failed to correct page metadata\n", paddr);
				err = -EFAULT;
				break;
			}
		}

		if (err < 0)
			break;

		/* Check data */
		offs = start + rawmetasz;
		for (i = 1; i < sizeof(aux->errors); i++) {
			if (aux->errors[i] != maxdataecc)
				printf("WARN: flipped present bad bit in data chunk %u? (expected %u errors, but got %u)\n", i, maxdataecc, aux->errors[i]);
			offs += rawdatasz;
		}

		for (i = 0; i < flashinfo.writesz; i++) {
			if (data[i] != (uint8_t)i) {
				printf("[%4u]: ECC failed to correct page data\n", paddr);
				err = -EFAULT;
				break;
			}
		}

		/* Test assumes no pre-existing bitflips in the page */
		printf("Checking erased page bitflips handling\n");

		/* Erase the block */
		err = flashdrv_erase(dma, paddr);
		if (err < 0) {
			printf("erase() failed: %d\n", err);
			break;
		}

		/* Flip metadata bits (we should correct up to 16 bit flips) */
		printf("[%4u]: adding bit flips to metadata\n", paddr);
		err = flip_bits(maxmetaecc, nblock, start, start + rawmetasz, 0);
		if (err < 0) {
			break;
		}

		/* Flip data bits (we should correct up to 14 bit flips) */
		offs = start + rawmetasz;
		for (i = 1; i < sizeof(aux->errors); i++) {
			printf("[%4u]: adding bit flips to data chunk %u\n", paddr, i);
			err = flip_bits(maxdataecc, nblock, offs, offs + rawdatasz, 0);
			if (err < 0) {
				break;
			}
			offs += rawdatasz;
		}

		/* Read the page */
		err = flashdrv_read(dma, paddr, data, aux);
		if (err < 0) {
			printf("read() failed: %d\n", err);
			break;
		}

		/* Check and correct bitflips (adapted _flashmtd_checkECC() implementation) */
		for (i = 0; i < sizeof(aux->errors); i++) {
			switch (aux->errors[i]) {
				case flash_no_errors:
				case flash_erased:
					break;

				case flash_uncorrectable:
					/* BCH reports chunk as uncorrectable in case of an erased page with bitflips within that chunk area */
					/* Check if that's the case by counting the bitflips with an assumption that the page is fully erased */

					/* Read raw page in order to check full chunk - both data and its ECC area */
					if (raw == NULL) {
						/* Map raw page buffer */
						raw = mmap(NULL, pagemapsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
						if (raw == MAP_FAILED) {
							raw = NULL;
							err = -ENOMEM;
							printf("mmap() failed: %d\n", err);
							break;
						}

						/* Read raw page */
						err = flashdrv_readraw(dma, paddr, raw, FLASHDRV_PAGESZ);
						if (err < 0) {
							printf("readraw() failed: %d\n", err);
							break;
						}
					}

					/* Metadata chunk */
					if (i == 0) {
						boffs = 0;
						blen = mlen;
					}
					/* Data chunk */
					else {
						boffs = mlen + (i - 1) * dlen;
						blen = dlen;
					}

					flips = flashdrv_checkErased(raw, boffs, blen);

					/* Nothing to do if there're no bitflips */
					if (flips == 0) {
						break;
					}

					/* Handle metadata chunk bitflips */
					if (i == 0) {
						/* Too many metadata bitflips, return error */
						if (flips > maxmetaecc) {
							err = -EBADMSG;
							printf("too many bitflips in metadata chunk: %d\n", err);
							break;
						}

						/* Correct metadata chunk */
						memset(aux, 0xff, flashinfo.oobsz);
					}
					/* Handle data chunk bitflips */
					else {
						/* Too many data bitflips, return error */
						if (flips > maxdataecc) {
							err = -EBADMSG;
							printf("too many bitflips in data chunk %d: %d\n", i, err);
							break;
						}

						/* Correct data chunk */
						memset(data + (i - 1) * 512, 0xff, 512);
					}

					/* Chunk corrected, update max number of bitflips */
					maxflips = max(flips, maxflips);
					break;

				default:
					/* Chunk corrected by BCH, update max number of bitflips */
					maxflips = max(aux->errors[i], maxflips);
					break;
			}

			if (err < 0) {
				break;
			}
		}

		if (err < 0) {
			break;
		}

		/* Verify metadata */
		for (i = 0; i < flashinfo.oobsz; i++) {
			if (aux->metadata[i] != 0xff) {
				err = -EBADMSG;
				printf("failed to correct metadata, %d\n", err);
				break;
			}
		}

		/* Verify data */
		for (i = 0; i < flashinfo.writesz; i++) {
			if (data[i] != 0xff) {
				err = -EBADMSG;
				printf("failed to correct data, %d\n", err);
				break;
			}
		}
	} while (0);

	if (err < 0) {
		printf("[%4u]: ECC test failed\n", paddr);
	}
	else {
		printf("[%4u]: ECC successfully corrected page data%s\n", paddr, (maxflips >= thresecc) ? ", page needs rewrite (dangerous number of bitflips)" : "");
	}

	if (raw != NULL) {
		munmap(raw, pagemapsz);
	}
	munmap(data, pagemapsz);
	flashdrv_dmadestroy(dma);
}


/* write predefined FCB block */
void test_write_fcb(void)
{
	flashdrv_dma_t *dma;
	uint8_t *data;
	int err;

	data = mmap(NULL, pagemapsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
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
	munmap(data, pagemapsz);
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

	data = mmap(NULL, pagemapsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	meta = data + _PAGE_SIZE;
	aux = (flashdrv_meta_t *)meta;

	if (data == MAP_FAILED)
		FAIL("failed to mmap data buffer\n");

	dma = flashdrv_dmanew();

	if ((err = flashdrv_erase(dma, paddr) < 0))
		printf("erase() failed: %d\n", err);

	/* test writing meta + data */
	memset(data, 0xff, pagemapsz);
	for (i = 0; i < sizeof(aux->metadata); ++i) {
		aux->metadata[i] = i;
	}

	if ((err = flashdrv_write(dma, paddr, data, (char *)meta)))
		printf("write() failed: %d\n", err);

	memset(data, 0x0, pagemapsz);

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

	/* test writing meta followed by data (partial page programming) */
	memset(meta, 0xff, _PAGE_SIZE);
	for (i = 0; i < sizeof(aux->metadata); ++i) {
		aux->metadata[i] = i;
	}

	if ((err = flashdrv_write(dma, paddr, NULL, (char *)meta)))
		printf("write() meta failed: %d\n", err);

	for (i = 0; i < flashinfo.writesz; ++i) {
		data[i] = i & 0xff;
	}

	if ((err = flashdrv_write(dma, paddr, data, NULL)))
		printf("write() data failed: %d\n", err);

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
		if (data[i] != (i & 0xff))
			printf("FAIL: data[%u] = 0x%02x\n", i, data[i]);
	}

	if ((err = flashdrv_erase(dma, paddr) < 0))
		printf("erase() failed: %d\n", err);

	/* test writing data followed by metadata (partial page programming) */
	for (i = 0; i < flashinfo.writesz; ++i) {
		data[i] = i & 0xff;
	}

	if ((err = flashdrv_write(dma, paddr, data, NULL)))
		printf("write() data failed: %d\n", err);

	memset(meta, 0xff, _PAGE_SIZE);
	for (i = 0; i < sizeof(aux->metadata); ++i) {
		aux->metadata[i] = i;
	}

	if ((err = flashdrv_write(dma, paddr, NULL, (char *)meta)))
		printf("write() meta failed: %d\n", err);

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
		if (data[i] != (i & 0xff))
			printf("FAIL: data[%u] = 0x%02x\n", i, data[i]);
	}

	if ((err = flashdrv_erase(dma, paddr) < 0))
		printf("erase() failed: %d\n", err);

	flashdrv_dmadestroy(dma);
	munmap(data, pagemapsz);
}


/* should be done on factory-new or completely ereased NAND flash */
void test_badblocks(void)
{
	flashdrv_dma_t *dma;
	uint8_t *data;
	unsigned int blockno;
	int err;
	int total_read_fails = 0, total_bad_blocks = 0;

	data = mmap(NULL, pagemapsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
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
	munmap(data, pagemapsz);

	printf("\n------------------\n");
	printf("total_read_fails = %d; total_bad_blocks = %d\n", total_read_fails, total_bad_blocks);
}


/* Continuously reads blocks and checks for appearing bad bits (reads few different blocks to avoid cache usage) */
void test_read_disturb(void)
{
	const unsigned int blocks[] = { rand() % TOTAL_BLOCKS_CNT, rand() % TOTAL_BLOCKS_CNT }; /* Tested blocks */
	const unsigned int nblocks = sizeof(blocks) / sizeof(blocks[0]);                        /* Number of tested blocks */
	flashdrv_dma_t *dma;
	flashdrv_meta_t *aux;
	uint8_t *data, errors[nblocks][BLOCK_PAGES_CNT][sizeof(aux->errors)];
	unsigned int i, j, k;
	int err, done;

	if ((dma = flashdrv_dmanew()) == MAP_FAILED) {
		printf("dmanew() failed, err: %d\n", -ENOMEM);
		return;
	}

	if ((data = mmap(NULL, pagemapsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0)) == MAP_FAILED) {
		printf("mmap() failed: %d\n", -ENOMEM);
		flashdrv_dmadestroy(dma);
		return;
	}
	aux = (flashdrv_meta_t *)(data + _PAGE_SIZE);
	memset(errors, 0, nblocks * BLOCK_PAGES_CNT * sizeof(aux->errors));

	/* Erase the blocks */
	for (i = 0; i < nblocks; i++) {
		if ((err = flashdrv_erase(dma, blocks[i] * BLOCK_PAGES_CNT)) < 0) {
			printf("erase() failed: %d\n", err);
			munmap(data, pagemapsz);
			flashdrv_dmadestroy(dma);
			return;
		}
	}

	/* Program the blocks */
	for (i = 0; i < flashinfo.writesz; i++)
		data[i] = i;
	memset(aux, 0xff, flashinfo.metasz);

	for (i = 0; i < nblocks; i++) {
		for (j = 0; j < BLOCK_PAGES_CNT; j++) {
			if ((err = flashdrv_write(dma, blocks[i] * BLOCK_PAGES_CNT + j, data, (char *)aux)) < 0) {
				printf("write() failed: %d\n", err);
				munmap(data, pagemapsz);
				flashdrv_dmadestroy(dma);
				return;
			}
		}
	}

	err = done = 0;
	for (i = 0; (err >= 0) && !done; i++) {
		printf("\rRead block %u time(s)...", i / nblocks + 1);

		/* Read block pages and check for errors */
		for (j = 0; (j < BLOCK_PAGES_CNT) && !done; j++) {
			if ((err = flashdrv_read(dma, blocks[i % nblocks] * BLOCK_PAGES_CNT + j, data, aux)) < 0) {
				printf("\rread() failed: %d\n", err);
				break;
			}

			for (k = 0; k < sizeof(aux->errors); k++) {
				switch (aux->errors[k]) {
					case flash_erased:
						break;

					case flash_uncorrectable:
						done = 1;
						break;

					default:
						if (errors[i % nblocks][j][k] != aux->errors[k]) {
							printf("\r[%4u]: %u corrections in chunk %u after %u block reads\n", blocks[i % nblocks] * BLOCK_PAGES_CNT + j, aux->errors[k], k, i / nblocks + 1);
							errors[i % nblocks][j][k] = aux->errors[k];
						}
				}
			}
		}
	}
	printf("\n");

	if (done)
		printf("[%4u]: uncorrectable number of errors appeared after %u block reads\n", blocks[i % nblocks] * BLOCK_PAGES_CNT + j, i / nblocks + 1);

	munmap(data, pagemapsz);
	flashdrv_dmadestroy(dma);
}


/* continuously erase/write/read single block */
void test_stress_one_block(void)
{
	uint8_t *data, *meta;
	flashdrv_dma_t *dma;
	int err;
	unsigned int block_no = TOTAL_BLOCKS_CNT - 1; /* use last block */
	uint32_t addr = block_no * 64;
	int i, b;

	unsigned long long corrected_errors = 0, uncorrectable_blocks = 0, failed_erase = 0, failed_write = 0, error_erased = 0;

	flashdrv_meta_t *m;

	dma = flashdrv_dmanew();

	data = mmap(NULL, pagemapsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	meta = data + _PAGE_SIZE;
	m = (flashdrv_meta_t *)meta;

	for (i = 0; i < flashinfo.writesz; ++i)
		data[i] = i;

	for (i = 0; i < flashinfo.metasz; ++i)
		meta[i] = 0xba;

	printf("reset\n");
	flashdrv_reset(dma, 0);

	for (i = 0; ; ++i) {
		printf("%d: ", i);

		if ((err = flashdrv_erase(dma, addr)))
			failed_erase++;

		if ((err = flashdrv_write(dma, addr, data, (char *)meta)))
			failed_write++;

		if ((err = flashdrv_read(dma, addr, data, m))) {
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
		printf("[%4u] writeraw(0x%02x) failed: %d\n", blockno, byte, err);
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

/* write, read and check */
void _write_and_check(flashdrv_dma_t *dma, uint32_t blockno, uint8_t *data, uint8_t *meta, uint8_t byte)
{
	uint32_t addr = blockno * 64;
	unsigned int i;
	int err;
	flashdrv_meta_t *aux = (flashdrv_meta_t *)meta;

	memset(data, byte, pagemapsz);
	for (i = 0; i < sizeof(aux->metadata); ++i) {
		aux->metadata[i] = i;
	}

	if ((err = flashdrv_write(dma, addr, data, (char *)meta)) < 0)
		printf("[%4u] write(0x%02x) failed: %d\n", blockno, byte, err);

	memset(data, 0, pagemapsz);
	if ((err = flashdrv_read(dma, addr, data, aux)) < 0)
		printf("[%4u] read(0x%02x) failed: %d\n", blockno, byte, err);

	for (i = 0; i < sizeof(aux->metadata); i++) {
		if (aux->metadata[i] != i)
			printf("[%4u] meta[%2u] invalid data 0x%02x\n", blockno, i, aux->metadata[i]);
	}

	for (i = 0; i < sizeof(aux->errors); i++) {
		switch (aux->errors[i]) {
			case flash_no_errors:
			case flash_erased:
				break;

			case flash_uncorrectable:
				printf("[%4u] uncorrectable data in chunk %u\n", blockno, i);
				break;

			default:
				printf("[%4u] %u corrections in chunk %u\n", blockno, aux->errors[i], i);
		}
	}

	for (i = 0; i < flashinfo.writesz; ++i) {
		if (data[i] != byte)
			printf("[%4u] write/read(0x%02x)[%u] invalid data: 0x%02x\n", blockno, byte, i, data[i]);
	}
}

void test_single_block(flashdrv_dma_t *dma, uint32_t blockno, uint8_t *data, uint8_t *meta)
{
	int err;
	uint32_t addr = blockno * 64;

	if (flashdrv_isbad(dma, addr) != 0) {
		printf("[%4u] bad block\n", blockno);
		return;
	}

	if ((err = flashdrv_erase(dma, addr)) < 0) {
		printf("[%4u] erase(1) failed: %d\n", blockno, err);
		return;
	}

	_write_and_check(dma, blockno, data, meta, 0x55);

	if ((err = flashdrv_erase(dma, addr) < 0))
		printf("[%4u] erase(2) failed: %d\n", blockno, err);

	_write_and_check(dma, blockno, data, meta, 0xAA);

	if ((err = flashdrv_erase(dma, addr) < 0))
		printf("[%4u] erase(3) failed: %d\n", blockno, err);
}

/* test writes with 0x55 and 0xAA pattarn (+ read, erase) */
void test_write_read_erase(void)
{
	flashdrv_dma_t *dma;
	uint8_t *data, *meta;
	unsigned int blockno;

	data = mmap(NULL, pagemapsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (data == MAP_FAILED)
		FAIL("failed to mmap data buffers\n");

	meta = data + _PAGE_SIZE;
	dma = flashdrv_dmanew();

	for (blockno = 0; blockno < TOTAL_BLOCKS_CNT; ++blockno)
		test_single_block(dma, blockno, data, meta);

	flashdrv_dmadestroy(dma);
	munmap(data, pagemapsz);
}

/* test raw writes with 0x55 and 0xAA pattarn (+ raw read, erase) */
void test_write_read_erase_raw(void)
{
	flashdrv_dma_t *dma;
	uint8_t *data, *meta;
	unsigned int blockno;

	data = mmap(NULL, pagemapsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);
	if (data == MAP_FAILED)
		FAIL("failed to mmap data buffers\n");

	meta = data + _PAGE_SIZE;
	dma = flashdrv_dmanew();

	/* check every erease block (read first page metadata as RAW to omit ECC checks) */
	for (blockno = 0; blockno < TOTAL_BLOCKS_CNT; ++blockno) {
		//printf("\rblock %4u\n", blockno);
		test_single_block_raw(dma, blockno, data, meta);
	}

	flashdrv_dmadestroy(dma);
	munmap(data, pagemapsz);

	printf("\n------------------\n");
}


/* Erase whole flash, write test pattern on block zero and check if it appeared in any other block */
void test_memory_aliasing(void)
{
	void *buffer = mmap(NULL, 16 * _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0x900000);
	flashdrv_dma_t *dma;
	int err;
	size_t pagesPerBlock = (flashinfo.erasesz / flashinfo.writesz);

	if (buffer == MAP_FAILED) {
		printf("test_memory_aliasing: mmap fail\n");
		return;
	}

	memset(buffer, 0, 16 * _PAGE_SIZE);

	for (int i = 0; i < 0x1000; ++i) {
		((char *)buffer)[i] = 0xb2;
		((char *)buffer)[0x1000 + i] = 0x8a;
	}

	printf("test_memory_aliasing: creating\n");

	dma = flashdrv_dmanew();
	if (dma == MAP_FAILED) {
		printf("test_memory_aliasing: dmanew error\n");
		return;
	}

	printf("test_memory_aliasing: chip erase\n");
	for (size_t i = 0; i < flashinfo.size / flashinfo.erasesz; ++i) {
		err = flashdrv_isbad(dma, i * flashinfo.erasesz / flashinfo.writesz);
		if (err != 0) {
			if (err < 0) {
				printf("test_memory_aliasing: isbad err %d\n", err);
				return;
			}
			continue;
		}

		err = flashdrv_erase(dma, i * flashinfo.erasesz / flashinfo.writesz);
		if (err < 0) {
			printf("test_memory_aliasing: erase err %d\n", err);
			return;
		}
	}

	/* Block 0 must be good */
	err = flashdrv_isbad(dma, 0);
	if (err != 0) {
		printf("test_memory_aliasing: block 0 not good err %d\n", err);
		return;
	}

	printf("test_memory_aliasing: writing test pattern\n");
	err = flashdrv_write(dma, 0, buffer, buffer + 0x1000);
	printf("test_memory_aliasing: write err %d\n", err);

	printf("test_memory_aliasing: test pattern verify\n");
	err = flashdrv_read(dma, 0, buffer + 0x2000, buffer + 0x3000);
	printf("test_memory_aliasing: error %d\n", err);

	if (memcmp(buffer, buffer + 0x2000, flashinfo.writesz) != 0) {
		printf("test_memory_aliasing: verify failed\n");
		return;
	}

	for (size_t i = pagesPerBlock; i < flashinfo.size / flashinfo.writesz; i += pagesPerBlock) {
		// printf("\rtest_memory_aliasing: Checking page %zu (block %zu)...", i, i / pagesPerBlock);
		err = flashdrv_isbad(dma, i);
		if (err != 0) {
			if (err < 0) {
				printf("test_memory_aliasing: isbad err %d\n", err);
				return;
			}
			continue;
		}

		(void)flashdrv_read(dma, i, buffer + 0x2000, buffer + 0x3000);

		for (size_t j = 0; j < flashinfo.writesz; ++j) {
			if (((char *)buffer)[0x2000 + j] == ((char *)buffer)[j]) {
				printf("test_memory_aliasing: Found unexpected data at %zu+%zu\n", i, j);
				return;
			}
		}
	}

	/* Restore flash to empty state */
	err = flashdrv_erase(dma, 0);
	if (err < 0) {
		printf("test_memory_aliasing: erase err %d\n", err);
		return;
	}

	printf("\ntest_memory_aliasing: No aliasing detected\n");
}


void test_3(void)
{
	void *buffer = mmap(NULL, 16 * _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0x900000);
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
	flashdrv_reset(dma, 0);

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
	assert(flashinfo.writesz <= _PAGE_SIZE);
	pagemapsz = (FLASHDRV_PAGESZ + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1);

	printf("%s: %s: size: %llu, writesz: %u, chips: %u\n",
		argv[0], flashinfo.name, flashinfo.size, flashinfo.writesz, flashinfo.chips);

	//test_meta();
	//test_write_fcb();
	//test_badblocks();
	test_memory_aliasing();
	test_write_read_erase();
	//test_write_read_erase_raw();
	//test_stress_one_block();
	//test_flashsrv("/dev/flash0");
	//test_flashsrv("/dev/flash1");
	//test_flashsrv( "/dev/flash2");
	//test_flashsrv( "/dev/flash3");
	//test_flashsrv( "/dev/flashsrv");
	//test_3();
	//test_ecc();
	//test_read_disturb();
	//test_bootrom();
	printf("%s: tests finished\n", argv[0]);
	usleep(3 * 1000 * 1000);
	reboot(PHOENIX_REBOOT_MAGIC);

	return 0;
}
