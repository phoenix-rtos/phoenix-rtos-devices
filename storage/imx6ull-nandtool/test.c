#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/mman.h>

#include "../imx6ull-flash/flashdrv.h"

#include "test.h"


int do_test(void *arg)
{
	flashdrv_dma_t *dma;
	unsigned block, page;
	int err = 0, i;
	char *wbuf, *rbuf;
	flashdrv_meta_t *meta;

	unsigned fail_erase = 0, fail_write = 0, mismatch = 0, errors = 0, uncorrectable = 0, erased = 0;

	if ((wbuf = mmap(NULL, SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0)) == NULL)
		return 1;

	if ((rbuf = mmap(NULL, SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0)) == NULL)
		return 1;

	if ((meta = mmap(NULL, SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0)) == NULL)
		return 1;

	for (i = 0; i < SIZE_PAGE; ++i)
		wbuf[i] = i;

	dma = flashdrv_dmanew();

	for (block = 0; block < 2 * 4096; ++block) {
		printf("\r0: testing block %6d", block);

		if ((err = flashdrv_erase(dma, 64 * block))) {
			printf(" erase failed on block %d\n", block);
			fail_erase++;
			continue;
		}

		for (page = 64 * block; page < 64 * (block + 1); ++page) {
			if ((err = flashdrv_write(dma, page, wbuf, (char *)meta))) {
				printf(" write failed on page %d\n", page);
				fail_write++;
				continue;
			}

			memset(rbuf, 0, SIZE_PAGE);

			flashdrv_read(dma, page, rbuf, meta);

			if (memcmp(wbuf, rbuf, SIZE_PAGE)) {
				printf(" r/w data mismatch on page %d\n", page);
				mismatch++;
			}

			for (i = 0; i < 9; ++i) {
				if (meta->errors[i] == flash_uncorrectable) {
					printf(" %d ECC block on page %d uncorrectable\n", i, page);
					uncorrectable++;
				}
				else if (meta->errors[i] == flash_erased) {
					printf(" %d ECC block on page %d erased\n", i, page);
					erased++;
				}
				else if (meta->errors[i] != flash_no_errors) {
					printf(" %d errors on %d ECC block, page %d\n", meta->errors[i], i, page);
					errors++;
				}
			}
		}
	}

	printf("\n");
	printf("0 summary: %u erase, %u write errors\n", fail_erase, fail_write);
	printf("           %u bit errors, %u uncorrectable ECC blocks, %u erased blocks\n", errors, uncorrectable, erased);
	printf("           %u pages with data mismatch errors\n", mismatch);

	flashdrv_dmadestroy(dma);
	return 0;
}


void init_tests(void)
{
	flashdrv_dma_t *dma;

	flashdrv_init();
	dma = flashdrv_dmanew();
	flashdrv_reset(dma);
	flashdrv_dmadestroy(dma);

	test_cnt = 1;
	test_func[0] = do_test;
}
