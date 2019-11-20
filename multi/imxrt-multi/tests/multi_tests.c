/*
 * Phoenix-RTOS
 *
 * i.MX RT Multi-driver's tests
 *
 * Copyright 2019 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <sys/msg.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>


#define SPI_TESTS


#define TEST_CATEGORY(category)                                         \
	do {                                                                \
		printf("\n\n##        %d. %s :", ++categoryCounter, category);  \
		testCounter = 1;                                                \
	} while(0)


#define TEST_CASE(test)                                                                    \
	do {                                                                                   \
		if ((test) == EOK) {                                                               \
			printf("\nTEST CASE %d.%d : %-45s\t", categoryCounter, testCounter++, #test);  \
			printf("\033[0;32m");                                                          \
			printf("-- PASSED"); }                                                         \
		else {                                                                             \
			printf("\033[1;31m");                                                          \
			printf("\nTEST CASE %d.%d : %-45s\t", categoryCounter, testCounter++, #test);  \
			printf("-- FAILED");                                                           \
		}									                                               \
		printf("\033[0m");																   \
	} while (0)


static int testCounter = 1;

static int categoryCounter = 0;

extern int test_spi_transfer_max_frame(void);

extern int test_spi_transfer_middle_data_sz(void);

extern int test_spi_transfer_overfilled_frame(void);

extern int test_spi_transfer_partially_filled_fifo(void);

extern int test_spi_multiple_transmission(void);


int main(int argc, char **argv)
{

#ifdef SPI_TESTS
	TEST_CATEGORY("SPI TESTS");

	TEST_CASE(test_spi_transfer_max_frame());
	TEST_CASE(test_spi_transfer_middle_data_sz());
	TEST_CASE(test_spi_transfer_partially_filled_fifo());
	TEST_CASE(test_spi_transfer_overfilled_frame());
	TEST_CASE(test_spi_multiple_transmission());
#endif

	return 0;
}
