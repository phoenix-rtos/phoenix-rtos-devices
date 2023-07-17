/*
 * Phoenix-RTOS
 *
 * GR716 flash tests
 *
 * Copyright 2020, 2023 Phoenix Systems
 * Author: Hubert Buczynski, Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <unistd.h>
#include <stdint.h>
#include <errno.h>

#include "tests.h"
#include "utils.h"

/* Tests module definitions */
#define TESTS_FLASH_SERVER_CHIP_CMD          0
#define TESTS_FLASH_SERVER_RAW_AND_MFS_PARTS 1

#define CLEAR_FLASHES 0


int main(int argc, char **argv)
{
/* It is recommended to clear flash memories and write partition table to one of them before first usage */
#if CLEAR_FLASHES
	TEST_CASE(test_flashsrv_eraseChip());
	TEST_CASE(write_pTable(EXTERNAL_FLASH_PATH));
#endif


/* NOTE: in order to perform flashsrv's tests, the gr716-flashsrv has to run simultaneously. */
#if TESTS_FLASH_SERVER_CHIP_CMD
	TEST_CATEGORY("FLASH SERVER TESTS: chip cmds interface");

	TEST_CASE(test_flashsrv_getFlashProperties());
	TEST_CASE(test_flashsrv_writeAndReadFlashPage());
	TEST_CASE(test_flashsrv_writeAndReadFlash());
	TEST_CASE(test_flashsrv_eraseFlashSector());
	TEST_CASE(test_flashsrv_eraseFlash());
#endif


/*
 * NOTE: in order to perform raw & mfs partitions tests, the partition table has to be located on flash memory
 *       and gr716-flashsrv has to run simultaneously.
 */
#if TESTS_FLASH_SERVER_RAW_AND_MFS_PARTS
	TEST_CATEGORY("FLASH SERVER TESTS: raw and mfs partitions interface");

	TEST_CASE(test_flashsrv_rawPartProperties());
	TEST_CASE(test_flashsrv_rawWriteAndRead());
	TEST_CASE(test_flashsrv_rawSectorErase());
	TEST_CASE(test_flashsrv_rawPartitionErase());
	TEST_CASE(test_flashsrv_mfsAllocate());
	TEST_CASE(test_flashsrv_mfsAllocateWriteAndRead());
	TEST_CASE(test_flashsrv_mfsFileResize());
#endif

	return 0;
}
