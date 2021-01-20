/*
 * Phoenix-RTOS
 *
 * i.MX RT flash tests
 *
 * Copyright 2020 Phoenix Systems
 * Author: Hubert Buczynski
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

#include "../flashdrv.h"

/* Tests module definitions */
#define TESTS_INTERNAL_FLASH_DRIVER             0
#define TESTS_EXTERNAL_FLASH_DRVIVER            1
// #define TESTS_FLASH_SERVER_CHIP_CMD             1
// #define TESTS_FLASH_SERVER_RAW_AND_MFS_PARTS    1

#define CLEAR_FLASHES                           0


int main(int argc, char **argv)
{
	// sleep(1);

/* It is recommended to clear flash memories and write partition table to one of them before first usage */
#if CLEAR_FLASHES
	TEST_CASE(test_flashdrv_eraseChip(FLASH_INTERNAL_DATA_ADDRESS));
	TEST_CASE(test_flashdrv_eraseChip(FLASH_EXT_DATA_ADDRESS));
	TEST_CASE(write_pTable(INTERNAL_FLASH_PATH));
#endif

#if TESTS_INTERNAL_FLASH_DRIVER
	TEST_CATEGORY("FLASHDRV TESTS: internal flash");

	TEST_CASE(test_flashdrv_init(FLASH_INTERNAL_DATA_ADDRESS));
	TEST_CASE(test_flashdrv_writeAndReadPage(FLASH_INTERNAL_DATA_ADDRESS));
	TEST_CASE(test_flashdrv_writeAndReadBytes(FLASH_INTERNAL_DATA_ADDRESS));
	TEST_CASE(test_flashdrv_eraseSector(FLASH_INTERNAL_DATA_ADDRESS));
	TEST_CASE(test_flashdrv_iterativeRead(FLASH_INTERNAL_DATA_ADDRESS));
	TEST_CASE(test_flashdrv_eraseChip(FLASH_INTERNAL_DATA_ADDRESS));
#endif


#if TESTS_EXTERNAL_FLASH_DRVIVER
	TEST_CATEGORY("FLASHDRV TESTS: external flash");

	TEST_CASE(test_flashdrv_init(FLASH_EXT_DATA_ADDRESS));
	TEST_CASE(test_flashdrv_writeAndReadPage(FLASH_EXT_DATA_ADDRESS));
	TEST_CASE(test_flashdrv_writeAndReadBytes(FLASH_EXT_DATA_ADDRESS));
	TEST_CASE(test_flashdrv_eraseSector(FLASH_EXT_DATA_ADDRESS));
	TEST_CASE(test_flashdrv_iterativeRead(FLASH_EXT_DATA_ADDRESS));
	TEST_CASE(test_flashdrv_eraseChip(FLASH_EXT_DATA_ADDRESS));
#endif


/* NOTE: in order to perform flashsrv's tests, the imxrt-flashsrv has to run simultaneously. */
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
 *       and imxrt-flashsrv has to run simultaneously.
*/
#if TESTS_FLASH_SERVER_RAW_AND_MFS_PARTS
	TEST_CATEGORY("FLASH SERVER TESTS: raw and mfs partitions interface");

	TEST_CASE(test_flashsrv_verifyPartitionTable());
	TEST_CASE(test_flashsrv_rawPartProperties());
	TEST_CASE(test_flashsrv_rawdWriteAndRead());
	TEST_CASE(test_flashsrv_rawSectorErase());
	TEST_CASE(test_flashsrv_rawPartitionErase());
	TEST_CASE(test_flashsrv_mfsAllocate());
	TEST_CASE(test_flashsrv_mfsAllocateWriteAndRead());
	TEST_CASE(test_flashsrv_mfsFileResize());
#endif

	while(1)
		;

	return 0;
}
