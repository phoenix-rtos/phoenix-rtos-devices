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

#include <stdint.h>
#include <ptable.h>

#define EXTERNAL_FLASH_PATH "/dev/flash0"
#define INTERNAL_FLASH_PATH "/dev/flash1"


/* Partition table uses in test cases */
static const ptable_header_t tHeader = {.pCnt = 4};
static const ptable_partition_t parts[] = {
	{.name = "raw1", .offset = 0x1000, .size = 4 * 0x1000, .type = ptable_raw},
	{.name = "raw2", .offset = 0x10000, .size = 6 * 0x1000, .type = ptable_raw},
	{.name = "mfs1", .offset = 0x20000, .size = 4 * 0x1000, .type = ptable_meterfs},
	{.name = "mfs2", .offset = 0x30000, .size = 10 * 0x1000, .type = ptable_meterfs}
};


/* Auxiliary functions */
extern int write_pTable(const char *flashName);
extern void serializepTable(char *buff, const ptable_header_t *tHeader, const ptable_partition_t *parts);


/* Flashdrv tests */
extern int test_flashdrv_init(uint32_t addr);
extern int test_flashdrv_writeAndReadPage(uint32_t addr);
extern int test_flashdrv_writeAndReadBytes(uint32_t addr);
extern int test_flashdrv_eraseSector(uint32_t addr);
extern int test_flashdrv_iterativeRead(uint32_t addr);
extern int test_flashdrv_eraseChip(uint32_t addr);


/* Flashsrv tests */
extern int test_flashsrv_getFlashProperties(void);
extern int test_flashsrv_writeAndReadFlashPage(void);
extern int test_flashsrv_writeAndReadFlash(void);
extern int test_flashsrv_eraseFlashSector(void);
extern int test_flashsrv_eraseFlash(void);

extern int test_flashsrv_verifyPartitionTable(void);

extern int test_flashsrv_rawPartProperties(void);
extern int test_flashsrv_rawdWriteAndRead(void);
extern int test_flashsrv_rawPartitionErase(void);
extern int test_flashsrv_rawSectorErase(void);

extern int test_flashsrv_mfsAllocate(void);
extern int test_flashsrv_mfsAllocateWriteAndRead(void);
extern int test_flashsrv_mfsFileResize(void);
