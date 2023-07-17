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

#ifndef _GR716_FLASH_TESTS_H_
#define _GR716_FLASH_TESTS_H_

#include <stdint.h>
#include <ptable.h>


#define EXTERNAL_FLASH_PATH "/dev/flash0"


/* Partition table uses in test cases */
static const ptable_t tHeader = { .count = 4 };
static const ptable_part_t parts[] = {
	{ .name = "raw1", .offset = 0x100000, .size = 4 * 0x1000, .type = ptable_raw },
	{ .name = "raw2", .offset = 0x110000, .size = 6 * 0x1000, .type = ptable_raw },
	{ .name = "mfs1", .offset = 0x120000, .size = 8 * 0x1000, .type = ptable_meterfs },
	{ .name = "mfs2", .offset = 0x130000, .size = 10 * 0x1000, .type = ptable_meterfs }
};


/* Auxiliary functions */
extern int write_pTable(const char *flashName);
extern void serializepTable(char *buff, const ptable_t *tHeader, const ptable_part_t *parts);

/* Flashsrv tests */
extern int test_flashsrv_getFlashProperties(void);
extern int test_flashsrv_writeAndReadFlashPage(void);
extern int test_flashsrv_writeAndReadFlash(void);
extern int test_flashsrv_eraseFlashSector(void);
extern int test_flashsrv_eraseFlash(void);
extern int test_flashsrv_eraseChip(void);

extern int test_flashsrv_verifyPartitionTable(void);

extern int test_flashsrv_rawPartProperties(void);
extern int test_flashsrv_rawWriteAndRead(void);
extern int test_flashsrv_rawPartitionErase(void);
extern int test_flashsrv_rawSectorErase(void);

extern int test_flashsrv_mfsAllocate(void);
extern int test_flashsrv_mfsAllocateWriteAndRead(void);
extern int test_flashsrv_mfsFileResize(void);


#endif
