/*
 * Phoenix-RTOS
 *
 * GRLIB SPIMCTRL Flash Server Tests
 *
 * Copyright 2026 Phoenix Systems
 * Author: Amelia Waszkowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHSRV_TESTS_H_
#define _FLASHSRV_TESTS_H_

#include <stdint.h>
#include <ptable.h>

/* Path created by flashsrv.c*/
#define DEFAULT_PARTITION "/dev/mtd0.part2"

#define PARTITION_1 "/dev/mtd0.part1"
#define PARTITION_2 "/dev/mtd0.part2"
#define PARTITION_3 "/dev/mtd0.part3"


/* Partition table verification test */
extern int test_flashsrv_verifyPartitionTable(void);

/* Flashsrv core msgHandler tests */
extern int test_flashsrv_openClose(void);
extern int test_flashsrv_getAttrSize(void);
extern int test_flashsrv_getAttrInvalidType(void);
extern int test_flashsrv_writeAndReadPage(void);
extern int test_flashsrv_writeAndReadUnaligned(void);
extern int test_flashsrv_eraseVerification(void);
extern int test_flashsrv_erasePartition(void);
extern int test_flashsrv_writeCrossPageBoundary(void);
extern int test_flashsrv_highAddressBoundary(void);
extern int test_flashsrv_invalidOffsetBounds(void);
extern int test_flashsrv_unsupportedMsgType(void);

#endif /* _FLASHSRV_TESTS_H_ */