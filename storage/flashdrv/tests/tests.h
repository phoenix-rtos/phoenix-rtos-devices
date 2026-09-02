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

/* Path created by flashsrv.c (#define STRG_PATH "mtd0") */
#define EXTERNAL_FLASH_PATH "/dev/mtd0"


/* Partition table verification test */
extern int test_flashsrv_verifyPartitionTable(void);

/* Flashsrv core msgHandler tests */
extern int test_flashsrv_openClose(void);
extern int test_flashsrv_getAttrSize(void);
extern int test_flashsrv_getAttrInvalidType(void);
extern int test_flashsrv_writeAndReadPage(void);
extern int test_flashsrv_writeAndReadUnaligned(void);
extern int test_flashsrv_sync(void);
extern int test_flashsrv_invalidOffsetBounds(void);
extern int test_flashsrv_unsupportedMsgType(void);

/* Partition operations */
extern int test_flashsrv_rawPartGetAttr(void);
extern int test_flashsrv_rawPartWriteAndRead(void);

/* Mount operations */
extern int test_flashsrv_mountFs(void);

#endif /* _FLASHSRV_TESTS_H_ */