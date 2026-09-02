/*
 * Phoenix-RTOS
 *
 * GRLIB SPIMCTRL Flash Server Tests Runner
 *
 * Copyright 2026 Phoenix Systems
 * Author: Amelia Waszkowska
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

/* Test runner module switches */
#define TEST_PTABLE_VERIFICATION     0
#define TEST_FLASH_SERVER_OPERATIONS 1
#define TEST_PARTITION_OPERATIONS   0
#define TEST_MOUNT_OPERATIONS       0


int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

#if TEST_PTABLE_VERIFICATION
    TEST_CATEGORY("FLASH SERVER TESTS: Partition table verification");
    TEST_CASE(test_flashsrv_verifyPartitionTable());
#endif

#if TEST_FLASH_SERVER_OPERATIONS
    TEST_CATEGORY("FLASH SERVER TESTS: Core operations (mtOpen, mtRead, mtWrite, mtSync, mtGetAttr)");
    // TEST_CASE(test_flashsrv_openClose());
    // TEST_CASE(test_flashsrv_getAttrSize());
    // TEST_CASE(test_flashsrv_getAttrInvalidType());
    TEST_CASE(test_flashsrv_writeAndReadPage());
    // TEST_CASE(test_flashsrv_writeAndReadUnaligned());
    // // TEST_CASE(test_flashsrv_sync());
    // TEST_CASE(test_flashsrv_invalidOffsetBounds());
    // TEST_CASE(test_flashsrv_unsupportedMsgType());
#endif

#if TEST_PARTITION_OPERATIONS
    TEST_CATEGORY("FLASH SERVER TESTS: Raw partitions interface");
    TEST_CASE(test_flashsrv_rawPartGetAttr());
    TEST_CASE(test_flashsrv_rawPartWriteAndRead());
#endif

#if TEST_MOUNT_OPERATIONS
    TEST_CATEGORY("FLASH SERVER TESTS: Filesystem Mount Interface");
    TEST_CASE(test_flashsrv_mountFs());
#endif

    return 0;
}