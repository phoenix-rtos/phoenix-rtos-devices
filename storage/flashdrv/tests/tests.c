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
#define TEST_PTABLE_VERIFICATION     1
#define TEST_FLASH_SERVER_OPERATIONS 1


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
    TEST_CASE(test_flashsrv_openClose());
    TEST_CASE(test_flashsrv_getAttrSize());
    TEST_CASE(test_flashsrv_getAttrInvalidType());
    TEST_CASE(test_flashsrv_writeAndReadPage());
    TEST_CASE(test_flashsrv_writeAndReadUnaligned());
    TEST_CASE(test_flashsrv_eraseVerification());
    TEST_CASE(test_flashsrv_erasePartition());
    TEST_CASE(test_flashsrv_writeCrossPageBoundary());
    TEST_CASE(test_flashsrv_highAddressBoundary());
    TEST_CASE(test_flashsrv_invalidOffsetBounds());
    TEST_CASE(test_flashsrv_unsupportedMsgType());
#endif

    return 0;
}