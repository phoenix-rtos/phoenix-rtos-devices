/*
 * Phoenix-RTOS
 *
 * GRLIB SPIMCTRL Flash server test
 *
 * Copyright 2026 Phoenix Systems
 * Author: Amelia Waszkowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include "test_helpers.h"


#if TRIPLE_REDUNDANCY_MODE
static const char *partitions[] = {
    PARTITION_1,
    PARTITION_2,
    PARTITION_3
};
#endif /* TRIPLE_REDUNDANCY_MODE */

#define NUM_PARTITIONS (sizeof(partitions) / sizeof(partitions[0]))


/* -------------------------------------------------------------------------
 * Test Cases
 * ------------------------------------------------------------------------- */


int test_flashsrv_verifyPartitionTable(void)
{
    oid_t oid;
    long long flashSize = 0;

    while (lookup("/dev/mtd0", NULL, &oid) < 0) {
    usleep(10000);
    }

    const char *part_path = "dev/mtd0";
    LOG_ERROR("LOOKUP %s -> port=%u, id=%ju", part_path, oid.port, (uintmax_t)oid.id);

    if (getAttrFlash(oid, atSize, &flashSize) < 0 || flashSize <= 0) {
        LOG_ERROR("Failed to query flash size");
        return -1;
    }
    LOG_ERROR("Flash size: 0x%llx (%lld MB)", flashSize, flashSize / (1024 * 1024));

    const size_t erasesz = 0x10000;
    off_t offs = (off_t)flashSize - erasesz;

    uint32_t count = 0;
    if (readFromFlash(oid, offs, &count, sizeof(count)) != sizeof(count)) {
        LOG_ERROR("Failed to read partition count");
        return -1;
    }
    count = le32toh(count);

    uint32_t pSize = ptable_size(count);
    if (pSize > erasesz) {
        LOG_ERROR("Invalid partition table size: %u", pSize);
        return -1;
    }

    uint8_t magic[sizeof(ptable_magic)];
    if (readFromFlash(oid, offs + pSize - sizeof(magic), magic, sizeof(magic)) != sizeof(magic)) {
        LOG_ERROR("Failed to read partition magic signature");
        return -1;
    }

    if (memcmp(magic, ptable_magic, sizeof(magic)) != 0) {
        LOG_ERROR("Partition table magic mismatch");
        return -1;
    }

    ptable_t *ptable = malloc(pSize);
    if (ptable == NULL) {
        return -ENOMEM;
    }

    if (readFromFlash(oid, offs, ptable, pSize) != (int)pSize) {
        LOG_ERROR("Failed to read full partition table");
        free(ptable);
        return -1;
    }

    if (ptable_deserialize(ptable, flashSize, erasesz) < 0) {
        LOG_ERROR("Failed to deserialize partition table");
        free(ptable);
        return -1;
    }

    free(ptable);
    return EOK;
}


/* Test mtOpen and mtClose msg handling */
int test_flashsrv_openClose(void)
{
    oid_t oid;

    while (lookup(DEFAULT_PARTITION, NULL, &oid) < 0) {
        usleep(10000);
    }

    if (sendOpenCloseMsg(oid, mtOpen) != 0) {
        return -1;
    }

    if (sendOpenCloseMsg(oid, mtClose) != 0) {
        return -1;
    }

    /* Test invalid OID lookup/id */
    oid_t invalidOid = oid;
    invalidOid.id = 99999;
    if (sendOpenCloseMsg(invalidOid, mtOpen) != -EINVAL) {
        return -1;
    }

    return EOK;
}


/* Test mtGetAttr with atSize */
int test_flashsrv_getAttrSize(void)
{
    oid_t oid;
    long long size = 0;

    while (lookup(DEFAULT_PARTITION, NULL, &oid) < 0) {
        usleep(10000);
    }

    if (getAttrFlash(oid, atSize, &size) < 0) {
        return -1;
    }

    if (size <= 0) {
        return -1;
    }

    return EOK;
}


/* Test mtGetAttr with invalid attribute type */
int test_flashsrv_getAttrInvalidType(void)
{
    oid_t oid;
    long long val = 0;

    while (lookup(DEFAULT_PARTITION, NULL, &oid) < 0) {
        usleep(10000);
    }

    /* Passing non-existent attribute type */
    if (getAttrFlash(oid, 0x7FFF, &val) != -EINVAL) {
        return -1;
    }

    return EOK;
}


int test_flashsrv_writeAndReadPage(void)
{
    oid_t oid;
    const off_t addr = 0x50000;
    const size_t size = 0x100;
    const size_t sectorSize = 0x10000;
    const uint8_t checkValue = 0x12;

    #if TRIPLE_REDUNDANCY_MODE
        for (size_t i = 0; i < NUM_PARTITIONS; i++) {
            const char *part_path = partitions[i];

            if(erase_write_read_print(oid, addr, size, sectorSize, checkValue, part_path) < 0) {
                return -1;
            }
        }
    #else
        const char *part_path = DEFAULT_PARTITION;
        if(erase_write_read_print(oid, addr, size, sectorSize, checkValue, part_path) < 0) {
            return -1;
        }
    #endif /* TRIPLE_REDUNDANCY_MODE */

    return EOK;
}


/* Test mtWrite -> mtSync -> mtRead with unaligned offsets */
int test_flashsrv_writeAndReadUnaligned(void)
{
    oid_t oid;
    const off_t addr = 0x2050;
    const size_t size = 0x180;
    const size_t sectorSize = 0x10000;
    const uint8_t checkValue = 0x3C;

    #if TRIPLE_REDUNDANCY_MODE
        for (size_t k = 0; k < NUM_PARTITIONS; k++) {
            const char *part_path = partitions[k];

            if(erase_write_read_print(oid, addr, size, sectorSize, checkValue, part_path) < 0) {
                return -1;
            }
        }
    #else
        const char *part_path = DEFAULT_PARTITION;

        if(erase_write_read_print(oid, addr, size, sectorSize, checkValue, part_path) < 0) {
            return -1;
        }
    #endif /* TRIPLE_REDUNDANCY_MODE */

    return EOK;
}


int test_flashsrv_eraseVerification(void)
{
    oid_t oid;
    const off_t addr = 0x30000;
    const size_t sectorSize = 0x10000;
    uint8_t buff[256];

    #if TRIPLE_REDUNDANCY_MODE
        for (size_t k = 0; k < NUM_PARTITIONS; k++) {
            const char *part_path = partitions[k];

            while (lookup(part_path, NULL, &oid) < 0) {
                usleep(10000);
            }

            LOG_ERROR("LOOKUP %s -> port=%u, id=%ju", part_path, oid.port, (uintmax_t)oid.id);
            LOG_ERROR("=== TEST PARTITION [%zu/%zu]: %s ===", k + 1, NUM_PARTITIONS, part_path);

            long long partSize = 0;
            if (getAttrFlash(oid, atSize, &partSize) < 0 || partSize <= 0) {
                LOG_ERROR("Failed to get partition size for %s!", part_path);
                return -1;
            }

            if (eraseSector(oid, addr, sectorSize) < 0) {
                LOG_ERROR("Failed to erase sector at 0x%lx on %s", (unsigned long)addr, part_path);
                return -1;
            }

            memset(buff, 0, sizeof(buff));
            if (readFromFlash(oid, addr, buff, sizeof(buff)) != (int)sizeof(buff)) {
                LOG_ERROR("Failed to read erased sector on %s", part_path);
                return -1;
            }

            for (size_t i = 0; i < sizeof(buff); ++i) {
                if (buff[i] != 0xFF) {
                    LOG_ERROR("Erase check failed on %s at 0x%lx: exp 0xFF, got 0x%02x",
                              part_path, (unsigned long)(addr + i), buff[i]);
                    return -1;
                }
            }
        }
    #else
        const char *part_path = DEFAULT_PARTITION;

        while (lookup(part_path, NULL, &oid) < 0) {
            usleep(10000);
        }

        LOG_ERROR("LOOKUP %s -> port=%u, id=%ju", part_path, oid.port, (uintmax_t)oid.id);

        long long partSize = 0;
        if (getAttrFlash(oid, atSize, &partSize) < 0 || partSize <= 0) {
            LOG_ERROR("Failed to get partition size for %s!", part_path);
            return -1;
        }

        if (eraseSector(oid, addr, sectorSize) < 0) {
            LOG_ERROR("Failed to erase sector at 0x%lx on %s", (unsigned long)addr, part_path);
            return -1;
        }

        memset(buff, 0, sizeof(buff));
        if (readFromFlash(oid, addr, buff, sizeof(buff)) != (int)sizeof(buff)) {
            LOG_ERROR("Failed to read erased sector on %s", part_path);
            return -1;
        }

        for (size_t i = 0; i < sizeof(buff); ++i) {
            if (buff[i] != 0xFF) {
                LOG_ERROR("Erase check failed on %s at 0x%lx: exp 0xFF, got 0x%02x",
                          part_path, (unsigned long)(addr + i), buff[i]);
                return -1;
            }
        }
    #endif /* TRIPLE_REDUNDANCY_MODE */

    return EOK;
}


int test_flashsrv_erasePartition(void)
{
    oid_t oid;
    const char *part_path = PARTITION_2;

    LOG_ERROR("Starting erasePartition test on %s...", part_path);

    if (lookup(part_path, NULL, &oid) < 0) {
        LOG_ERROR("Lookup failed for %s", part_path);
        return -1;
    }

    /* Użycie nowej funkcji pomocniczej do czyszczenia partycji */
    if (erasePartition(oid) < 0) {
        LOG_ERROR("erasePartition failed on %s", part_path);
        return -1;
    }

    long long partSize = 0;
    if (getAttrFlash(oid, atSize, &partSize) < 0 || partSize <= 0) {
        LOG_ERROR("Failed to get partition size for %s!", part_path);
        return -1;
    }

    const size_t sectorSize = 0x10000;
    uint8_t *sectorBuff = malloc(sectorSize);
    if (sectorBuff == NULL) {
        LOG_ERROR("Failed to allocate sector buffer");
        return -ENOMEM;
    }

    int badSectorsCount = 0;

    for (off_t offs = 0; offs < (off_t)partSize; offs += sectorSize) {
        size_t rem = (size_t)(partSize - offs);
        size_t len = (rem < sectorSize) ? rem : sectorSize;

        if (readFromFlash(oid, offs, sectorBuff, len) != (ssize_t)len) {
            LOG_ERROR("readFromFlash failed on %s at offset 0x%lx", part_path, (unsigned long)offs);
            free(sectorBuff);
            return -1;
        }

        for (size_t i = 0; i < len; ++i) {
            if (sectorBuff[i] != 0xFF) {
                badSectorsCount++;
                LOG_ERROR("Erase check failed in sector 0x%lx (first error at 0x%lx, got 0x%02x, expected 0xFF)",
                          (unsigned long)offs, (unsigned long)(offs + i), sectorBuff[i]);
                break;
            }
        }
    }

    free(sectorBuff);

    if (badSectorsCount > 0) {
        LOG_ERROR("erasePartition test FAILED on %s: %d bad sectors found.", part_path, badSectorsCount);
        return -1;
    }

    LOG_ERROR("erasePartition test PASSED on %s", part_path);
    return EOK;
}


int test_flashsrv_writeCrossPageBoundary(void)
{
    oid_t oid;
    const off_t addr = 0x10080;
    const size_t size = 0x100;  
    const size_t sectorSize = 0x10000;
    const uint8_t checkValue = 0x24;

    #if TRIPLE_REDUNDANCY_MODE
        for (size_t k = 0; k < NUM_PARTITIONS; k++) {
            const char *part_path = partitions[k];

            if(erase_write_read_print(oid, addr, size, sectorSize, checkValue, part_path) < 0) {
                return -1;
            }
        }
    #else
        const char *part_path = DEFAULT_PARTITION;

        if(erase_write_read_print(oid, addr, size, sectorSize, checkValue, part_path) < 0) {
            return -1;
        }
    #endif /* TRIPLE_REDUNDANCY_MODE */

    return EOK;
}


/* Test: Address mode check */
int test_flashsrv_highAddressBoundary(void)
{
    oid_t oid;
    const off_t addr = 0x01000080; /* > 16 MB */
    const size_t size = 0x80;
    const size_t sectorSize = 0x10000;
    const uint8_t checkValue = 0x2E;

    #if TRIPLE_REDUNDANCY_MODE
        for (size_t k = 0; k < NUM_PARTITIONS; k++) {
            const char *part_path = partitions[k];

            if(erase_write_read_print(oid, addr, size, sectorSize, checkValue, part_path) < 0) {
                return -1;
            }
        }
    #else
        const char *part_path = DEFAULT_PARTITION;

        if(erase_write_read_print(oid, addr, size, sectorSize, checkValue, part_path) < 0) {
            return -1;
        }
    #endif /* TRIPLE_REDUNDANCY_MODE */

    return EOK;
}


/* Test flashsrv_read and flashsrv_write validation (-EINVAL checks) */
int test_flashsrv_invalidOffsetBounds(void)
{
    oid_t oid;
    long long flashSize = 0;
    uint8_t dummy[16] = { 0 };

    while (lookup(DEFAULT_PARTITION, NULL, &oid) < 0) {
        usleep(10000);
    }

    if (getAttrFlash(oid, atSize, &flashSize) < 0) {
        return -1;
    }

    /* 1. Negative offset */
    if (readFromFlash(oid, -50, dummy, sizeof(dummy)) != -EINVAL) {
        return -1;
    }

    /* 2. (offs + size) > strg->size */
    if (readFromFlash(oid, flashSize - 8, dummy, 16) != -EINVAL) {
        return -1;
    }

    /* 3. Write past bounds */
    if (writeToFlash(oid, flashSize + 0x10, dummy, sizeof(dummy)) != -EINVAL) {
        return -1;
    }

    /* 4. Zero size read/write should return 0 */
    if (readFromFlash(oid, 0x100, dummy, 0) != 0) {
        return -1;
    }

    return EOK;
}


/* Test unknown message type handling in msgHandler (should return -ENOSYS) */
int test_flashsrv_unsupportedMsgType(void)
{
    oid_t oid;
    msg_t msg;
    memset(&msg, 0, sizeof(msg));

    while (lookup(DEFAULT_PARTITION, NULL, &oid) < 0) {
        usleep(10000);
    }

    msg.type = 0xFFFF; /* Unknown msg type */
    msg.oid = oid;

    if (msgSend(oid.port, &msg) != 0) {
        return -1;
    }

    if (msg.o.err != -ENOSYS) {
        return -1;
    }

    return EOK;
}


int test_setSPIMode(void)
{
    oid_t oid;
    const off_t testAddr = 0x20000;
    const size_t testSize = 0x100;
    const size_t sectorSize = 0x10000;
    const uint8_t checkPattern = 0x6C;

    const char *part_path = DEFAULT_PARTITION;

    while (lookup(DEFAULT_PARTITION, NULL, &oid) < 0) {
    usleep(10000);
    }

    /* Switch SPI - i = 0 is default SPI, i = 1 is DOUT */

    for (int i = 2; i < SPI_MAX; i++) {
        if (setSPI(oid, i) < 0) {
            LOG_ERROR("setSPI failed");
            return -1;
        }

        if(erase_write_read_print(oid, testAddr, testSize, sectorSize, checkPattern, part_path) < 0) {
            return -1;
        }
    }

    // uint64_t t0 = get_time_us();
    // readFromFlash(oid, 0x10000, big_buffer, 1024 * 1024); // 1 MB
    // uint64_t t1 = get_time_us();

    // LOG_INFO("Read time: %llu us, Speed: %f MB/s", (t1 - t0), (1.0 / ((t1 - t0) / 1000000.0)));

    return EOK;
}


int test_setSPIModeDifferentPartition(void)
{
    oid_t oid;
    const off_t testAddr = 0x20000;
    const size_t testSize = 0x100;
    const size_t sectorSize = 0x10000;
    const uint8_t checkPattern = 0x15;

    while (lookup(PARTITION_2, NULL, &oid) < 0) {
        usleep(10000);
    }

    if (setSPI(oid, QSPI) < 0) {
        LOG_ERROR("setSPI failed");
        return -1;
    }

    const char *part_path = PARTITION_3;

    if(erase_write_read_print(oid, testAddr, testSize, sectorSize, checkPattern, part_path) < 0) {
        return -1;
    }

    return EOK;
}
