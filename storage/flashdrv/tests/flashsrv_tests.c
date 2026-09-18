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

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <endian.h>

#include <sys/msg.h>
#include <sys/file.h>
#include <posix/utils.h>
#include <ptable.h>
#include <storage/storage.h>
#include <flashdrv/flashsrv.h>
#include <board_config.h>
#include <time.h>

#include "tests.h"

#if TRIPLE_REDUNDANCY_MODE
static const char *partitions[] = {
    PARTITION_1,
    PARTITION_2,
    PARTITION_3
};
#endif /* TRIPLE_REDUNDANCY_MODE */

#define NUM_PARTITIONS (sizeof(partitions) / sizeof(partitions[0]))


static inline uint64_t get_time_us(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}


/* -------------------------------------------------------------------------
 * IPC helpers sending messages directly handled by flashsrv_msgHandler
 * ------------------------------------------------------------------------- */

static int sendOpenCloseMsg(oid_t oid, int type)
{
    msg_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.type = type;
    msg.oid = oid;

    if (msgSend(oid.port, &msg) != 0) {
        LOG_ERROR("Cannot send open/close msg");
        return -1;
    }

    return msg.o.err;
}


static int writeToFlash(oid_t oid, off_t offs, const void *data, size_t size)
{
    msg_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.type = mtWrite;
    msg.oid = oid;
    msg.i.io.offs = offs;
    msg.o.data = (void *)data;
    msg.o.size = size;

    if (msgSend(oid.port, &msg) != 0) {
        LOG_ERROR("Cannot send mtWrite msg");
        return -1;
    }

    return msg.o.err;
}


static int eraseFlash(oid_t oid, uint32_t offs, size_t size)
{
    msg_t msg;

    int res = 0;

    msg.type = mtDevCtl;
    msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
    msg.oid = oid;
    msg.o.size = 0;

    flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg.i.raw;
    idevctl->type = flashdrv_devctl_eraseSector;
    idevctl->erase.addr = offs;
    idevctl->erase.size = size;

    res = msgSend(oid.port, &msg);

    if (res != 0) {
        LOG_ERROR("Cannot send mtDevCtl (eraseSector) msg %d\n", res);
        return -1;
    }

	if (msg.o.err < 0)
		LOG_ERROR("Cannot erase sector, err: (%s).", strerror(msg.o.err));

    return msg.o.err;
}


static int setSPI(oid_t oid, SPIMode_t mode)
{
    msg_t msg;

    int res = 0;

    msg.type = mtDevCtl;
    msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
    msg.oid = oid;
    msg.o.size = 0;

    flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg.i.raw;
    idevctl->type = flashdrv_devctl_SPIMode;
    idevctl->spi.mode = mode;

    res = msgSend(oid.port, &msg);

    if (res != 0) {
        LOG_ERROR("Cannot send mtDevCtl (setSPI) msg %d\n", res);
        return -1;
    }

	if (msg.o.err < 0)
		LOG_ERROR("Cannot set SPI mode, err: (%s).", strerror(msg.o.err));

    return msg.o.err;
}


static int readFromFlash(oid_t oid, off_t offs, void *data, size_t size)
{
    msg_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.type = mtRead;
    msg.oid = oid;
    msg.i.io.offs = offs;
    msg.o.data = data;
    msg.o.size = size;

    if (msgSend(oid.port, &msg) != 0) {
        LOG_ERROR("Cannot send mtRead msg");
        return -1;
    }

    return msg.o.err;
}


static int getAttrFlash(oid_t oid, int type, long long *val)
{
    msg_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.type = mtGetAttr;
    msg.oid = oid;
    msg.i.attr.type = type;

    if (msgSend(oid.port, &msg) != 0) {
        LOG_ERROR("Cannot send mtGetAttr msg");
        return -1;
    }

    if (msg.o.err == 0) {
        *val = msg.o.attr.val;
    }

    return msg.o.err;
}


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
    printf("Flash size: 0x%llx (%lld MB)\n", flashSize, flashSize / (1024 * 1024));

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
    int j;
    const off_t addr = 0x50000;
    const size_t size = 0x100;
    const size_t sectorSize = 0x10000;
    const uint8_t checkValue = 0x12;
    uint8_t buff[size];


    #if TRIPLE_REDUNDANCY_MODE
        for (size_t i = 0; i < NUM_PARTITIONS; i++) {
            const char *part_path = partitions[i];

            while (lookup(part_path, NULL, &oid) < 0) {
                usleep(10000);
            }

            lookup(part_path, NULL, &oid);
            LOG_ERROR("LOOKUP %s -> port=%u, id=%ju", part_path, oid.port, (uintmax_t)oid.id);

            LOG_ERROR("=== TEST PARTITION [%zu/%zu]: %s ===", i + 1, NUM_PARTITIONS, part_path);

            long long partSize = 0;
            if (getAttrFlash(oid, atSize, &partSize) < 0 || partSize <= 0) {
                LOG_ERROR("Failed to get partition size for %s!", part_path);
                return -1;
            }

            LOG_ERROR("Partition %s verified: oid.id=%ju, size=0x%llx",
                    part_path, (uintmax_t)oid.id, partSize);

            off_t sectorAddr = addr - (addr % sectorSize);
            if (eraseFlash(oid, sectorAddr, sectorSize) < 0) {
                LOG_ERROR("Failed to erase sector at 0x%lx on %s", (unsigned long)sectorAddr, part_path);
                return -1;
            }

            memset(buff, checkValue, size);
            if (writeToFlash(oid, addr, buff, size) != (int)size) {
                LOG_ERROR("Failed to write to %s", part_path);
                return -1;
            }

            memset(buff, 0, size);
            if (readFromFlash(oid, addr, buff, size) != (int)size) {
                LOG_ERROR("Failed to read from %s", part_path);
                return -1;
            }

            for (j = 0; j < (int)size; ++j) {
                if (buff[j] != checkValue) {
                    LOG_ERROR("Mismatch on %s at index %d: expected 0x%02x, got 0x%02x",
                            part_path, j, checkValue, buff[j]);
                    return -1;
                }
            }
        }
    #else
        const char *part_path = DEFAULT_PARTITION;

        while (lookup(part_path, NULL, &oid) < 0) {
            usleep(10000);
        }

        lookup(part_path, NULL, &oid);
        LOG_ERROR("LOOKUP %s -> port=%u, id=%ju", part_path, oid.port, (uintmax_t)oid.id);

        long long partSize = 0;
        if (getAttrFlash(oid, atSize, &partSize) < 0 || partSize <= 0) {
            LOG_ERROR("Failed to get partition size for %s!", part_path);
            return -1;
        }
        printf("Part size: 0x%llx (%lld MB)\n", partSize, partSize / (1024 * 1024));

        LOG_ERROR("Partition %s verified: oid.id=%ju, size=0x%llx",
                part_path, (uintmax_t)oid.id, partSize);

        off_t sectorAddr = addr - (addr % sectorSize);
        if (eraseFlash(oid, sectorAddr, sectorSize) < 0) {
            LOG_ERROR("Failed to erase sector at 0x%lx on %s", (unsigned long)sectorAddr, part_path);
            return -1;
        }

        memset(buff, 0, sizeof(buff));
        ssize_t readRes = readFromFlash(oid, addr, buff, sizeof(buff));
        if (readRes != (ssize_t)sizeof(buff)) {
            LOG_ERROR("readFromFlash failed on %s: expected %zu, got %zd", 
                    part_path, sizeof(buff), readRes);
            return -1;
        }

        for (size_t j = 0; j < sizeof(buff); ++j) {
            if (buff[j] != 0xFF) {
                LOG_ERROR("Erase check failed on %s at offset %zu: expected 0xFF, got 0x%02x",
                        part_path, j, buff[j]);
                return -1;
            }
        }

        memset(buff, checkValue, size);
        if (writeToFlash(oid, addr, buff, size) != (int)size) {
            LOG_ERROR("Failed to write to %s", part_path);
            return -1;
        }

        memset(buff, 0, size);
        if (readFromFlash(oid, addr, buff, size) != (int)size) {
            LOG_ERROR("Failed to read from %s", part_path);
            return -1;
        }

        for (j = 0; j < (int)size; ++j) {
            if (buff[j] != checkValue) {
                LOG_ERROR("Mismatch on %s at index %d: expected 0x%02x, got 0x%02x",
                        part_path, j, checkValue, buff[j]);
                return -1;
            }
        }
    #endif /* TRIPLE_REDUNDANCY_MODE */

    return EOK;
}


/* Test mtWrite -> mtSync -> mtRead with unaligned offsets */
int test_flashsrv_writeAndReadUnaligned(void)
{
    oid_t oid;
    int i;
    const off_t addr = 0x2050;
    const size_t size = 0x180;
    const size_t sectorSize = 0x10000;
    const uint8_t checkValue = 0x3C;
    uint8_t buff[size];

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

            memset(buff, checkValue, size);

            off_t startSector = addr - (addr % sectorSize);
            off_t endSector = (addr + size - 1) - ((addr + size - 1) % sectorSize);
            size_t eraseLen = (endSector - startSector) + sectorSize;

            if (eraseFlash(oid, startSector, eraseLen) < 0) {
                LOG_ERROR("Failed to erase sector at 0x%lx on %s", (unsigned long)startSector, part_path);
                return -1;
            }

            if (writeToFlash(oid, addr, buff, size) != (int)size) {
                LOG_ERROR("Failed to write flash on %s", part_path);
                return -1;
            }

            memset(buff, 0, size);

            if (readFromFlash(oid, addr, buff, size) != (int)size) {
                LOG_ERROR("Failed to read from flash on %s", part_path);
                return -1;
            }

            for (i = 0; i < (int)size; ++i) {
                if (buff[i] != checkValue) {
                    LOG_ERROR("Unaligned mismatch on %s at %d: exp 0x%02x, got 0x%02x", part_path, i, checkValue, buff[i]);
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

        memset(buff, checkValue, size);

        off_t startSector = addr - (addr % sectorSize);
        off_t endSector = (addr + size - 1) - ((addr + size - 1) % sectorSize);
        size_t eraseLen = (endSector - startSector) + sectorSize;

        if (eraseFlash(oid, startSector, eraseLen) < 0) {
            LOG_ERROR("Failed to erase sector at 0x%lx on %s", (unsigned long)startSector, part_path);
            return -1;
        }

        if (writeToFlash(oid, addr, buff, size) != (int)size) {
            LOG_ERROR("Failed to write flash on %s", part_path);
            return -1;
        }

        memset(buff, 0, size);

        if (readFromFlash(oid, addr, buff, size) != (int)size) {
            LOG_ERROR("Failed to read from flash on %s", part_path);
            return -1;
        }

        for (i = 0; i < (int)size; ++i) {
            if (buff[i] != checkValue) {
                LOG_ERROR("Unaligned mismatch on %s at %d: exp 0x%02x, got 0x%02x", part_path, i, checkValue, buff[i]);
                return -1;
            }
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

            if (eraseFlash(oid, addr, sectorSize) < 0) {
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

        if (eraseFlash(oid, addr, sectorSize) < 0) {
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
    msg_t msg;
    const char *part_path = PARTITION_2;

    LOG_ERROR("Starting erasePartition test on %s...", part_path);

    if (lookup(part_path, NULL, &oid) < 0) {
        LOG_ERROR("Lookup failed for %s", part_path);
        return -1;
    }

    memset(&msg, 0, sizeof(msg));
    msg.type = mtDevCtl;
    msg.oid = oid;

    flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg.i.raw;
    idevctl->type = flashdrv_devctl_erasePartition;

    if (msgSend(oid.port, &msg) != 0 || msg.o.err < 0) {
        LOG_ERROR("erasePartition failed on %s (err=%d)", part_path, msg.o.err);
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
    uint8_t txBuff[size];
    uint8_t rxBuff[size];

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

            off_t sectorAddr = addr - (addr % sectorSize);
            if (eraseFlash(oid, sectorAddr, sectorSize) < 0) {
                LOG_ERROR("Failed to erase sector at 0x%lx on %s", (unsigned long)sectorAddr, part_path);
                return -1;
            }

            for (size_t i = 0; i < size; ++i) {
                txBuff[i] = (uint8_t)(i & 0xFF);
            }

            if (writeToFlash(oid, addr, txBuff, size) != (int)size) {
                LOG_ERROR("Failed cross-page write on %s", part_path);
                return -1;
            }

            memset(rxBuff, 0, size);
            if (readFromFlash(oid, addr, rxBuff, size) != (int)size) {
                LOG_ERROR("Failed cross-page read on %s", part_path);
                return -1;
            }

            for (size_t i = 0; i < size; ++i) {
                if (rxBuff[i] != txBuff[i]) {
                    LOG_ERROR("Cross-page mismatch on %s at index %zu: exp 0x%02x, got 0x%02x",
                              part_path, i, txBuff[i], rxBuff[i]);
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

        off_t sectorAddr = addr - (addr % sectorSize);
        if (eraseFlash(oid, sectorAddr, sectorSize) < 0) {
            LOG_ERROR("Failed to erase sector at 0x%lx on %s", (unsigned long)sectorAddr, part_path);
            return -1;
        }

        for (size_t i = 0; i < size; ++i) {
            txBuff[i] = (uint8_t)(i & 0xFF);
        }

        if (writeToFlash(oid, addr, txBuff, size) != (int)size) {
            LOG_ERROR("Failed cross-page write on %s", part_path);
            return -1;
        }

        memset(rxBuff, 0, size);
        if (readFromFlash(oid, addr, rxBuff, size) != (int)size) {
            LOG_ERROR("Failed cross-page read on %s", part_path);
            return -1;
        }

        for (size_t i = 0; i < size; ++i) {
            if (rxBuff[i] != txBuff[i]) {
                LOG_ERROR("Cross-page mismatch on %s at index %zu: exp 0x%02x, got 0x%02x",
                          part_path, i, txBuff[i], rxBuff[i]);
                return -1;
            }
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
    uint8_t txBuff[size];
    uint8_t rxBuff[size];

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

            if (partSize <= addr + (off_t)size) {
                LOG_ERROR("Skipping high address test on %s: partition size (0x%llx) smaller than target addr (0x%lx)",
                          part_path, partSize, (unsigned long)addr);
                continue;
            }

            off_t sectorAddr = addr - (addr % sectorSize);
            if (eraseFlash(oid, sectorAddr, sectorSize) < 0) {
                LOG_ERROR("Failed to erase sector at 0x%lx on %s", (unsigned long)sectorAddr, part_path);
                return -1;
            }

            memset(txBuff, 0x5A, size);
            if (writeToFlash(oid, addr, txBuff, size) != (int)size) {
                LOG_ERROR("Failed write on %s", part_path);
                return -1;
            }

            memset(rxBuff, 0, size);
            if (readFromFlash(oid, addr, rxBuff, size) != (int)size) {
                LOG_ERROR("Failed read on %s", part_path);
                return -1;
            }

            if (memcmp(txBuff, rxBuff, size) != 0) {
                LOG_ERROR("Data mismatch on 32-bit address boundary test on %s", part_path);
                return -1;
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

        if (partSize > addr + (off_t)size) {
            off_t sectorAddr = addr - (addr % sectorSize);
            if (eraseFlash(oid, sectorAddr, sectorSize) < 0) {
                LOG_ERROR("Failed to erase sector at 0x%lx on %s", (unsigned long)sectorAddr, part_path);
                return -1;
            }

            memset(txBuff, 0x5A, size);
            if (writeToFlash(oid, addr, txBuff, size) != (int)size) {
                LOG_ERROR("Failed write on %s", part_path);
                return -1;
            }

            memset(rxBuff, 0, size);
            if (readFromFlash(oid, addr, rxBuff, size) != (int)size) {
                LOG_ERROR("Failed read on %s", part_path);
                return -1;
            }

            if (memcmp(txBuff, rxBuff, size) != 0) {
                LOG_ERROR("Data mismatch on 32-bit address boundary test on %s", part_path);
                return -1;
            }
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
    const uint8_t checkPattern = 0x7B;
    uint8_t txBuff[testSize];
    uint8_t rxBuff[testSize];

    while (lookup(PARTITION_2, NULL, &oid) < 0) {
    usleep(10000);
    }

    /* Switch SPI */

    // for (int i = 0; i < SPI_MAX; i++) {
    //     if (setSPI(oid, i) < 0) {
    //         LOG_ERROR("setSPI failed on /dev/mtd0");
    //         return -1;
    //     }
    // }

    if (setSPI(oid, QSPI) < 0) {
        LOG_ERROR("setSPI failed \n");
        return -1;
    }


    // uint64_t t0 = get_time_us();
    // readFromFlash(oid, 0x10000, big_buffer, 1024 * 1024); // 1 MB
    // uint64_t t1 = get_time_us();

    // LOG_INFO("Read time: %llu us, Speed: %f MB/s", (t1 - t0), (1.0 / ((t1 - t0) / 1000000.0)));

    off_t sectorAddr = testAddr - (testAddr % sectorSize);
    if (eraseFlash(oid, sectorAddr, sectorSize) < 0) {
        LOG_ERROR("Failed to erase sector at 0x%lx after setSPI", (unsigned long)sectorAddr);
        return -1;
    }

    memset(rxBuff, 0, testSize);
    if (readFromFlash(oid, testAddr, rxBuff, testSize) != (int)testSize) {
        LOG_ERROR("Failed to read flash after setSPI");
        return -1;
    }

    printf("Read %zu bytes from address 0x%lx:\n", testSize, (unsigned long)testAddr);
    for (size_t i = 0; i < testSize; i++) {
        printf("%02X ", rxBuff[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");

    memset(txBuff, checkPattern, testSize);
    if (writeToFlash(oid, testAddr, txBuff, testSize) != (int)testSize) {
        LOG_ERROR("Failed to write flash after setSPI");
        return -1;
    }

    memset(rxBuff, 0, testSize);
    if (readFromFlash(oid, testAddr, rxBuff, testSize) != (int)testSize) {
        LOG_ERROR("Failed to read flash after setSPI");
        return -1;
    }

    printf("Read %zu bytes from address 0x%lx:\n", testSize, (unsigned long)testAddr);
    for (size_t i = 0; i < testSize; i++) {
        printf("%02X ", rxBuff[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");

    if (memcmp(txBuff, rxBuff, testSize) != 0) {
        LOG_ERROR("Data verification failed after setSPI");
        return -1;
    }

    return EOK;
}


int test_setSPIModeDifferentPartition(void)
{
    oid_t oid;
    const off_t testAddr = 0x20000;
    const size_t testSize = 0x100;
    const size_t sectorSize = 0x10000;
    const uint8_t checkPattern = 0x15;
    uint8_t txBuff[testSize];
    uint8_t rxBuff[testSize];

    while (lookup(PARTITION_2, NULL, &oid) < 0) {
        usleep(10000);
    }

    if (setSPI(oid, QSPI) < 0) {
        LOG_ERROR("setSPI failed");
        return -1;
    }

    while (lookup(PARTITION_3, NULL, &oid) < 0) {
        usleep(10000);
    }

    off_t sectorAddr = testAddr - (testAddr % sectorSize);
    if (eraseFlash(oid, sectorAddr, sectorSize) < 0) {
        LOG_ERROR("Failed to erase sector at 0x%lx after setSPI", (unsigned long)sectorAddr);
        return -1;
    }

    memset(rxBuff, 0, testSize);
    if (readFromFlash(oid, testAddr, rxBuff, testSize) != (int)testSize) {
        LOG_ERROR("Failed to read flash after setSPI");
        return -1;
    }

    printf("Read %zu bytes from address 0x%lx:\n", testSize, (unsigned long)testAddr);
    for (size_t i = 0; i < testSize; i++) {
        printf("%02X ", rxBuff[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");

    memset(txBuff, checkPattern, testSize);
    if (writeToFlash(oid, testAddr, txBuff, testSize) != (int)testSize) {
        LOG_ERROR("Failed to write flash after setSPI");
        return -1;
    }

    memset(rxBuff, 0, testSize);
    if (readFromFlash(oid, testAddr, rxBuff, testSize) != (int)testSize) {
        LOG_ERROR("Failed to read flash after setSPI");
        return -1;
    }

    printf("Read %zu bytes from address 0x%lx:\n", testSize, (unsigned long)testAddr);
    for (size_t i = 0; i < testSize; i++) {
        printf("%02X ", rxBuff[i]);
        if ((i + 1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");

    if (memcmp(txBuff, rxBuff, testSize) != 0) {
        LOG_ERROR("Data verification failed after setSPI");
        return -1;
    }

    return EOK;
}
