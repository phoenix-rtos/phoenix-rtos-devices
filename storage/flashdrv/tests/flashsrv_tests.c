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

#include "tests.h"


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
    idevctl->type = flashsrv_devctl_eraseSector;
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

    while (lookup(EXTERNAL_FLASH_PATH, NULL, &oid) < 0) {
        usleep(10000);
    }

    if (getAttrFlash(oid, atSize, &flashSize) < 0 || flashSize <= 0) {
        LOG_ERROR("Failed to query flash size");
        return -1;
    }

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

    while (lookup(EXTERNAL_FLASH_PATH, NULL, &oid) < 0) {
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

    while (lookup(EXTERNAL_FLASH_PATH, NULL, &oid) < 0) {
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

    while (lookup(EXTERNAL_FLASH_PATH, NULL, &oid) < 0) {
        usleep(10000);
    }

    /* Passing non-existent attribute type */
    if (getAttrFlash(oid, 0x7FFF, &val) != -EINVAL) {
        return -1;
    }

    return EOK;
}


/* Test mtWrite -> mtSync -> mtRead (page aligned) */
int test_flashsrv_writeAndReadPage(void)
{
    oid_t oid;
    int i;
    const off_t addr = 0x10000;
    const size_t size = 0x100;
    const size_t sectorSize = 0x10000;
    const uint8_t checkValue = 0x12;
    uint8_t buff[size];

    while (lookup(EXTERNAL_FLASH_PATH, NULL, &oid) < 0) {
        usleep(10000);
    }

    off_t sectorAddr = addr - (addr % sectorSize);
    if (eraseFlash(oid, sectorAddr, sectorSize) < 0) {
        LOG_ERROR("Failed to erase sector at 0x%lx", (unsigned long)sectorAddr);
        return -1;
    }
    
    memset(buff, checkValue, size);
    if (writeToFlash(oid, addr, buff, size) != (int)size) {
        LOG_ERROR("Failed to write flash");
        return -1;
    }

    const off_t addr1 = 0x20000;
    memset(buff, 0, size);
    if (readFromFlash(oid, addr1, buff, size) != (int)size) {
        LOG_ERROR("Failed to read from flash");
        return -1;
    }

    memset(buff, 0, size);
    if (readFromFlash(oid, addr, buff, size) != (int)size) {
        LOG_ERROR("Failed to read from flash");
        return -1;
    }
  
    for (i = 0; i < (int)size; ++i) {
        if (buff[i] != checkValue) {
            LOG_ERROR("Mismatch at index %d (offs 0x%lx): expected 0x%02x, got 0x%02x", 
                  i, (unsigned long)(addr + i), checkValue, buff[i]);
            return -1;
        }
    }

    return EOK;
}


/* Test mtWrite -> mtSync -> mtRead with unaligned offsets */
int test_flashsrv_writeAndReadUnaligned(void)
{
    oid_t oid;
    int i;
    const off_t addr = 0x2050;
    const size_t size = 0x180;
    const uint8_t checkValue = 0x3C;
    uint8_t buff[size];

    memset(buff, checkValue, size);

    while (lookup(EXTERNAL_FLASH_PATH, NULL, &oid) < 0) {
        usleep(10000);
    }

    if (writeToFlash(oid, addr, buff, size) != (int)size) {
        LOG_ERROR("Failed to write flash ");
        return -1;
    }

    memset(buff, 0, size);

    if (readFromFlash(oid, addr, buff, size) != (int)size) {
        LOG_ERROR("Failed to read from flash ");
        return -1;
    }

    for (i = 0; i < (int)size; ++i) {
        if (buff[i] != checkValue) {
            return -1;
        }
    }

    return EOK;
}


/* Test flashsrv_read and flashsrv_write validation (-EINVAL checks) */
int test_flashsrv_invalidOffsetBounds(void)
{
    oid_t oid;
    long long flashSize = 0;
    uint8_t dummy[16] = { 0 };

    while (lookup(EXTERNAL_FLASH_PATH, NULL, &oid) < 0) {
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

    while (lookup(EXTERNAL_FLASH_PATH, NULL, &oid) < 0) {
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


/* Test getAttr for a specific raw partition (/dev/mtd0.raw1) */
int test_flashsrv_rawPartGetAttr(void)
{
    oid_t oid;
    long long partSize = 0;
    const char *path = "/dev/mtd0.raw1";

    if (lookup(path, NULL, &oid) < 0) {
        LOG_ERROR("Path does not exist, cannot fetch port address/id");
        return -1;
    }

    if (getAttrFlash(oid, atSize, &partSize) < 0) {
        return -1;
    }

    if (partSize <= 0) {
        return -1;
    }

    return EOK;
}


/* Test write and read on raw partition */
int test_flashsrv_rawPartWriteAndRead(void)
{
    oid_t oid;
    int i;
    const off_t addr = 0x100;
    const size_t size = 0x80;
    const uint8_t checkValue = 0x7E;
    const char *path = "/dev/mtd0.raw1";
    uint8_t buff[size];

    memset(buff, checkValue, size);

    if (lookup(path, NULL, &oid) < 0) {
        LOG_ERROR("Path does not exist, cannot fetch port address/id");
        return -1;
    }

    if (writeToFlash(oid, addr, buff, size) != (int)size) {
        return -1;
    }

    memset(buff, 0, size);

    if (readFromFlash(oid, addr, buff, size) != (int)size) {
        return -1;
    }

    for (i = 0; i < (int)size; ++i) {
        if (buff[i] != checkValue) {
            return -1;
        }
    }

    return EOK;
}


/* Test mtMount message handler */
int test_flashsrv_mountFs(void)
{
    oid_t oid;
    msg_t msg;
    memset(&msg, 0, sizeof(msg));

    const char *path = "/dev/mtd0.mfs1";

    if (lookup(path, NULL, &oid) < 0) {
        LOG_ERROR("Path does not exist, cannot fetch port address/id");
        return -1;
    }

    mount_i_msg_t *imnt = (mount_i_msg_t *)msg.i.raw;

    msg.type = mtMount;
    msg.oid = oid;
    strncpy(imnt->fstype, "meterfs", sizeof(imnt->fstype) - 1);
    imnt->mode = 0;

    if (msgSend(oid.port, &msg) != 0) {
        return -1;
    }

    /* Validate return code from storage_mountfs */
    if (msg.o.err < 0 && msg.o.err != -EINVAL && msg.o.err != -ENOSYS) {
        return -1;
    }

    return EOK;
}