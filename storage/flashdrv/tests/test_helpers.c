/*
 * Phoenix-RTOS
 *
 * GRLIB SPIMCTRL Flash server helpers
 *
 * Copyright 2026 Phoenix Systems
 * Author: Amelia Waszkowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include "test_helpers.h"


// uint64_t get_time_us(void)
// {

// }


/* -------------------------------------------------------------------------
 * IPC helpers sending messages directly handled by flashsrv_msgHandler
 * ------------------------------------------------------------------------- */

int sendOpenCloseMsg(oid_t oid, int type)
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


int writeToFlash(oid_t oid, off_t offs, const void *data, size_t size)
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


int eraseSector(oid_t oid, uint32_t offs, size_t size)
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
    idevctl->type = flashdrv_devctl_Erase;
    idevctl->erase.addr = offs;
    idevctl->erase.size = size;
    idevctl->erase.type = flashdrv_devctl_eraseSector;

    res = msgSend(oid.port, &msg);

    if (res != 0) {
        LOG_ERROR("Cannot send mtDevCtl (eraseSector) msg %d\n", res);
        return -1;
    }

	if (msg.o.err < 0)
		LOG_ERROR("Cannot erase sector, err: (%s).", strerror(msg.o.err));

    return msg.o.err;
}


int erasePartition(oid_t oid)
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
    idevctl->type = flashdrv_devctl_Erase;
    idevctl->erase.type = flashdrv_devctl_erasePartition;

    res = msgSend(oid.port, &msg);

    if (res != 0) {
        LOG_ERROR("Cannot send mtDevCtl (eraseSector) msg %d\n", res);
        return -1;
    }

	if (msg.o.err < 0)
		LOG_ERROR("Cannot erase sector, err: (%s).", strerror(msg.o.err));

    return msg.o.err;  
}


int eraseChip(oid_t oid)
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
    idevctl->type = flashdrv_devctl_Erase;
    idevctl->erase.type = flashdrv_devctl_eraseChip;

    res = msgSend(oid.port, &msg);

    if (res != 0) {
        LOG_ERROR("Cannot send mtDevCtl (eraseSector) msg %d\n", res);
        return -1;
    }

	if (msg.o.err < 0)
		LOG_ERROR("Cannot erase sector, err: (%s).", strerror(msg.o.err));

    return msg.o.err;  
}


int setSPI(oid_t oid, SPIMode_t mode)
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


int readFromFlash(oid_t oid, off_t offs, void *data, size_t size)
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


int getAttrFlash(oid_t oid, int type, long long *val)
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


int erase_write_read_print(oid_t oid, const off_t testAddr, const size_t testSize, const size_t sectorSize, 
                        const uint8_t checkPattern, const char* part_path)
{
    uint8_t txBuff[testSize];
    uint8_t rxBuff[testSize];

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

    LOG_ERROR("Partition %s verified: oid.id=%ju, size=0x%llx (%lld MB)",
            part_path, (uintmax_t)oid.id, partSize, partSize / (1024 * 1024));

    off_t sectorAddr = testAddr - (testAddr % sectorSize);
    if (eraseSector(oid, sectorAddr, sectorSize) < 0) {
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
        LOG_ERROR("Data verification failed");
        return -1;
    }

    return 1;
}