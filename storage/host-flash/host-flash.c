/*
 * Phoenix-RTOS
 *
 * Flash emulator
 *
 * Copyright 2021 Phoenix Systems
 * Author: Tomasz Korniluk
 *
 * %LICENSE%
 */

#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "host-flash.h"

static struct {
    int filefd;
    size_t flashsz;
    size_t sectorsz;
} hostflash_common;

ssize_t hostflash_read(unsigned int addr, void *buff, size_t bufflen)
{
    ssize_t stat;
    int read = 0;

    if (addr + bufflen > hostflash_common.flashsz)
        return -EINVAL;

    while ((stat = pread(hostflash_common.filefd, (void *)((char *)buff + read), bufflen - read, addr + read)) != 0) {
        if (stat < 0) {
            if (errno == -EINTR)
                continue;

            return stat;
        }

        read += stat;
        if (read == bufflen)
            break;
    }

    return read;
}


ssize_t hostflash_write(unsigned int addr, void *buff, size_t bufflen)
{
    char currentByte = 0;
    size_t wrote = 0;
    ssize_t stat;

    if (addr + bufflen > hostflash_common.flashsz)
        return -EINVAL;

    while (wrote != bufflen) {
        stat = hostflash_read(addr + wrote, &currentByte, 1);
        if (stat <= 0)
            return stat;

        currentByte = currentByte & *((char *)buff + wrote);
        stat = pwrite(hostflash_common.filefd, &currentByte, 1, addr + wrote);
        if (stat < 0) {
            if (errno == -EINTR)
                continue;

            return stat;
        }

        wrote++;
    }

    return (ssize_t)wrote;
}


void hostflash_sectorErase(unsigned int addr)
{
    char tempTab[256];
    ssize_t len = sizeof(tempTab);
    int erased = 0;
    unsigned int sectorNum;
    unsigned int sectorAddr;
    int stat;

    memset(tempTab, 0xFF, sizeof(tempTab));

    if (addr < hostflash_common.flashsz) {
        sectorNum = addr / hostflash_common.sectorsz;
        sectorAddr = sectorNum * hostflash_common.sectorsz;
    }
    else {
        return;
    }

    while (erased != hostflash_common.sectorsz) {
        stat = pwrite(hostflash_common.filefd, tempTab, len, sectorAddr + erased);
        if (stat < 0) {
            if (errno == -EINTR)
                continue;

            return;
        }
        else {
            len = len - stat;
            if (len == 0)
                len = sizeof(tempTab);
        }

        erased += stat;
    }

    return;
}


void hostflash_chipErase(void)
{
    char tempTab[256];
    size_t erased = 0, len = sizeof(tempTab);
    ssize_t stat;

    memset(tempTab, 0xFF, sizeof(tempTab));

    while (erased != hostflash_common.flashsz) {
        stat = pwrite(hostflash_common.filefd, tempTab, len, erased);
        if (stat < 0) {
            if (errno == -EINTR)
                continue;

            return;
        }
        else {
            len = len - stat;
            if (len == 0)
                len = sizeof(tempTab);
        }

        erased += stat;
    }

    return;
}


int hostflash_init(size_t *flashsz, size_t *sectorsz, const char *fileName)
{
    int stat;

    if ((*sectorsz % 2) || (*flashsz % *sectorsz) || (*sectorsz > *flashsz) || *sectorsz < 256 || *flashsz == 0)
        return -EINVAL;

    hostflash_common.flashsz = *flashsz;
    hostflash_common.sectorsz = *sectorsz;
    hostflash_common.filefd = open(fileName, O_CREAT | O_RDWR, 0666);
    if (hostflash_common.filefd < 0)
        return hostflash_common.filefd;
    stat = ftruncate(hostflash_common.filefd, hostflash_common.flashsz);
    if (stat < 0) {
        close(hostflash_common.filefd);
        return stat;
    }
    return 0;
}
