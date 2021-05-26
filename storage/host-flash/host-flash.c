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
#include <string.h>

#include "host-flash.h"

static struct {
	int filefd;
	size_t flashsz;
	size_t sectorsz;
} hostflash_common;

ssize_t hostflash_read(unsigned int addr, void *buff, size_t bufflen)
{
	ssize_t stat;
	int readsz = 0;

	if (addr + bufflen > hostflash_common.flashsz)
		return -EINVAL;

	while ((stat = pread(hostflash_common.filefd, (void *)((char *)buff + readsz), bufflen - readsz, addr + readsz)) != 0) {
		if (stat < 0) {
			if (errno == EINTR)
				continue;

			return stat;
		}

		readsz += stat;
		if (readsz == bufflen)
			break;
	}

	return readsz;
}


ssize_t hostflash_write(unsigned int addr, void *buff, size_t bufflen)
{
	char tempTab[256];
	size_t wrote = 0, i;
	int len;
	ssize_t stat, readsz;

	if (addr + bufflen > hostflash_common.flashsz)
		return -EINVAL;

	while (wrote != bufflen) {
		if ((bufflen - wrote) >= sizeof(tempTab))
			len = sizeof(tempTab);
		else
			len = bufflen - wrote;

		readsz = hostflash_read(addr + wrote, tempTab, len);
		if (readsz <= 0)
			return readsz;

		for (i = 0; i < readsz; i++)
			tempTab[i] = tempTab[i] & *((char *)buff + wrote + i);

		len = 0;
		while (len != readsz) {
			stat = pwrite(hostflash_common.filefd, &tempTab, readsz - len, addr + wrote + len);
			if (stat < 0) {
				if (errno == EINTR)
					continue;

				return stat;
			}
			len += stat;
		}
		wrote += len;
	}

	return (ssize_t)wrote;
}


int hostflash_sectorErase(unsigned int addr)
{
	char tempTab[256];
	ssize_t len = sizeof(tempTab);
	int erased = 0;
	unsigned int sectorAddr;
	int stat;

	memset(tempTab, 0xff, sizeof(tempTab));

	if (addr >= hostflash_common.flashsz)
		return -EINVAL;

	sectorAddr = (addr / hostflash_common.sectorsz) * hostflash_common.sectorsz;

	while (erased != hostflash_common.sectorsz) {
		stat = pwrite(hostflash_common.filefd, tempTab, len, sectorAddr + erased);
		if (stat < 0) {
			if (errno == EINTR)
				continue;

			return stat;
		}

		len -= stat;
		if (len == 0)
			len = sizeof(tempTab);

		erased += stat;
	}

	return 0;
}


int hostflash_chipErase(void)
{
	char tempTab[256];
	size_t erased = 0, len = sizeof(tempTab);
	ssize_t stat;

	memset(tempTab, 0xff, sizeof(tempTab));

	while (erased != hostflash_common.flashsz) {
		stat = pwrite(hostflash_common.filefd, tempTab, len, erased);
		if (stat < 0) {
			if (errno == EINTR)
				continue;

			return stat;
		}

		len -= stat;
		if (len == 0)
			len = sizeof(tempTab);

		erased += stat;
	}

	return 0;
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
