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
#include <assert.h>

#include "host-flash.h"


struct _meterfs_devCtx_t {
	volatile int state;
};


static struct {
	int filefd;
	size_t flashsz;
	size_t sectorsz;
	struct _meterfs_devCtx_t devCtx;
} hostflash_common;


static ssize_t hostflash_safeRead(void *buff, size_t bufflen, off_t offset)
{
	int ret = lseek(hostflash_common.filefd, offset, SEEK_SET);
	if (ret < 0) {
		return (ssize_t)ret;
	}

	int readsz = 0;
	ssize_t len;
	do {
		len = read(hostflash_common.filefd, (char *)buff + readsz, bufflen - readsz);
		if (len < 0) {
			if (errno == EINTR) {
				continue;
			}
			return len;
		}
		readsz += len;
	} while ((len > 0) && (readsz != bufflen));

	return readsz;
}


static ssize_t hostflash_safeWrite(const void *buff, size_t bufflen, off_t offset)
{
	int ret = lseek(hostflash_common.filefd, offset, SEEK_SET);
	if (ret < 0) {
		return (ssize_t)ret;
	}

	int writesz = 0;
	ssize_t len;
	do {
		len = write(hostflash_common.filefd, (char *)buff + writesz, bufflen - writesz);
		if (len < 0) {
			if (errno == EINTR) {
				continue;
			}
			return len;
		}
		writesz += len;
	} while ((len > 0) && (writesz != bufflen));

	return writesz;
}


ssize_t hostflash_read(struct _meterfs_devCtx_t *devCtx, off_t offs, void *buff, size_t bufflen)
{
	if ((devCtx == NULL) || (devCtx->state == 0) || ((offs + bufflen) > hostflash_common.flashsz)) {
		return -EINVAL;
	}

	return hostflash_safeRead(buff, bufflen, offs);
}


ssize_t hostflash_write(struct _meterfs_devCtx_t *devCtx, off_t offs, const void *buff, size_t bufflen)
{
	char tempTab[256];
	size_t wrote = 0;

	if ((devCtx == NULL) || (devCtx->state == 0) || ((offs + bufflen) > hostflash_common.flashsz)) {
		return -EINVAL;
	}

	while (wrote != bufflen) {
		size_t len;
		if ((bufflen - wrote) >= sizeof(tempTab)) {
			len = sizeof(tempTab);
		}
		else {
			len = bufflen - wrote;
		}

		ssize_t readsz = hostflash_safeRead(tempTab, len, offs + wrote);
		if (readsz <= 0) {
			return readsz;
		}

		for (ssize_t i = 0; i < readsz; i++) {
			tempTab[i] &= *((const char *)buff + wrote + i);
		}

		ssize_t stat = hostflash_safeWrite(tempTab, readsz, offs + wrote);
		if (stat < 0) {
			return stat;
		}
		wrote += stat;
	}

	return (ssize_t)wrote;
}


int hostflash_sectorErase(struct _meterfs_devCtx_t *devCtx, off_t offs)
{
	char tempTab[256];

	if ((devCtx == NULL) || (devCtx->state == 0) || (offs >= hostflash_common.flashsz)) {
		return -EINVAL;
	}

	(void)memset(tempTab, 0xff, sizeof(tempTab));

	off_t sectorAddr = (offs / hostflash_common.sectorsz) * hostflash_common.sectorsz;

	assert(hostflash_common.sectorsz % sizeof(tempTab) == 0);

	for (size_t erased = 0; erased < hostflash_common.sectorsz; erased += sizeof(tempTab)) {
		ssize_t stat = hostflash_safeWrite(tempTab, sizeof(tempTab), sectorAddr + erased);
		if (stat < 0) {
			return (int)stat;
		}
	}

	return 0;
}


void hostflash_powerCtrl(struct _meterfs_devCtx_t *devCtx, int state)
{
	devCtx->state = state;
}


struct _meterfs_devCtx_t *hostflash_devCtx(void)
{
	return &hostflash_common.devCtx;
}


int hostflash_init(size_t *flashsz, size_t *sectorsz, const char *fileName)
{
	int stat;

	if ((*sectorsz % 2) || (*flashsz % *sectorsz) || (*sectorsz > *flashsz) || (*sectorsz < 256) || (*flashsz == 0)) {
		return -EINVAL;
	}

	hostflash_common.devCtx.state = 1;
	hostflash_common.flashsz = *flashsz;
	hostflash_common.sectorsz = *sectorsz;
	hostflash_common.filefd = open(fileName, O_CREAT | O_RDWR, 0666);
	if (hostflash_common.filefd < 0) {
		return hostflash_common.filefd;
	}
	stat = ftruncate(hostflash_common.filefd, hostflash_common.flashsz);
	if (stat < 0) {
		close(hostflash_common.filefd);
		return stat;
	}
	return 0;
}
