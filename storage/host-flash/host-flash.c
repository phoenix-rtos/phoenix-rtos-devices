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


struct _meterfs_devCtx_t {
	volatile int state;
};


static struct {
	int filefd;
	size_t flashsz;
	size_t sectorsz;
	struct _meterfs_devCtx_t devCtx;
} hostflash_common;


ssize_t hostflash_read(struct _meterfs_devCtx_t *devCtx, off_t offs, void *buff, size_t bufflen)
{
	ssize_t stat;
	int readsz = 0;

	(void)devCtx;

	if ((devCtx == NULL) || (devCtx->state == 0) || ((offs + bufflen) > hostflash_common.flashsz)) {
		return -EINVAL;
	}

	while ((stat = pread(hostflash_common.filefd, (void *)((char *)buff + readsz), bufflen - readsz, offs + readsz)) != 0) {
		if (stat < 0) {
			if (errno == EINTR) {
				continue;
			}

			return stat;
		}

		readsz += stat;
		if (readsz == bufflen) {
			break;
		}
	}

	return readsz;
}


ssize_t hostflash_write(struct _meterfs_devCtx_t *devCtx, off_t offs, const void *buff, size_t bufflen)
{
	char tempTab[256];
	size_t wrote = 0, i;
	int len;
	ssize_t stat, readsz;

	if ((devCtx == NULL) || (devCtx->state == 0) || ((offs + bufflen) > hostflash_common.flashsz)) {
		return -EINVAL;
	}

	while (wrote != bufflen) {
		if ((bufflen - wrote) >= sizeof(tempTab)) {
			len = sizeof(tempTab);
		}
		else {
			len = bufflen - wrote;
		}

		readsz = hostflash_read(devCtx, offs + wrote, tempTab, len);
		if (readsz <= 0) {
			return readsz;
		}

		for (i = 0; i < readsz; i++) {
			tempTab[i] = tempTab[i] & *((const char *)buff + wrote + i);
		}

		len = 0;
		while (len != readsz) {
			stat = pwrite(hostflash_common.filefd, &tempTab, readsz - len, offs + wrote + len);
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


int hostflash_sectorErase(struct _meterfs_devCtx_t *devCtx, off_t offs)
{
	char tempTab[256];
	ssize_t len = sizeof(tempTab);
	size_t erased = 0;
	unsigned int sectorAddr;
	int stat;

	if ((devCtx == NULL) || (devCtx->state == 0) || (offs >= hostflash_common.flashsz)) {
		return -EINVAL;
	}

	(void)memset(tempTab, 0xff, sizeof(tempTab));

	sectorAddr = (offs / hostflash_common.sectorsz) * hostflash_common.sectorsz;

	while (erased != hostflash_common.sectorsz) {
		stat = pwrite(hostflash_common.filefd, tempTab, len, sectorAddr + erased);
		if (stat < 0) {
			if (errno == EINTR) {
				continue;
			}

			return stat;
		}

		len -= stat;
		if (len == 0) {
			len = sizeof(tempTab);
		}

		erased += stat;
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
