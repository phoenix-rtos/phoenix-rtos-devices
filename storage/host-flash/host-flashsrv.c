/*
 * Phoenix-RTOS
 *
 * Emulated flash server
 *
 * Copyright 2021 Phoenix Systems
 * Author: Tomasz Korniluk
 *
 * %LICENSE%
 */

#include <string.h>
#include <errno.h>
#include <pthread.h>

#include "host-flash.h"
#include "host-flashsrv.h"

static struct {
	pthread_mutex_t lock;
	meterfs_ctx_t ctx;
} hostflashsrv_common;

int hostflashsrv_readFile(id_t *id, off_t off, char *buff, size_t bufflen)
{
	int ret = 0;

	pthread_mutex_lock(&hostflashsrv_common.lock);
	ret = meterfs_readFile(*id, off, buff, bufflen, &hostflashsrv_common.ctx);
	pthread_mutex_unlock(&hostflashsrv_common.lock);

	return ret;
}


int hostflashsrv_writeFile(id_t *id, const char *buff, size_t bufflen)
{
	int ret = 0;

	pthread_mutex_lock(&hostflashsrv_common.lock);
	ret = meterfs_writeFile(*id, buff, bufflen, &hostflashsrv_common.ctx);
	pthread_mutex_unlock(&hostflashsrv_common.lock);

	return ret;
}


int hostflashsrv_lookup(const char *name, id_t *res)
{
	int ret = 0;

	pthread_mutex_lock(&hostflashsrv_common.lock);
	ret = meterfs_lookup(name, res, &hostflashsrv_common.ctx);
	pthread_mutex_unlock(&hostflashsrv_common.lock);

	return ret;
}


int hostflashsrv_open(id_t *id)
{
	int ret = 0;

	pthread_mutex_lock(&hostflashsrv_common.lock);
	ret = meterfs_open(*id, &hostflashsrv_common.ctx);
	pthread_mutex_unlock(&hostflashsrv_common.lock);

	return ret;
}


int hostflashsrv_close(id_t *id)
{
	int ret = 0;

	pthread_mutex_lock(&hostflashsrv_common.lock);
	ret = meterfs_close(*id, &hostflashsrv_common.ctx);
	pthread_mutex_unlock(&hostflashsrv_common.lock);

	return ret;
}


int hostflashsrv_devctl(meterfs_i_devctl_t *i, meterfs_o_devctl_t *o)
{
	int ret = 0;

	pthread_mutex_lock(&hostflashsrv_common.lock);
	ret = meterfs_devctl(i, o, &hostflashsrv_common.ctx);
	pthread_mutex_unlock(&hostflashsrv_common.lock);

	return ret;
}


int hostflashsrv_init(size_t *flashsz, size_t *sectorsz, const char *fileName)
{
	hostflashsrv_common.ctx.sz = *flashsz;
	hostflashsrv_common.ctx.sectorsz = *sectorsz;

	if (hostflash_init(&hostflashsrv_common.ctx.sz, &hostflashsrv_common.ctx.sectorsz, fileName) < 0) {
		printf("hostflashsrv: hostflash init failed\n");
		return -1;
	}

	hostflashsrv_common.ctx.offset = 0;
	hostflashsrv_common.ctx.read = hostflash_read;
	hostflashsrv_common.ctx.write = hostflash_write;
	hostflashsrv_common.ctx.eraseSector = hostflash_sectorErase;
	hostflashsrv_common.ctx.powerCtrl = hostflash_powerCtrl;

	hostflashsrv_common.ctx.devCtx = hostflash_devCtx();

	if (meterfs_init(&hostflashsrv_common.ctx) < 0) {
		printf("hostflashsrv: meterfs init failed\n");
		return -1;
	}

	if (pthread_mutex_init(&hostflashsrv_common.lock, NULL) != 0)
		return -1;

	return 0;
}
