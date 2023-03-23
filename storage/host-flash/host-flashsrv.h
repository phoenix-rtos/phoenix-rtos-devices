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

#ifndef HOST_FLASHSRV_H
#define HOST_FLASHSRV_H

#include <sys/types.h>
#include <meterfs.h>


int hostflashsrv_readFile(id_t *id, off_t off, char *buff, size_t bufflen);


int hostflashsrv_writeFile(id_t *id, const char *buff, size_t bufflen);


int hostflashsrv_lookup(const char *name, id_t *res);


int hostflashsrv_open(id_t *id);


int hostflashsrv_close(id_t *id);


int hostflashsrv_devctl(meterfs_i_devctl_t *i, meterfs_o_devctl_t *o);


int hostflashsrv_init(size_t *flashsz, size_t *sectorsz, const char *fileName);

#endif
