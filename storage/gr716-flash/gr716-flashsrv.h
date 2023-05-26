/*
 * Phoenix-RTOS
 *
 * GR716 Flash server
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _GR716_FLASHSRV_H_
#define _GR716_FLASHSRV_H_


#include <sys/types.h>


/* clang-format off */
enum { flashsrv_devctl_properties = 0, flashsrv_devctl_sync, flashsrv_devctl_eraseSector, flashsrv_devctl_erasePartition,
	flashsrv_devctl_directWrite };
/* clang-format on */


typedef struct {
	int type;
	oid_t oid;
	uint32_t addr;
} __attribute__((packed)) flash_i_devctl_t;


typedef struct {
	int err;

	struct {
		uint32_t size;
		uint32_t psize;
		uint32_t ssize;
		uint32_t offs;
	} properties;

} __attribute__((packed)) flash_o_devctl_t;


#endif
