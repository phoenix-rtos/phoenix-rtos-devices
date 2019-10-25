/*
 * Phoenix-RTOS
 *
 * i.MX RT Flash server
 *
 * Copyright 2019 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _IMXRT_FLASH_SERVER_H_
#define _IMXRT_FLASH_SERVER_H_


#include <sys/msg.h>


enum { flashsrv_devctl_properties = 0, flashsrv_devctl_sync };


typedef struct {
	int type;
	oid_t oid;
} __attribute__((packed)) flash_i_devctl_t;


typedef struct {
	int err;

	struct {
		uint32_t fsize;
		uint32_t psize;
		uint32_t ssize;
	} properties;

} __attribute__((packed)) flash_o_devctl_t;


#endif
