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


enum { flashsrv_devctl_properties = 0, flashsrv_devctl_sync, flashsrv_devctl_eraseSector, flashsrv_devctl_erasePartition, flashsrv_devctl_directWrite };


typedef struct {
	int type;
	oid_t oid;

	struct {
		uint32_t addr;
	} erase;

	struct {
		uint32_t addr;
	} write;

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
