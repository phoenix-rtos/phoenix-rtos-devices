/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND flash server.
 *
 * Copyright 2018 - 2019 Phoenix Systems
 * Author: Jan Sikorski, Hubert Buczyński
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMX6ULL_FLASHSRV_H_
#define _IMX6ULL_FLASHSRV_H_

enum { flashsrv_devctl_erase = 0 };

typedef struct {
	int type;

	union {
		struct {
			size_t size;
			size_t offset;
			oid_t oid;
		} erase;
	};
} __attribute__((packed)) flash_i_devctl_t;


typedef struct {
	int err;
} __attribute__((packed)) flash_o_devctl_t;

#endif
