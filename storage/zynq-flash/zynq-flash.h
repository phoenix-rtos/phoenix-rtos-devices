/*
 * Phoenix-RTOS
 *
 * Zynq Flash server
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jakub Klimek
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef _ZYNQ_FLASH_H_
#define _ZYNQ_FLASH_H_


#include <stddef.h>

#include <storage/dev.h>


enum { flashsrv_devctl_info = 0,
	flashsrv_devctl_erase };


typedef struct {
	mtd_type_t type;
	size_t size;        /* Total size */
	size_t erasesz;     /* Erase size */
	size_t writesz;     /* Minimal writable flash unit size. For NOR it is 1, for NAND it is one page */
	size_t writeBuffsz; /* For NOR flash it is page size, for NAND should be equal writesz */
	size_t metaSize;    /* Amount of meta data per block */
	size_t oobSize;     /* out-of-bound (oob) data size */
	size_t oobAvail;    /* available out-of-bound (oob) data size */
} flashsrv_info_t;


typedef struct {
	int type;
	struct {
		size_t address;
		size_t size;
	} erase;
} __attribute__((packed)) flash_i_devctl_t;


typedef struct {
	union {
		flashsrv_info_t info;
	};
} __attribute__((packed)) flash_o_devctl_t;

#endif
