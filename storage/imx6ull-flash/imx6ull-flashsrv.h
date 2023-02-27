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

#include <sys/types.h>
#include <stddef.h>
#include <stdint.h>


enum { flashsrv_devctl_info = 0, flashsrv_devctl_erase, flashsrv_devctl_writeraw, flashsrv_devctl_writemeta,
	 flashsrv_devctl_readraw, flashsrv_devctl_readmeta, flashsrv_devctl_isbad, flashsrv_devctl_markbad, flashsrv_devctl_maxbitflips,
	 flashsrv_devctl_readptable, flashsrv_devctl_writeptable };

/* information about NAND flash configuration */
typedef struct {
	uint64_t size;    /* total NAND size in bytes */
	uint32_t writesz; /* write page DATA size in bytes */
	uint32_t metasz;  /* write page METADATA size in bytes */
	uint32_t oobsz;   /* OOB size in bytes */
	uint32_t erasesz; /* erase block size in bytes (multiply of writesize) */
} flashsrv_info_t;

/* message to /dev/flashX   - chip operation - absolute address
 * message to /dev/flashXpY - partition operation - address relative to the beginning of the partition
 */

typedef struct {
	int type;

	union {
		/* erase: automatically skips bad blocks, returns erased blocks count */
		struct {
			oid_t oid;
			size_t address; /* multiply of erasesz */
			size_t size;    /* multiply of erasesz, 0 == full partition / device */
		} erase;

		/* writeraw, writemeta */
		struct {
			oid_t oid;
			uint32_t address;
			size_t size;
		} write;

		/* readraw, readmeta */
		struct {
			oid_t oid;
			uint32_t address; /* multiply of (writesz + metasz) or oobsz */
			size_t size;      /* multiply of (writesz + metasz) or oobsz */
		} read;

		/* isbad, markbad */
		struct {
			oid_t oid;
			uint32_t address; /* multiply of erasesz */
		} badblock;

		/* maxbitflips */
		struct {
			oid_t oid;
			uint32_t address; /* multiply of erasesz */
		} maxbitflips;

		/* ptable */
		struct {
			oid_t oid;
		} ptable;
	};
} __attribute__((packed)) flash_i_devctl_t;


typedef struct {
	int err;
	flashsrv_info_t info; /* valid only for flashsrv_devctl_info */
} __attribute__((packed)) flash_o_devctl_t;

#endif
