/*
 * Phoenix-RTOS
 *
 * GRLIB NANDFCTRL2 NAND flash server.
 *
 * Flash server message interface.
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef NANDFCTRL2_FLASHSRV_H_
#define NANDFCTRL2_FLASHSRV_H_

#include <sys/types.h>
#include <stddef.h>
#include <stdint.h>


/* clang-format off */
enum {
	flashsrv_devctl_info = 0, flashsrv_devctl_erase, flashsrv_devctl_writeraw, flashsrv_devctl_writemeta,
	flashsrv_devctl_readraw, flashsrv_devctl_readmeta, flashsrv_devctl_isbad, flashsrv_devctl_markbad,
	flashsrv_devctl_maxbitflips, flashsrv_devctl_readptable, flashsrv_devctl_writeptable
};
/* clang-format on */


/* Information about NAND flash configuration */
typedef struct {
	uint64_t size;    /* Total NAND size in bytes */
	uint32_t writesz; /* Page data size in bytes */
	uint32_t metasz;  /* Spare/OOB area size in bytes (user-accessible) */
	uint32_t oobsz;   /* Total OOB size (same as metasz here) */
	uint32_t erasesz; /* Erase block size in bytes */
} flashsrv_info_t;


/*
 * message to /dev/mtdX   - chip operation, absolute address
 * message to /dev/mtdXpY - partition operation, address relative to partition start
 */

typedef struct {
	int type;

	union {
		/* erase: automatically skips bad blocks, returns erased blocks count */
		struct {
			size_t address; /* multiple of erasesz */
			size_t size;    /* multiple of erasesz, 0 == full partition / device */
		} erase;

		/* writeraw, writemeta */
		struct {
			uint32_t address;
			size_t size;
		} write;

		/* readraw, readmeta */
		struct {
			uint32_t address;
			size_t size;
		} read;

		/* isbad, markbad */
		struct {
			uint32_t address; /* multiple of erasesz */
		} badblock;

		/* maxbitflips */
		struct {
			uint32_t address; /* multiple of erasesz */
		} maxbitflips;
	};
} __attribute__((packed)) flash_i_devctl_t;


typedef struct {
	flashsrv_info_t info; /* Valid only for flashsrv_devctl_info */
} __attribute__((packed)) flash_o_devctl_t;


#endif /* NANDFCTRL2_FLASHSRV_H_ */
