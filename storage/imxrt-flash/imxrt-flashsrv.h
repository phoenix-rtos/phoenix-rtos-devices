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


/* clang-format off */

enum { flashsrv_devctl_properties = 0, flashsrv_devctl_sync, flashsrv_devctl_eraseSector,
	flashsrv_devctl_erasePartition, flashsrv_devctl_directWrite, flashsrv_devctl_directRead,
	flashsrv_devctl_calcCrc32 };

/* clang-format on */


typedef struct {
	int type;

	union {
		/* eraseSector */
		struct {
			uint32_t addr;
		} erase;

		/* directWrite */
		struct {
			uint32_t addr;
		} write;

		/* directRead */
		struct {
			uint32_t addr;
		} read;

		/* calcCrc32 */
		struct {
			/* set addr & len to 0 for full range */
			uint32_t addr;
			uint32_t len;
			uint32_t base;
		} crc32;
	};
} __attribute__((packed)) flash_i_devctl_t;


typedef struct {
	uint32_t size;
	uint32_t psize;
	uint32_t ssize;
	uint32_t offs;
} flashsrv_properties_t;


typedef struct {
	union {
		flashsrv_properties_t properties;
		uint32_t crc32;
	};
} __attribute__((packed)) flash_o_devctl_t;


/*
 * Device/partition operations
 *
 * Includes parameters and callbacks which allow to perform direct operations (read/write/sector erase) on specific
 * device/partition. Callbacks are initialized by flashsrv. User should provide *fID* inside flashsrv_customIntInit()
 * call. It should be passed as the first argument to callbacks.
 */
typedef struct {
	uint8_t fID;

	ssize_t (*read)(uint8_t fID, size_t offset, void *data, size_t size);
	ssize_t (*write)(uint8_t fID, size_t offset, const void *data, size_t size);
	int (*eraseSector)(uint8_t fID, uint32_t offs);
	int (*getProperties)(uint8_t fID, flashsrv_properties_t *p);
	int (*calcCrc32)(uint8_t fID, size_t offset, size_t len, uint32_t *crc32);
} flashsrv_partitionOps_t;


#endif
