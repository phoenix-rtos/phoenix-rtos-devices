/*
 * Phoenix-RTOS
 *
 * Flash server
 *
 * Copyright 2025 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASHDRV_FLASHSRV_H_
#define _FLASHDRV_FLASHSRV_H_

#include <stdio.h>
#include <flashdrv/flash_interface.h>

/* clang-format off */
#define LOG(fmt, ...) do { (void)fprintf(stdout, "flashsrv: " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(fmt, ...) do { (void)fprintf(stdout, "flashsrv:%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } while (0)
#define TRACE(fmt, ...) do { if (0) { (void)fprintf(stdout, "flashsrv:%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } } while (0)
/* clang-format on */

#ifndef FLASHSRV_ENABLE_JFFS2
#define FLASHSRV_ENABLE_JFFS2 0
#endif


#ifndef FLASHSRV_ENABLE_LITTLEFS
#define FLASHSRV_ENABLE_LITTLEFS 0
#endif


enum {
    flashsrv_devctl_eraseSector,
    flashsrv_devctl_erasePartition
};


typedef struct {
	int type;

	union {
		/* eraseSector */
		struct {
			size_t size;
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


void flashsrv_register(const struct flash_driver *driver);

#endif /* _FLASHDRV_FLASHSRV_H_ */