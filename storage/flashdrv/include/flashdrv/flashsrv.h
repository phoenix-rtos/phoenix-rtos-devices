/*
 * Phoenix-RTOS
 *
 * Flash server
 *
 * Copyright 2024 Phoenix Systems
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


void flashsrv_register(const struct flash_driver *driver);


#endif
