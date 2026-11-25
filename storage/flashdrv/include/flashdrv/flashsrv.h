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

#define LOG_LEVEL_NONE  0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_INFO  2
#define LOG_LEVEL_TRACE 3

#ifndef LOG_MODULE
  #define LOG_MODULE __FILE__
#endif

#ifndef LOG_LEVEL
  #define LOG_LEVEL LOG_LEVEL_INFO
#endif

#define LOG_ERROR(fmt, ...) \
    do { \
        if (LOG_LEVEL >= LOG_LEVEL_ERROR) { \
            (void)fprintf(stdout, "%s:%s:%d: [ERR] " fmt "\n", LOG_MODULE, __func__, __LINE__, ##__VA_ARGS__); \
        } \
    } while (0)

#define LOG_INFO(fmt, ...) \
    do { \
        if (LOG_LEVEL >= LOG_LEVEL_INFO) { \
            (void)fprintf(stdout, "%s:%s:%d: [INF] " fmt "\n", LOG_MODULE, __func__, __LINE__, ##__VA_ARGS__); \
        } \
    } while (0)

#define TRACE(fmt, ...) \
    do { \
        if (LOG_LEVEL >= LOG_LEVEL_TRACE) { \
            (void)fprintf(stdout, "%s:%s:%d: [TRC] " fmt "\n", LOG_MODULE, __func__, __LINE__, ##__VA_ARGS__); \
        } \
    } while (0)


/* clang-format on */

#ifndef FLASHSRV_ENABLE_JFFS2
#define FLASHSRV_ENABLE_JFFS2 0
#endif


#ifndef FLASHSRV_ENABLE_LITTLEFS
#define FLASHSRV_ENABLE_LITTLEFS 0
#endif


void flashsrv_register(const struct flash_driver *driver);

#endif /* _FLASHDRV_FLASHSRV_H_ */
