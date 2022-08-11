/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4 HASH driver
 *
 * Copyright 2022 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef LIBHASH_H_
#define LIBHASH_H_

#include <stdint.h>
#include <unistd.h>

#define LIBHASH_MD5_DIGESTSZ    (128 / 8)
#define LIBHASH_SHA1_DIGESTSZ   (160 / 8)
#define LIBHASH_SHA224_DIGESTSZ (224 / 8)
#define LIBHASH_SHA256_DIGESTSZ (256 / 8)

/* clang-format off */
typedef enum { libhash_sha1, libhash_sha224, libhash_sha256, libhash_md5 } libhash_algo_t;
/* clang-format on */

int libhash_start(libhash_algo_t algorithm);


ssize_t libhash_feed(void *buff, size_t size);


ssize_t libhash_finish(uint8_t *digest);


int libhash_init(void);


#endif
