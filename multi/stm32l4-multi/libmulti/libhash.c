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

#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <sys/threads.h>
#include "include/libmulti/libhash.h"
#include "../common.h"

/* IP registers */
#define HASH_CR  0
#define HASH_DIN 1
#define HASH_STR 2
#define HASH_IMR 8
#define HASH_SR  9
#define HASH_HR0 196

static struct {
	volatile uint32_t *base;
	size_t remsz;
	libhash_algo_t algorithm;
	uint8_t rem[4];
} libhash_common;


static inline uint32_t libhash_pack(const uint8_t *in)
{
	return ((uint32_t)(in[0]) << 24) | ((uint32_t)(in[1]) << 16) | ((uint32_t)(in[2]) << 8) | (uint32_t)(in[3]);
}


static inline void libhash_unpack(uint8_t *out, uint32_t in)
{
	out[3] = in & 0xff;
	out[2] = (in >> 8) & 0xff;
	out[1] = (in >> 16) & 0xff;
	out[0] = (in >> 24) & 0xff;
}


#if defined(__CPU_STM32L4X6)
static int libhash_selectAlgorithm(libhash_algo_t algorithm, uint32_t *crValue)
{
	*crValue &= ~((1 << 18) | (1 << 7));
	switch (algorithm) {
		case libhash_sha1:
			*crValue |= 0;
			break;

		case libhash_md5:
			*crValue |= 1 << 7;
			break;

		case libhash_sha224:
			*crValue |= 1 << 18;
			break;

		case libhash_sha256:
			*crValue |= (1 << 7) | (1 << 18);
			break;

		default:
			return -EINVAL;
	}

	return 0;
}
#elif defined(__CPU_STM32N6)
static int libhash_selectAlgorithm(libhash_algo_t algorithm, uint32_t *crValue)
{
	/* MD5 not supported on this device */
	if ((algorithm == libhash_md5) || (algorithm < 0) || (algorithm > libhash_sha512)) {
		return -EINVAL;
	}

	static const uint8_t lookup[] = {
		[libhash_sha1] = 0x0,
		[libhash_sha224] = 0x2,
		[libhash_sha256] = 0x3,
		[libhash_sha384] = 0xc,
		[libhash_sha512_224] = 0xd,
		[libhash_sha512_256] = 0xe,
		[libhash_sha512] = 0xf,
	};

	*crValue &= ~(0xf << 17);
	*crValue |= (uint32_t)lookup[algorithm] << 17;
	return 0;
}
#endif


int libhash_start(libhash_algo_t algorithm)
{
	uint32_t t = *(libhash_common.base + HASH_CR) & ~(0x13f7cU);
	if (libhash_selectAlgorithm(algorithm, &t) < 0) {
		return -EINVAL;
	}

	*(libhash_common.base + HASH_CR) = t | (1 << 2);
	libhash_common.remsz = 0;
	libhash_common.algorithm = algorithm;

	return EOK;
}


ssize_t libhash_feed(const void *buff, size_t size)
{
	size_t chunk, written = 0, i;
	const uint8_t *data = buff;

	if (size == 0) {
		return 0;
	}

	if (libhash_common.remsz != 0) {
		if (libhash_common.remsz + size >= sizeof(uint32_t)) {
			for (i = libhash_common.remsz; i < sizeof(uint32_t); ++i) {
				libhash_common.rem[i] = data[i];
			}

			*(libhash_common.base + HASH_DIN) = libhash_pack(libhash_common.rem);

			written = sizeof(uint32_t) - libhash_common.remsz;
			data += written;
			size -= written;
			libhash_common.remsz = 0;
		}
		else {
			memcpy(libhash_common.rem + libhash_common.remsz, data, size);
			written = size;
			libhash_common.remsz += size;
			size = 0;
		}
	}

	while (size != 0) {
		if (size >= sizeof(uint32_t)) {
			chunk = sizeof(uint32_t);
			*(libhash_common.base + HASH_DIN) = libhash_pack(data + written);
		}
		else {
			chunk = size;
			memcpy(libhash_common.rem, data + written, chunk);
			libhash_common.remsz = chunk;
		}

		written += chunk;
		size -= chunk;
	}

	return written;
}


ssize_t libhash_finish(uint8_t *digest)
{
	static const uint8_t diglen_lookup[] = {
		[libhash_sha1] = LIBHASH_SHA1_DIGESTSZ / 4,
		[libhash_sha224] = LIBHASH_SHA224_DIGESTSZ / 4,
		[libhash_sha256] = LIBHASH_SHA256_DIGESTSZ / 4,
		[libhash_md5] = LIBHASH_MD5_DIGESTSZ / 4,
		[libhash_sha384] = LIBHASH_SHA384_DIGESTSZ / 4,
		[libhash_sha512_224] = LIBHASH_SHA512_224_DIGESTSZ / 4,
		[libhash_sha512_256] = LIBHASH_SHA512_256_DIGESTSZ / 4,
		[libhash_sha512] = LIBHASH_SHA512_DIGESTSZ / 4,
	};

	uint32_t t = *(libhash_common.base + HASH_STR) & ~(0x11fU);

	if (libhash_common.remsz != 0) {
		*(libhash_common.base + HASH_DIN) = libhash_pack(libhash_common.rem);
		t |= libhash_common.remsz * 8;
	}

	*(libhash_common.base + HASH_STR) = t;
	*(libhash_common.base + HASH_STR) |= 1 << 8;

	if ((libhash_common.algorithm < 0) || (libhash_common.algorithm > libhash_sha512)) {
		return -EINVAL;
	}

	while (!(*(libhash_common.base + HASH_SR) & (1 << 1))) {
		/* Wait for calculation completion */
	}

	size_t diglen = diglen_lookup[libhash_common.algorithm];
	for (size_t i = 0; i < diglen; ++i) {
		libhash_unpack(&digest[i * 4], *(libhash_common.base + HASH_HR0 + i));
	}

	return diglen * sizeof(uint32_t);
}


int libhash_init(void)
{
	libhash_common.base = HASH_BASE;
#if defined(__CPU_STM32L4X6)
	devClk(pctl_hash, 3);
#elif defined(__CPU_STM32N6)
	devClk(pctl_hash, 1);
#endif

	return EOK;
}
