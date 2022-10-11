/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4 AES driver
 *
 * Copyright 2020, 2022 Phoenix Systems
 * Author: Daniel Sawka
 *
 * %LICENSE%
 */

#include "../common.h"
#include "libmulti/libaes.h"


enum { cr = 0, sr, dinr, doutr, keyr0, ivr0 = keyr0 + 4, keyr4 = ivr0 + 4, susp0r = keyr4 + 4 };


static struct {
	volatile unsigned int *base;
} common;


static inline void libaes_enable(void)
{
	dataBarier();
	*(common.base + cr) |= 0x1;
	dataBarier();
}


static inline void libaes_disable(void)
{
	dataBarier();
	*(common.base + cr) &= ~0x1;
	dataBarier();
}


static void storeVector4(int reg, const unsigned char *vector)
{
	int i;
	unsigned int tmp;
	volatile unsigned int *addr = common.base + reg + 3;

	for (i = 0; i < 4; i++) {
		tmp = *vector++ << 24;
		tmp |= *vector++ << 16;
		tmp |= *vector++ << 8;
		tmp |= *vector++;
		*addr = tmp;
		addr -= 1;
	}
}


static void retrieveVector4(int reg, unsigned char *vector)
{
	int i;
	unsigned int tmp;
	volatile unsigned int *addr = common.base + reg + 3;

	for (i = 0; i < 4; i++) {
		tmp = *addr;
		addr -= 1;
		*vector++ = tmp >> 24;
		*vector++ = tmp >> 16;
		*vector++ = tmp >> 8;
		*vector++ = tmp;
	}
}


void libaes_setKey(const unsigned char *key, int keylen)
{
	if (keylen == aes_128) {
		storeVector4(keyr0, key);
		*(common.base + cr) &= ~(1 << 18);
	}
	else {
		storeVector4(keyr4, key);
		storeVector4(keyr0, key + 16);
		*(common.base + cr) |= (1 << 18);
	}
}


void libaes_getKey(unsigned char *key, int keylen)
{
	if (keylen == aes_128) {
		retrieveVector4(keyr0, key);
	}
	else {
		retrieveVector4(keyr4, key);
		retrieveVector4(keyr0, key + 16);
	}
}


void libaes_setIv(const unsigned char *iv)
{
	storeVector4(ivr0, iv);
}


void libaes_getIv(unsigned char *iv)
{
	retrieveVector4(ivr0, iv);
}


void libaes_deriveDecryptionKey(void)
{
	unsigned int t;

	t = *(common.base + cr) & ~(0x3 << 3);
	*(common.base + cr) = t | (0x1 << 3);

	libaes_enable();

	while ((*(common.base + sr) & 0x1) == 0)
		;

	*(common.base + cr) |= (1 << 7);
}


void libaes_prepare(int mode, int dir)
{
	unsigned int t;

	t = *(common.base + cr) & ~0x1007E;
	/* byte swap */
	*(common.base + cr) = t | (mode << 5) | (dir << 3) | (0x2 << 1);

	libaes_enable();
}


void libaes_unprepare(void)
{
	libaes_disable();
}


void libaes_processBlock(const unsigned char *in, unsigned char *out)
{
	int i;
	unsigned int tmp;

	for (i = 0; i < 4; i++) {
		tmp = *in++;
		tmp |= *in++ << 8;
		tmp |= *in++ << 16;
		tmp |= *in++ << 24;
		*(common.base + dinr) = tmp;
	}

	while ((*(common.base + sr) & 0x1) == 0)
		;

	for (i = 0; i < 4; i++) {
		tmp = *(common.base + doutr);
		*out++ = tmp;
		*out++ = tmp >> 8;
		*out++ = tmp >> 16;
		*out++ = tmp >> 24;
	}

	*(common.base + cr) |= (1 << 7);
}


int libaes_init(void)
{
	common.base = (void *)0x50060000;

	devClk(pctl_aes, 1);

	libaes_disable();

	return 0;
}
