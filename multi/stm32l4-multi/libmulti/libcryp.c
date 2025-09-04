/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32N6 CRYP driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Krzysztof Radzewicz
 * %LICENSE%
 */

#include "../common.h"
#include "libmulti/libcryp.h"
#include <errno.h>
#include <stdio.h>
#include <unistd.h>


#define CR_IPRST      (31)
#define CR_KMOD       (24)
#define CR_NPBLB      (20)
#define CR_ALGOMODE_H (19)
#define CR_GCM_CCMPH  (16)
#define CR_ENABLE     (15)
#define CR_FFLUSH     (14)
#define CR_KEYSIZE    (8)
#define CR_DATATYPE   (6)
#define CR_ALGOMODE_L (3)
#define CR_ALGODIR    (2)

#define SR_KEYVALID (7)
#define SR_KERF     (6)
#define SR_BUSY     (4)
#define SR_OFFU     (3)
#define SR_OFNE     (2)
#define SR_IFNF     (1)
#define SR_IFEM     (0)

#define CRYP_BASE ((void *)0x54020800)


enum {
	cryp_cr,
	cryp_sr,
	cryp_dinr,
	cryp_doutr,
	cryp_dmacr,
	cryp_imscr,
	cryp_risr,
	cryp_misr,
	cryp_k0lr,
	cryp_k0rr,
	cryp_k1lr,
	cryp_k1rr,
	cryp_k2lr,
	cryp_k2rr,
	cryp_k3lr,
	cryp_k3rr,
	cryp_iv0lr,
	cryp_iv0rr,
	cryp_iv1lr,
	cryp_iv1rr,
	cryp_csgcmccm0r,
	cryp_csgcmccm1r,
	cryp_csgcmccm2r,
	cryp_csgcmccm3r,
	cryp_csgcmccm4r,
	cryp_csgcmccm5r,
	cryp_csgcmccm6r,
	cryp_csgcmccm7r,
	cryp_csgcm0r,
	cryp_csgcm1r,
	cryp_csgcm2r,
	cryp_csgcm3r,
	cryp_csgcm4r,
	cryp_csgcm5r,
	cryp_csgcm6r,
	cryp_csgcm7r,
};


static struct {
	volatile unsigned int *base;
} common;


/* Can only be enabled when KEYVALID is set */
inline void libcryp_enable(void)
{
	dataBarier();
	*(common.base + cryp_cr) |= (0x1 << CR_ENABLE);
	dataBarier();
}


inline void libcryp_disable(void)
{
	dataBarier();
	*(common.base + cryp_cr) &= ~(0x1 << CR_ENABLE);
	dataBarier();
}


static inline void waitBusy(void)
{
	while ((*(common.base + cryp_sr) & (0x1 << SR_BUSY)) != 0)
		;
}


/* Treating each consecutive register write as less significant. Write msb first.
 * Veclen must be multiple of 4
 */
static void storeVector(int reg, const unsigned char *vector, int veclen)
{
	int i;
	unsigned int tmp;
	volatile unsigned int *addr = common.base + reg;

	for (i = 0; i * 4 < veclen; i++) {
		tmp = *vector++ << 24;
		tmp |= *vector++ << 16;
		tmp |= *vector++ << 8;
		tmp |= *vector++;
		*addr++ = tmp;
	}
}


/* Treating each consecutive registyer read as less significant. Read msb first.
 * Veclen must multiple of 4
 */
static void retrieveVector(int reg, unsigned char *vector, int veclen)
{
	int i;
	volatile unsigned int *addr = common.base + reg;
	unsigned int tmp;

	for (i = 0; i * 4 < veclen; i++) {
		tmp = *addr++;
		*vector++ = tmp >> 24;
		*vector++ = tmp >> 16;
		*vector++ = tmp >> 8;
		*vector++ = tmp;
	}
}


int libcryp_setKey(const unsigned char *key, int keylen)
{
	unsigned int t;
	waitBusy();

	t = *(common.base + cryp_cr) & ~(0x3 << CR_KEYSIZE);

	if (keylen == aes_128) {
		*(common.base + cryp_cr) = t | (0x0 << CR_KEYSIZE);
		storeVector(cryp_k2lr, key, 16);
	}
	else if (keylen == aes_192) {
		*(common.base + cryp_cr) = t | (0x1 << CR_KEYSIZE);
		storeVector(cryp_k1lr, key, 24);
	}
	else {
		*(common.base + cryp_cr) = t | (0x2 << CR_KEYSIZE);
		storeVector(cryp_k0lr, key, 32);
	}

	/* Wait untill key writing sequence succeded */
	while ((*(common.base + cryp_sr) & (0x1 << SR_KEYVALID)) == 0)
		;

	libcryp_enable();
	waitBusy();

	return EOK;
}


void libcryp_getKey(unsigned char *key, int keylen)
{
	return;
}

void libcryp_deriveDecryptionKey(void)
{
	unsigned int t;

	libcryp_disable();
	waitBusy();

	/* Flush I/O FIFO */
	*(common.base + cryp_cr) |= (0x1 << CR_FFLUSH);

	/* Clear ALGOMODE and ALGODIR to set them later */
	t = *(common.base + cryp_cr) & ~((0x1 << CR_ALGOMODE_H) | (0x7 << CR_ALGOMODE_L) | (0x1 << CR_ALGODIR));

	/* Set key derivation mode */
	t |= (0x7 << CR_ALGOMODE_L);
	*(common.base + cryp_cr) = t;

	/* Disable data swapping */
	*(common.base + cryp_cr) &= ~(0x3 << CR_DATATYPE);
}


void libcryp_setIv(const unsigned char *iv)
{
	storeVector(cryp_iv0lr, iv, 16);
}


void libcryp_getIv(unsigned char *iv)
{
	retrieveVector(cryp_iv0lr, iv, 16);
}


void libcryp_prepare(int mode, int dir)
{
	unsigned int t;

	waitBusy();

	/* Flush I/O FIFO */
	*(common.base + cryp_cr) |= (0x1 << CR_FFLUSH);

	/* Clear ALGOMODE and ALGODIR to set them later */
	t = *(common.base + cryp_cr) & ~((0x1 << CR_ALGOMODE_H) | (0x7 << CR_ALGOMODE_L) | (0x1 << CR_ALGODIR));
	switch (mode) {
		case aes_ecb:
			t |= (0x4 << CR_ALGOMODE_L);
			break;
		case aes_cbc:
			t |= (0x5 << CR_ALGOMODE_L);
			break;
		case aes_ctr:
			t |= (0x6 << CR_ALGOMODE_L);
			break;
	}
	if (dir == aes_decrypt) {
		t |= (0x1 << CR_ALGODIR);
	}
	*(common.base + cryp_cr) = t;

	/* Disable data swapping */
	*(common.base + cryp_cr) &= ~(0x3 << CR_DATATYPE);
}


void libcryp_unprepare(void)
{
	waitBusy();
	libcryp_disable();
}


/* Block must be 16 bytes long */
void libcryp_processBlock(const unsigned char *in, unsigned char *out)
{
	int i;
	unsigned int tmp;

	dataBarier();

	/* Wait untill input FIFO not full */
	while ((*(common.base + cryp_sr) & (0x1 << SR_IFNF)) == 0)
		;

	dataBarier();

	/* Treating each consecutive register read as less significant. Store msb first */
	for (i = 0; i < 4; i++) {
		tmp = *in++ << 24;
		tmp |= *in++ << 16;
		tmp |= *in++ << 8;
		tmp |= *in++;
		*(common.base + cryp_dinr) = tmp;
	}

	dataBarier();

	/* Wait untill output FIFO not empty */
	while ((*(common.base + cryp_sr) & (0x1 << SR_OFNE)) == 0)
		;

	dataBarier();

	/* Treating each consecutive register read as less significant. Read msb first */
	for (i = 0; i < 4; i++) {
		tmp = *(common.base + cryp_doutr);
		*out++ = tmp >> 24;
		*out++ = tmp >> 16;
		*out++ = tmp >> 8;
		*out++ = tmp;
	}
}


int libcryp_init(void)
{
	common.base = CRYP_BASE;

	/* The clock is only enabled if the user chooses to use the peripheral */
	devClk(pctl_cryp, 1);
	libcryp_disable();

	waitBusy();

	/* Set normal key mode - don't share with SAES */
	*(common.base + cryp_cr) &= ~(0x3 << CR_KMOD);

	return 0;
}
