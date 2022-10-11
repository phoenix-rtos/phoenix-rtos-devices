/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4 AES driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Daniel Sawka
 *
 * %LICENSE%
 */


#ifndef LIBAES_H_
#define LIBAES_H_


enum { aes_128 = 0, aes_256 = 1 };


enum { aes_ecb = 0, aes_cbc = 1, aes_ctr = 2 };


enum { aes_encrypt = 0, aes_decrypt = 2 };


void libaes_setKey(const unsigned char *key, int keylen);


void libaes_getKey(unsigned char *key, int keylen);


void libaes_setIv(const unsigned char *iv);


void libaes_getIv(unsigned char *iv);


void libaes_deriveDecryptionKey(void);


void libaes_prepare(int mode, int dir);


void libaes_unprepare(void);


void libaes_processBlock(const unsigned char *in, unsigned char *out);


int libaes_init(void);


#endif
