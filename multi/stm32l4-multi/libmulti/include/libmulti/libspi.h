/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4 SPI driver
 *
 * Copyright 2018, 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef LIBSPI_H_
#define LIBSPI_H_


#include <stddef.h>
#include <sys/threads.h>


typedef struct {
	volatile uint16_t *base;

	unsigned char *ibuff;
	const unsigned char *obuff;
	size_t cnt;

	int usedma;
} libspi_ctx_t;


enum { spi1 = 0, spi2, spi3 };


#define SPI_ADDRSHIFT 3
#define SPI_ADDRMASK 0x3


enum { spi_cmd = 0x1, spi_dummy = 0x2, /* 3-bits for SPI_ADDR* ,*/ spi_addrlsb = 0x20 };


enum { spi_dir_read = 0, spi_dir_write, spi_dir_readwrite };


int libspi_transaction(libspi_ctx_t *ctx, int dir, unsigned char cmd, unsigned int addr, unsigned char flags,
	unsigned char *ibuff, const unsigned char *obuff, size_t bufflen);


int libspi_configure(libspi_ctx_t *ctx, char mode, char bdiv, int enable);


int libspi_init(libspi_ctx_t *ctx, unsigned int spi, int useDma);


#endif
