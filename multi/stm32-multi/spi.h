/*
 * Phoenix-RTOS
 *
 * STM32L1 SPI driver
 *
 * Copyright 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _SPI_H_
#define _SPI_H_


enum { spi_read = 0, spi_write, spi_readwrite };


int spi_transaction(int spi, int dir, unsigned char cmd, unsigned int addr, unsigned char flags,
	unsigned char *ibuff, unsigned char *obuff, size_t bufflen);


int spi_configure(int spi, char mode, char bdiv, int enable);


void spi_init(void);


#endif
