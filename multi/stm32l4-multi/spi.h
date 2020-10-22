/*
 * Phoenix-RTOS
 *
 * STM32L4 SPI driver
 *
 * Copyright 2018 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef SPI_H_
#define SPI_H_


#include <stddef.h>


int spi_transaction(int spi, int dir, unsigned char cmd, unsigned int addr, unsigned char flags,
	unsigned char *ibuff, unsigned char *obuff, size_t bufflen);


int spi_configure(int spi, char mode, char bdiv, int enable);


void spi_init(void);


#endif
