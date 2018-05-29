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


enum { cmd_wrsr = 0x01, cmd_write, cmd_read, cmd_wrdi, cmd_rdsr, cmd_wren, cmd_hsread = 0x0b, cmd_sector_erase = 0x20,
	cmd_ewsr = 0x50, cmd_32erase = 0x52, cmd_chip_erase = 0x60, cmd_ebsy = 0x70, cmd_dbsy = 0x80, cmd_rdid = 0x90,
	cmd_jedecid = 0x9f, cmd_aai_write = 0xad, cmd_64erase = 0xd8 };


enum { spi_read = 0, spi_write };


int spi_transaction(int spi, int dir, unsigned char cmd, unsigned int addr, unsigned char flags, unsigned char *buff, size_t bufflen);


int spi_configure(int spi, char mode, char bdiv, int enable);


void spi_init(void);


#endif
