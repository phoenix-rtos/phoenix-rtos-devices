/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1x SPI routines
 *
 * Copyright 2017 Phoenix Systems
 * Author: Aleksander Kaminski, Jan Sikorski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _SPIDRV_H_
#define _SPIDRV_H_


enum { cmd_wrsr = 0x01, cmd_write, cmd_read, cmd_wrdi, cmd_rdsr, cmd_wren, cmd_hsread = 0x0b, cmd_sector_erase = 0x20,
	cmd_ewsr = 0x50, cmd_32erase = 0x52, cmd_chip_erase = 0x60, cmd_ebsy = 0x70, cmd_dbsy = 0x80, cmd_rdid = 0x90,
	cmd_jedecid = 0x9f, cmd_aai_write = 0xad, cmd_64erase = 0xd8 };


enum { spi_read = 0x1, spi_address = 0x2, spi_dummy = 0x4 };


typedef struct {
	unsigned int addr;
	unsigned char cmd, flags, bufflen, buff[9];
} __attribute__((packed)) spi_msg_t;



#endif
