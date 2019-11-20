/*
 * Phoenix-RTOS
 *
 * i.MX RT SPI driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _SPI_H_
#define _SPI_H_


int spi_handleMsg(msg_t *msg, int dev);


int spi_init(void);


#endif
