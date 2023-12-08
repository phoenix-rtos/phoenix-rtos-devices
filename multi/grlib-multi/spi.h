/*
 * Phoenix-RTOS
 *
 * GRLIB SPI driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _MULTI_SPI_H_
#define _MULTI_SPI_H_


#include <sys/msg.h>


void spi_handleMsg(msg_t *msg, int dev);


int spi_createDevs(oid_t *oid);


int spi_init(void);


#endif
