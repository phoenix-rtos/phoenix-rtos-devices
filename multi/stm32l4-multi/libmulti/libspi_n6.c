/*
 * Phoenix-RTOS
 *
 * Multidrv-lib: STM32L4 SPI driver
 *
 * Copyright 2018, 2020 Phoenix Systems
 * Author: Aleksander Kaminski, Daniel Sawka
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <sys/pwman.h>
#include <errno.h>
#include <stdint.h>

#include "../common.h"
#include "libmulti/libdma.h"
#include "libmulti/libspi.h"


int libspi_transaction(libspi_ctx_t *ctx, int dir, unsigned char cmd, unsigned int addr, unsigned char flags, unsigned char *ibuff, const unsigned char *obuff, size_t bufflen)
{
	return -ENOSYS;
}


int libspi_configure(libspi_ctx_t *ctx, char mode, char bdiv, int enable)
{
	return -ENOSYS;
}


int libspi_init(libspi_ctx_t *ctx, unsigned int spi, int useDma)
{
	return -ENOSYS;
}
