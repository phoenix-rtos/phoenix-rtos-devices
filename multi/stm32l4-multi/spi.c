/*
 * Phoenix-RTOS
 *
 * STM32L4 SPI driver
 *
 * Copyright 2018, 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <sys/threads.h>
#include <sys/msg.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

#include "libmulti/libdma.h"
#include "libmulti/libspi.h"

#include "stm32l4-multi.h"
#include "common.h"
#include "config.h"
#include "rcc.h"
#include "spi.h"


#define SPI1_POS 0
#define SPI2_POS (SPI1_POS + SPI1)
#define SPI3_POS (SPI2_POS + SPI2)


static libspi_ctx_t spi_common[SPI1 + SPI2 + SPI3];
static handle_t spi_locks[SPI1 + SPI2 + SPI3];


static const int spiConfig[] = { SPI1, SPI2, SPI3 };


static const int spiPos[] = { SPI1_POS, SPI2_POS, SPI3_POS };


static const int spiUseDma[] = { SPI1_USEDMA, SPI2_USEDMA, SPI3_USEDMA };


int spi_transaction(int spi, int dir, unsigned char cmd, unsigned int addr, unsigned char flags, unsigned char *ibuff, unsigned char *obuff, size_t bufflen)
{
	int ret;

	if (spi < spi1 || spi > spi3 || !spiConfig[spi])
		return -EINVAL;

	mutexLock(spi_locks[spiPos[spi]]);
	ret = libspi_transaction(&spi_common[spiPos[spi]], dir, cmd, addr, flags, ibuff, obuff, bufflen);
	mutexUnlock(spi_locks[spiPos[spi]]);

	return ret;
}


int spi_configure(int spi, char mode, char bdiv, int enable)
{
	int ret;

	if (spi < spi1 || spi > spi3 || !spiConfig[spi])
		return -EINVAL;

	mutexLock(spi_locks[spiPos[spi]]);
	ret = libspi_configure(&spi_common[spiPos[spi]], mode, bdiv, enable);
	mutexUnlock(spi_locks[spiPos[spi]]);

	return ret;
}


void spi_init(void)
{
	int spi;

	for (spi = 0; spi < 3; ++spi) {
		if (!spiConfig[spi])
			continue;

		mutexCreate(&spi_locks[spiPos[spi]]);
		libspi_init(&spi_common[spiPos[spi]], spi, spiUseDma[spi]);
	}
}
