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


#include <sys/threads.h>
#include <sys/msg.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <unistd.h>
#include <errno.h>

#include "stm32l1-multi.h"
#include "common.h"
#include "dma.h"
#include "rcc.h"
#include "spi.h"


#define SPI1_POS 0
#define SPI2_POS (SPI1_POS + SPI1)
#define SPI3_POS (SPI2_POS + SPI2)


struct {
	volatile unsigned int *base;

	handle_t mutex;
} spi_common[SPI1 + SPI2 + SPI3];


static const int spi2pctl[] = { pctl_spi1, pctl_spi2, pctl_spi3 };


static const int spiConfig[] = { SPI1, SPI2, SPI3 };


static const int spiPos[] = { SPI1_POS, SPI2_POS, SPI3_POS };


enum { cr1 = 0, cr2, sr, dr, crcpr, rxcrcr, txcrcr, i2scfgr, i2spr };


static unsigned char _spi_readwrite(int spi, unsigned char txd)
{
	unsigned char rxd;

	/* Initiate transmission */
	*(spi_common[spi].base + dr) = txd;

	/* Wait until RXNE==1 */
	while (!(*(spi_common[spi].base + sr) & 0x1))
		;

	rxd = *(spi_common[spi].base + dr);

	return rxd;
}


static void _spi_readwrite_dma(int spi, unsigned char *ibuff, unsigned char *obuff, size_t bufflen)
{
	unsigned int dmach = (1 << 1) | (ibuff != NULL);

	*(spi_common[spi].base + cr2) |= dmach;
	dma_transfer_spi(spi, ibuff, obuff, bufflen);
	*(spi_common[spi].base + cr2) &= ~dmach;

	if (ibuff != NULL)
		return;

	/* Wait until RXNE=1 and TXE=1 */
	while ((*(spi_common[spi].base + sr) & 0x3) != 0x3)
		;

	/* Empty out the RX buffer - a crucial step */
	(void) *(spi_common[spi].base + dr);
}


int spi_transaction(int spi, int dir, unsigned char cmd, unsigned int addr, unsigned char flags, unsigned char *ibuff, unsigned char *obuff, size_t bufflen)
{
	int i;
	unsigned int addrsz;

	if (spi < spi1 || spi > spi3 || !spiConfig[spi])
		return -EINVAL;

	spi = spiPos[spi];
	addrsz = (flags >> SPI_ADDRSHIFT) & SPI_ADDRMASK;

	mutexLock(spi_common[spi].mutex);
	keepidle(1);

	if (flags & spi_cmd)
		_spi_readwrite(spi, cmd);

	if (addrsz > 0) {
		if (flags & spi_addrlsb) {
			for (i = 0; i < addrsz; ++i) {
				_spi_readwrite(spi, addr & 0xFF);
				addr >>= 8;
			}
		}
		else {
			for (i = 0; i < addrsz; ++i) {
				_spi_readwrite(spi, (addr >> (addrsz - 1) * 8) & 0xFF);
				addr <<= 8;
			}
		}
	}

	if (flags & spi_dummy)
		_spi_readwrite(spi, 0);

	if (bufflen >= 6) {
		_spi_readwrite_dma(spi, ibuff, obuff, bufflen);
	}
	else {
		if (dir == spi_read) {
			for (i = 0; i < bufflen; ++i)
				ibuff[i] = _spi_readwrite(spi, 0);
		}
		else if (dir == spi_write) {
			for (i = 0; i < bufflen; ++i)
				_spi_readwrite(spi, obuff[i]);
		}
		else {
			for (i = 0; i < bufflen; ++i)
				ibuff[i] = _spi_readwrite(spi, obuff[i]);
		}
	}

	/* Wait until BSY=0 */
	while ((*(spi_common[spi].base + sr) & (1 << 7)) != 0x0)
		;

	keepidle(0);
	mutexUnlock(spi_common[spi].mutex);

	return bufflen;
}


int spi_configure(int spi, char mode, char bdiv, int enable)
{
	int pos;
	unsigned int t;

	if (spi < spi1 || spi > spi3 || !spiConfig[spi])
		return -EINVAL;

	pos = spiPos[spi];

	mutexLock(spi_common[pos].mutex);

	rcc_devClk(spi2pctl[spi], 1);
	*(spi_common[pos].base + cr1) &= ~(1 << 6);

	/* Set mode and baud div */
	t = *(spi_common[pos].base + cr1) & ~((0x7 << 3) | 0x3);
	*(spi_common[pos].base + cr1) = t | (((unsigned int)bdiv & 0x7) << 3) | (mode & 0x3);

	if (enable)
		*(spi_common[pos].base + cr1) |= 1 << 6;
	else
		rcc_devClk(spi2pctl[spi], 0);

	mutexUnlock(spi_common[pos].mutex);

	return EOK;
}


void spi_init(void)
{
	int i, spi;

	static const struct {
		unsigned int base;
		int irq;
	} spiinfo[3] = { { 0x40013000, 35 }, { 0x40003800, 36 }, { 0x40003c00, 47 } };

	for (i = 0, spi = 0; spi < 3; ++spi) {
		if (!spiConfig[spi])
			continue;

		spi_common[i].base = (void *)spiinfo[spi].base;

		mutexCreate(&spi_common[i].mutex);

		rcc_devClk(spi2pctl[spi], 1);

		/* Disable SPI */
		*(spi_common[i].base + cr1) &= ~(1 << 6);
		dataBarier();

		/* fPCLK/2 baudrate, master, mode 0 */
		*(spi_common[i].base + cr1) = (1 << 2);

		/* Motorola frame format, SS output enable */
		*(spi_common[i].base + cr2) = (1 << 2);

		/* SPI mode enabled */
		*(spi_common[i].base + i2scfgr) = 0;

		dma_configure_spi(spi, dma_mem2per, 0x1, (void*) (spi_common[i].base + dr), 0x0, 0x0, 0x1, 0x0);
		dma_configure_spi(spi, dma_per2mem, 0x1, (void*) (spi_common[i].base + dr), 0x0, 0x0, 0x1, 0x0);

		/* Enable SPI */
		*(spi_common[i].base + cr1) |= 1 << 6;

		++i;
	}
}
