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

#include "lib/libdma.h"

#include "stm32-multi.h"
#include "common.h"
#include "rcc.h"
#include "spi.h"


#define SPI1_POS 0
#define SPI2_POS (SPI1_POS + SPI1)
#define SPI3_POS (SPI2_POS + SPI2)


struct {
	volatile uint16_t *base;

	unsigned char *ibuff;
	unsigned char *obuff;
	size_t cnt;

	int usedma;

	handle_t mutex;
	handle_t irqLock;
	handle_t cond;
	handle_t inth;
} spi_common[SPI1 + SPI2 + SPI3];


static const int spi2pctl[] = { pctl_spi1, pctl_spi2, pctl_spi3 };


static const int spiConfig[] = { SPI1, SPI2, SPI3 };


static const int spiPos[] = { SPI1_POS, SPI2_POS, SPI3_POS };


static const int spiUseDma[] = { SPI1_USEDMA, SPI2_USEDMA, SPI3_USEDMA };


enum { cr1 = 0, cr2 = 2, sr = 4, dr = 6, crcpr = 8, rxcrcr = 10, txcrcr = 12 };


static int spi_irqHandler(unsigned int n, void *arg)
{
	*(spi_common[(int)arg].ibuff++) = *((volatile uint8_t *)(spi_common[(int)arg].base + dr));
	if (--spi_common[(int)arg].cnt)
		*((volatile uint8_t *)(spi_common[(int)arg].base + dr)) = *(spi_common[(int)arg].obuff++);
	else {
		*(spi_common[(int)arg].base + cr2) &= ~(1 << 6);
		return 1;
	}

	return -1;
}


static unsigned char _spi_readwrite(int spi, unsigned char txd)
{
	unsigned char rxd;

	/* Initiate transmission */
	*((volatile uint8_t *)(spi_common[spi].base + dr)) = txd;

	/* Wait until RXNE==1 */
	while (!(*(spi_common[spi].base + sr) & 0x1))
		;

	rxd = *((volatile uint8_t *)(spi_common[spi].base + dr));

	return rxd;
}


static void _spi_readwriteIrq(int spi, unsigned char *ibuff, unsigned char *obuff, size_t bufflen)
{
	spi_common[spi].ibuff = ibuff;
	spi_common[spi].obuff = obuff + 1;
	spi_common[spi].cnt = bufflen;

	/* Initiate transmission */
	*(spi_common[spi].base + cr2) |= 1 << 6;
	*((volatile uint8_t *)(spi_common[spi].base + dr)) = obuff[0];

	mutexLock(spi_common[spi].irqLock);
	while (spi_common[spi].cnt)
		condWait(spi_common[spi].cond, spi_common[spi].irqLock, 1);
	mutexUnlock(spi_common[spi].irqLock);
}


static void _spi_readwriteDma(int spi, unsigned char *ibuff, unsigned char *obuff, size_t bufflen)
{
	unsigned int dmach = (1 << 1) | (ibuff != NULL);

	*(spi_common[spi].base + cr2) |= dmach;
	libdma_transferSpi(spi, ibuff, obuff, bufflen);
	*(spi_common[spi].base + cr2) &= ~dmach;

	if (ibuff != NULL)
		return;

	/* Wait until RXNE=1 and TXE=1, i.e. the whole transfer is complete */
	while ((*(spi_common[spi].base + sr) & 0x3) != 0x3)
		;

	/* Empty out the RX FIFO completely - leaving anything in the FIFO will corrupt subsequent transfers */
	while (*(spi_common[spi].base + sr) & 0x1)
		(void) *((volatile uint8_t *)(spi_common[spi].base + dr));
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
		if (spi_common[spi].usedma)
			_spi_readwriteDma(spi, ibuff, obuff, bufflen);
		else
			_spi_readwriteIrq(spi, ibuff, obuff, bufflen);
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

	devClk(spi2pctl[spi], 1);
	*(spi_common[pos].base + cr1) &= ~(1 << 6);

	/* Set mode and baud div */
	t = *(spi_common[pos].base + cr1) & ~((0x7 << 3) | 0x3);
	*(spi_common[pos].base + cr1) = t | (((unsigned int)bdiv & 0x7) << 3) | (1 << 2) | (mode & 0x3);

	/* 8-bit RXNE threshold, 8 bits, motorola frame format, SS output enable */
	*(spi_common[pos].base + cr2) |= (1 << 12) | (0x7 << 8) | (1 << 2);

	if (enable)
		*(spi_common[pos].base + cr1) |= 1 << 6;
	else
		devClk(spi2pctl[spi], 0);

	mutexUnlock(spi_common[pos].mutex);

	return EOK;
}


void spi_init(void)
{
	int i, spi;

	static const struct {
		unsigned int base;
		int irq;
	} spiinfo[3] = { { 0x40013000, spi1_irq }, { 0x40003800, spi2_irq }, { 0x40003c00, spi3_irq } };

	libdma_init();

	for (i = 0, spi = 0; spi < 3; ++spi) {
		if (!spiConfig[spi])
			continue;

		spi_common[i].base = (void *)spiinfo[spi].base;

		spi_common[i].usedma = spiUseDma[spi];

		mutexCreate(&spi_common[i].mutex);
		mutexCreate(&spi_common[i].irqLock);
		condCreate(&spi_common[i].cond);

		spi_configure(spi, 0, 0, 1);

		if (spi_common[i].usedma) {
			libdma_configureSpi(spi, dma_mem2per, 0x1, (void*) (spi_common[i].base + dr), 0x0, 0x0, 0x1, 0x0);
			libdma_configureSpi(spi, dma_per2mem, 0x1, (void*) (spi_common[i].base + dr), 0x0, 0x0, 0x1, 0x0);
		}
		else {
			interrupt(spiinfo[spi].irq, spi_irqHandler, (void *)i, spi_common[i].cond, &spi_common[i].inth);
		}

		++i;
	}
}
