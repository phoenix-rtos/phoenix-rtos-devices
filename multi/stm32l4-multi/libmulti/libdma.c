/*
 * Phoenix-RTOS
 *
 * STM32L4 DMA driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Daniel Sawka, Aleksander Kaminski
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include "../common.h"
#include "libmulti/libdma.h"

struct {
	volatile unsigned int *base;
	int irq_base;
	handle_t irqLock;
	handle_t cond;
} dma_common[2];


enum { dma1 = 0, dma2 };


static const int dma2pctl[] = { pctl_dma1, pctl_dma2 };


static const struct dma_map {
	char dma;
	char channel[2];
	char reqmap;
} spiChanMap[3] = {
	{ dma1, { 1, 2 }, 0x1 },
	{ dma1, { 3, 4 }, 0x1 },
	{ dma2, { 0, 1 }, 0x3 },
};


static const struct {
	unsigned int base;
	int irq_base;
} dmainfo[2] = { { 0x40020000, 11 }, { 0x40020400, 56 } };


enum { isr = 0, ifcr, cselr = 42 };


enum { ccr = 0, cndtr, cpar, cmar };


static int irqHandler(unsigned int n, void *arg)
{
	int channel = n - dma_common[(int)arg].irq_base - 16;
	volatile unsigned int *base = dma_common[(int)arg].base;

	if (*(base + isr) & (0xF << channel * 4)) {
		*(base + ifcr) = (0xF << channel * 4);
		return 1;
	} else {
		return -1;
	}
}


static void _configureChannel(int dma, int channel, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc, unsigned char reqmap)
{
	unsigned int tmp, irqnum;
	volatile unsigned int *channel_base = dma_common[dma].base + 2 + 5 * channel;

	/* Channels of DMA2 are noncontiguous */
	if (dma == dma2 && channel >= 5) {
		irqnum = 16 + 68 - 5 + channel;
	}
	else {
		irqnum = 16 + dmainfo[dma].irq_base + channel;
	}

	interrupt(irqnum, irqHandler, (void *)dma, dma_common[dma].cond, NULL);

	*(channel_base + ccr) = ((priority & 0x3) << 12) | ((msize & 0x3) << 10) | ((psize & 0x3) << 8) |
		((minc & 0x1) << 7) | ((pinc & 0x1) << 6) | ((dir & 0x1) << 4);
	*(channel_base + cpar) = (unsigned int) paddr;
	tmp = *(dma_common[dma].base + cselr) & ~(0xF << channel * 4);
	*(dma_common[dma].base + cselr) = tmp | ((unsigned int) reqmap << channel * 4);
}


int libdma_configureSpi(int num, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc)
{
	int dma = spiChanMap[num].dma;
	int channel = spiChanMap[num].channel[dir];
	char reqmap = spiChanMap[num].reqmap;

	_configureChannel(dma, channel, dir, priority, paddr, msize, psize, minc, pinc, reqmap);

	return 0;
}


static void _prepare_transfer(volatile unsigned int *channel_base, void *maddr, size_t len, int tcie)
{
	*(channel_base + cmar) = (unsigned int) maddr;
	*(channel_base + cndtr) = len;

	if (tcie)
		/* Enable Transfer Complete interrupt, enable channel */
		*(channel_base + ccr) |= (1 << 1) | 0x1;
	else
		/* Enable channel */
		*(channel_base + ccr) |= 0x1;
}


static void _unprepare_transfer(volatile unsigned int *channel_base)
{
	/* Disable Transfer Complete interrupt, disable channel */
	*(channel_base + ccr) &= ~((1 << 1) | 0x1);
}


static int _has_channel_finished(const void *maddr, volatile unsigned int *channel_base)
{
	return (maddr != NULL && *(channel_base + cndtr) > 0);
}


static int _transfer(int dma, int rx_channel, int tx_channel, void *rx_maddr, const void *tx_maddr, size_t len)
{
	/* Empirically chosen value to avoid mutex+cond overhead for short transactions */
	int use_interrupts = len > 24;

	volatile unsigned int *rx_channel_base = dma_common[dma].base + 2 + 5 * rx_channel;
	volatile unsigned int *tx_channel_base = dma_common[dma].base + 2 + 5 * tx_channel;

	if (rx_maddr != NULL)
		_prepare_transfer(rx_channel_base, rx_maddr, len, use_interrupts);

	if (tx_maddr != NULL)
		/* When doing rw transfer, avoid unnecessary interrupt handling and condSignal()
		by waiting only for RX transfer completion, ignoring TX */
		_prepare_transfer(tx_channel_base, (void *)tx_maddr, len, use_interrupts && rx_maddr == NULL);

	if (use_interrupts) {
		mutexLock(dma_common[dma].irqLock);
		while (_has_channel_finished(rx_maddr, rx_channel_base) || _has_channel_finished(tx_maddr, tx_channel_base))
			condWait(dma_common[dma].cond, dma_common[dma].irqLock, 1);
		mutexUnlock(dma_common[dma].irqLock);
	}
	else {
		while (_has_channel_finished(rx_maddr, rx_channel_base) || _has_channel_finished(tx_maddr, tx_channel_base))
			;
	}

	if (rx_maddr != NULL)
		_unprepare_transfer(rx_channel_base);

	if (tx_maddr != NULL)
		_unprepare_transfer(tx_channel_base);

	return len;
}


int libdma_transferSpi(int num, void *rx_maddr, const void *tx_maddr, size_t len)
{
	int res;
	volatile unsigned int *tx_channel_base;
	int dma = spiChanMap[num].dma;
	int rx_channel = spiChanMap[num].channel[dma_per2mem];
	int tx_channel = spiChanMap[num].channel[dma_mem2per];
	unsigned char txbuf = 0;

	if (tx_maddr == NULL) {
		/* In case no tx buffer is provided, use a 1-byte dummy
		and configure DMA not to increment the memory address. */
		tx_channel_base = dma_common[dma].base + 2 + 5 * tx_channel;
		*(tx_channel_base + ccr) &= ~(1 << 7);
		res = _transfer(dma, rx_channel, tx_channel, rx_maddr, &txbuf, len);
		*(tx_channel_base + ccr) |= (1 << 7);
	}
	else {
		res = _transfer(dma, rx_channel, tx_channel, rx_maddr, tx_maddr, len);
	}

	return res;
}


int libdma_init(void)
{
	int dma;
	static int init = 0;

	if (init)
		return 0;

	init = 1;

	for (dma = 0; dma < 2; ++dma) {
		dma_common[dma].base = (void *)dmainfo[dma].base;
		dma_common[dma].irq_base = dmainfo[dma].irq_base;

		mutexCreate(&dma_common[dma].irqLock);
		condCreate(&dma_common[dma].cond);

		devClk(dma2pctl[dma], 1);
	}

	return 0;
}
