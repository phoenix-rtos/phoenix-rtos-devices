/*
 * Phoenix-RTOS
 *
 * STM32L1 DMA driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Daniel Sawka
 *
 * %LICENSE%
 */


#include <errno.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include "common.h"
#include "dma.h"
#include "rcc.h"


#define DMA1_CH1 0
#define DMA1_CH2 (SPI1)
#define DMA1_CH3 (SPI1)
#define DMA1_CH4 (SPI2)
#define DMA1_CH5 (SPI2)
#define DMA1_CH6 0
#define DMA1_CH7 0

#define DMA2_CH1 (SPI3)
#define DMA2_CH2 (SPI3)
#define DMA2_CH3 0
#define DMA2_CH4 0
#define DMA2_CH5 0

#define DMA1 (DMA1_CH1 || DMA1_CH2 || DMA1_CH3 || DMA1_CH4 || DMA1_CH5 || DMA1_CH6 || DMA1_CH7)
#define DMA2 (DMA2_CH1 || DMA2_CH2 || DMA2_CH3 || DMA2_CH4 || DMA2_CH5)

#define DMA1_POS 0
#define DMA2_POS (DMA1_POS + DMA1)


struct {
	volatile unsigned int *base;
	int irq_base;
	handle_t irqLock;
	handle_t cond;
} dma_common[DMA1 + DMA2];


enum { dma1 = 0, dma2 };


static const int dma2pctl[] = { pctl_dma1, pctl_dma2 };


static const unsigned char dmaConfig[] = { DMA1, DMA2 };


static const unsigned char dmaChannelConfig[2][7] = {
	{ DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7 },
	{ DMA2_CH1, DMA2_CH2, DMA2_CH3, DMA2_CH4, DMA2_CH5, 0, 0 }
};


static const int dmaPos[] = { DMA1_POS, DMA2_POS };


static const struct dma_map {
	char dma;
	char channel[2];
} spiChanMap[3] = {
	{ dma1, { 1, 2 } },
	{ dma1, { 3, 4 } },
	{ dma2, { 0, 1 } },
};


enum { isr = 0, ifcr };


enum { ccr = 0, cndtr, cpar, cmar };


static int dma_irqHandler(unsigned int n, void *arg)
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


static void _configure_channel(int dma, int channel, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc)
{
	int pos = dmaPos[dma];
	volatile unsigned int *channel_base = dma_common[pos].base + 2 + 5 * channel;

	*(channel_base + ccr) = ((priority & 0x3) << 12) | ((msize & 0x3) << 10) | ((psize & 0x3) << 8) |
		((minc & 0x1) << 7) | ((pinc & 0x1) << 6) | ((dir & 0x1) << 4);
	*(channel_base + cpar) = (unsigned int) paddr;
}


int dma_configure_spi(int num, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc)
{
	int dma = spiChanMap[num].dma;
	int channel = spiChanMap[num].channel[dir];

	_configure_channel(dma, channel, dir, priority, paddr, msize, psize, minc, pinc);

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


static int _has_channel_finished(void *maddr, volatile unsigned int *channel_base)
{
	return (maddr != NULL && *(channel_base + cndtr) > 0);
}


static int _transfer(int dma, int rx_channel, int tx_channel, void *rx_maddr, void *tx_maddr, size_t len)
{
	/* Empirically chosen value to avoid mutex+cond overhead for short transactions */
	int use_interrupts = len > 24;
	int pos = dmaPos[dma];

	volatile unsigned int *rx_channel_base = dma_common[pos].base + 2 + 5 * rx_channel;
	volatile unsigned int *tx_channel_base = dma_common[pos].base + 2 + 5 * tx_channel;

	if (rx_maddr != NULL)
		_prepare_transfer(rx_channel_base, rx_maddr, len, use_interrupts);

	if (tx_maddr != NULL)
		/* When doing rw transfer, avoid unnecessary interrupt handling and condSignal()
		by waiting only for RX transfer completion, ignoring TX */
		_prepare_transfer(tx_channel_base, tx_maddr, len, use_interrupts && rx_maddr == NULL);

	if (use_interrupts) {
		mutexLock(dma_common[pos].irqLock);
		while (_has_channel_finished(rx_maddr, rx_channel_base) || _has_channel_finished(tx_maddr, tx_channel_base))
			condWait(dma_common[pos].cond, dma_common[pos].irqLock, 1);
		mutexUnlock(dma_common[pos].irqLock);
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


int dma_transfer_spi(int num, void *rx_maddr, void *tx_maddr, size_t len)
{
	int pos;
	int res;
	volatile unsigned int *tx_channel_base;
	int dma = spiChanMap[num].dma;
	int rx_channel = spiChanMap[num].channel[dma_per2mem];
	int tx_channel = spiChanMap[num].channel[dma_mem2per];
	unsigned char txbuf = 0;

	if (tx_maddr == NULL) {
		/* In case no tx buffer is provided, use a 1-byte dummy
		and configure DMA not to increment the memory address. */
		pos = dmaPos[dma];
		tx_channel_base = dma_common[pos].base + 2 + 5 * tx_channel;
		*(tx_channel_base + ccr) &= ~(1 << 7);
		res = _transfer(dma, rx_channel, tx_channel, rx_maddr, &txbuf, len);
		*(tx_channel_base + ccr) |= (1 << 7);
	}
	else {
		res = _transfer(dma, rx_channel, tx_channel, rx_maddr, tx_maddr, len);
	}

	return res;
}


int dma_init(void)
{
	int i, j, dma;

	static const struct {
		unsigned int base;
		int irq_base;
	} dmainfo[2] = { { 0x40026000, 11 }, { 0x40026400, 50 } };

	for (i = 0, dma = 0; dma < 2; ++dma) {
		if (!dmaConfig[dma])
			continue;

		dma_common[i].base = (void *)dmainfo[dma].base;
		dma_common[i].irq_base = dmainfo[dma].irq_base;

		mutexCreate(&dma_common[i].irqLock);
		condCreate(&dma_common[i].cond);

		for (j = 0; j < 7; j++) {
			if (!dmaChannelConfig[dma][j])
				continue;

			interrupt(16 + dmainfo[dma].irq_base + j, dma_irqHandler, (void *)i, dma_common[i].cond, NULL);
		}

		rcc_devClk(dma2pctl[dma], 1);

		++i;
	}

	return 0;
}
