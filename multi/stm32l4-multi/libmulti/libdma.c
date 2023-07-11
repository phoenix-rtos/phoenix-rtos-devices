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
#include <sys/interrupt.h>

#include "../common.h"
#include "libmulti/libdma.h"


#define DMA_TCIE_FLAG     (1 << 1)
#define DMA_HTIE_FLAG     (1 << 2)
#define DMA_CIRCULAR_FLAG (1 << 5)


struct {
	volatile unsigned int *base;
	int irq_base;
	handle_t irqLock;
	handle_t cond;
} dma_common[2];


static const int dma2pctl[] = { pctl_dma1, pctl_dma2 };


struct libdma_per {
	char dma;
	char channel[2];
	char reqmap;
};


/* clang-format off */
enum { dma1 = 0, dma2 };


enum { isr = 0, ifcr, cselr = 42 };


enum { ccr = 0, cndtr, cpar, cmar };
/* clang-format on */


static const struct libdma_per libdma_persSpi[] = {
	{ dma1, { 1, 2 }, 0x1 },
	{ dma1, { 3, 4 }, 0x1 },
	{ dma2, { 0, 1 }, 0x3 },
};


static const struct libdma_per libdma_persUart[] = {
	{ dma1, { 4, 3 }, 0x2 },  // or { dma2, { 6, 5 }, 0x2 }
	{ dma1, { 5, 6 }, 0x2 },
	{ dma1, { 2, 1 }, 0x2 },
	{ dma2, { 4, 2 }, 0x2 },
	{ dma2, { 1, 0 }, 0x2 },
};


static const struct {
	unsigned int base;
	int irq_base;
} dmainfo[2] = { { 0x40020000, 11 }, { 0x40020400, 56 } };


static struct {
	enum {
		dma_transferNull = 0,
		dma_transferInf,
		dma_transferOnce,
	} type;
	union {
		struct {
			void (*fn)(void *arg, int type);
			void *arg;
		} inf;
		struct {
			volatile int *done_flag;
			volatile unsigned int *channel_base;
		} once;
	};
} dma_transfers[2][8];


static void _prepare_transfer(volatile unsigned int *channel_base, void *maddr, size_t len, int flags)
{
	*(channel_base + cmar) = (unsigned int)maddr;
	*(channel_base + cndtr) = len;
	dataBarier();
	*(channel_base + ccr) |= flags | 0x1;
	dataBarier();
}


static void _unprepare_transfer(volatile unsigned int *channel_base)
{
	dataBarier();
	/* Disable interrupts, disable channel */
	*(channel_base + ccr) &= ~(DMA_TCIE_FLAG | DMA_HTIE_FLAG | 0x1);
	dataBarier();
}


static int irqHandler(unsigned int n, void *arg)
{
	int dma = (int)arg;
	int channel = n - dma_common[dma].irq_base - 16;
	volatile unsigned int *base = dma_common[dma].base;
	unsigned int flags = (*(base + isr) & (0xF << (channel * 4))) >> (channel * 4);

	if (flags == 0) {
		return -1;
	}

	*(base + ifcr) = (0xF << (channel * 4));

	switch (dma_transfers[dma][channel].type) {
		case dma_transferOnce:
			_unprepare_transfer(dma_transfers[dma][channel].once.channel_base);

			*dma_transfers[dma][channel].once.done_flag = 1;

			dma_transfers[dma][channel].type = dma_transferNull;
			break;
		case dma_transferInf:
			if ((flags & DMA_HTIE_FLAG) != 0) {
				dma_transfers[dma][channel].inf.fn(dma_transfers[dma][channel].inf.arg, dma_ht);
			}
			if ((flags & DMA_TCIE_FLAG) != 0) {
				dma_transfers[dma][channel].inf.fn(dma_transfers[dma][channel].inf.arg, dma_tc);
			}
			break;
		case dma_transferNull:
			/* Shouldn't happen */
			break;
	}

	return 1;
}


static void _configureChannel(int dma, int channel, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc, unsigned char reqmap, handle_t *cond)
{
	unsigned int tmp, irqnum;
	volatile unsigned int *channel_base = dma_common[dma].base + 2 + (5 * channel);
	handle_t interruptCond = (cond == NULL) ? dma_common[dma].cond : *cond;

	/* Channels of DMA2 are noncontiguous */
	if (dma == dma2 && channel >= 5) {
		irqnum = 16 + 68 - 5 + channel;
	}
	else {
		irqnum = 16 + dmainfo[dma].irq_base + channel;
	}

	interrupt(irqnum, irqHandler, (void *)dma, interruptCond, NULL);

	*(channel_base + ccr) = ((priority & 0x3) << 12) | ((msize & 0x3) << 10) | ((psize & 0x3) << 8) |
		((minc & 0x1) << 7) | ((pinc & 0x1) << 6) | ((dir & 0x1) << 4);
	*(channel_base + cpar) = (unsigned int)paddr;
	tmp = *(dma_common[dma].base + cselr) & ~(0xF << channel * 4);
	*(dma_common[dma].base + cselr) = tmp | ((unsigned int)reqmap << channel * 4);
}


int libdma_configurePeripheral(const struct libdma_per *per, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc, handle_t *cond)
{
	int dma = per->dma, channel = per->channel[dir];
	char reqmap = per->reqmap;

	_configureChannel(dma, channel, dir, priority, paddr, msize, psize, minc, pinc, reqmap, cond);

	return 0;
}


static int _has_channel_finished(const void *maddr, volatile unsigned int *channel_base)
{
	return ((maddr != NULL) && (*(channel_base + cndtr) > 0)) ? 1 : 0;
}


static int _transfer(int dma, int rx_channel, int tx_channel, void *rx_maddr, const void *tx_maddr, size_t len)
{
	/* Empirically chosen value to avoid mutex+cond overhead for short transactions */
	int use_interrupts = len > 24;
	int interrupts_flags = (use_interrupts == 0) ? 0 : DMA_TCIE_FLAG;

	volatile unsigned int *rx_channel_base = dma_common[dma].base + 2 + (5 * rx_channel);
	volatile unsigned int *tx_channel_base = dma_common[dma].base + 2 + (5 * tx_channel);

	if (rx_maddr != NULL) {
		_prepare_transfer(rx_channel_base, rx_maddr, len, interrupts_flags);
	}

	if (tx_maddr != NULL) {
		/* When doing rw transfer, avoid unnecessary interrupt handling and condSignal()
		by waiting only for RX transfer completion, ignoring TX */
		_prepare_transfer(tx_channel_base, (void *)tx_maddr, len, rx_maddr == NULL ? interrupts_flags : 0);
	}

	if (use_interrupts != 0) {
		mutexLock(dma_common[dma].irqLock);
		while (_has_channel_finished(rx_maddr, rx_channel_base) || _has_channel_finished(tx_maddr, tx_channel_base)) {
			condWait(dma_common[dma].cond, dma_common[dma].irqLock, 1);
		}
		mutexUnlock(dma_common[dma].irqLock);
	}
	else {
		while (_has_channel_finished(rx_maddr, rx_channel_base) || _has_channel_finished(tx_maddr, tx_channel_base))
			;
	}

	if (rx_maddr != NULL) {
		_unprepare_transfer(rx_channel_base);
	}

	if (tx_maddr != NULL) {
		_unprepare_transfer(tx_channel_base);
	}

	return len;
}


static int libdma_transferAsync(const struct libdma_per *per, void *maddr, int dir, size_t len, volatile int *done_flag)
{
	int dma = per->dma;
	int channel = per->channel[dir];
	volatile unsigned int *channel_base = dma_common[dma].base + 2 + (5 * channel);

	/* Only one request may be issued on one channel at one time. */
	if ((dma_transfers[dma][channel].type != dma_transferNull) || (DMA_MAX_LEN < len)) {
		return -EINVAL;
	}

	*done_flag = 0;

	dma_transfers[dma][channel].type = dma_transferOnce;
	dma_transfers[dma][channel].once.done_flag = done_flag;
	dma_transfers[dma][channel].once.channel_base = channel_base;

	_prepare_transfer(channel_base, maddr, len, DMA_TCIE_FLAG);

	return 0;
}


int libdma_txAsync(const struct libdma_per *per, const void *tx_maddr, size_t len, volatile int *done_flag)
{
	return libdma_transferAsync(per, (void *)tx_maddr, dma_mem2per, len, done_flag);
}


int libdma_rxAsync(const struct libdma_per *per, void *rx_maddr, size_t len, volatile int *done_flag)
{
	return libdma_transferAsync(per, rx_maddr, dma_per2mem, len, done_flag);
}


int libdma_infiniteRxAsync(const struct libdma_per *per, void *rx_maddr, size_t len, void fn(void *arg, int type), void *arg)
{
	int dma = per->dma;
	int channel = per->channel[dma_per2mem];
	volatile unsigned int *channel_base = dma_common[dma].base + 2 + (5 * channel);

	/* Only one request may be issued on one channel at one time. */
	if ((dma_transfers[dma][channel].type != dma_transferNull) || (DMA_MAX_LEN < len)) {
		return -EINVAL;
	}

	dma_transfers[dma][channel].type = dma_transferInf;
	dma_transfers[dma][channel].inf.fn = fn;
	dma_transfers[dma][channel].inf.arg = arg;

	_prepare_transfer(channel_base, rx_maddr, len, DMA_TCIE_FLAG | DMA_HTIE_FLAG | DMA_CIRCULAR_FLAG);

	return 0;
}


int libdma_transfer(const struct libdma_per *per, void *rx_maddr, const void *tx_maddr, size_t len)
{
	int res;
	volatile unsigned int *tx_channel_base;
	int dma = per->dma;
	int rx_channel = per->channel[dma_per2mem];
	int tx_channel = per->channel[dma_mem2per];
	unsigned char txbuf = 0;

	if (DMA_MAX_LEN < len) {
		return -EINVAL;
	}

	if (tx_maddr == NULL) {
		/* In case no tx buffer is provided, use a 1-byte dummy
		and configure DMA not to increment the memory address. */
		tx_channel_base = dma_common[dma].base + 2 + (5 * tx_channel);
		*(tx_channel_base + ccr) &= ~(1 << 7);
		res = _transfer(dma, rx_channel, tx_channel, rx_maddr, &txbuf, len);
		*(tx_channel_base + ccr) |= (1 << 7);
	}
	else {
		res = _transfer(dma, rx_channel, tx_channel, rx_maddr, tx_maddr, len);
	}

	return res;
}


uint16_t libdma_leftToRx(const struct libdma_per *per)
{
	int dma = per->dma;
	int channel = per->channel[dma_per2mem];
	volatile unsigned int *channel_base = dma_common[dma].base + 2 + (5 * channel);

	/* Only bottom 16 bits contain data. */
	return (uint16_t)(*(channel_base + cndtr));
}


const struct libdma_per *libdma_getPeripheral(int per, unsigned int num)
{
	switch (per) {
		case dma_spi:
			if (num >= sizeof(libdma_persSpi) / sizeof(libdma_persSpi[0])) {
				return NULL;
			}
			return &libdma_persSpi[num];
		case dma_uart:
			if (num >= sizeof(libdma_persUart) / sizeof(libdma_persUart[0])) {
				return NULL;
			}
			return &libdma_persUart[num];
		default:
			return NULL;
	}
}


int libdma_init(void)
{
	int dma;
	static int init = 0;

	if (init != 0) {
		return 0;
	}

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
