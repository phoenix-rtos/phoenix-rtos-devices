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
#include <sys/time.h>
#include <sys/pwman.h>

#include "../common.h"
#include "libmulti/libdma.h"


#define DMA_TCIE_FLAG     (1 << 1)
#define DMA_HTIE_FLAG     (1 << 2)
#define DMA_CIRCULAR_FLAG (1 << 5)

#define DMA_NUM_CONTROLLERS 2
#define DMA_NUM_CHANNELS    7

#define DMA_DMA2_ALT_IRQ_BASE 68

struct {
	volatile unsigned int *base;
	struct {
		handle_t irqLock;
		handle_t cond;
		int taken;
	} channels[DMA_NUM_CHANNELS];
	handle_t takenLock;
} dma_common[DMA_NUM_CONTROLLERS];


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
	{ dma2, { 6, 5 }, 0x2 },  // or { dma1, { 4, 3 }, 0x2 }
	{ dma1, { 5, 6 }, 0x2 },
	{ dma1, { 2, 1 }, 0x2 },
	{ dma2, { 4, 2 }, 0x2 },
	{ dma2, { 1, 0 }, 0x2 },
};


static const struct {
	uintptr_t base;
	int irqBase;
} dmainfo[DMA_NUM_CONTROLLERS] = { { 0x40020000, 11 }, { 0x40020400, 56 } };


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
			volatile int *doneFlag;
		} once;
	};
} dma_transfers[DMA_NUM_CONTROLLERS][DMA_NUM_CHANNELS];


volatile unsigned int *libdma_channelBase(int dma, int channel)
{
	return dma_common[dma].base + 2 + (5 * channel);
}


static void libdma_prepareTransfer(int dma, int channel, void *maddr, size_t len, int flags)
{
	volatile unsigned int *channelBase = libdma_channelBase(dma, channel);
	volatile unsigned int *base = dma_common[dma].base;

	*(base + ifcr) = 1 << (4 * channel);
	dataBarier();
	*(channelBase + cmar) = (unsigned int)maddr;
	*(channelBase + cndtr) = len;
	dataBarier();
	*(channelBase + ccr) |= flags | 0x1;
	dataBarier();
}


static void libdma_unprepareTransfer(int dma, int channel)
{
	volatile unsigned int *channelBase = libdma_channelBase(dma, channel);

	dataBarier();
	/* Disable interrupts, disable circular mode, disable channel */
	*(channelBase + ccr) &= ~(DMA_TCIE_FLAG | DMA_HTIE_FLAG | DMA_CIRCULAR_FLAG | 0x1);
	dataBarier();
}


static unsigned int libdma_irqnum(int dma, int channel)
{
	/* Channels of DMA2 are noncontiguous */
	if ((dma == dma2) && (channel >= 5)) {
		return 16 + DMA_DMA2_ALT_IRQ_BASE - 5 + channel;
	}
	return 16 + dmainfo[dma].irqBase + channel;
}


static int libdma_irqnumToChannel(int dma, int irqnum)
{
	/* Channels of DMA2 are noncontiguous */
	if ((dma == dma2) && ((irqnum == 16 + DMA_DMA2_ALT_IRQ_BASE) || (irqnum == 16 + DMA_DMA2_ALT_IRQ_BASE + 1))) {
		return irqnum - (16 + DMA_DMA2_ALT_IRQ_BASE - 5);
	}
	return irqnum - dmainfo[dma].irqBase - 16;
}


static int libdma_irqHandler(unsigned int n, void *arg)
{
	int dma = (int)arg;
	int channel = libdma_irqnumToChannel(dma, n);
	volatile unsigned int *base = dma_common[dma].base;
	unsigned int flags = (*(base + isr) >> (channel * 4)) & 0xF;

	if (flags == 0) {
		return -1;
	}

	*(base + ifcr) = (1 << (channel * 4));

	switch (dma_transfers[dma][channel].type) {
		case dma_transferOnce:
			libdma_unprepareTransfer(dma, channel);

			if (dma_transfers[dma][channel].once.doneFlag != NULL) {
				*dma_transfers[dma][channel].once.doneFlag = 1;
			}

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
		default:
			/* Shouldn't happen */
			break;
	}

	return 1;
}


static void libdma_configureChannel(int dma, int channel, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc, unsigned char reqmap, handle_t *cond)
{
	unsigned int tmp, irqnum = libdma_irqnum(dma, channel);
	volatile unsigned int *channelBase = libdma_channelBase(dma, channel);
	handle_t interruptCond = (cond == NULL) ? dma_common[dma].channels[channel].cond : *cond;

	interrupt(irqnum, libdma_irqHandler, (void *)dma, interruptCond, NULL);

	*(channelBase + ccr) = ((priority & 0x3) << 12) | ((msize & 0x3) << 10) | ((psize & 0x3) << 8) |
		((minc & 0x1) << 7) | ((pinc & 0x1) << 6) | ((dir & 0x1) << 4);
	*(channelBase + cpar) = (unsigned int)paddr;
	tmp = *(dma_common[dma].base + cselr) & ~(0xF << channel * 4);
	*(dma_common[dma].base + cselr) = tmp | ((unsigned int)reqmap << channel * 4);
}


int libdma_configurePeripheral(const struct libdma_per *per, int dir, int priority, void *paddr, int msize, int psize, int minc, int pinc, handle_t *cond)
{
	int dma = per->dma, channel = per->channel[dir];
	char reqmap = per->reqmap;

	libdma_configureChannel(dma, channel, dir, priority, paddr, msize, psize, minc, pinc, reqmap, cond);

	return 0;
}


static int libdma_hasChannelFinished(volatile unsigned int *channelBase)
{
	return ((channelBase == NULL) || (*(channelBase + cndtr) == 0)) ? 1 : 0;
}


static int libdma_transferTimeout(int dma, int channel, void *maddr, size_t len, int mode, time_t timeout)
{
	time_t now, end, condTimeout;
	volatile unsigned int *channelBase = libdma_channelBase(dma, channel);
	volatile int done = 0;

	dma_transfers[dma][channel].type = dma_transferOnce;
	dma_transfers[dma][channel].once.doneFlag = &done;
	libdma_prepareTransfer(dma, channel, maddr, len, DMA_TCIE_FLAG);

	condTimeout = timeout;
	if (timeout > 0) {
		gettime(&now, NULL);
		end = now + timeout;
	}

	mutexLock(dma_common[dma].channels[channel].irqLock);
	while (done == 0) {
		condWait(dma_common[dma].channels[channel].cond, dma_common[dma].channels[channel].irqLock, condTimeout);
		if (mode == dma_modeNoBlock) {
			break;
		}
		if (timeout != 0) {
			gettime(&now, NULL);
			if (end <= now) {
				break;
			}
			condTimeout = end - now;
		}
	}
	mutexUnlock(dma_common[dma].channels[channel].irqLock);

	if (done == 1) {
		return len;
	}
	/* May result in unpreparing channel twice but that's valid. */
	libdma_unprepareTransfer(dma, channel);
	dma_transfers[dma][channel].type = dma_transferNull;

	return len - *(channelBase + cndtr);
}


static int libdma_transferHelperInterrupts(int dma, int rxChannel, int txChannel, void *rxMAddr, const void *txMAddr, size_t len)
{
	volatile unsigned int *txChannelBase = libdma_channelBase(dma, txChannel);
	volatile int rxDone = 0;

	if (rxMAddr == NULL) {
		return libdma_transferTimeout(dma, txChannel, (void *)txMAddr, len, dma_modeNormal, 0);
	}

	dma_transfers[dma][rxChannel].type = dma_transferOnce;
	dma_transfers[dma][rxChannel].once.doneFlag = &rxDone;
	libdma_prepareTransfer(dma, rxChannel, rxMAddr, len, DMA_TCIE_FLAG);

	/* When doing rw transfer, avoid unnecessary interrupt handling and condSignal()
	by waiting only for RX transfer completion, ignoring TX */
	libdma_prepareTransfer(dma, txChannel, (void *)txMAddr, len, 0);

	mutexLock(dma_common[dma].channels[rxChannel].irqLock);
	while ((rxDone == 0) || (libdma_hasChannelFinished(txChannelBase) == 0)) {
		condWait(dma_common[dma].channels[rxChannel].cond, dma_common[dma].channels[rxChannel].irqLock, 0);
	}
	mutexUnlock(dma_common[dma].channels[rxChannel].irqLock);

	/* Unprepare rx is handled by irq */
	libdma_unprepareTransfer(dma, txChannel);

	return len;
}


static int libdma_transferHelperNoInterrupts(int dma, int rxChannel, int txChannel, void *rxMAddr, const void *txMAddr, size_t len)
{
	volatile unsigned int *rxChannelBase;
	volatile unsigned int *txChannelBase = libdma_channelBase(dma, txChannel);

	if (rxMAddr == NULL) {
		rxChannelBase = NULL;
	}
	else {
		rxChannelBase = libdma_channelBase(dma, rxChannel);
		libdma_prepareTransfer(dma, rxChannel, rxMAddr, len, 0);
	}

	libdma_prepareTransfer(dma, txChannel, (void *)txMAddr, len, 0);

	while ((libdma_hasChannelFinished(rxChannelBase) == 0) || (libdma_hasChannelFinished(txChannelBase) == 0)) {
		;
	}

	if (rxMAddr != NULL) {
		libdma_unprepareTransfer(dma, rxChannel);
	}

	libdma_unprepareTransfer(dma, txChannel);

	return len;
}


static int libdma_transferHelper(int dma, int rxChannel, int txChannel, void *rxMAddr, const void *txMAddr, size_t len)
{
	/* rxMAddr may be NULL, txMAddr must be a valid address */

	/* Empirically chosen value to avoid mutex+cond overhead for short transactions */
	/* TODO: this value was chosen before refactor reducing locking overhead. */
	if (len > 24) {
		return libdma_transferHelperInterrupts(dma, rxChannel, txChannel, rxMAddr, txMAddr, len);
	}
	return libdma_transferHelperNoInterrupts(dma, rxChannel, txChannel, rxMAddr, txMAddr, len);
}


static int libdma_transferAsync(const struct libdma_per *per, void *maddr, int dir, size_t len, volatile int *doneFlag)
{
	int dma = per->dma;
	int channel = per->channel[dir];

	if (DMA_MAX_LEN < len) {
		return -EINVAL;
	}

	*doneFlag = 0;

	dma_transfers[dma][channel].type = dma_transferOnce;
	dma_transfers[dma][channel].once.doneFlag = doneFlag;

	libdma_prepareTransfer(dma, channel, maddr, len, DMA_TCIE_FLAG);

	return 0;
}


int libdma_txAsync(const struct libdma_per *per, const void *txMAddr, size_t len, volatile int *doneFlag)
{
	return libdma_transferAsync(per, (void *)txMAddr, dma_mem2per, len, doneFlag);
}


int libdma_rxAsync(const struct libdma_per *per, void *rxMAddr, size_t len, volatile int *doneFlag)
{
	return libdma_transferAsync(per, rxMAddr, dma_per2mem, len, doneFlag);
}


int libdma_infiniteRxAsync(const struct libdma_per *per, void *rxMAddr, size_t len, void fn(void *arg, int type), void *arg)
{
	int dma = per->dma;
	int channel = per->channel[dma_per2mem];

	if (DMA_MAX_LEN < len) {
		return -EINVAL;
	}

	dma_transfers[dma][channel].type = dma_transferInf;
	dma_transfers[dma][channel].inf.fn = fn;
	dma_transfers[dma][channel].inf.arg = arg;

	libdma_prepareTransfer(dma, channel, rxMAddr, len, DMA_TCIE_FLAG | DMA_HTIE_FLAG | DMA_CIRCULAR_FLAG);

	return 0;
}


int libdma_transfer(const struct libdma_per *per, void *rxMAddr, const void *txMAddr, size_t len)
{
	int res;
	volatile unsigned int *txChannelBase;
	int dma = per->dma;
	int rxChannel = per->channel[dma_per2mem];
	int txChannel = per->channel[dma_mem2per];
	unsigned char txbuf = 0;

	if (DMA_MAX_LEN < len) {
		return -EINVAL;
	}

	if (txMAddr == NULL) {
		/* In case no tx buffer is provided, use a 1-byte dummy
		and configure DMA not to increment the memory address. */
		txChannelBase = libdma_channelBase(dma, txChannel);
		*(txChannelBase + ccr) &= ~(1 << 7);
		res = libdma_transferHelper(dma, rxChannel, txChannel, rxMAddr, &txbuf, len);
		*(txChannelBase + ccr) |= (1 << 7);
	}
	else {
		res = libdma_transferHelper(dma, rxChannel, txChannel, rxMAddr, txMAddr, len);
	}

	return res;
}


int libdma_tx(const struct libdma_per *per, const void *txMAddr, size_t len, int mode, time_t timeout)
{
	int dma = per->dma;
	int txChannel = per->channel[dma_mem2per];

	if (len > DMA_MAX_LEN) {
		return -EINVAL;
	}

	return libdma_transferTimeout(dma, txChannel, (void *)txMAddr, len, mode, timeout);
}


int libdma_rx(const struct libdma_per *per, void *rxMAddr, size_t len, int mode, time_t timeout)
{
	int dma = per->dma;
	int rxChannel = per->channel[dma_per2mem];

	if (len > DMA_MAX_LEN) {
		return -EINVAL;
	}

	return libdma_transferTimeout(dma, rxChannel, rxMAddr, len, mode, timeout);
}


uint16_t libdma_leftToRx(const struct libdma_per *per)
{
	int dma = per->dma;
	int channel = per->channel[dma_per2mem];
	volatile unsigned int *channelBase = libdma_channelBase(dma, channel);

	/* Only bottom 16 bits contain data. */
	return (uint16_t)(*(channelBase + cndtr));
}


int libdma_acquirePeripheral(int per, unsigned int num, const struct libdma_per **perP)
{
	const struct libdma_per *p = NULL;
	int err = 0;

	if (per == dma_spi) {
		if (num < sizeof(libdma_persSpi) / sizeof(libdma_persSpi[0])) {
			p = &libdma_persSpi[num];
		}
	}
	else if (per == dma_uart) {
		if (num < sizeof(libdma_persUart) / sizeof(libdma_persUart[0])) {
			p = &libdma_persUart[num];
		}
	}

	if (p == NULL) {
		return -EINVAL;
	}

	mutexLock(dma_common[(int)p->dma].takenLock);

	if ((dma_common[(int)p->dma].channels[(int)p->channel[0]].taken == 0) && (dma_common[(int)p->dma].channels[(int)p->channel[1]].taken == 0)) {
		dma_common[(int)p->dma].channels[(int)p->channel[0]].taken = 1;
		dma_common[(int)p->dma].channels[(int)p->channel[1]].taken = 1;

		*perP = p;
	}
	else {
		err = -EBUSY;
	}

	mutexUnlock(dma_common[(int)p->dma].takenLock);

	return err;
}


int libdma_init(void)
{
	int dma;
	int channel;
	static int init = 0;

	if (init != 0) {
		return 0;
	}

	init = 1;

	for (dma = 0; dma < DMA_NUM_CONTROLLERS; ++dma) {
		dma_common[dma].base = (void *)dmainfo[dma].base;

		mutexCreate(&dma_common[dma].takenLock);

		for (channel = 0; channel < DMA_NUM_CHANNELS; channel++) {
			condCreate(&dma_common[dma].channels[channel].cond);

			/* Only purpose of the mutex is to be able to use condWait. */
			/* Synchronization and exclusion on channels must be done externally by the user. */
			mutexCreate(&dma_common[dma].channels[channel].irqLock);

			dma_common[dma].channels[channel].taken = 0;
		}

		devClk(dma2pctl[dma], 1);
	}

	return 0;
}
