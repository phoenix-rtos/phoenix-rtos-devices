/*
 * Phoenix-RTOS
 *
 * RTT pipes: communication through debug probe
 *
 * Copyright 2023-2024 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <board_config.h>


static inline void dataMemoryBarrier(void)
{
#if (defined __ARM_ARCH_7EM__)                                  /* Cortex-M4/M7 */ \
	|| (defined __ARM_ARCH_8M_BASE__)                           /* Cortex-M23 */ \
	|| (defined __ARM_ARCH_8M_MAIN__)                           /* Cortex-M33 */ \
	|| ((defined __ARM_ARCH_7A__) || (defined __ARM_ARCH_7R__)) /* Cortex-A/R 32-bit ARMv7-A/R */

	/* clang-format off */
	__asm__ volatile("dmb\n":::);
	/* clang-format on */
#endif
}


/*
 * Note: LIBRTT_TAG needs to be backward written string. This tag is used by
 * the RTT remote side e.g. openocd to find descriptor location during memory
 * scan, so as not to mislead the descriptor scan procedure, this tag needs
 * to be simply hidden before it is copied into RAM together with initialized
 * descriptors, that's why it is written backwards.
 * - Default LIBRTT_TAG_REVERSED="EPIP TTR", which corresponds to "RTT PIPE"
 */

/* clang-format off */

#ifndef LIBRTT_TAG_BACKWARD
#define LIBRTT_TAG_BACKWARD     "EPIP TTR"
#endif


#ifndef LIBRTT_TXCHANNELS
#define LIBRTT_TXCHANNELS       2
#define LIBRTT_TXCHANNELS_NAMES { "rtt1_tx", "rtt2_tx" }
#endif

#ifndef LIBRTT_RXCHANNELS
#define LIBRTT_RXCHANNELS       2
#define LIBRTT_RXCHANNELS_NAMES { "rtt1_rx", "rtt2_rx" }
#endif

/* clang-format on */


struct rtt_pipe {
	const char *name;
	unsigned char *ptr;
	unsigned int sz;
	volatile unsigned int wr;
	volatile unsigned int rd;
	unsigned int flags;
};


struct rtt_desc {
	char tag[16];
	unsigned int txChannels;
	unsigned int rxChannels;
	struct rtt_pipe txChannel[LIBRTT_TXCHANNELS];
	struct rtt_pipe rxChannel[LIBRTT_RXCHANNELS];
};


static const char rtt_tagReversed[] = LIBRTT_TAG_BACKWARD;
static const char *const rtt_txName[LIBRTT_TXCHANNELS] = LIBRTT_TXCHANNELS_NAMES;
static const char *const rtt_rxName[LIBRTT_RXCHANNELS] = LIBRTT_RXCHANNELS_NAMES;


static struct {
	volatile struct rtt_desc *rtt;
	void *memptr;
} librtt_common = { 0 };


int rtt_check(int chan)
{
	if (librtt_common.rtt == NULL || chan < 0 || chan >= LIBRTT_TXCHANNELS) {
		return -ENODEV;
	}

	return 0;
}


ssize_t rtt_read(int chan, void *buf, size_t count)
{
	volatile struct rtt_desc *rtt = librtt_common.rtt;

	if (rtt_check(chan) < 0) {
		return -ENODEV;
	}

	dataMemoryBarrier();

	unsigned char *srcBuf = (unsigned char *)rtt->rxChannel[chan].ptr;
	unsigned char *dstBuf = (unsigned char *)buf;
	unsigned int sz = rtt->rxChannel[chan].sz - 1;
	unsigned int rd = rtt->rxChannel[chan].rd & sz;
	unsigned int wr = rtt->rxChannel[chan].wr & sz;
	size_t todo = count;

	while ((todo != 0) && (rd != wr)) {
		*dstBuf++ = srcBuf[rd];
		rd = (rd + 1) & sz;
		todo--;
	}

	rtt->rxChannel[chan].rd = rd;

	dataMemoryBarrier();

	return count - todo;
}


ssize_t rtt_write(int chan, const void *buf, size_t count)
{
	volatile struct rtt_desc *rtt = librtt_common.rtt;

	if (rtt_check(chan) < 0) {
		return -ENODEV;
	}

	dataMemoryBarrier();

	const unsigned char *srcBuf = (const unsigned char *)buf;
	unsigned char *dstBuf = (unsigned char *)rtt->txChannel[chan].ptr;
	unsigned int sz = rtt->txChannel[chan].sz - 1;
	unsigned int rd = (rtt->txChannel[chan].rd + sz) & sz;
	unsigned int wr = rtt->txChannel[chan].wr & sz;
	size_t todo = count;

	while ((todo != 0) && (rd != wr)) {
		dstBuf[wr] = *srcBuf++;
		wr = (wr + 1) & sz;
		todo--;
	}

	rtt->txChannel[chan].wr = wr;

	dataMemoryBarrier();

	return count - todo;
}


ssize_t rtt_txAvail(int chan)
{
	volatile struct rtt_desc *rtt = librtt_common.rtt;

	dataMemoryBarrier();
	unsigned int sz = rtt->txChannel[chan].sz - 1;
	unsigned int rd = (rtt->txChannel[chan].rd + sz) & sz;
	unsigned int wr = rtt->txChannel[chan].wr & sz;

	if (wr > rd) {
		return sz + 1 - (wr - rd);
	}
	else {
		return rd - wr;
	}
}


void rtt_rxReset(int chan)
{
	dataMemoryBarrier();
	librtt_common.rtt->rxChannel[chan].rd = librtt_common.rtt->rxChannel[chan].wr;
	dataMemoryBarrier();
}


void rtt_txReset(int chan)
{
	dataMemoryBarrier();
	librtt_common.rtt->txChannel[chan].wr = librtt_common.rtt->txChannel[chan].rd;
	dataMemoryBarrier();
}


int rtt_init(void *addr, void *bufptr, size_t bufsz)
{
	unsigned int n, m;
	volatile struct rtt_desc *rtt = librtt_common.rtt;

	if ((rtt != NULL) && (bufsz != 0) && ((bufsz & (bufsz - 1)) != 0)) {
		return -EINVAL;
	}

	if (bufptr == NULL) {
		bufptr = mmap(NULL,
			bufsz * rtt->txChannels * rtt->rxChannels,
			PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_UNCACHED,
			-1,
			0);
		if (bufptr == MAP_FAILED) {
			return -ENOMEM;
		}
		librtt_common.memptr = bufptr;
	}

	rtt = (volatile struct rtt_desc *)addr;
	memset((void *)rtt, 0, sizeof(*rtt));

	rtt->txChannels = LIBRTT_TXCHANNELS;
	rtt->rxChannels = LIBRTT_RXCHANNELS;

	bufsz /= (rtt->txChannels + rtt->rxChannels);

	m = 0;
	for (n = 0; n < rtt->txChannels; n++) {
		rtt->txChannel[n].name = rtt_txName[n];
		rtt->txChannel[n].ptr = (unsigned char *)bufptr + m * bufsz;
		rtt->txChannel[n].sz = bufsz;
		m++;
	}

	for (n = 0; n < rtt->rxChannels; n++) {
		rtt->rxChannel[n].name = rtt_rxName[n];
		rtt->rxChannel[n].ptr = (unsigned char *)bufptr + m * bufsz;
		rtt->rxChannel[n].sz = bufsz;
		m++;
	}

	n = 0;
	m = sizeof(rtt_tagReversed) - 1;
	while (n < sizeof(rtt->tag) && m > 0) {
		rtt->tag[n++] = rtt_tagReversed[--m];
	}

	return 0;
}


void rtt_done(void)
{
	volatile struct rtt_desc *rtt = librtt_common.rtt;

	if (rtt != NULL) {
		if (librtt_common.memptr != NULL) {
			size_t n, sz = 0;
			for (n = 0; rtt->txChannels; n++) {
				sz += rtt->txChannel[n].sz;
			}
			for (n = 0; rtt->rxChannels; n++) {
				sz += rtt->rxChannel[n].sz;
			}
			munmap(librtt_common.memptr, sz);
		}
		memset((void *)rtt, 0, sizeof(*rtt));
		librtt_common.rtt = NULL;
	}
}
