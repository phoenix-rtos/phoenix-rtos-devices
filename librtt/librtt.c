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

#include "librtt.h"

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
	volatile unsigned char *ptr;
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


static const char librtt_tagReversed[] = LIBRTT_TAG_BACKWARD;
static const char *const librtt_txName[LIBRTT_TXCHANNELS] = LIBRTT_TXCHANNELS_NAMES;
static const char *const librtt_rxName[LIBRTT_RXCHANNELS] = LIBRTT_RXCHANNELS_NAMES;


static struct {
	volatile struct rtt_desc *rtt;
	void *memptr;
	librtt_cacheOp_t invalFn;
	librtt_cacheOp_t cleanFn;
} librtt_common = { 0 };


int librtt_check(int chan)
{
	if ((librtt_common.rtt == NULL) || (chan < 0) || (chan >= LIBRTT_TXCHANNELS)) {
		return -ENODEV;
	}

	return 0;
}


static void performCacheOp(librtt_cacheOp_t op, volatile unsigned char *buf, unsigned int sz, unsigned int rd, unsigned int wr)
{
	if ((rd == wr) || (op == NULL)) {
		return;
	}

	if (wr < rd) {
		op((void *)(buf + rd), sz - rd);
		op((void *)buf, wr + 1);
	}
	else {
		op((void *)(buf + rd), wr - rd + 1);
	}
}


ssize_t librtt_read(int chan, void *buf, size_t count)
{
	volatile struct rtt_desc *rtt = librtt_common.rtt;

	if (librtt_check(chan) < 0) {
		return -ENODEV;
	}

	dataMemoryBarrier();

	volatile unsigned char *srcBuf = rtt->rxChannel[chan].ptr;
	unsigned char *dstBuf = (unsigned char *)buf;
	unsigned int sz = rtt->rxChannel[chan].sz - 1;
	unsigned int rd = rtt->rxChannel[chan].rd & sz;
	unsigned int wr = rtt->rxChannel[chan].wr & sz;
	size_t todo = count;

	performCacheOp(librtt_common.invalFn, srcBuf, sz + 1, rd, wr);
	while ((todo != 0) && (rd != wr)) {
		*dstBuf++ = srcBuf[rd];
		rd = (rd + 1) & sz;
		todo--;
	}

	dataMemoryBarrier();

	rtt->channel[ch].rd = rd;

	return count - todo;
}


ssize_t librtt_write(int chan, const void *buf, size_t count)
{
	volatile struct rtt_desc *rtt = librtt_common.rtt;

	if (librtt_check(chan) < 0) {
		return -ENODEV;
	}

	dataMemoryBarrier();

	const unsigned char *srcBuf = (const unsigned char *)buf;
	volatile unsigned char *dstBuf = rtt->txChannel[chan].ptr;
	unsigned int sz = rtt->txChannel[chan].sz - 1;
	unsigned int rd = (rtt->txChannel[chan].rd + sz) & sz;
	unsigned int wr = rtt->txChannel[chan].wr & sz;
	size_t todo = count;

	while ((todo != 0) && (rd != wr)) {
		dstBuf[wr] = *srcBuf++;
		wr = (wr + 1) & sz;
		todo--;
	}

	performCacheOp(librtt_common.cleanFn, dstBuf, sz + 1, rd, wr);

	dataMemoryBarrier();

	rtt->channel[ch].wr = wr;

	return count - todo;
}


ssize_t librtt_txAvail(int chan)
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


void librtt_rxReset(int chan)
{
	dataMemoryBarrier();
	librtt_common.rtt->rxChannel[chan].rd = librtt_common.rtt->rxChannel[chan].wr;
	dataMemoryBarrier();
}


void librtt_txReset(int chan)
{
	dataMemoryBarrier();
	librtt_common.rtt->txChannel[chan].wr = librtt_common.rtt->txChannel[chan].rd;
	dataMemoryBarrier();
}


int librtt_init(void *addr, void *bufptr, size_t bufsz, librtt_cacheOp_t invalFn, librtt_cacheOp_t cleanFn)
{
	unsigned int n, m;
	const size_t bufSzPerChannel = bufsz / (LIBRTT_TXCHANNELS + LIBRTT_RXCHANNELS);
	if ((LIBRTT_DESC_SIZE < sizeof(struct rtt_desc)) ||
		(librtt_common.rtt != NULL) ||
		(bufSzPerChannel == 0) ||
		((bufSzPerChannel & (bufSzPerChannel - 1)) != 0)) {
		return -EINVAL;
	}

	if (bufptr == NULL) {
		bufptr = mmap(NULL,
			bufsz,
			PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_UNCACHED,
			-1,
			0);

		if (bufptr == MAP_FAILED) {
			return -ENOMEM;
		}
	}

	librtt_common.memptr = bufptr;
	librtt_common.invalFn = invalFn;
	librtt_common.cleanFn = cleanFn;

	volatile struct rtt_desc *rtt = addr;
	memset((void *)rtt, 0, sizeof(*rtt));

	rtt->txChannels = LIBRTT_TXCHANNELS;
	rtt->rxChannels = LIBRTT_RXCHANNELS;

	m = 0;
	for (n = 0; n < rtt->txChannels; n++) {
		rtt->txChannel[n].name = librtt_txName[n];
		rtt->txChannel[n].ptr = (unsigned char *)bufptr + m * bufSzPerChannel;
		rtt->txChannel[n].sz = bufSzPerChannel;
		m++;
	}

	for (n = 0; n < rtt->rxChannels; n++) {
		rtt->rxChannel[n].name = librtt_rxName[n];
		rtt->rxChannel[n].ptr = (unsigned char *)bufptr + m * bufSzPerChannel;
		rtt->rxChannel[n].sz = bufSzPerChannel;
		m++;
	}

	n = 0;
	m = sizeof(librtt_tagReversed) - 1;
	while ((n < sizeof(rtt->tag)) && (m > 0)) {
		rtt->tag[n++] = librtt_tagReversed[--m];
	}

	librtt_common.rtt = rtt;
	return 0;
}


void librtt_done(void)
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
