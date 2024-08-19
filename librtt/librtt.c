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
#if (defined __ARM_ARCH_7EM__)                                      /* Cortex-M4/M7 */ \
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
	struct rtt_pipe channel[LIBRTT_TXCHANNELS + LIBRTT_RXCHANNELS];
};


static const char librtt_tagReversed[] = LIBRTT_TAG_BACKWARD;
static const size_t librtt_tagLength = sizeof(librtt_tagReversed) - 1;
static const char *const librtt_txName[LIBRTT_TXCHANNELS] = LIBRTT_TXCHANNELS_NAMES;
static const char *const librtt_rxName[LIBRTT_RXCHANNELS] = LIBRTT_RXCHANNELS_NAMES;


static struct {
	volatile struct rtt_desc *rtt;
	librtt_cacheOp_t invalFn;
	librtt_cacheOp_t cleanFn;
	unsigned int lastRd[LIBRTT_TXCHANNELS];
} librtt_common = { 0 };


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


int librtt_checkTx(unsigned int ch)
{
	if ((librtt_common.rtt == NULL) || (ch >= librtt_common.rtt->txChannels)) {
		return -ENODEV;
	}

	return 0;
}


int librtt_checkRx(unsigned int ch)
{
	if ((librtt_common.rtt == NULL) || (ch >= librtt_common.rtt->rxChannels)) {
		return -ENODEV;
	}

	return 0;
}


ssize_t librtt_read(unsigned int ch, void *buf, size_t count)
{
	volatile struct rtt_desc *rtt = librtt_common.rtt;

	if (librtt_checkRx(ch) < 0) {
		return -ENODEV;
	}

	dataMemoryBarrier();
	ch += rtt->txChannels;
	volatile unsigned char *srcBuf = rtt->channel[ch].ptr;
	unsigned char *dstBuf = (unsigned char *)buf;
	unsigned int sz = rtt->channel[ch].sz - 1;
	unsigned int rd = rtt->channel[ch].rd & sz;
	unsigned int wr = rtt->channel[ch].wr & sz;
	size_t todo = count;

	performCacheOp(librtt_common.invalFn, (void *)srcBuf, sz + 1, rd, wr);
	while ((todo != 0) && (rd != wr)) {
		*dstBuf++ = srcBuf[rd];
		rd = (rd + 1) & sz;
		todo--;
	}

	dataMemoryBarrier();

	rtt->channel[ch].rd = rd;

	return count - todo;
}


ssize_t librtt_write(unsigned int ch, const void *buf, size_t count, int allowOverwrite)
{
	volatile struct rtt_desc *rtt = librtt_common.rtt;

	if (librtt_checkTx(ch) < 0) {
		return -ENODEV;
	}

	dataMemoryBarrier();

	const unsigned char *srcBuf = (const unsigned char *)buf;
	volatile unsigned char *dstBuf = rtt->channel[ch].ptr;
	unsigned int sz = rtt->channel[ch].sz - 1;
	unsigned int rd = (rtt->channel[ch].rd + sz) & sz;
	unsigned int wr = rtt->channel[ch].wr & sz;
	size_t todo = count;

	while ((todo != 0) && (rd != wr)) {
		dstBuf[wr] = *srcBuf++;
		wr = (wr + 1) & sz;
		todo--;
	}

	if ((rd == wr) && (allowOverwrite != 0)) {
		while (todo != 0) {
			dstBuf[wr] = *srcBuf++;
			wr = (wr + 1) & sz;
			todo--;
		}

		rtt->channel[ch].rd = wr;
	}

	performCacheOp(librtt_common.cleanFn, (void *)dstBuf, sz + 1, rd, wr);

	dataMemoryBarrier();

	rtt->channel[ch].wr = wr;

	return count - todo;
}


ssize_t librtt_rxAvail(unsigned int ch)
{
	volatile struct rtt_desc *rtt = librtt_common.rtt;
	if (librtt_checkRx(ch) < 0) {
		return 0;
	}

	dataMemoryBarrier();
	ch += librtt_common.rtt->txChannels;
	unsigned int sz = rtt->channel[ch].sz - 1;
	unsigned int rd = rtt->channel[ch].rd & sz;
	unsigned int wr = rtt->channel[ch].wr & sz;

	if (rd > wr) {
		return sz + 1 - (rd - wr);
	}
	else {
		return wr - rd;
	}
}


ssize_t librtt_txAvail(unsigned int ch)
{
	volatile struct rtt_desc *rtt = librtt_common.rtt;
	if (librtt_checkTx(ch) < 0) {
		return 0;
	}

	dataMemoryBarrier();
	unsigned int sz = rtt->channel[ch].sz - 1;
	unsigned int rd = (rtt->channel[ch].rd + sz) & sz;
	unsigned int wr = rtt->channel[ch].wr & sz;

	if (wr > rd) {
		return sz + 1 - (wr - rd);
	}
	else {
		return rd - wr;
	}
}


void librtt_rxReset(unsigned int ch)
{
	dataMemoryBarrier();
	if (librtt_checkRx(ch) < 0) {
		return;
	}

	ch += librtt_common.rtt->txChannels;
	librtt_common.rtt->channel[ch].rd = librtt_common.rtt->channel[ch].wr;
	dataMemoryBarrier();
}


void librtt_txReset(unsigned int ch)
{
	dataMemoryBarrier();
	if (librtt_checkTx(ch) < 0) {
		return;
	}

	librtt_common.rtt->channel[ch].wr = librtt_common.rtt->channel[ch].rd;
	dataMemoryBarrier();
}


int librtt_txCheckReaderAttached(unsigned int ch)
{
	if (librtt_checkTx(ch) < 0) {
		return 0;
	}

	unsigned int rd = librtt_common.rtt->channel[ch].rd;
	if (librtt_common.lastRd[ch] != rd) {
		librtt_common.lastRd[ch] = rd;
		return 1;
	}

	return 0;
}


int librtt_init(void *addr, librtt_cacheOp_t invalFn, librtt_cacheOp_t cleanFn)
{
	if ((LIBRTT_DESC_SIZE < sizeof(struct rtt_desc)) || (librtt_common.rtt != NULL)) {
		return -EINVAL;
	}

	librtt_common.invalFn = invalFn;
	librtt_common.cleanFn = cleanFn;

	volatile struct rtt_desc *rtt = addr;

	int n;
	for (n = 0; n < librtt_tagLength; n++) {
		if (librtt_tagReversed[n] != rtt->tag[librtt_tagLength - 1 - n]) {
			break;
		}
	}

	if (n == librtt_tagLength) {
		if ((rtt->txChannels + rtt->rxChannels) <= (LIBRTT_TXCHANNELS + LIBRTT_RXCHANNELS)) {
			librtt_common.rtt = rtt;
			return 0;
		}
	}

	memset((void *)rtt, 0, sizeof(*rtt));

	rtt->txChannels = LIBRTT_TXCHANNELS;
	rtt->rxChannels = LIBRTT_RXCHANNELS;

	for (n = 0; n < librtt_tagLength; n++) {
		rtt->tag[librtt_tagLength - 1 - n] = librtt_tagReversed[n];
	}

	librtt_common.rtt = rtt;
	return 0;
}


int librtt_initChannel(int isTx, unsigned int ch, unsigned char *buf, size_t sz)
{
	volatile struct rtt_desc *rtt = librtt_common.rtt;
	if (rtt == NULL) {
		return -EINVAL;
	}

	unsigned int chMax = (isTx != 0) ? rtt->txChannels : rtt->rxChannels;
	if ((ch >= chMax)) {
		return -EINVAL;
	}

	/* Check buffer size is non-zero and power of 2 */
	if ((sz == 0) || ((sz & (sz - 1)) != 0)) {
		return -EINVAL;
	}

	const char *name = NULL;
	if (isTx != 0) {
		if (ch < LIBRTT_TXCHANNELS) {
			name = librtt_txName[ch];
		}
	}
	else {
		if (ch < LIBRTT_RXCHANNELS) {
			name = librtt_rxName[ch];
		}
	}

	if (isTx != 0) {
		librtt_common.lastRd[ch] = 0;
	}
	else {
		ch += rtt->txChannels;
	}

	rtt->channel[ch].name = name;
	rtt->channel[ch].ptr = buf;
	rtt->channel[ch].sz = sz;
	rtt->channel[ch].rd = 0;
	rtt->channel[ch].wr = 0;
	return 0;
}


void librtt_done(void)
{
	volatile struct rtt_desc *rtt = librtt_common.rtt;

	if (rtt != NULL) {
		memset((void *)rtt, 0, sizeof(*rtt));
		librtt_common.rtt = NULL;
	}
}
