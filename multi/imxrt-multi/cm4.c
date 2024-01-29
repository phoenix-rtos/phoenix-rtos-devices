/*
 * Phoenix-RTOS
 *
 * i.MX RT117x M4 cpu core driver
 *
 * Copyright 2021, 2023 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <sys/msg.h>
#include <posix/utils.h>
#include <sys/platform.h>
#include <sys/interrupt.h>
#include <sys/threads.h>
#include <phoenix/arch/imxrt1170.h>
#include <board_config.h>

#include <libtty-lf-fifo.h>

#include "common.h"

#ifndef CM4_MU_CHANNELS
#define CM4_MU_CHANNELS 4
#endif

#ifndef CM4_MU_0_RX_FIFO_SIZE
#define CM4_MU_0_RX_FIFO_SIZE 256
#endif
#ifndef CM4_MU_1_RX_FIFO_SIZE
#define CM4_MU_1_RX_FIFO_SIZE 256
#endif
#ifndef CM4_MU_2_RX_FIFO_SIZE
#define CM4_MU_2_RX_FIFO_SIZE 256
#endif
#ifndef CM4_MU_3_RX_FIFO_SIZE
#define CM4_MU_3_RX_FIFO_SIZE 256
#endif

#ifndef CM4_MU_0_TX_FIFO_SIZE
#define CM4_MU_0_TX_FIFO_SIZE 256
#endif
#ifndef CM4_MU_1_TX_FIFO_SIZE
#define CM4_MU_1_TX_FIFO_SIZE 256
#endif
#ifndef CM4_MU_2_TX_FIFO_SIZE
#define CM4_MU_2_TX_FIFO_SIZE 256
#endif
#ifndef CM4_MU_3_TX_FIFO_SIZE
#define CM4_MU_3_TX_FIFO_SIZE 256
#endif

#define CM4_MU_FIFO_SIZE(chan) (CM4_MU_##chan##_RX_FIFO_SIZE + CM4_MU_##chan##_TX_FIFO_SIZE)

#if CM4_MU_CHANNELS == 1
#define CM4_MU_TOTAL_FIFO_SIZE (CM4_MU_FIFO_SIZE(0))
#elif CM4_MU_CHANNELS == 2
#define CM4_MU_TOTAL_FIFO_SIZE (CM4_MU_FIFO_SIZE(0) + CM4_MU_FIFO_SIZE(1))
#elif CM4_MU_CHANNELS == 3
#define CM4_MU_TOTAL_FIFO_SIZE (CM4_MU_FIFO_SIZE(0) + CM4_MU_FIFO_SIZE(1) + CM4_MU_FIFO_SIZE(2))
#elif CM4_MU_CHANNELS == 4
#define CM4_MU_TOTAL_FIFO_SIZE (CM4_MU_FIFO_SIZE(0) + CM4_MU_FIFO_SIZE(1) + CM4_MU_FIFO_SIZE(2) + CM4_MU_FIFO_SIZE(3))
#else
#define CM4_MU_TOTAL_FIFO_SIZE 0
#endif

#define CM4_MEMORY_START ((void *)0x20200000)
#define CM4_MEMORY_SIZE  (256 * 1024)


/* clang-format off */
enum { atr0 = 0, atr1, atr2, atr3, arr0, arr1, arr2, arr3, asr, acr };


enum { chan0 = 0, chan1, chan2, chan3, chanLen };


enum { src_scr = 0, src_srmr };
/* clang-format on */


typedef union {
	struct {
		unsigned char bno;
		unsigned char payload[3];
	} __attribute__((packed)) byte;
	uint32_t word;
} packet_t;


struct {
	unsigned int port;
	volatile void *m4memory;
	size_t m4memorysz;

	volatile uint32_t *src;
	volatile uint32_t *mu;

	struct {
		lf_fifo_t rx_fifo;
		lf_fifo_t tx_fifo;
		handle_t rx_lock;
		handle_t tx_lock;
		struct {
			uint8_t busy : 1;
			uint8_t nonblock : 1;
		} state;
	} channel[CM4_MU_CHANNELS];

	handle_t lock;
	uint8_t data[CM4_MU_TOTAL_FIFO_SIZE];
} m4_common;


static unsigned int fifo_getRXSize(int channel)
{
	switch (channel) {
		case 0:
			return CM4_MU_0_RX_FIFO_SIZE;
		case 1:
			return CM4_MU_1_RX_FIFO_SIZE;
		case 2:
			return CM4_MU_2_RX_FIFO_SIZE;
		case 3:
			return CM4_MU_3_RX_FIFO_SIZE;
	}

	return 0;
}


static unsigned int fifo_getTXSize(int channel)
{
	switch (channel) {
		case 0:
			return CM4_MU_0_TX_FIFO_SIZE;
		case 1:
			return CM4_MU_1_TX_FIFO_SIZE;
		case 2:
			return CM4_MU_2_TX_FIFO_SIZE;
		case 3:
			return CM4_MU_3_TX_FIFO_SIZE;
	}

	return 0;
}


static void setTXirq(int channel, int state)
{
	if ((channel < 0) || (channel >= CM4_MU_CHANNELS)) {
		return;
	}

	if (state != 0) {
		*(m4_common.mu + acr) |= (1 << (23 - channel));
	}
	else {
		*(m4_common.mu + acr) &= ~(1 << (23 - channel));
	}
}


static void setRXirq(int channel, int state)
{
	if ((channel < 0) || (channel >= CM4_MU_CHANNELS)) {
		return;
	}

	if (state != 0) {
		*(m4_common.mu + acr) |= (1 << (27 - channel));
	}
	else {
		*(m4_common.mu + acr) &= ~(1 << (27 - channel));
	}
}


static inline int fifo_popPacket(lf_fifo_t *fifo, packet_t *packet)
{
	size_t i;

	for (i = 0; i < sizeof(packet->byte.payload); ++i) {
		if (lf_fifo_pop(fifo, &packet->byte.payload[i]) == 0)
			break;
	}

	packet->byte.bno = i;

	return i;
}


static inline int fifo_pushPacket(lf_fifo_t *fifo, packet_t *packet)
{
	int i, size;

	if (packet->byte.bno > sizeof(packet->byte.payload)) {
		size = sizeof(packet->byte.payload);
	}
	else {
		size = packet->byte.bno;
	}

	for (i = 0; i < size; ++i) {
		if (lf_fifo_push(fifo, packet->byte.payload[i]) == 0) {
			/* overrun */
			break;
		}
	}

	return i;
}


static int mu_irqHandler(unsigned int n, void *arg)
{
	uint32_t sr;
	int chan;
	packet_t packet;

	(void)n;
	(void)arg;

	sr = *(m4_common.mu + asr);

	/* TX */
	for (chan = 0; chan < CM4_MU_CHANNELS; ++chan) {
		if ((sr & (1 << (23 - chan))) != 0) {
			if (fifo_popPacket(&m4_common.channel[chan].tx_fifo, &packet) == 0) {
				setTXirq(chan, 0);
			}
			else {
				*(m4_common.mu + atr0 + chan) = packet.word;
			}
		}
	}

	/* RX */
	for (chan = 0; chan < CM4_MU_CHANNELS; ++chan) {
		if ((sr & (1 << (27 - chan))) != 0) {
			packet.word = *(m4_common.mu + arr0 + chan);
			fifo_pushPacket(&m4_common.channel[chan].rx_fifo, &packet);
		}
	}

	return -1;
}


static ssize_t chanOpen(id_t id, int flags)
{
	if ((id < chan0) || (id >= CM4_MU_CHANNELS)) {
		return -EINVAL;
	}

	mutexLock(m4_common.lock);

	if (m4_common.channel[id].state.busy == 1) {
		mutexUnlock(m4_common.lock);
		return -EBUSY;
	}

	m4_common.channel[id].state.busy = 1;
	m4_common.channel[id].state.nonblock = ((flags & O_NONBLOCK) != 0) ? 1 : 0;

	mutexUnlock(m4_common.lock);

	return EOK;
}


static ssize_t chanClose(id_t id)
{
	if ((id < chan0) || (id >= CM4_MU_CHANNELS)) {
		return -EINVAL;
	}

	mutexLock(m4_common.lock);

	if (m4_common.channel[id].state.busy == 0) {
		mutexUnlock(m4_common.lock);
		return -EBADF;
	}

	m4_common.channel[id].state.busy = 0;

	mutexUnlock(m4_common.lock);

	return EOK;
}


static ssize_t chanRead(id_t id, unsigned char *buff, size_t bufflen)
{
	size_t i;
	lf_fifo_t *fifo;

	if ((id < chan0) || (id >= CM4_MU_CHANNELS) || (buff == NULL)) {
		return -EINVAL;
	}

	/* Enforce SPSC FIFO access by mutex */
	mutexLock(m4_common.channel[id].rx_lock);

	fifo = &m4_common.channel[id].rx_fifo;

	for (i = 0; i < bufflen; ++i) {
		if (lf_fifo_pop(fifo, &buff[i]) == 0) {
			break;
		}
	}

	if (i == 0 && m4_common.channel[id].state.nonblock == 1) {
		i = -EAGAIN;
	}

	mutexUnlock(m4_common.channel[id].rx_lock);

	return (ssize_t)i;
}


static ssize_t chanWrite(id_t id, const unsigned char *buff, size_t bufflen)
{
	size_t i;
	lf_fifo_t *fifo;

	if ((id < chan0) || (id >= CM4_MU_CHANNELS) || (buff == NULL)) {
		return -EINVAL;
	}

	/* Enforce SPSC FIFO access by mutex */
	mutexLock(m4_common.channel[id].tx_lock);

	fifo = &m4_common.channel[id].tx_fifo;

	for (i = 0; i < bufflen; ++i) {
		if (lf_fifo_push(fifo, buff[i]) == 0) {
			break;
		}
	}

	if (i != 0) {
		setTXirq(id, 1);
	}

	if (i == 0 && m4_common.channel[id].state.nonblock == 1) {
		i = -EAGAIN;
	}

	mutexUnlock(m4_common.channel[id].tx_lock);

	return (ssize_t)i;
}


static int runCore(unsigned int offset)
{
	platformctl_t pctl;
	unsigned int vectors = (unsigned int)CM4_MEMORY_START + offset;

	/* Set CM4's VTOR to the start of it's memory */
	pctl.action = pctl_set;
	pctl.type = pctl_iolpsrgpr;
	pctl.iogpr.field = 0;
	pctl.iogpr.val = vectors & 0xfff8;

	if (platformctl(&pctl) != 0) {
		return -EIO;
	}

	pctl.action = pctl_set;
	pctl.type = pctl_iolpsrgpr;
	pctl.iogpr.field = 1;
	pctl.iogpr.val = (vectors >> 16) & 0xffff;

	if (platformctl(&pctl) != 0) {
		return -EIO;
	}

	/* Release the Kraken */
	*(m4_common.src + src_scr) |= 1;

	return EOK;
}


static int cleanInvalDCache(void *addr, unsigned int sz)
{
	platformctl_t pctl;

	pctl.action = pctl_set,
	pctl.type = pctl_cleanInvalDCache,
	pctl.cleanInvalDCache.addr = addr;
	pctl.cleanInvalDCache.sz = sz;

	return platformctl(&pctl);
}


static int loadFromFile(const char *path)
{
	unsigned char buff[64]; /* Can't be big due to small multi stacks */
	FILE *f;
	size_t r, total = 0;
	volatile unsigned char *ptr = m4_common.m4memory;

	f = fopen(path, "r");
	if (f == NULL) {
		return -ENOENT;
	}

	cleanInvalDCache((void *)m4_common.m4memory, m4_common.m4memorysz);

	while (total < m4_common.m4memorysz) {
		r = fread(buff, sizeof(buff), 1, f);
		if (r == 0) {
			if (ferror(f) != 0) {
				fclose(f);
				return -EIO;
			}
			break;
		}

		if (total + r > m4_common.m4memorysz) {
			fclose(f);
			return -EINVAL;
		}

		memcpy((void *)ptr, buff, r);
		ptr += r;
		total += r;
	}

	cleanInvalDCache((void *)m4_common.m4memory, m4_common.m4memorysz);

	fclose(f);

	return (int)total;
}


static int loadFromBuff(const void *buff, size_t bufflen)
{
	if (bufflen > m4_common.m4memorysz) {
		return -EINVAL;
	}

	cleanInvalDCache((void *)m4_common.m4memory, bufflen);

	memcpy((void *)m4_common.m4memory, buff, bufflen);

	cleanInvalDCache((void *)m4_common.m4memory, bufflen);

	return (int)bufflen;
}


static void devctl(msg_t *msg)
{
	multi_i_t *iptr = (multi_i_t *)msg->i.raw;
	multi_o_t *optr = (multi_o_t *)msg->o.raw;

	mutexLock(m4_common.lock);

	switch (iptr->cm4_type) {
		case CM4_LOAD_BUFF:
			if ((msg->i.data == NULL) || (msg->i.size == 0)) {
				optr->err = -EINVAL;
				break;
			}

			optr->err = loadFromBuff(msg->i.data, msg->i.size);
			break;

		case CM4_LOAD_FILE:
			if ((msg->i.data == NULL) || (msg->i.size == 0)) {
				optr->err = -EINVAL;
				break;
			}

			optr->err = loadFromFile(msg->i.data);
			break;

		case CM4_RUN_CORE:
			if ((msg->i.data == NULL) || (msg->i.size != sizeof(unsigned int))) {
				optr->err = -EINVAL;
				break;
			}
			optr->err = runCore(*(unsigned int *)msg->i.data);
			break;

		default:
			optr->err = -ENOSYS;
			break;
	}

	mutexUnlock(m4_common.lock);
}


void cm4_handleMsg(msg_t *msg)
{
	switch (msg->type) {
		case mtOpen:
			msg->o.io.err = chanOpen(msg->i.io.oid.id - id_cm4_0, msg->i.openclose.flags);
			break;

		case mtClose:
			msg->o.io.err = chanClose(msg->i.io.oid.id - id_cm4_0);
			break;

		case mtRead:
			msg->o.io.err = chanRead(msg->i.io.oid.id - id_cm4_0, msg->o.data, msg->o.size);
			break;

		case mtWrite:
			msg->o.io.err = chanWrite(msg->i.io.oid.id - id_cm4_0, msg->i.data, msg->i.size);
			break;

		case mtGetAttr:
		case mtSetAttr:
			msg->o.attr.err = -ENOSYS;
			break;

		case mtDevCtl:
			devctl(msg);
			break;

		default:
			msg->o.io.err = -EINVAL;
			break;
	}
}


void cm4_init(void)
{
	int chan;
	uint8_t *data;

	/* TODO - get this info from a memory map? */
	m4_common.m4memory = CM4_MEMORY_START;
	m4_common.m4memorysz = CM4_MEMORY_SIZE;

	m4_common.src = (void *)0x40c04000;
	m4_common.mu = (void *)0x40c48000;

	for (chan = 0, data = m4_common.data; chan < CM4_MU_CHANNELS; ++chan) {
		lf_fifo_init(&m4_common.channel[chan].rx_fifo, data, fifo_getRXSize(chan));
		data += fifo_getRXSize(chan);
		lf_fifo_init(&m4_common.channel[chan].tx_fifo, data, fifo_getTXSize(chan));
		data += fifo_getTXSize(chan);
	}

	mutexCreate(&m4_common.lock);
	for (chan = 0; chan < CM4_MU_CHANNELS; ++chan) {
		mutexCreate(&m4_common.channel[chan].rx_lock);
		mutexCreate(&m4_common.channel[chan].tx_lock);
	}

	interrupt(mu_irq, mu_irqHandler, NULL, 0, NULL);

	for (chan = 0; chan < CM4_MU_CHANNELS; ++chan) {
		setRXirq(chan, 1);
	}
}
