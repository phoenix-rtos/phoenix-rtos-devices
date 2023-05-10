/*
 * Phoenix-RTOS
 *
 * i.MX RT117x M4 cpu core driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <sys/msg.h>
#include <posix/utils.h>
#include <sys/platform.h>
#include <sys/interrupt.h>
#include <phoenix/arch/imxrt1170.h>

#include "imxrt117x-cm4.h"

#define M4_MEMORY_START ((void *)0x20200000)
#define M4_VTORS        (0x1ffe0000)
#define M4_MEMORY_SIZE  (256 * 1024)


// clang-format off
enum { atr0 = 0, atr1, atr2, atr3, arr0, arr1, arr2, arr3, asr, acr };


enum { chan0 = 0, chan1, chan2, chan3, chanLen };


enum { src_scr = 0, src_srmr };
// clang-format on


typedef union {
	struct {
		char bno;
		char payload[3];
	} __attribute__((packed)) byte;
	uint32_t word;
} packet_t;


typedef struct {
	char buff[256];
	volatile int wptr;
	volatile int rptr;
} fifo_t;


struct {
	unsigned int port;
	volatile void *m4memory;
	size_t m4memorysz;

	volatile uint32_t *src;
	volatile uint32_t *mu;

	struct {
		fifo_t rx;
		fifo_t tx;
	} channel[chanLen];
} m4_common;


static void setTXirq(int channel, int state)
{
	if (channel < 0 || channel >= chanLen)
		return;

	if (state)
		*(m4_common.mu + acr) |= (1 << (23 - channel));
	else
		*(m4_common.mu + acr) &= ~(1 << (23 - channel));
}


static inline int fifo_pop(char *byte, fifo_t *fifo)
{
	if (fifo->rptr == fifo->wptr)
		return 0;

	*byte = fifo->buff[fifo->rptr];
	fifo->rptr = (fifo->rptr + 1) % sizeof(fifo->buff);

	return 1;
}


static inline int fifo_popPacket(packet_t *packet, fifo_t *fifo)
{
	int i;

	for (i = 0; i < sizeof(packet->byte.payload); ++i) {
		if (!fifo_pop(&packet->byte.payload[i], fifo))
			break;
	}

	packet->byte.bno = i;

	return i;
}


static inline int fifo_push(char byte, fifo_t *fifo)
{
	if ((fifo->wptr + 1) % sizeof(fifo->buff) == fifo->wptr)
		return 0;

	fifo->buff[fifo->wptr] = byte;
	fifo->wptr = (fifo->wptr + 1) % sizeof(fifo->buff);

	return 1;
}


static inline int fifo_pushPacket(packet_t *packet, fifo_t *fifo)
{
	int i, size;

	if (packet->byte.bno > sizeof(packet->byte.payload))
		size = sizeof(packet->byte.payload);
	else
		size = packet->byte.bno;

	for (i = 0; i < size; ++i) {
		if (!fifo_push(packet->byte.payload[i], fifo))
			break;
	}

	return i;
}


static int mu_irqHandler(unsigned int n, void *arg)
{
	int chan;
	packet_t packet;

	(void)n;
	(void)arg;

	/* TX */
	for (chan = 0; chan < chanLen; ++chan) {
		if (*(m4_common.mu + asr) & (1 << (23 - chan))) {
			if (!fifo_popPacket(&packet, &m4_common.channel[chan].tx))
				setTXirq(chan, 0);
			else
				*(m4_common.mu + atr0 + chan) = packet.word;
		}
	}

	/* RX */
	for (chan = 0; chan < chanLen; ++chan) {
		if (*(m4_common.mu + asr) & (1 << (27 - chan))) {
			packet.word = *(m4_common.mu + arr0 + chan);
			fifo_pushPacket(&packet, &m4_common.channel[chan].rx);
		}
	}

	return -1;
}


static ssize_t chanRead(id_t id, void *buff, size_t bufflen)
{
	char *tbuff = buff;
	ssize_t i;
	fifo_t *fifo;

	if (id < chan0 || id > chan3 || buff == NULL)
		return -1;

	fifo = &m4_common.channel[id].rx;

	for (i = 0; i < bufflen; ++i) {
		if (!fifo_pop(&tbuff[i], fifo))
			break;
	}

	return i;
}


static ssize_t chanWrite(id_t id, const void *buff, size_t bufflen)
{
	const char *tbuff = buff;
	int i;
	fifo_t *fifo;

	if (id < chan0 || id > chan3 || buff == NULL)
		return -1;

	fifo = &m4_common.channel[id].tx;

	for (i = 0; i < bufflen; ++i) {
		if (!fifo_push(tbuff[i], fifo))
			break;
	}

	if (i)
		setTXirq(id, 1);

	return i;
}


static int runCore(unsigned int offset)
{
	platformctl_t pctl;
	unsigned int vectors = (unsigned int)M4_MEMORY_START + offset;

	/* Set CM4's VTOR to the start of it's memory */
	pctl.action = pctl_set;
	pctl.type = pctl_iolpsrgpr;
	pctl.iogpr.field = 0;
	pctl.iogpr.val = vectors & 0xffff;

	if (platformctl(&pctl))
		return -EIO;

	pctl.action = pctl_set;
	pctl.type = pctl_iolpsrgpr;
	pctl.iogpr.field = 1;
	pctl.iogpr.val = (vectors >> 16) & 0xffff;

	if (platformctl(&pctl))
		return -EIO;

	/* Release the Kraken */
	*(m4_common.src + src_srmr) |= 0x3u << 10;
	__asm__ volatile("dmb");
	*(m4_common.src + src_scr) |= 1;

	return EOK;
}


static int loadFromFile(const char *path)
{
	char buff[256];
	FILE *f;
	size_t r, total = 0;
	volatile char *ptr = m4_common.m4memory;

	if ((f = fopen(path, "r")) == NULL)
		return -ENOENT;

	while ((r = fread(buff, sizeof(buff), 1, f)) > 0 && total < m4_common.m4memorysz) {
		if (total + r > m4_common.m4memorysz)
			r = m4_common.m4memorysz - total;

		memcpy((void *)ptr, buff, r);
		ptr += r;
		total += r;
	}
	/* FIXME: clean & invalidate dcache */

	return (int)total;
}


static int loadFromBuff(const void *buff, size_t bufflen)
{
	if (bufflen > m4_common.m4memorysz)
		bufflen = m4_common.m4memorysz;

	memcpy((void *)m4_common.m4memory, buff, bufflen);
	/* FIXME: clean & invalidate dcache */

	return (int)bufflen;
}


static void devctl(msg_t *msg)
{
	imxrt117xM4DevCtli_t *i = (void *)msg->i.raw;
	imxrt117xM4DevCtlo_t *o = (void *)msg->o.raw;

	switch (i->type) {
	case m4_loadBuff:
		if (msg->i.data == NULL || msg->i.size == 0) {
			o->err = -EINVAL;
			break;
		}

		o->err = loadFromBuff(msg->i.data, msg->i.size);
		break;

	case m4_loadFile:
		if (msg->i.data == NULL || msg->i.size == 0) {
			o->err = -EINVAL;
			break;
		}

		o->err = loadFromFile(msg->i.data);
		break;

	case m4_runCore:
		if (msg->i.data == NULL || msg->i.size != sizeof(unsigned int)) {
			o->err = -EINVAL;
			break;
		}
		o->err = runCore(*(unsigned int *)msg->i.data);
		break;

	default:
		o->err = -EINVAL;
		break;
	}
}


__attribute__((noreturn)) static void thread(void)
{
	msg_t msg;
	unsigned long rid;

	for (;;) {
		if (msgRecv(m4_common.port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtRead:
			msg.o.io.err = chanRead(msg.i.io.oid.id, msg.o.data, msg.o.size);
			break;

		case mtWrite:
			msg.o.io.err = chanWrite(msg.i.io.oid.id, msg.i.data, msg.i.size);
			break;

		case mtDevCtl:
			devctl(&msg);
			break;

		case mtOpen:
		case mtClose:
			if (msg.i.openclose.oid.port == m4_common.port && msg.i.openclose.oid.id >= chan0 && msg.i.openclose.oid.id < chanLen)
				msg.o.io.err = EOK;
			else
				msg.o.io.err = -EINVAL;
			break;

		/* Unsupported operations */
		case mtCreate:
			msg.o.create.err = -ENOSYS;
			break;
		case mtLookup:
			msg.o.lookup.err = -ENOSYS;
			break;
		default:
			msg.o.io.err = -ENOSYS;
			break;
		}

		msgRespond(m4_common.port, &msg, rid);
	}
}


int main(int argc, char *argv[])
{
	oid_t oid;
	unsigned int i;
	char path[] = "/dev/cpuM4x";
	const size_t pathlen = sizeof(path);

	/* TODO - get this info from a memory map? */
	m4_common.m4memory = M4_MEMORY_START;
	m4_common.m4memorysz = M4_MEMORY_SIZE;

	m4_common.src = (void *)0x40c04000;
	m4_common.mu = (void *)0x40c48000;

	if (portCreate(&m4_common.port) < 0) {
		fprintf(stderr, "imxrt117x-m4: Failed to create port\n");
		return -1;
	}

	oid.port = m4_common.port;
	for (i = 0; i < chanLen - chan0; ++i) {
		path[pathlen - 2] = '0' + i - chan0;
		oid.id = chan0 + i;
		if (create_dev(&oid, path) < 0) {
			fprintf(stderr, "imxrt117x-m4: Failed to register the device %s\n", path);
			return -1;
		}
	}

	interrupt(mu_irq, mu_irqHandler, NULL, 0, NULL);

	*(m4_common.mu + acr) |= 0xf << 24;

	fprintf(stderr, "%s: Started\n", argv[0]);

	thread();

	/* Never reached */
	return 0;
}
