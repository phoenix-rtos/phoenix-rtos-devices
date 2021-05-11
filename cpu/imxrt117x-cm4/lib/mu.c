/*
 * Phoenix-RTOS
 *
 * i.MX RT117x Messaging Unit driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include "mu.h"
#include "interrupt.h"
#include "cm4.h"
#include "string.h"


// clang-format off
enum { btr0 = 0, btr1, btr2, btr3, brr0, brr1, brr2, brr3, bsr, bcr };
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


static struct {
	volatile uint32_t *base;

	struct {
		fifo_t rx;
		fifo_t tx;
	} channel[CHANNEL_NO];
} common;


static void setTXirq(int channel, int state)
{
	if (channel < 0 || channel >= CHANNEL_NO)
		return;

	if (state)
		*(common.base + bcr) |= (1 << (23 - channel));
	else
		*(common.base + bcr) &= ~(1 << (23 - channel));
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


static void mu_irqHandler(int n)
{
	int chan;
	packet_t packet;

	(void)n;

	/* TX */
	for (chan = 0; chan < CHANNEL_NO; ++chan) {
		if (*(common.base + bsr) & (1 << (23 - chan))) {
			if (!fifo_popPacket(&packet, &common.channel[chan].tx))
				setTXirq(chan, 0);
			else
				*(common.base + btr0 + chan) = packet.word;
		}
	}

	/* RX */
	for (chan = 0; chan < CHANNEL_NO; ++chan) {
		if (*(common.base + bsr) & (1 << (27 - chan))) {
			packet.word = *(common.base + brr0 + chan);
			fifo_pushPacket(&packet, &common.channel[chan].rx);
		}
	}
}


int mu_read(int channel, void *buff, int len)
{
	char *tbuff = buff;
	int i;
	fifo_t *fifo;

	if (channel < 0 || channel >= CHANNEL_NO || buff == NULL)
		return -1;

	fifo = &common.channel[channel].rx;

	for (i = 0; i < len; ++i) {
		if (!fifo_pop(&tbuff[i], fifo))
			break;
	}

	return i;
}


int mu_write(int channel, const void *buff, int len)
{
	const char *tbuff = buff;
	int i;
	fifo_t *fifo;

	if (channel < 0 || channel >= CHANNEL_NO || buff == NULL)
		return -1;

	fifo = &common.channel[channel].tx;

	for (i = 0; i < len; ++i) {
		if (!fifo_push(tbuff[i], fifo))
			break;
	}

	if (i)
		setTXirq(channel, 1);

	return i;
}


void mu_init(void)
{
	common.base = (void *)0x40c4c000;

	interrupt_register(mu_irq, mu_irqHandler, -1);

	/* Enable RX IRQ */
	*(common.base + bcr) |= 0xf << 24;
}
