/*
 * Phoenix-RTOS
 *
 * iMX RT RTT communication driver
 *
 * Copyright 2017-2024 Phoenix Systems
 * Author: Kamil Amanowicz, Marek Bialowas, Aleksander Kaminski, Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/file.h>

#include <librtt.h>

#include "common.h"
#include "rtt.h"


#ifndef RTT_ADDR
/* RTT descriptors location, last 256 bytes of DTCM */
#define RTT_ADDR (0x20040000 - 0x100)
#endif

#define RTT_TX_BUF_SIZE          1024
#define RTT_RX_BUF_SIZE          256
#define RTT_POLLING_RATE_MS      20
#define RTT_NO_PICKUP_TIMEOUT_MS (2 * RTT_POLLING_RATE_MS)
#define RTT_RETRIES              (RTT_NO_PICKUP_TIMEOUT_MS / RTT_POLLING_RATE_MS)

/* Doesn't need to be large, data will mostly be stored in RTT buffers */
#define TTY_BUF_SIZE 64

#define RTT_CHANNEL0_POS 0
#define RTT_CHANNEL1_POS (RTT_CHANNEL0_POS + RTT_CHANNEL0)
#define RTT_ACTIVE_CNT   (RTT_CHANNEL0 + RTT_CHANNEL1)

typedef struct rtt_s {
	int chn;
	handle_t lock;
	libtty_common_t tty_common;
	volatile unsigned int diag_txSkipped; /* Accessed using a debugger */
} rtt_t;

static struct {
	char stack[1024] __attribute__((aligned(8)));
	rtt_t uarts[RTT_ACTIVE_CNT];
} rtt_common;


static const int rttConfig[] = { RTT_CHANNEL0, RTT_CHANNEL1 };


static const int rttBlocking[] = { RTT_CHANNEL0_BLOCKING, RTT_CHANNEL1_BLOCKING };


static const int rttPos[] = { RTT_CHANNEL0_POS, RTT_CHANNEL1_POS };


#define RTT_CHANNEL_CNT (sizeof(rttConfig) / sizeof(rttConfig[0]))


static inline ssize_t rtt_txAvailMode(unsigned int chn)
{
	return (rttBlocking[chn] != 0) ? librtt_txAvail(chn) : 1;
}


static void rtt_thread(void *arg)
{
	unsigned retries[RTT_ACTIVE_CNT];
	memset(retries, 0, sizeof(retries));

	for (;;) {
		for (int chn_idx = 0; chn_idx < RTT_ACTIVE_CNT; chn_idx++) {
			rtt_t *uart = &rtt_common.uarts[chn_idx];
			unsigned char data;
			ssize_t onRx = librtt_rxAvail(uart->chn);
			ssize_t onTx = rtt_txAvailMode(uart->chn);
			int txReady = libtty_txready(&uart->tty_common);

			if (rttBlocking[uart->chn] == 0) {
				/* Do nothing, in this case the remaining code is unnecessary */
			}
			else if (librtt_txCheckReaderAttached(uart->chn) != 0) {
				retries[chn_idx] = RTT_RETRIES;
			}
			else if (onTx == 0) {
				if (retries[chn_idx] == 0) {
					onTx = 1;
				}
				else {
					retries[chn_idx]--;
				}
			}


			if ((onRx == 0) && ((txReady == 0) || (onTx == 0))) {
				continue;
			}

			mutexLock(uart->lock);
			const unsigned char mask = ((uart->tty_common.term.c_cflag & CSIZE) == CS7) ? 0x7f : 0xff;
			while (onRx > 0) {
				librtt_read(uart->chn, &data, 1);
				libtty_putchar(&uart->tty_common, data & mask, NULL);
				onRx = librtt_rxAvail(uart->chn);
			}

			while (onTx > 0 && txReady) {
				data = libtty_getchar(&uart->tty_common, NULL);
				ssize_t written = librtt_write(uart->chn, &data, 1, 0);
				if (written <= 0) {
					uart->diag_txSkipped++;
				}

				onTx = rtt_txAvailMode(uart->chn);
				txReady = libtty_txready(&uart->tty_common);
			}

			mutexUnlock(uart->lock);
		}

		usleep(RTT_POLLING_RATE_MS * 1000);
	}
}


static int rtt_initOne(rtt_t *uart, int chn, unsigned char *buf)
{
	libtty_callbacks_t callbacks;
	callbacks.arg = uart;
	callbacks.set_baudrate = NULL;
	callbacks.set_cflag = NULL;
	callbacks.signal_txready = NULL;

	uart->chn = chn;

	int ret = 0;
	ret = (ret == 0) ? librtt_initChannel(1, chn, buf, RTT_TX_BUF_SIZE) : ret;
	ret = (ret == 0) ? librtt_initChannel(0, chn, buf + RTT_TX_BUF_SIZE, RTT_RX_BUF_SIZE) : ret;
	ret = (ret == 0) ? mutexCreate(&uart->lock) : ret;
	/* TODO: calculate approx. baud rate based on buffer size and polling rate */
	ret = (ret == 0) ? libtty_init(&uart->tty_common, &callbacks, TTY_BUF_SIZE, libtty_int_to_baudrate(115200)) : ret;

	return ret;
}


int rtt_init(void)
{
	if (RTT_ACTIVE_CNT == 0) {
		return EOK;
	}

	/* Reserve memory for the descriptor and buffers */
	intptr_t startAddr = (RTT_ADDR - (RTT_ACTIVE_CNT * (RTT_TX_BUF_SIZE + RTT_RX_BUF_SIZE))) & ~(_PAGE_SIZE - 1);
	size_t mapSize = RTT_ADDR + LIBRTT_DESC_SIZE - startAddr;
	unsigned char *rttBuffer = mmap(
		NULL,
		mapSize,
		PROT_WRITE | PROT_READ,
		MAP_ANONYMOUS | MAP_UNCACHED | MAP_PHYSMEM,
		-1,
		startAddr);

	if (rttBuffer == MAP_FAILED) {
		return -ENOMEM;
	}

	int ret = librtt_init((void *)RTT_ADDR, NULL, NULL);
	if (ret != 0) {
		librtt_done();
		munmap(rttBuffer, mapSize);
		return ret;
	}

	unsigned char *buf = rttBuffer;
	for (int i = 0, chn = 0; chn < RTT_CHANNEL_CNT; ++chn) {
		if (rttConfig[chn] == 0) {
			continue;
		}

		rtt_t *uart = &rtt_common.uarts[i++];
		int ret = rtt_initOne(uart, chn, buf);
		if (ret != 0) {
			librtt_done();
			munmap(rttBuffer, mapSize);
			return ret;
		}

		buf += RTT_RX_BUF_SIZE + RTT_TX_BUF_SIZE;
	}

	beginthread(rtt_thread, IMXRT_MULTI_PRIO, rtt_common.stack, sizeof(rtt_common.stack), NULL);
	return ret;
}

void rtt_klogCblk(const char *data, size_t size)
{
#ifdef RTT_CHANNEL_CONSOLE
	libtty_write(&rtt_common.uarts[rttPos[RTT_CHANNEL_CONSOLE]].tty_common, data, size, 0);
#endif
}


int rtt_handleMsg(msg_t *msg, int dev)
{
	unsigned long request;
	const void *in_data, *out_data = NULL;
	pid_t pid;
	int err;
	rtt_t *uart;

	dev -= id_rtt0;

	if ((dev < 0) || (dev >= RTT_CHANNEL_CNT) || (rttConfig[dev] == 0)) {
		return -EINVAL;
	}

	uart = &rtt_common.uarts[rttPos[dev]];

	switch (msg->type) {
		case mtWrite:
			msg->o.err = libtty_write(&uart->tty_common, msg->i.data, msg->i.size, msg->i.io.mode);
			break;

		case mtRead:
			msg->o.err = libtty_read(&uart->tty_common, msg->o.data, msg->o.size, msg->i.io.mode);
			break;

		case mtGetAttr:
			if (msg->i.attr.type != atPollStatus) {
				msg->o.err = -ENOSYS;
				break;
			}

			msg->o.attr.val = libtty_poll_status(&uart->tty_common);
			msg->o.err = EOK;
			break;

		case mtDevCtl:
			in_data = ioctl_unpack(msg, &request, NULL);
			pid = ioctl_getSenderPid(msg);
			err = libtty_ioctl(&uart->tty_common, pid, request, in_data, &out_data);
			ioctl_setResponse(msg, request, err, out_data);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}

	return EOK;
}