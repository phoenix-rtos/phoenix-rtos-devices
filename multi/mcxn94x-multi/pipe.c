/*
 * Phoenix-RTOS
 *
 * MCX N94x M33 Mailbox-signalled inter-CPU datagram-based shared memory pipes
 *
 * Copyright 2025 Phoenix Systems
 * Author: Daniel Sawka
 *
 * %LICENSE%
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/minmax.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/threads.h>

#define LF_FIFO_CACHELINE 4
#include <lf-fifo.h>

#include "common.h"
#include "cpu.h"
#include "dev.h"


#ifndef PIPE_CNT
#define PIPE_CNT 0
#endif

/* Sizes may be 0 - a tx/rx buffer or a whole pipe is then omitted */
#ifndef PIPE_DEFS
/* clang-format off */
#define PIPE_DEFS { { .txsz = 0, .rxsz = 0 } }
/* clang-format on */
#endif

#if PIPE_CNT > 0
/* TODO: Use statically-allocated buffers if not using a provided memory map */
#ifndef PIPE_MEMMAP
#error PIPE_MEMMAP must be defined
#endif
#else
#define PIPE_MEMMAP ""
#endif

typedef struct {
	void *start;
	size_t size;
} memrange_t;


static const struct {
	size_t txsz;
	size_t rxsz;
	/* TODO: Other parameters, e.g. overwriting FIFOs */
} pipe_defs[PIPE_CNT + 1] = PIPE_DEFS; /* +1 in case PIPE_CNT==0 */


static struct {
	unsigned int major;
	/* Pointer to array of fifos in the destination memory map (PIPE_MEMMAP) */
	/* This is the layout the other core will see and use - keep in sync with
	other implementations */
	struct pipe {
		size_t txsz;
		size_t rxsz;
		lf_fifo_t txfifo;
		lf_fifo_t rxfifo;
	} *pipes;
	/* Per-pipe data, not available through the memory map */
	struct pipe_priv {
		handle_t txLock;
		handle_t rxLock;
		addr_t txBufaddr;
		addr_t rxBufaddr;
	} pipes_priv[PIPE_CNT + 1]; /* +1 in case PIPE_CNT==0 */
} common;


static int getMapRange(const char *name, memrange_t *range)
{
	meminfo_t mi;
	mi.page.mapsz = -1;
	mi.entry.kmapsz = -1;
	mi.entry.mapsz = -1;
	mi.maps.mapsz = 0;
	mi.maps.map = NULL;
	meminfo(&mi);

	mi.maps.map = malloc(mi.maps.mapsz * sizeof(mapinfo_t));
	if (mi.maps.map == NULL) {
		return -ENOMEM;
	}

	range->start = NULL;

	meminfo(&mi);
	for (int i = 0; i < mi.maps.mapsz; i++) {
		if (strcmp(mi.maps.map[i].name, name) == 0) {
			range->start = (void *)mi.maps.map[i].vstart;
			range->size = mi.maps.map[i].vend - mi.maps.map[i].vstart;
			break;
		}
	}

	free(mi.maps.map);
	if (range->start == NULL) {
		return -ENODEV;
	}

	return 0;
}


static ssize_t pipeRead(unsigned int minor, uint8_t *buf, size_t size, bool blocking)
{
	size_t bytesRead = 0;
	size_t recordLength = 0;

	struct pipe *pipe = &common.pipes[minor];
	struct pipe_priv *pipe_priv = &common.pipes_priv[minor];

	if (buf == NULL || size == 0 || pipe->rxsz == 0) {
		return -EINVAL;
	}

	mutexLock(pipe_priv->rxLock);

	/* First, get the record length */

	while (lf_fifo_used(&pipe->rxfifo) < sizeof(size_t)) {
		if (!blocking) {
			mutexUnlock(pipe_priv->rxLock);
			return -EAGAIN;
		}

		cpu_waitForEvent(0);
	}

	lf_fifo_pop_many(&pipe->rxfifo, (uint8_t *)&recordLength, sizeof(recordLength));

	/* Now get the record itself */

	while (lf_fifo_used(&pipe->rxfifo) < recordLength) {
		/* Record length already popped (no peeking function), must block */
		/* Should be safe, because writing functions must write the whole record at once */

		/* Pipe is notified once via Mailbox (when write completes), so we cannot
		condWait() here as we would wait forever. Active waiting should happen only
		when pipeRead() is entered just before the other end finishes writing
		to fifo but before it sends a notification (i.e. wait for a short time). */
	}

	bytesRead = lf_fifo_pop_many(&pipe->rxfifo, buf, min(recordLength, size));

	/* Truncate record if it does not fit into read buffer, like in unix sockets */
	while (recordLength > size) {
		uint8_t dummy;
		lf_fifo_pop(&pipe->rxfifo, &dummy);
		recordLength -= 1;
	}

	/* TODO: Notify the other core that some fifo operation has completed */
	/* cpu_fifoWrite(NULL, 0, true, 1); */

	mutexUnlock(pipe_priv->rxLock);

	return (ssize_t)bytesRead;
}


static ssize_t pipeWrite(unsigned int minor, const uint8_t *buf, size_t len, bool blocking)
{
	size_t bytesWritten = 0;

	struct pipe *pipe = &common.pipes[minor];
	struct pipe_priv *pipe_priv = &common.pipes_priv[minor];

	if (buf == NULL || len == 0 || pipe->txsz == 0) {
		return -EINVAL;
	}

	/* The whole datagram must fit, i.e. record size, data itself, and 1 byte
	must remain free for FIFO state itself */
	if (len > pipe->txsz - sizeof(size_t) - 1) {
		len = pipe->txsz - sizeof(size_t) - 1;
	}

	mutexLock(pipe_priv->txLock);

	while (lf_fifo_free(&pipe->txfifo) < sizeof(size_t) + len) {
		if (!blocking) {
			mutexUnlock(pipe_priv->txLock);
			return -EAGAIN;
		}

		cpu_waitForEvent(0);
	}

	lf_fifo_push_many(&pipe->txfifo, (uint8_t *)&len, sizeof(len));
	bytesWritten = lf_fifo_push_many(&pipe->txfifo, buf, len);

	/* TODO: Notify the other core that some fifo operation has completed */
	/* cpu_fifoWrite(NULL, 0, true, 1); */

	mutexUnlock(pipe_priv->txLock);

	return (ssize_t)bytesWritten;
}


static int pipePollStatus(unsigned int minor)
{
	int revents = 0;

	struct pipe *pipe = &common.pipes[minor];

	if (!lf_fifo_empty(&pipe->rxfifo)) {
		revents |= POLLIN | POLLRDNORM;
	}

	if (lf_fifo_free(&pipe->txfifo) > sizeof(size_t)) {
		revents |= POLLOUT | POLLWRNORM;
	}

	return revents;
}


static void pipe_handleMsg(msg_t *msg, msg_rid_t rid, unsigned int major, unsigned int minor)
{
	(void)major;
	bool blocking;

	if (minor >= PIPE_CNT) {
		msg->o.err = -ENODEV;
		msgRespond(msg->oid.port, msg, rid);
		return;
	}

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.err = 0;
			break;

		case mtWrite:
			blocking = ((msg->i.io.mode & O_NONBLOCK) == 0u);
			msg->o.err = pipeWrite(minor, msg->i.data, msg->i.size, blocking);
			break;

		case mtRead:
			blocking = ((msg->i.io.mode & O_NONBLOCK) == 0u);
			msg->o.err = pipeRead(minor, msg->o.data, msg->o.size, blocking);
			break;

		case mtGetAttr:
			if (msg->i.attr.type != atPollStatus) {
				msg->o.err = -ENOSYS;
				break;
			}
			msg->o.attr.val = pipePollStatus(minor);
			msg->o.err = 0;
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}

	msgRespond(msg->oid.port, msg, rid);
}


static int pipe_init(void)
{
	addr_t bufaddr;
	memrange_t range;

	int res = getMapRange(PIPE_MEMMAP, &range);

	if (res < 0 || range.size < (sizeof(*common.pipes) * PIPE_CNT)) {
		return -1;
	}

	common.pipes = range.start;
	bufaddr = (addr_t)((uint8_t *)common.pipes + sizeof(*common.pipes) * PIPE_CNT);
	bufaddr = ALIGN_ADDR(bufaddr, 8);

	/* Check map bounds first, to avoid resource allocation */

	for (size_t i = 0; i < PIPE_CNT; i++) {
		common.pipes[i].txsz = pipe_defs[i].txsz;
		common.pipes[i].rxsz = pipe_defs[i].rxsz;

		if (common.pipes[i].txsz > 0) {
			if (bufaddr + common.pipes[i].txsz > (addr_t)range.start + range.size) {
				return -1;
			}

			common.pipes_priv[i].txBufaddr = bufaddr;
			bufaddr = ALIGN_ADDR(bufaddr + common.pipes[i].txsz, 8);
		}

		if (common.pipes[i].rxsz > 0) {
			if (bufaddr + common.pipes[i].rxsz > (addr_t)range.start + range.size) {
				return -1;
			}

			common.pipes_priv[i].rxBufaddr = bufaddr;
			bufaddr = ALIGN_ADDR(bufaddr + common.pipes[i].rxsz, 8);
		}
	}

	/* Initialize FIFOs and allocate resources */

	for (size_t i = 0; i < PIPE_CNT; i++) {
		if (common.pipes[i].txsz > 0) {
			uint8_t *buf = (uint8_t *)common.pipes_priv[i].txBufaddr;
			lf_fifo_init(&common.pipes[i].txfifo, buf, common.pipes[i].txsz);
			mutexCreate(&common.pipes_priv[i].txLock);
		}

		if (common.pipes[i].rxsz > 0) {
			uint8_t *buf = (uint8_t *)common.pipes_priv[i].rxBufaddr;
			lf_fifo_init(&common.pipes[i].rxfifo, buf, common.pipes[i].rxsz);
			mutexCreate(&common.pipes_priv[i].rxLock);
		}
	}

	/* Send the address and number of shared FIFOs to the other core */
	/* TODO: 0x1 is an arbitrary message type for sending FIFO config - consider using enums */
	uint8_t buf[6] = { 0x1, PIPE_CNT };
	memcpy(&buf[2], (uint8_t *)&common.pipes, sizeof(common.pipes));
	cpu_fifoWrite(buf, sizeof(buf), true, 1);

	return 0;
}


static void __attribute__((constructor(1500))) pipe_register(void)
{
	if (PIPE_CNT == 0 || pipe_init() < 0) {
		return;
	}

	if (dev_allocMajor(&common.major) < 0) {
		/* TODO - exit the driver? trigger the reset? report the error? */
		return;
	}

	dev_register(pipe_handleMsg, common.major);

	for (size_t i = 0; i < PIPE_CNT; i++) {
		char fname[8];

		if (common.pipes[i].txsz > 0 || common.pipes[i].rxsz > 0) {
			snprintf(fname, sizeof(fname), "pipe%u", i);

			if (dev_registerFile(fname, common.major, i) < 0) {
				/* TODO - exit the driver? trigger the reset? report the error? */
			}
		}
	}
}
