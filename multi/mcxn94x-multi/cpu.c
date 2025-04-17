/*
 * Phoenix-RTOS
 *
 * MCX N94x M33 CPU core driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Daniel Sawka
 *
 * %LICENSE%
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/minmax.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>

#include <libtty-lf-fifo.h>

#include "common.h"
#include "dev.h"

#define REG_IRQ_OFFSET 4
/* clang-format off */
enum {
	reg_irq = 0, reg_irq_set, reg_irq_clr, mutex = 62
};
/* clang-format on */

#define REG_IRQ(CPUID, REG) (*(common.mailbox + (REG_IRQ_OFFSET * (CPUID)) + (REG)))


static struct {
	uint8_t rxBuf[64];
	lf_fifo_t rxFifo;
	handle_t rxLock;
	handle_t txLock;
	handle_t irqLock;
	handle_t rxCond;

	volatile uint32_t *mailbox;
	unsigned int major;
	uint8_t coreidSelf;
	uint8_t coreidRemote;
} common;


static int mailbox_irqHandler(unsigned int n, void *arg)
{
	size_t consumed = 0;
	uint32_t value = REG_IRQ(common.coreidSelf, reg_irq);

	if (value != 0) {
		REG_IRQ(common.coreidSelf, reg_irq_clr) = value;

		size_t rxLen = (value >> 24) & 0x3;

		for (; consumed < rxLen; consumed++) {
			uint8_t byte = (value >> (8 * consumed)) & 0xff;

			/* TODO: Data is overwritten if rx fifo is full. Consider disabling
			the interrupt somehow and leaving the value in the register until
			the application reads from rx fifo. Also, fifo needs to be more
			generic and at least have function to get its current length. */
			lf_fifo_push(&common.rxFifo, byte);
		}
	}

	return (consumed > 0) ? 1 : -1;
}


static ssize_t fifoRead(unsigned char *buf, size_t size, bool blocking)
{
	size_t bytesRead = 0;

	if (buf == NULL || size == 0) {
		return -EINVAL;
	}

	/* Synchronize multiple readers */
	mutexLock(common.rxLock);

	while (bytesRead < size) {
		mutexLock(common.irqLock);
		while (lf_fifo_empty(&common.rxFifo) != 0) {
			if (!blocking) {
				mutexUnlock(common.irqLock);
				mutexUnlock(common.rxLock);
				return (bytesRead > 0) ? (ssize_t)bytesRead : -EAGAIN;
			}
			condWait(common.rxCond, common.irqLock, 0);
		}
		mutexUnlock(common.irqLock);

		while (bytesRead < size) {
			if (lf_fifo_pop(&common.rxFifo, &buf[bytesRead]) != 0) {
				bytesRead += 1;
			}
			else {
				break;
			}
		}
	}

	mutexUnlock(common.rxLock);

	return (ssize_t)bytesRead;
}


static ssize_t fifoWrite(const unsigned char *buf, size_t len, bool blocking)
{
	size_t bytesWritten = 0;

	if (buf == NULL || len == 0) {
		return -EINVAL;
	}

	/* Synchronize multiple writers */
	mutexLock(common.txLock);

	while (bytesWritten < len) {
		if (!blocking && REG_IRQ(common.coreidRemote, reg_irq) != 0u) {
			mutexUnlock(common.txLock);
			return (bytesWritten > 0) ? bytesWritten : -EAGAIN;
		}

		while (blocking && REG_IRQ(common.coreidRemote, reg_irq) != 0u) {
			; /* Wait until the other CPU reads IRQx register */
		}

		/* Message structure: MSB[ 6b reserved | 2b len(data) | 24b data ]LSB */
		uint32_t value = 0;
		size_t toWrite = min(len - (size_t)bytesWritten, 3);

		for (size_t i = 0; i < toWrite; i++) {
			value |= (uint32_t)buf[bytesWritten] << (8 * i);
			bytesWritten += 1;
		}

		value |= ((uint32_t)toWrite & 0x3) << 24;

		REG_IRQ(common.coreidRemote, reg_irq_set) = value;
	}

	mutexUnlock(common.txLock);

	return (ssize_t)bytesWritten;
}


static int fifoPollStatus(void)
{
	int revents = 0;

	if (lf_fifo_empty(&common.rxFifo) == 0) {
		revents |= POLLIN | POLLRDNORM;
	}

	if (REG_IRQ(common.coreidRemote, reg_irq) == 0u) {
		revents |= POLLOUT | POLLWRNORM;
	}

	return revents;
}


static void cpu_handleMsg(msg_t *msg, msg_rid_t rid, unsigned int major, unsigned int minor)
{
	bool blocking;

	if (minor > 0) {
		msg->o.err = -EINVAL;
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
			msg->o.err = fifoWrite(msg->i.data, msg->i.size, blocking);
			break;

		case mtRead:
			blocking = ((msg->i.io.mode & O_NONBLOCK) != 0u);
			msg->o.err = fifoRead(msg->o.data, msg->o.size, blocking);
			break;

		case mtGetAttr:
			if (msg->i.attr.type != atPollStatus) {
				msg->o.err = -ENOSYS;
				break;
			}
			msg->o.attr.val = fifoPollStatus();
			msg->o.err = 0;
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}

	msgRespond(msg->oid.port, msg, rid);
}


static void cpu_init(void)
{
	common.mailbox = (void *)0x400b2000;

	lf_fifo_init(&common.rxFifo, common.rxBuf, sizeof(common.rxBuf));

	mutexCreate(&common.rxLock);
	mutexCreate(&common.txLock);
	mutexCreate(&common.irqLock);
	condCreate(&common.rxCond);

	platformctl_t pctl = {
		.action = pctl_get,
		.type = pctl_cpuid
	};

	platformctl(&pctl);
	common.coreidSelf = pctl.cpuid;
	common.coreidRemote = (pctl.cpuid + 1) % 2;

	interrupt(mailbox0_irq, mailbox_irqHandler, NULL, common.rxCond, NULL);
}


static void __attribute__((constructor)) cpu_register(void)
{
	cpu_init();

	if (dev_allocMajor(&common.major) < 0) {
		/* TODO - exit the driver? trigger the reset? report the error? */
		return;
	}

	dev_register(cpu_handleMsg, common.major);

	if (dev_registerFile("cpuM33", common.major, 0) < 0) {
		/* TODO - exit the driver? trigger the reset? report the error? */
	}
}
