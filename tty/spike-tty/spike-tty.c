/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Spike tty (HTIF console)
 *
 * Copyright 2020 Phoenix Systems
 * Author: Pawel Pisarczyk
 *
 * %LICENSE%
 */

#include <errno.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <sys/io.h>
#include <sys/file.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/mman.h>

#include <libtty.h>


typedef struct {
//	handle_t mutex;
	oid_t oid;
	libtty_common_t tty;
} spiketty_t;


static spiketty_t *spikettys[1];

static spiketty_t spiketty = {0};

static void set_baudrate(void *_uart, speed_t baud)
{
	/* TODO */
}


static void set_cflag(void *_uart, tcflag_t *cflag)
{
	/* TODO */
}


static void signal_txready(void *_uart)
{
/*	uart_t *uart = _uart;
	outb(uart->base + REG_IMR, IMR_THRE | IMR_DR);
	condSignal(uart->intcond);*/
}


static int spiketty_write(uint8_t d, size_t len, char *buff, int mode)
{
	spiketty_t *spiketty;

	if (d >= sizeof(spikettys) / sizeof(spiketty_t *))
		return -EINVAL;

	if ((spiketty = spikettys[d]) == NULL)
		return -ENOENT;

	if (!len)
		return 0;

	return libtty_write(&spiketty->tty, buff, len, mode);
}


static int spiketty_read(uint8_t d, size_t len, char *buff, int mode)
{
	spiketty_t *spiketty;

	if (d >= sizeof(spikettys) / sizeof(spiketty_t *))
		return -EINVAL;

	if ((spiketty = spikettys[d]) == NULL)
		return -ENOENT;

	return libtty_read(&spiketty->tty, buff, len, mode);
}


static int spiketty_status(uint8_t d)
{
	spiketty_t *spiketty;

	if (d >= sizeof(spikettys) / sizeof(spiketty_t *))
		return POLLNVAL;

	if ((spiketty = spikettys[d]) == NULL)
		return POLLNVAL;

	return libtty_poll_status(&spiketty->tty);
}


uint8_t spiketty_get(oid_t *oid)
{
	unsigned int i;

	for (i = 0; i < sizeof(spikettys) / sizeof(spiketty_t); i++) {
		if ((spikettys[i]->oid.id == oid->id) && (spikettys[i]->oid.port == oid->port))
			return i;
	}
	return 0;
}


static void spiketty_ioctl(unsigned port, msg_t *msg)
{
	spiketty_t *spiketty;
	unsigned long request;
	const void *in_data, *out_data;
	pid_t pid;
	int err;
	oid_t oid;
	uint8_t d;

	oid.port = port;

	in_data = ioctl_unpack(msg, &request, &oid.id);
	out_data = NULL;
	pid = ioctl_getSenderPid(msg);

	if (!(d = spiketty_get(&oid)))
		err = -EINVAL;

	else if ((spiketty = spikettys[d]) == NULL)
		err = -ENOENT;

	else
		err = libtty_ioctl(&spiketty->tty, pid, request, in_data, &out_data);

	ioctl_setResponse(msg, request, err, out_data);
}


void poolthr(void *arg)
{
	uint32_t port = (uint32_t)(uint64_t)arg;
	msg_t msg;
	unsigned long rid;

	for (;;) {

		if (msgRecv(port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtOpen:
			break;
		case mtWrite:
			msg.o.io.err = spiketty_write(spiketty_get(&msg.i.io.oid), msg.i.size, msg.i.data, msg.i.io.mode);
			break;
		case mtRead:
			msg.o.io.err = spiketty_read(spiketty_get(&msg.i.io.oid), msg.o.size, msg.o.data, msg.i.io.mode);
			break;
		case mtClose:
			break;
		case mtGetAttr:
			if (msg.i.attr.type == atPollStatus)
				msg.o.attr.val = spiketty_status(spiketty_get(&msg.i.io.oid));
			else
				msg.o.attr.val = -EINVAL;
			break;
		case mtDevCtl:
			spiketty_ioctl(port, &msg);
			break;
		}
		msgRespond(port, &msg, rid);
	}
}

void spiketty_thr(void *arg)
{
	spiketty_t *spiketty = (spiketty_t *)arg;

	int c;

	for (;;) {
		do {
			c = sbi_getchar();
			if (c > 0) {
				libtty_putchar(&spiketty->tty, c, NULL);
			}
		} while (c > 0);

		/* Transmit */
		while (libtty_txready(&spiketty->tty)) {
			c = libtty_getchar(&spiketty->tty, NULL);
			sbi_putchar(c);
		}
		usleep(10000);
	}
}


int _spiketty_init(spiketty_t **spiketty)
{
	libtty_callbacks_t callbacks;

	callbacks.arg = *spiketty;
	callbacks.set_baudrate = set_baudrate;
	callbacks.set_cflag = set_cflag;
	callbacks.signal_txready = signal_txready;

	libtty_init(&(*spiketty)->tty, &callbacks, _PAGE_SIZE);

	uint8_t *stack;
	stack = (uint8_t *)malloc(2 * 4096);

	beginthread(spiketty_thr, 1, stack, 2 * 4096, (void *)*spiketty);

	return EOK;
}


int main(void)
{
	uint32_t port;

	/* printf("riscv-spiketty: Initializing RISCV HTIF console driver %s\n", ""); */

	spikettys[0] = &spiketty;
	_spiketty_init(&spikettys[0]);

	portCreate(&port);
	if (portRegister(port, "/dev/tty0", &spikettys[0]->oid) < 0) {
		printf("Can't register port %d\n", port);
		return -1;
	}

	/* Run threads */
	void *stack = malloc(2048);	
	beginthread(poolthr, 4, stack, 2048, (void *)(uint64_t)port);

	poolthr((void *)(uint64_t)port);

	return 0;
}
