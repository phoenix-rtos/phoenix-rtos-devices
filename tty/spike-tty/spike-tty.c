/*
 * Phoenix-RTOS
 *
 * Spike tty (HTIF console)
 *
 * Copyright 2020, 2021 Phoenix Systems
 * Author: Pawel Pisarczyk, Julia Kosowska, Lukasz Kosinski
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <paths.h>

#include <board_config.h>
#include <posix/utils.h>
#include <sys/file.h>
#include <sys/io.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <libtty.h>
#include <libklog.h>

#define KMSG_CTRL_ID 100


typedef struct {
	int active;
	oid_t oid;
	libtty_common_t tty;
	char stack[1024] __attribute__((aligned(8)));
} spiketty_t;


static struct {
	spiketty_t spikettys[1];
	char stack[2048] __attribute__((aligned(8)));
} spiketty_common;


static spiketty_t *spiketty_get(oid_t *oid)
{
	spiketty_t *spiketty;
	unsigned int i;

	for (i = 0; i < sizeof(spiketty_common.spikettys) / sizeof(spiketty_common.spikettys[0]); i++) {
		spiketty = spiketty_common.spikettys + i;

		if (spiketty->active && (spiketty->oid.port == oid->port) && (spiketty->oid.id == oid->id))
			return spiketty;
	}

	return NULL;
}


static void set_baudrate(void *arg, speed_t baud)
{
	/* TODO */
}


static void set_cflag(void *arg, tcflag_t *cflag)
{
	/* TODO */
}


static void signal_txready(void *arg)
{
	spiketty_t *spiketty = (spiketty_t *)arg;

	while (libtty_txready(&spiketty->tty))
		sbi_putchar(libtty_popchar(&spiketty->tty));

	libtty_wake_writer(&spiketty->tty);
}


static void spiketty_ioctl(unsigned int port, msg_t *msg)
{
	const void *idata, *odata = NULL;
	oid_t oid = { .port = port };
	spiketty_t *spiketty;
	unsigned long req;
	int err;

	idata = ioctl_unpack(msg, &req, &oid.id);

	if ((spiketty = spiketty_get(&oid)) == NULL)
		err = -EINVAL;
	else
		err = libtty_ioctl(&spiketty->tty, ioctl_getSenderPid(msg), req, idata, &odata);

	ioctl_setResponse(msg, req, err, odata);
}


static void poolthr(void *arg)
{
	unsigned int port = (uintptr_t)arg;
	spiketty_t *spiketty;
	msg_rid_t rid;
	msg_t msg;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0)
			continue;

		if (libklog_ctrlHandle(port, &msg, rid) == 0) {
			/* msg has been handled by libklog */
			continue;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				if ((spiketty = spiketty_get(&msg.i.io.oid)) == NULL) {
					msg.o.io.err = -EINVAL;
					break;
				}
				msg.o.io.err = EOK;
				break;

			case mtRead:
				if ((spiketty = spiketty_get(&msg.i.io.oid)) == NULL)
					msg.o.io.err = -EINVAL;
				else
					msg.o.io.err = libtty_read(&spiketty->tty, msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtWrite:
				if ((spiketty = spiketty_get(&msg.i.io.oid)) == NULL)
					msg.o.io.err = -EINVAL;
				else
					msg.o.io.err = libtty_write(&spiketty->tty, msg.i.data, msg.i.size, msg.i.io.mode);
				break;

			case mtGetAttr:
				if ((msg.i.attr.type != atPollStatus) || ((spiketty = spiketty_get(&msg.i.attr.oid)) == NULL)) {
					msg.o.attr.err = -EINVAL;
					break;
				}
				msg.o.attr.val = libtty_poll_status(&spiketty->tty);
				msg.o.attr.err = EOK;
				break;

			case mtDevCtl:
				spiketty_ioctl(port, &msg);
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void spiketty_thr(void *arg)
{
	spiketty_t *spiketty = (spiketty_t *)arg;
	int c;

	for (;;) {
		if ((c = sbi_getchar()) > 0)
			libtty_putchar(&spiketty->tty, c, NULL);
		usleep(10);
	}
}


static void spiketty_klogClbk(const char *data, size_t size)
{
	libtty_write(&spiketty_common.spikettys[SPIKETTY_CONSOLE_USER].tty, data, size, 0);
}


static int _spiketty_init(spiketty_t *spiketty, unsigned int port, unsigned int id)
{
	libtty_callbacks_t callbacks;
	int err;

	callbacks.arg = spiketty;
	callbacks.set_baudrate = set_baudrate;
	callbacks.set_cflag = set_cflag;
	callbacks.signal_txready = signal_txready;

	if ((err = libtty_init(&spiketty->tty, &callbacks, _PAGE_SIZE, TTYDEF_SPEED)) < 0)
		return err;

	spiketty->active = 1;
	spiketty->oid.port = port;
	spiketty->oid.id = id;
	beginthread(spiketty_thr, 4, spiketty->stack, sizeof(spiketty->stack), spiketty);

	return EOK;
}


int main(void)
{
	unsigned int i, port;
	char path[12];
	int err;

	portCreate(&port);

	for (i = 0; i < sizeof(spiketty_common.spikettys) / sizeof(spiketty_common.spikettys[0]); i++) {
		snprintf(path, sizeof(path), "/dev/tty%u", i);

		if ((err = _spiketty_init(&spiketty_common.spikettys[i], port, i)) < 0) {
			if (err != -ENODEV)
				fprintf(stderr, "spike-tty: failed to init %s, err: %d\n", path, err);
			continue;
		}

		if ((err = portRegister(port, path, &spiketty_common.spikettys[i].oid)) < 0) {
			fprintf(stderr, "spike-tty: failed to register %s, err: %d\n", path, err);
			return err;
		}

		if (i == SPIKETTY_CONSOLE_USER) {

			if (create_dev(&spiketty_common.spikettys[i].oid, _PATH_CONSOLE) < 0) {
				fprintf(stderr, "spike-tty: failed to register %s\n", path);
			}

			libklog_init(spiketty_klogClbk);
			oid_t kmsgctrl = { .port = port, .id = KMSG_CTRL_ID };
			libklog_ctrlRegister(&kmsgctrl);
		}
	}

	beginthread(poolthr, 4, spiketty_common.stack, sizeof(spiketty_common.stack), (void *)(uintptr_t)port);
	poolthr((void *)(uintptr_t)port);

	return EOK;
}
