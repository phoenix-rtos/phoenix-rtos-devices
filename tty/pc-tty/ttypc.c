/*
 * Phoenix-RTOS
 *
 * ttypc - VT220 terminal emulator based on VGA and 101-key US keyboard
 *
 * Driver initialization
 *
 * Copyright 2012, 2017, 2019 Phoenix Systems
 * Copyright 2006, 2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Pawel Kolodziej, Janusz Gralak, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>
#include <sys/io.h>
#include <sys/debug.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/msg.h>
#include <posix/utils.h>
#include <phoenix/stat.h>

#include "ttypc.h"
#include "ttypc_vga.h"
#include "ttypc_kbd.h"

#define PORT_DESCRIPTOR 3

ttypc_t ttypc_common;


static unsigned int ttypc_virt_get(id_t *id)
{
	unsigned int i;

	for (i = 0; i < sizeof(ttypc_common.virtuals) / sizeof(ttypc_virt_t); i++) {
		if ((ttypc_common.virtuals[i].oid.id == *id))
			return i;
	}

	return 0;
}


static int ttypc_read(unsigned int d, char *buff, size_t len, int mode)
{
	if (d >= sizeof(ttypc_common.virtuals) / sizeof(ttypc_virt_t))
		return -EINVAL;

	return ttypc_virt_sget(&ttypc_common.virtuals[d], buff, len, mode);
}


static int ttypc_write(unsigned int d, char *buff, size_t len, int mode)
{
	if (d >= sizeof(ttypc_common.virtuals) / sizeof(ttypc_virt_t))
		return -EINVAL;

	if (!len)
		return 0;

	return ttypc_virt_sadd(&ttypc_common.virtuals[d], buff, len, mode);
}


static int ttypc_poll_status(unsigned int d)
{
	if (d >= sizeof(ttypc_common.virtuals) / sizeof(ttypc_virt_t))
		return POLLNVAL;

	return ttypc_virt_poll_status(&ttypc_common.virtuals[d]);
}


static void ttypc_ioctl(unsigned int port, msg_t *msg)
{
	unsigned long request;
	const void *in_data, *out_data;
	pid_t pid = msg->pid;
	int err;
	unsigned int d;

	request = msg->i.devctl;
	in_data = msg->i.data; //ioctl_unpack(msg, &request, &oid.id);
	out_data = NULL;

	d = ttypc_virt_get(&msg->object);

	if (d >= sizeof(ttypc_common.virtuals) / sizeof(ttypc_virt_t))
		err = -EINVAL;
	else
		err = ttypc_virt_ioctl(&ttypc_common.virtuals[d], pid, request, in_data, &out_data);

	ioctl_setResponse(msg, request, err, out_data);
}


static int _ttypc_init(void *base, unsigned int irq)
{
	unsigned int i;

	/* Test monitor type */
	memset(&ttypc_common, 0, sizeof(ttypc_t));
	ttypc_common.color = (inb((void *)0x3cc) & 0x01);

	ttypc_common.out_base = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, 0, FD_PHYSMEM, ttypc_common.color ? 0xb8000 : 0xb0000);
	ttypc_common.out_crtc = ttypc_common.color ? (void *)0x3d4 : (void *)0x3b4;

	/* Initialize virtual terminals */
	for (i = 0; i < sizeof(ttypc_common.virtuals) / sizeof(ttypc_virt_t); i++) {
		if (_ttypc_virt_init(&ttypc_common.virtuals[i], _PAGE_SIZE, &ttypc_common) < 0) {
			printf("ttypc: Can't initialize virtual terminal %d!\n", i);
			return -1;
		}
	}

	ttypc_common.cv = &ttypc_common.virtuals[0];
	ttypc_common.cv->vram = ttypc_common.out_base;
	ttypc_common.cv->active = 1;

	_ttypc_vga_getcursor(ttypc_common.cv);
	memsetw(ttypc_common.out_base + ttypc_common.cv->cur_offset * 2, 0x0700, 2000 - ttypc_common.cv->cur_offset);

	ttypc_common.irq = irq;
	ttypc_common.base = base;

	mutexCreate(&ttypc_common.mutex);

	/* Initialize keyboard */
	_ttypc_kbd_init(&ttypc_common);

	return 0;
}


static void poolthr(void *arg)
{
	uint32_t port = PORT_DESCRIPTOR;
	msg_t msg;
	unsigned int rid;
	int err = 0;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtOpen:
			msg.o.open = msg.object;
			err = EOK;
			break;
		case mtWrite:
			if ((err = ttypc_write(ttypc_virt_get(&msg.object), msg.i.data, msg.i.size, 0)) < 0) {
				msg.o.io = 0;
			}
			else {
				msg.o.io = err;
				err = EOK;
			}
			break;
		case mtRead:
			if ((err = ttypc_read(ttypc_virt_get(&msg.object), msg.o.data, msg.o.size, 0)) < 0) {
				msg.o.io = 0;
			}
			else {
				msg.o.io = err;
				err = EOK;
			}
			break;
		case mtClose:
			err = EOK;
			break;
		case mtGetAttr:
			if (msg.i.attr == atEvents) {
				*(int *)msg.o.data = ttypc_poll_status(ttypc_virt_get(&msg.object));
			}
			else
				err = -EINVAL;
			break;
		case mtDevCtl:
			ttypc_ioctl(port, &msg);
			err = msg.o.io;
			break;
		}

		msgRespond(port, err, &msg, rid);
	}
}


int main(void)
{
	void *base = (void *)0x60;
	unsigned int n = 1;
	uint32_t port;

	debug("pc-tty: Initializing VGA VT220 terminal emulator\n");

	/* Register port in the namespace */
	if (create_dev(PORT_DESCRIPTOR, 0, "/dev/ttyS0", S_IFCHR) < 0) {
		debug("pc-tty: Could not create device file\n");
		return -1;
	}

	if (fork())
		exit(EXIT_SUCCESS);
	setsid();

	_ttypc_init(base, n);

	/* Run threads */
	beginthread(poolthr, 3, &ttypc_common.poolthr_stack[0], sizeof(ttypc_common.poolthr_stack[0]), (void *)port);
	beginthread(poolthr, 3, &ttypc_common.poolthr_stack[1], sizeof(ttypc_common.poolthr_stack[1]), (void *)port);
	poolthr((void *)port);

	return 0;
}
