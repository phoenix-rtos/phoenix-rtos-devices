/*
 * Phoenix-RTOS
 *
 * Terminal emulator using VGA display and 101-key US keyboard (based on FreeBSD 4.4 pcvt)
 *
 * Copyright 2006, 2008 Pawel Pisarczyk
 * Copyright 2012, 2017, 2019, 2020 Phoenix Systems
 * Author: Pawel Pisarczyk, Pawel Kolodziej, Janusz Gralak, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <paths.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/file.h>
#include <sys/io.h>
#include <sys/ioctl.h>
#include <sys/msg.h>
#include <sys/threads.h>

#include <libklog.h>
#include <posix/utils.h>

#include "ttypc.h"
#include "ttypc_bioskbd.h"
#include "ttypc_kbd.h"
#include "ttypc_vga.h"

#define KMSG_CTRL_ID 100

ttypc_t ttypc_common;


static void ttypc_poolthr(void *arg)
{
	ttypc_t* ttypc = (ttypc_t *)arg;
	const void *idata, *odata;
	unsigned long req;
	msg_rid_t rid;
	msg_t msg;
	id_t id;
	int err;

	for (;;) {
		if (msgRecv(ttypc->port, &msg, &rid) < 0)
			continue;

		if (libklog_ctrlHandle(ttypc->port, &msg, rid) == 0) {
			/* msg has been handled by libklog */
			continue;
		}

		switch (msg.type) {
		case mtOpen:
			break;

		case mtRead:
			if (msg.i.io.oid.id < NVTS)
				msg.o.io.err = ttypc_vt_read(ttypc->vts + msg.i.io.oid.id, msg.i.io.mode, msg.o.data, msg.o.size);
			else
				msg.o.io.err = -EINVAL;
			break;

		case mtWrite:
			if (msg.i.io.oid.id < NVTS)
				msg.o.io.err = ttypc_vt_write(ttypc->vts + msg.i.io.oid.id, msg.i.io.mode, msg.i.data, msg.i.size);
			else
				msg.o.io.err = -EINVAL;
			break;

		case mtClose:
			break;

		case mtGetAttr:
			if ((msg.i.attr.type != atPollStatus) || (msg.i.attr.oid.id >= NVTS)) {
				msg.o.attr.err = -EINVAL;
				break;
			}
			msg.o.attr.val = ttypc_vt_pollstatus(ttypc->vts + msg.i.attr.oid.id);
			msg.o.attr.err = EOK;
			break;

		case mtDevCtl:
			idata = ioctl_unpack(&msg, &req, &id);
			if (id < NVTS) {
				err = ttypc_vt_ioctl(ttypc->vts + id, ioctl_getSenderPid(&msg), req, idata, &odata);
			}
			else {
				odata = NULL;
				err = -EINVAL;
			}
			ioctl_setResponse(&msg, req, err, odata);
			break;
		}

		msgRespond(ttypc->port, &msg, rid);
	}
}


static void ttypc_klogClbk(const char *data, size_t size)
{
	libtty_write(&ttypc_common.vts[0].tty, data, size, 0);
}


int main(void)
{
	unsigned int i;
	char path[12];
	oid_t oid;
	int err;

	/* Initialize driver */
	memset(&ttypc_common, 0, sizeof(ttypc_t));

	if ((err = portCreate(&ttypc_common.port)) < 0)
		return err;

	if ((err = mutexCreate(&ttypc_common.lock)) < 0)
		return err;

	/* Initialize VTs */
	for (i = 0; i < NVTS; i++) {
		if ((err = ttypc_vt_init(&ttypc_common, _PAGE_SIZE, ttypc_common.vts + i)) < 0)
			return err;
	}

	/* Initialize VGA display */
	if ((err = ttypc_vga_init(&ttypc_common)) < 0)
		return err;

	/* Set active virtual terminal */
	ttypc_common.vt = ttypc_common.vts;
	ttypc_common.vt->vram = ttypc_common.vga;

	/* Initialize cursor */
	_ttypc_vga_getcursor(ttypc_common.vt);
	/* Set default cursor color */
	_ttypc_vga_set(ttypc_common.vt->vram + ttypc_common.vt->cpos, FG_LIGHTGREY << 8, ttypc_common.vt->rows * ttypc_common.vt->cols - ttypc_common.vt->cpos);

	/* Run pool threads */
	if ((err = beginthread(ttypc_poolthr, 1, ttypc_common.pstack, sizeof(ttypc_common.pstack), &ttypc_common)) < 0)
		return err;

	/* Initialize keyboard */
	if (inb((void *)0x64) == 0xff) {
		fprintf(stderr, "pc-tty: no PS/2 keyboard detected\n");
		if ((err = ttypc_bioskbd_init(&ttypc_common)) < 0)
			return err;
		ttypc_common.ktype = KBD_BIOS;
	}
	else {
		if ((err = ttypc_kbd_init(&ttypc_common)) < 0)
			return err;
		ttypc_common.ktype = KBD_PS2;
	}

	/* Wait for the filesystem */
	while (lookup("/", NULL, &oid) < 0)
		usleep(10000);

	oid.port = ttypc_common.port;
	oid.id = 0;
	if (create_dev(&oid, _PATH_CONSOLE) < 0) {
		fprintf(stderr, "pc-tty: failed to register device %s\n", _PATH_CONSOLE);
	}

	libklog_init(ttypc_klogClbk);
	oid_t kmsgctrl = { .port = ttypc_common.port, .id = KMSG_CTRL_ID };
	libklog_ctrlRegister(&kmsgctrl);

	/* Register devices */
	for (i = 0; i < NVTS; i++) {
		snprintf(path, sizeof(path), "/dev/tty%u", i);
		oid.port = ttypc_common.port;
		oid.id = i;

		if (create_dev(&oid, path) < 0) {
			fprintf(stderr, "pc-tty: failed to register device %s\n", path);
		}

	}

	ttypc_poolthr(&ttypc_common);

	return EOK;
}
