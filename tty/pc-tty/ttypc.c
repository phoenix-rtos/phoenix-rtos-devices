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
#include <poll.h>
#include <stdio.h>
#include <string.h>

#include <sys/file.h>
#include <sys/io.h>
#include <sys/ioctl.h>
#include <sys/msg.h>
#include <sys/threads.h>

#include "ttypc.h"
#include "ttypc_bioskbd.h"
#include "ttypc_kbd.h"
#include "ttypc_vga.h"


ttypc_t ttypc_common;


static void ttypc_poolthr(void *arg)
{
	ttypc_t* ttypc = (ttypc_t *)arg;
	const void *idata, *odata;
	unsigned long req;
	unsigned int rid;
	msg_t msg;
	id_t id;
	int err;

	for (;;) {
		if (msgRecv(ttypc->port, &msg, &rid) < 0)
			continue;

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
			if ((msg.i.attr.type == atPollStatus) && (msg.i.io.oid.id < NVTS))
				msg.o.attr.val = ttypc_vt_pollstatus(ttypc->vts + msg.i.io.oid.id);
			else
				msg.o.attr.val = -EINVAL;
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


int main(void)
{
	unsigned int i;
	char name[10];
	oid_t oid;
	int err;

	/* Initialize driver */
	memset(&ttypc_common, 0, sizeof(ttypc_t));

	if ((err = portCreate(&ttypc_common.port)) < 0)
		return err;

	if ((err = mutexCreate(&ttypc_common.lock)) < 0)
		return err;

	/* Initialize and register virtual terminals */
	for (i = 0; i < NVTS; i++) {
		snprintf(name, sizeof(name), "/dev/tty%u", i);
		oid.port = ttypc_common.port;
		oid.id = i;

		if ((err = ttypc_vt_init(&ttypc_common, _PAGE_SIZE, ttypc_common.vts + i)) < 0)
			return err;

		if ((err = portRegister(oid.port, name, &oid)) < 0)
			return err;
	}
	printf("pc-tty: Initialized %u virtual terminals\n", NVTS);

	/* Initialize VGA display */
	if ((err = ttypc_vga_init(&ttypc_common)) < 0)
		return err;
	printf("pc-tty: Initialized VGA display\n");

	/* Set active virtual terminal */
	ttypc_common.vt = ttypc_common.vts;
	ttypc_common.vt->vram = ttypc_common.vga;

	/* Initialize cursor */
	_ttypc_vga_getcursor(ttypc_common.vt);
	/* Set default cursor color */
	_ttypc_vga_set(ttypc_common.vt->vram + ttypc_common.vt->cpos, FG_LIGHTGREY << 8, ttypc_common.vt->rows * ttypc_common.vt->cols - ttypc_common.vt->cpos);

	/* Initialize keyboard */
	if (ttypc_kbd_configure(&ttypc_common)) {
		if ((err = ttypc_kbd_init(&ttypc_common)) < 0)
			return err;
		ttypc_common.ktype = KBD_PS2;
	}
	else {
		if ((err = ttypc_bioskbd_init(&ttypc_common)) < 0)
			return err;
		ttypc_common.ktype = KBD_BIOS;
	}
	printf("pc-tty: Initialized %s keyboard\n", (ttypc_common.ktype) ? "PS/2" : "BIOS");

	/* Start poolthreads */
	if ((err = beginthread(ttypc_poolthr, 1, ttypc_common.pstack, sizeof(ttypc_common.pstack), &ttypc_common)) < 0)
		return err;
	ttypc_poolthr(&ttypc_common);

	return EOK;
}
