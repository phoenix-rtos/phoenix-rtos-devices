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
#include <stdint.h>
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
#include <phoenix/fbcon.h>

#include "ttypc.h"
#include "ttypc_bioskbd.h"
#include "ttypc_kbd.h"
#include "ttypc_vga.h"

#define KMSG_CTRL_ID 100

ttypc_t ttypc_common;


static void ttypc_poolthr(void *arg)
{
	ttypc_t *ttypc = (ttypc_t *)arg;
	const void *idata, *odata = NULL;
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
				msg.o.err = EOK;
				break;

			case mtRead:
				if (msg.oid.id < NVTS)
					msg.o.err = ttypc_vt_read(ttypc->vts + msg.oid.id, msg.i.io.mode, msg.o.data, msg.o.size);
				else
					msg.o.err = -EINVAL;
				break;

			case mtWrite:
				if (msg.oid.id < NVTS)
					msg.o.err = ttypc_vt_write(ttypc->vts + msg.oid.id, msg.i.io.mode, msg.i.data, msg.i.size);
				else
					msg.o.err = -EINVAL;
				break;

			case mtClose:
				break;

			case mtGetAttr:
				if ((msg.i.attr.type != atPollStatus) || (msg.oid.id >= NVTS)) {
					msg.o.err = -EINVAL;
					break;
				}
				msg.o.attr.val = ttypc_vt_pollstatus(ttypc->vts + msg.oid.id);
				msg.o.err = EOK;
				break;

			case mtDevCtl:
				idata = ioctl_unpack(&msg, &req, &id);
				if (req == KIOEN) {
					if (id == 0) {
						libklog_enable((int)(intptr_t)idata);
						err = EOK;
					}
					else {
						err = -EINVAL;
					}
				}
				else {
					if (id < NVTS) {
						err = ttypc_vt_ioctl(ttypc->vts + id, ioctl_getSenderPid(&msg), req, idata, &odata);
					}
					else {
						err = -EINVAL;
					}
				}
				ioctl_setResponse(&msg, req, err, odata);
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(ttypc->port, &msg, rid);
	}
}


static void ttypc_klogClbk(const char *data, size_t size)
{
	libtty_write(&ttypc_common.vts[0].tty, data, size, 0);
}


int main(int argc, char **argv)
{
	unsigned int i;
	char path[12];
	oid_t oid;
	int err;

	int isconsole = 1;
	if ((argc == 2) && (strcmp(argv[1], "-n") == 0)) {
		isconsole = 0;
	}
	else if (argc != 1) {
		return -1;
	}

	/* Initialize driver */
	memset(&ttypc_common, 0, sizeof(ttypc_t));

	if ((err = portCreate(&ttypc_common.port)) < 0)
		return err;

	if ((err = mutexCreate(&ttypc_common.lock)) < 0)
		return err;

	/* Initialize VGA display */
	err = ttypc_vga_init(&ttypc_common);
	if (err < 0) {
		return err;
	}

	/* Initialize VTs */
	for (i = 0; i < NVTS; i++) {
		err = ttypc_vt_init(&ttypc_common, ttypc_common.memsz, ttypc_common.vts + i);
		if (err < 0) {
			return err;
		}
	}

	/* Enable fbcon in VTs if available */
	if ((ttypc_common.fbmaxcols != -1) && (ttypc_common.fbmaxrows != -1)) {
		for (i = 0; i < NVTS; i++) {
			ttypc_common.vts[i].fbmode = FBCON_ENABLED;
		}
	}

	/* Set active virtual terminal */
	ttypc_common.vt = ttypc_common.vts;
	ttypc_common.vt->vram = ttypc_common.vga;

	if (ttypc_common.vt->fbmode == FBCON_ENABLED) {
		/* Resize active virtual terminal to match fbcon maximum resolution */
		ttypc_vt_resize(ttypc_common.vt, ttypc_common.fbmaxcols, ttypc_common.fbmaxrows);
	}

	/* Initialize cursor */
	if (ttypc_common.vt->fbmode == FBCON_UNSUPPORTED) {
		/* If fbcon is unsupported, retrieve the cursor so we don't overwrite the tty as
		 * some earlier component might have written something to the text mode buffer (i.e. plo) */
		_ttypc_vga_getcursor(ttypc_common.vt);
	}
	/* else: In case of fbcon, we don't care about the text mode buffer, because we're
	 * in the graphic mode already and the text mode buffer may contain garbage */

	/* Set default cursor color */
	_ttypc_vga_set(ttypc_common.vt, ttypc_common.vt->cpos, FG_LIGHTGREY << 8, ttypc_common.vt->rows * ttypc_common.vt->cols - ttypc_common.vt->cpos);

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

	if (isconsole != 0) {
		libklog_init(ttypc_klogClbk);
		oid.port = ttypc_common.port;
		oid.id = 0;
		if (create_dev(&oid, _PATH_CONSOLE) < 0) {
			fprintf(stderr, "pc-tty: failed to register device %s\n", _PATH_CONSOLE);
		}

		oid_t kmsgctrl = { .port = ttypc_common.port, .id = KMSG_CTRL_ID };
		libklog_ctrlRegister(&kmsgctrl);
	}
	else {
		libklog_initNoDev(ttypc_klogClbk);
	}

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
