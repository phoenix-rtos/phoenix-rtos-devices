/*
 * Phoenix-RTOS
 *
 * PS/2 3-button mouse
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>

#include <board_config.h>

#include "ttypc_mouse.h"

#if PC_TTY_MOUSE_ENABLE

#include <poll.h>
#include <unistd.h>

#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/io.h>
#include <sys/threads.h>

#include <libklog.h>
#include <posix/utils.h>

#include "event_queue.h"
#include "ttypc_ps2.h"

#ifndef TTYPC_MOUSE_POOL_PRIO
#define TTYPC_MOUSE_POOL_PRIO 1
#endif

#ifndef PC_TTY_MOUSE_SKIP_CONFIG
#define PC_TTY_MOUSE_SKIP_CONFIG 0
#endif


static int _ttypc_mouse_setup(ttypc_t *ttypc)
{
	unsigned char status;
	int err;

	do {
/*
 * Some BIOS-es do this part of configuration already when emulating PS/2 and
 * redoing it may not be necessary or could even hang the PS/2 controller
 * completely (as is the case on ThinkPad/Phoenix BIOS 2.26)
 */
#if !PC_TTY_MOUSE_SKIP_CONFIG
		/* Enable mouse. Should receive ACK (0xFA) afterwards */
		err = ttypc_ps2_write_ctrl(ttypc, 0xA8);
		if (err < 0) {
			break;
		}
		err = ttypc_ps2_read(ttypc);
		if (err != 0xFA) {
			(void)fprintf(stderr, "pc-tty: mouse not found\n");
			err = -1;
			break;
		}

		/* Get compaq status byte */
		err = ttypc_ps2_write_ctrl(ttypc, 0x20);
		if (err < 0) {
			break;
		}

		status = ttypc_ps2_read(ttypc);
		status |= (1u << 1u); /* Enable IRQ12 interrupt */

/*
 * Some sources say the bit 5 should be unset for proper functioning, but on QEMU
 * and real hardware it is not necessary
 */
#if 0
		status &= ~(1u << 5u); /* Enable mouse clock (unset bit 5) */
#endif

		/* Set compaq status byte */
		err = ttypc_ps2_write_ctrl(ttypc, 0x60);
		if (err < 0) {
			break;
		}
		err = ttypc_ps2_write(ttypc, status);
		if (err < 0) {
			break;
		}
#endif /* !PC_TTY_MOUSE_SKIP_CONFIG */

		/* Set defaults. Should receive ACK afterwards */
		err = ttypc_ps2_write_ctrl(ttypc, 0xD4);
		if (err < 0) {
			break;
		}
		err = ttypc_ps2_write(ttypc, 0xF6);
		if (err < 0) {
			break;
		}
		err = ttypc_ps2_read(ttypc);
		if (err != 0xFA) {
			(void)fprintf(stderr, "pc-tty: mouse initialization failed - no ACK after setting defaults\n");
			err = -1;
			break;
		}

		/* Send mouse cmd - enable packet streaming. Should receive ACK afterwards */
		err = ttypc_ps2_write_ctrl(ttypc, 0xD4);
		if (err < 0) {
			break;
		}
		err = ttypc_ps2_write(ttypc, 0xF4);
		if (err < 0) {
			break;
		}
		err = ttypc_ps2_read(ttypc);
		if (err != 0xFA) {
			(void)fprintf(stderr, "pc-tty: mouse initialization failed - no ACK after enable packet streaming cmd\n");
			err = -1;
			break;
		}
	} while (0);

	if (err < 0) {
		(void)fprintf(stderr, "pc-tty: failed to initialize mouse\n");
		return -1;
	}

	return EOK;
}


#if PC_TTY_CREATE_PS2_VDEVS
static void ttypc_mouse_poolthr(void *arg)
{
	ttypc_t *ttypc = (ttypc_t *)arg;
	msg_rid_t rid;
	msg_t msg;
	int rv;

	for (;;) {
		rv = msgRecv(ttypc->mport, &msg, &rid);
		if (rv < 0) {
			continue;
		}

		rv = libklog_ctrlHandle(ttypc->mport, &msg, rid);
		if (rv == 0) {
			/* msg has been handled by libklog */
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				msg.o.err = EOK;
				break;

			case mtRead:
				msg.o.err = event_queue_get_mouse(&ttypc->meq, msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtWrite:
				rv = ttypc_ps2_write(ttypc, ((unsigned char *)msg.i.data)[0]);
				if (rv < 0) {
					msg.o.err = -EINVAL;
					break;
				}
				msg.o.err = 1;
				break;

			case mtGetAttr:
				if (msg.i.attr.type != atPollStatus) {
					msg.o.err = -EINVAL;
					break;
				}

				msg.o.attr.val = 0;
				rv = event_queue_count(&ttypc->meq);
				if (rv >= 3) {
					msg.o.attr.val |= POLLIN;
				}

				msg.o.err = EOK;
				break;

			case mtClose:
				msg.o.err = EOK;
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(ttypc->mport, &msg, rid);
	}
}
#endif /* PC_TTY_CREATE_PS2_VDEVS */


/* Mouse interrupt handler */
static int _ttypc_mouse_interrupt(unsigned int n, void *arg)
{
	return 0;
}


int ttypc_mouse_init(ttypc_t *ttypc)
{
	int err;

	do {
		err = _ttypc_mouse_setup(ttypc);
		if (err < 0) {
			break;
		}

#if PC_TTY_CREATE_PS2_VDEVS
		oid_t oid;

		/* Create virtual mouse device. Assumes the filesystem is available already */
		err = portCreate(&ttypc->mport);
		if (err < 0) {
			(void)fprintf(stderr, "pc-tty: failed to create mouse port\n");
			break;
		}
		oid.port = ttypc->mport;
		oid.id = 0;

		err = create_dev(&oid, "/dev/mouse");
		if (err < 0) {
			(void)fprintf(stderr, "pc-tty: failed to register mouse device\n");
			portDestroy(ttypc->mport);
			break;
		}

		err = event_queue_init(&ttypc->meq);
		if (err < 0) {
			portDestroy(ttypc->mport);
			break;
		}
#endif

		/* Attach KIRQ12 (mouse event) interrupt handle */
		ttypc->mirq = 12;
		err = interrupt(ttypc->mirq, _ttypc_mouse_interrupt, ttypc, ttypc->kmcond, &ttypc->minth);
		if (err < 0) {
#if PC_TTY_CREATE_PS2_VDEVS
			portDestroy(ttypc->mport);
			event_queue_destroy(&ttypc->meq);
#endif
			break;
		}

#if PC_TTY_CREATE_PS2_VDEVS
		/* Launch mouse pool thread */
		err = beginthread(ttypc_mouse_poolthr, TTYPC_MOUSE_POOL_PRIO, ttypc->mpstack, sizeof(ttypc->mpstack), ttypc);
		if (err < 0) {
			portDestroy(ttypc->mport);
			event_queue_destroy(&ttypc->meq);
			resourceDestroy(ttypc->minth);
			break;
		}
#endif
	} while (0);

	if (err < 0) {
		/*
		 * If mouse initialization has failed at any point, it's better to disable
		 * mouse so that it doesn't clutter the I/O port with events we won't handle
		 */

		/* Disable mouse */
		err = ttypc_ps2_write_ctrl(ttypc, 0xA7);
		if (err < 0) {
			return err;
		}

		/*
		 * Read a byte in case mouse responded. This could be ACK (0xFA) or something
		 * else if mouse is broken. Ignore return value - if read fails, it's ok.
		 */
		(void)ttypc_ps2_read(ttypc);
	}

	return err;
}


int ttypc_mouse_handle_event(ttypc_t *ttypc, unsigned char b)
{
#if PC_TTY_CREATE_PS2_VDEVS
	return event_queue_put(&ttypc->meq, b, 3);
#else
	return EOK;
#endif
}


void ttypc_mouse_destroy(ttypc_t *ttypc)
{
	resourceDestroy(ttypc->minth);
#if PC_TTY_CREATE_PS2_VDEVS
	event_queue_destroy(&ttypc->meq);
	portDestroy(ttypc->mport);
#endif
}

#else /* PC_TTY_ENABLE_MOUSE */

int ttypc_mouse_handle_event(ttypc_t *ttypc, unsigned char b)
{
	fprintf(stderr, "pc-tty: ttypc_mouse_handle_event() called while mouse disabled\n");
	return -1;
}


int ttypc_mouse_init(ttypc_t *ttypc)
{
	return EOK;
}


void ttypc_mouse_destroy(ttypc_t *ttypc)
{
}

#endif /* PC_TTY_ENABLE_MOUSE */
