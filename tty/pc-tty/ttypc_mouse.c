#include <errno.h>
#include <paths.h>
#include <poll.h>
#include <unistd.h>
#include <stdbool.h>

#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/io.h>
#include <sys/threads.h>
#include <sys/reboot.h>

#include <libklog.h>
#include <fcntl.h>
#include <posix/utils.h>

#include "event_queue.h"
#include "ttypc_ps2.h"


static int _ttypc_mouse_setup(ttypc_t *ttypc)
{
	unsigned char status;

	do {
		/* Enable mouse */
		ttypc_ps2_write_ctrl(ttypc, 0xA8);
		if ((status = ttypc_ps2_read(ttypc)) != 0xFA) {
			fprintf(stderr, "pc-tty: mouse not found\n");
			break;
		}

		/* Get compaq status byte */
		ttypc_ps2_write_ctrl(ttypc, 0x20);

		status = ttypc_ps2_read(ttypc);
		status |= (1 << 1);  /* Enable IRQ12 interrupt */
		status &= ~(1 << 5); /* Enable mouse clock (unset bit 5) */

		/* Set compaq status byte */
		ttypc_ps2_write_ctrl(ttypc, 0x60);
		ttypc_ps2_write(ttypc, status);

		/* Set defaults */
		ttypc_ps2_write_ctrl(ttypc, 0xD4);
		ttypc_ps2_write(ttypc, 0xF6);
		if ((status = ttypc_ps2_read(ttypc)) != 0xFA) {
			fprintf(stderr, "pc-tty: mouse initialization failed - no ACK after setting defaults\n");
			break;
		}

		/* Send mouse cmd - enable packet streaming */
		ttypc_ps2_write_ctrl(ttypc, 0xD4);
		ttypc_ps2_write(ttypc, 0xF4);
		if ((status = ttypc_ps2_read(ttypc)) != 0xFA) {
			fprintf(stderr, "pc-tty: mouse initialization failed - no ACK after enable packet streaming cmd\n");
			break;
		}

		/* Is the mouse alive? */
		ttypc_ps2_write_ctrl(ttypc, 0xD4);
		ttypc_ps2_write(ttypc, 0xEB);
		if ((status = ttypc_ps2_read(ttypc)) != 0xFA)
			fprintf(stderr, "pc-tty: mouse died\n");
	} while (0);

	if (status != 0xFA) {
		/** If mouse initialization had failed at any point, disable mouse
		  so that it doesn't clutter the I/O port with events we won't handle */
		ttypc_ps2_write_ctrl(ttypc, 0xA7);
		if ((status = ttypc_ps2_read(ttypc)) != 0xFA) {
			fprintf(stderr, "pc-tty: could not disable mouse (?)\n");
		}

		return -1;
	}

	return EOK;
}


static void _ttypc_kbd_mouse_poolthr(void *arg)
{
	ttypc_t *ttypc = (ttypc_t *)arg;
	msg_rid_t rid;
	msg_t msg;

	for (;;) {
		if (msgRecv(ttypc->mport, &msg, &rid) < 0)
			continue;

		if (libklog_ctrlHandle(ttypc->mport, &msg, rid) == 0) {
			/* msg has been handled by libklog */
			continue;
		}

		switch (msg.type) {
			case mtOpen:
				msg.o.err = EOK;
				break;

			case mtRead:
				if (msg.o.size < 3) {
					/* Pretend there's nothing to read. We don't want someone to
			 read partial mouse packets - this would mess up byte ordering
			 and confuse recipients. */
					msg.o.err = 0;
					break;
				}
				msg.o.err = event_queue_get(ttypc->meq, msg.o.data, 3, msg.i.io.mode);
				break;

			case mtWrite:
				if (ttypc_ps2_write(ttypc, ((unsigned char *)msg.i.data)[0]) < 0) {
					msg.o.err = -EINVAL;
					break;
				}
				msg.o.err = 1;
				break;

			case mtGetAttr:
				if ((msg.i.attr.type != atPollStatus)) {
					msg.o.err = -EINVAL;
					break;
				}

				msg.o.attr.val = 0;
				if (event_queue_count(ttypc->meq) >= 3)
					msg.o.attr.val |= POLLIN;

				msg.o.err = EOK;
				break;

			case mtClose:
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(ttypc->mport, &msg, rid);
	}
}


/* Mouse interrupt handler */
static int _ttypc_mouse_interrupt(unsigned int n, void *arg)
{
	ttypc_t *ttypc = (ttypc_t *)arg;

	return ttypc->kmcond;
}


int ttypc_mouse_init(ttypc_t *ttypc)
{
	int err;
	oid_t oid;

	if ((err = _ttypc_mouse_setup(ttypc)) < 0) {
		return err;
	}

	/* Create virtual mouse device */
	if ((err = portCreate(&ttypc->mport)) < 0) {
		fprintf(stderr, "pc-tty: failed to create mouse port\n");
		return err;
	}
	oid.port = ttypc->mport;
	oid.id = 0;
	if ((err = create_dev(&oid, _PATH_MOUSE)) < 0) {
		fprintf(stderr, "pc-tty: failed to register device %s\n", _PATH_MOUSE);
		return err;
	}

	if ((err = event_queue_init(&ttypc->meq, 256)) < 0)
		return err;

	/* Attach KIRQ12 (mouse event) interrupt handle */
	if ((err = interrupt((ttypc->mirq = 12), _ttypc_mouse_interrupt, ttypc, ttypc->kmcond, &ttypc->minth)) < 0)
		return err;

	/* Launch mouse pool thread */
	if ((err = beginthread(_ttypc_kbd_mouse_poolthr, 1, ttypc->mpstack, sizeof(ttypc->mpstack), ttypc)) < 0)
		return err;

	// TODO destroy on cleanup

	return EOK;
}


void ttypc_mouse_handle_event(ttypc_t *ttypc)
{
	unsigned char b = inb((void *)ttypc->kbd);

	event_queue_put(ttypc->meq, b, 3, 0);
}


void ttypc_mouse_destroy(ttypc_t *ttypc)
{
	event_queue_destroy(ttypc->meq);
	resourceDestroy(ttypc->minth);
	portDestroy(ttypc->mport);
}
