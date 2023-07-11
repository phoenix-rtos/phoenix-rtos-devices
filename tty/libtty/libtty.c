/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * TTY abstraction layer
 *
 * Copyright 2018 Phoenix Systems
 * Author: Marek Białowąs
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "libtty.h"
#include "libtty_disc.h"
#include "fifo.h"

#include <sys/ioctl.h>
#include <sys/threads.h>
#include <termios.h>
#include <poll.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

#define TTYDEFCHARS
#include "ttydefaults.h"
#undef TTYDEFCHARS

/* DEBUG { */
#include <stdio.h> /* printf */

#define COL_RED    "\033[1;31m"
#define COL_CYAN   "\033[1;36m"
#define COL_YELLOW "\033[1;33m"
#define COL_NORMAL "\033[0m"

#define LOG_TAG "libtty: "

/* clang-format off */
#define log_debug(fmt, ...)     do { if (0) printf(LOG_TAG fmt "\n", ##__VA_ARGS__); } while (0)
#define log_ioctl(fmt, ...)     do { if (0) printf(COL_CYAN LOG_TAG "IOCTL: " fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
#define log_info(fmt, ...)      do { if (0) printf(COL_CYAN LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
#define log_warn(fmt, ...)      do { if (1) printf(COL_YELLOW LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
#define log_error(fmt, ...)     do { if (1) printf(COL_RED  LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
/* clang-format on */

/* } DEBUG */

/* NOT supported: IXON|IXOFF|IXANY|PARMRK|INPCK|IGNPAR */
#define TTYSUP_IFLAG (IGNBRK | BRKINT | ISTRIP | INLCR | IGNCR | ICRNL | IMAXBEL)

#define TTYSUP_OFLAG (OPOST | ONLCR | TAB3 | OCRNL | ONOCR | ONLRET)
/* NOT supported: TOSTOP|FLUSHO|NOFLSH|ECHOPRT */
#define TTYSUP_LFLAG (ECHOKE | ECHOE | ECHOK | ECHO | ECHONL | ECHOCTL | ISIG | ICANON | IEXTEN)

#define CALLBACK(cb_name, ...) \
	do { \
		if (tty->cb.cb_name != NULL) \
			tty->cb.cb_name(tty->cb.arg, ##__VA_ARGS__); \
	} while (0)

#if 0
#define DEBUG_CHAR(c) \
	do { \
		*(tty->debug + 16) = (c); \
	} while (0)
#endif

#define TX_FIFO_NOTFULL_WATERMARK 16 /* amount of free space in fifo before we will wake up the writer */

static void termios_optimize(libtty_common_t *tty)
{
	/* check break characters list */
	tty->breakchars[0] = CNL;
	int n = 1;

	if (tty->term.c_cc[VEOF] != _POSIX_VDISABLE)
		tty->breakchars[n++] = tty->term.c_cc[VEOF];

	if (tty->term.c_cc[VEOL] != _POSIX_VDISABLE)
		tty->breakchars[n++] = tty->term.c_cc[VEOL];

	tty->breakchars[n] = '\0';

	/* check if we have break char in the RX FIFO */
	tty->t_flags &= ~TF_HAVEBREAK;
	if (CMP_FLAG(l, ICANON)) {
		if (libttydisc_rx_have_breakchar(tty))
			tty->t_flags |= TF_HAVEBREAK;
	}
}

static void termios_init(struct termios *term, speed_t speed)
{
	memset(term, 0, sizeof(*term));

	term->c_iflag = TTYDEF_IFLAG & TTYSUP_IFLAG;
	term->c_oflag = TTYDEF_OFLAG & TTYSUP_OFLAG;
	term->c_lflag = TTYDEF_LFLAG & TTYSUP_LFLAG;
	term->c_cflag = TTYDEF_CFLAG;

	term->c_ispeed = speed;
	term->c_ospeed = speed;

	memcpy(term->c_cc, ttydefchars, sizeof(ttydefchars));
}

static void termios_print_flags(const struct termios *termios_p)
{

	return;

	log_info("TERMIOS :");
	log_info("input flags: (0x%x)", termios_p->c_iflag);
	log_info("	INLCR  = %u // Translate NL to CR on input.", (termios_p->c_iflag & INLCR) != 0);
	log_info("	IGNCR  = %u // Ignore carriage return on input.", (termios_p->c_iflag & IGNCR) != 0);
	log_info("	ICRNL  = %u // Translate carriage return to newline on input (unless IGNCR is set).", (termios_p->c_iflag & ICRNL) != 0);
	log_info("output flags: (0x%x)", termios_p->c_oflag);
	log_info("	OPOST  = %u // POSTPROCESS output", (termios_p->c_oflag & OPOST) != 0);
	log_info("	ONLCR  = %u // (XSI) Map NL to CR-NL on output.", (termios_p->c_oflag & ONLCR) != 0);
	log_info("	OCRNL  = %u // Map CR to NL on output.", (termios_p->c_oflag & OCRNL) != 0);
	log_info("	ONOCR  = %u // Don't output CR at column 0.", (termios_p->c_oflag & ONOCR) != 0);
	log_info("	ONLRET = %u // Don't output CR.", (termios_p->c_oflag & ONLRET) != 0);
	log_info("local flags:  (0x%x)", termios_p->c_lflag);
	log_info("	ICANON = %u // Enable canonical mode", (termios_p->c_lflag & ICANON) != 0);
	log_info("	IEXTEN = %u // Enable implementation-defined input processing (EOL2, LNEXT, REPRINT, WERASE).", (termios_p->c_lflag & IEXTEN) != 0);
	log_info("	ECHO   = %u // Echo input characters.", (termios_p->c_lflag & ECHO) != 0);
	log_info("	ECHOE  = %u // If ICANON is also set, the ERASE character erases the preceding input character.", (termios_p->c_lflag & ECHOE) != 0);
	log_info("	ECHOK  = %u // If ICANON is also set, the KILL character erases the current line.", (termios_p->c_lflag & ECHOK) != 0);
	log_info("	ECHOCTL= %u // If ECHO is also set, terminal special characters other than TAB, NL, START, and STOP are echoed as ^X", (termios_p->c_lflag & ECHOCTL) != 0);
	log_info("cc flags:");
	log_info("	[VMIN]  = %u", termios_p->c_cc[VMIN]);
	log_info("	[VTIME] = %u", termios_p->c_cc[VTIME]);

#if 0
	int i;
	for (i=1; i<NCCS; ++i)
		if (termios_p->c_cc[i])
			log_info("	[%2u]    = %u", i, termios_p->c_cc[i]);
#endif
}


ssize_t libtty_read(libtty_common_t *tty, char *data, size_t size, unsigned mode)
{
	ssize_t ret = 0;

	if (tty->t_flags & TF_CLOSING)
		return -EBADF;

	if (CMP_FLAG(l, ICANON))
		ret = libttydisc_read_canonical(tty, data, size, mode, NULL);
	else
		ret = libttydisc_read_raw(tty, data, size, mode, NULL);

	return ret;
}

ssize_t libtty_read_nonblock(libtty_common_t *tty, char *data, size_t size, unsigned mode, libtty_read_state_t *st)
{
	ssize_t ret = 0;

	if (tty->t_flags & TF_CLOSING)
		return -EBADF;

	if (CMP_FLAG(l, ICANON))
		ret = libttydisc_read_canonical(tty, data, size, mode, st);
	else
		ret = libttydisc_read_raw(tty, data, size, mode, st);

	return ret;
}


/* writer wake up is done outside of libtty if wake_writer is not NULL */
unsigned char libtty_getchar(libtty_common_t *tty, int *wake_writer)
{
	unsigned char c = fifo_pop_back(tty->tx_fifo);

	if (wake_writer != NULL) {
		*wake_writer = fifo_freespace(tty->tx_fifo) >= TX_FIFO_NOTFULL_WATERMARK;
	}
	else {
		libtty_wake_writer(tty);
	}

	return c;
}


unsigned char libtty_popchar(libtty_common_t *tty)
{
	return fifo_pop_back(tty->tx_fifo);
}


void libtty_wake_writer(libtty_common_t *tty)
{
	if (fifo_freespace(tty->tx_fifo) >= TX_FIFO_NOTFULL_WATERMARK) {
		condSignal(tty->tx_waitq);
	}
}


int libtty_init(libtty_common_t *tty, libtty_callbacks_t *callbacks, unsigned int bufsize, speed_t speed)
{
	/* bufsize must be a power of 2 */
	if (bufsize == 0 || (bufsize & (bufsize - 1)) != 0) {
		return -1;
	}

	memset(tty, 0, sizeof(*tty));
	tty->cb = *callbacks;

	tty->tx_fifo = malloc(sizeof(fifo_t) + bufsize * sizeof(tty->tx_fifo->data[0]));
	tty->rx_fifo = malloc(sizeof(fifo_t) + bufsize * sizeof(tty->rx_fifo->data[0]));
	if (tty->tx_fifo == NULL || tty->rx_fifo == NULL) {
		free(tty->tx_fifo);
		free(tty->rx_fifo);
		return -1;
	}

	if (condCreate(&tty->tx_waitq) != EOK)
		return -1;

	if (condCreate(&tty->rx_waitq) != EOK)
		return -1;

	if (mutexCreate(&tty->tx_mutex) != EOK)
		return -1;

	if (mutexCreate(&tty->rx_mutex) != EOK)
		return -1;

	fifo_init(tty->tx_fifo, bufsize);
	fifo_init(tty->rx_fifo, bufsize);

	termios_init(&tty->term, speed);
	termios_optimize(tty);

	tty->ws.ws_row = 25;
	tty->ws.ws_col = 80;
	tty->pgrp = -1;

	return 0;
}


int libtty_close(libtty_common_t *tty)
{
	mutexLock2(tty->tx_mutex, tty->rx_mutex);
	tty->t_flags |= TF_CLOSING;

	condBroadcast(tty->tx_waitq);
	condBroadcast(tty->rx_waitq);

	mutexUnlock(tty->tx_mutex);
	mutexUnlock(tty->rx_mutex);

	return 0;
}


/* Note: only call after all readers/writers have finished */
int libtty_destroy(libtty_common_t *tty)
{
	resourceDestroy(tty->tx_waitq);
	resourceDestroy(tty->rx_waitq);
	resourceDestroy(tty->tx_mutex);
	resourceDestroy(tty->rx_mutex);

	free(tty->tx_fifo);
	free(tty->rx_fifo);

	return 0;
}


ssize_t libtty_write(libtty_common_t *tty, const char *data, size_t size, unsigned mode)
{
	ssize_t len = 0;

	/* short path */
	if (tty->t_flags & TF_CLOSING)
		return -EPIPE;
	else if (fifo_is_full(tty->tx_fifo) && (mode & O_NONBLOCK))
		return -EWOULDBLOCK;
	else if (size == 0)
		return 0;

	mutexLock(tty->tx_mutex);

	int fifo_freespace_for_single_char = CMP_FLAG(o, OPOST) ? LIBTTYDISC_WRITE_OPROC_MAXLEN : 1;

	/* write contents of the buffer */
	while (len < size) {
		while (fifo_freespace(tty->tx_fifo) < fifo_freespace_for_single_char) {
			if (tty->t_flags & TF_CLOSING)
				goto exit;

			if (mode & O_NONBLOCK)
				goto exit;

			CALLBACK(signal_txready);
			condWait(tty->tx_waitq, tty->tx_mutex, 0);
		}

		if (CMP_FLAG(o, OPOST) && (CTL_VALID(*data))) { /* we need to process this char */
			libttydisc_write_oproc(tty, *data);
		}
		else {
			fifo_push(tty->tx_fifo, *data);
		}

		len += 1;
		data += 1;
	}

	/* DEBUG_CHAR('W'); */
	CALLBACK(signal_txready);
#if 0
	/* TODO: test O_SYNC */
	while ((mode & O_SYNC) && !fifo_is_empty(tty->tx_fifo) && !(tty->t_flags & TF_CLOSING))
		condWait(tty->tx_waitq, tty->tx_mutex, 0);
#endif

exit:

	if (tty->t_flags & TF_CLOSING)
		len = -EPIPE;
	else if ((len == 0) && (mode & O_NONBLOCK))
		len = -EWOULDBLOCK;

	mutexUnlock(tty->tx_mutex);

	return len;
}

int libtty_txready(libtty_common_t *tty)
{
#if 0
	/* DEBUG_CHAR('0' + fifo_count(tty->tx_fifo)); */
	if (fifo_is_empty(tty->tx_fifo))
		DEBUG_CHAR('E');
	else
		DEBUG_CHAR('F');
#endif

	return !fifo_is_empty(tty->tx_fifo);
}

int libtty_txfull(libtty_common_t *tty)
{
	return fifo_is_full(tty->tx_fifo);
}

int libtty_rxready(libtty_common_t *tty)
{
	return !fifo_is_empty(tty->rx_fifo);
}

int libtty_poll_status(libtty_common_t *tty)
{
	int revents = 0;

	/* poll in ICANON mode should return POLLIN only if breakchar is present */
	if (!CMP_FLAG(l, ICANON)) {
		if (libtty_rxready(tty))
			revents |= POLLIN | POLLRDNORM;
	}
	else {
		if (tty->t_flags & TF_HAVEBREAK)
			revents |= POLLIN | POLLRDNORM;
	}

	if (!libtty_txfull(tty))
		revents |= POLLOUT | POLLWRNORM;
	if (tty->t_flags & TF_CLOSING)
		revents |= POLLHUP;

	return revents;
}

void libtty_signal_pgrp(libtty_common_t *tty, int signal)
{
	if (tty->pgrp > 0) {
		log_debug("signal(%u): %d", tty->pgrp, signal);
		kill(-tty->pgrp, signal);
	}
}

void libtty_drain(libtty_common_t *tty)
{
	mutexLock(tty->tx_mutex);
	while (!fifo_is_empty(tty->tx_fifo))
		condWait(tty->tx_waitq, tty->tx_mutex, 0);

	mutexUnlock(tty->tx_mutex);
}
void libtty_flush(libtty_common_t *tty, int type)
{
	if (type == TCIFLUSH || type == TCIOFLUSH) {
		mutexLock(tty->rx_mutex);
		fifo_remove_all(tty->rx_fifo);
		mutexUnlock(tty->rx_mutex);
	}

	if (type == TCOFLUSH || type == TCIOFLUSH) {
		/* leaving one char in TX fifo should allow us to avoid */
		/* undefined behaviour if writer is in the middle of operation */
		mutexLock(tty->tx_mutex);
		fifo_remove_all_but_one(tty->tx_fifo);
		mutexUnlock(tty->tx_mutex);
	}

	/* check for breakchars, etc. */
	termios_optimize(tty);
}

int libtty_ioctl(libtty_common_t *tty, pid_t sender_pid, unsigned int cmd, const void *in_arg, const void **out_arg)
{
	struct termios *termios_p = (struct termios *)in_arg;
	struct winsize *ws = (struct winsize *)in_arg;
	pid_t *pid = (pid_t *)in_arg;
	int ret = 0;

	*out_arg = NULL;

	/* TODO: locking */

	switch (cmd) {
		case TIOCGWINSZ:
			log_ioctl("TIOCGWINSZ");
			*out_arg = (const void *)&tty->ws;
			break;
		case TIOCSWINSZ:
			log_ioctl("TIOCSWINSZ(col=%u, row=%u)", ws->ws_col, ws->ws_row);
			tty->ws.ws_row = ws->ws_row;
			tty->ws.ws_col = ws->ws_col;
			libtty_signal_pgrp(tty, SIGWINCH);
			break;

		case TCDRAIN:
			log_ioctl("TCDRAIN");
			libtty_drain(tty);
			break;
		case TCFLSH:
			log_ioctl("TCFLSH");
			/* WARN: passing ioctl attr by value */
			libtty_flush(tty, (long)in_arg);
			break;
		case TCSETS:
		case TCSETSW:
		case TCSETSF: {
			log_ioctl("TCSETS (%s)", ((termios_p->c_lflag & ICANON) ? "cooked" : "raw"));
			/* TODO: SW SF */

			/* need local copy to be able to change values */
			struct termios temp_term = *termios_p;
			if (temp_term.c_ispeed == 0) /* required by POSIX */
				temp_term.c_ispeed = temp_term.c_ospeed;
			if (temp_term.c_ispeed != temp_term.c_ospeed) {
				log_warn("ispeed (%u) != ospeed (%u)", temp_term.c_ispeed, temp_term.c_ospeed);
				return -EINVAL;
			}

			if (temp_term.c_ospeed != tty->term.c_ospeed) {
				log_info("old baud: %u (B%u), new_baud: %u (B%u)",
					tty->term.c_ospeed, libtty_baudrate_to_int(tty->term.c_ospeed),
					temp_term.c_ospeed, libtty_baudrate_to_int(temp_term.c_ospeed));
				CALLBACK(set_baudrate, temp_term.c_ospeed);
			}

			if (temp_term.c_cflag != tty->term.c_cflag)
				CALLBACK(set_cflag, &temp_term.c_cflag);

			/* all succeded, we can apply params now (only supported ones) */
			temp_term.c_iflag &= TTYSUP_IFLAG;
			temp_term.c_oflag &= TTYSUP_OFLAG;
			temp_term.c_lflag &= TTYSUP_LFLAG;
			tty->term = temp_term;

			termios_optimize(tty);
			termios_print_flags(&tty->term);
			break;
		}
		case TCGETS:
			log_ioctl("TCGETS (%s)", ((tty->term.c_lflag & ICANON) ? "cooked" : "raw"));
			*out_arg = (const void *)&tty->term;
			break;
		case TIOCGPGRP:
			log_ioctl("TIOCGPGRP = %u", tty->pgrp);
			*out_arg = (const void *)&tty->pgrp;
			break;
		case TIOCSPGRP:
			log_ioctl("TIOCSPGRP(%u)", *pid);
			/* FIXME: check permissions */
			tty->pgrp = getpgid(*pid);
			break;
		case TIOCNOTTY:
			log_ioctl("TIOCNOTTY");
			tty->pgrp = -1; /* process detached from the console */
			break;
		case TIOCSCTTY:
			log_ioctl("TIOCSCTTY: pid=%X", sender_pid);
			/* FIXME: check permissions */
			tty->pgrp = getpgid(sender_pid);
			break;
		case TIOCGSID:
			/* NOTE: simulating sessions with process groups */
			log_ioctl("TIOCGSID = %u", tty->pgrp);
			*out_arg = (const void *)&tty->pgrp;
			break;
		default:
			log_warn("unsupported ioctl: 0x%x", cmd);
			ret = -EINVAL;
			break;
	}

	return ret;
}
