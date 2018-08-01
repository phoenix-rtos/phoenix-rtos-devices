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
#include "fifo.h"

#include <sys/ioctl.h>
#include <sys/threads.h>
#include <termios.h>
#include <poll.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

#include <stdio.h> // printf

#define COL_RED     "\033[1;31m"
#define COL_CYAN    "\033[1;36m"
#define COL_YELLOW  "\033[1;33m"
#define COL_NORMAL  "\033[0m"

#define LOG_TAG "libtty: "
#define log_debug(fmt, ...)     do { if (1) printf(LOG_TAG fmt "\n", ##__VA_ARGS__); } while (0)
#define log_ioctl(fmt, ...)     do { if (1) printf(COL_CYAN LOG_TAG "IOCTL: " fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
#define log_info(fmt, ...)      do { if (1) printf(COL_CYAN LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
#define log_warn(fmt, ...)      do { if (1) printf(COL_YELLOW LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
#define log_error(fmt, ...)     do { if (1) printf(COL_RED  LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)


#define CALLBACK(cb_name, ...) do {\
	if (tty->cb.cb_name != NULL)\
		/*log_warn("callback " #cb_name);*/\
		tty->cb.cb_name(tty->cb.arg, ##__VA_ARGS__);\
	} while (0)

/* termios comparison macro's. */
#define	CMP_CC(v,c) (tty->term.c_cc[v] != _POSIX_VDISABLE && \
			tty->term.c_cc[v] == (c))
#define	CMP_FLAG(field,opt) (tty->term.c_ ## field ## flag & (opt))

/* Characters that cannot be modified through c_cc. */
#define CTAB	'\t'
#define CNL	'\n'
#define CCR	'\r'

/* Character is a control character. */
#define CTL_VALID(c)	((c) == 0x7f || (unsigned char)(c) < 0x20)
/* Control character should be processed on echo. */
#define CTL_ECHO(c,q)	(!(q) && ((c) == CERASE2 || (c) == CTAB || \
    (c) == CNL || (c) == CCR))
/* Control character should be printed using ^X notation. */
#define CTL_PRINT(c,q)	((c) == 0x7f || ((unsigned char)(c) < 0x20 && \
    ((q) || ((c) != CTAB && (c) != CNL))))
/* Character is whitespace. */
#define CTL_WHITE(c)	((c) == ' ' || (c) == CTAB)
/* Character is alphanumeric. */
#define CTL_ALNUM(c)	(((c) >= '0' && (c) <= '9') || \
    ((c) >= 'a' && (c) <= 'z') || ((c) >= 'A' && (c) <= 'Z'))

#define DEBUG_CHAR(c) do {\
		*(tty->debug + 16) = (c);\
	} while (0)

static void termios_init(struct termios* term)
{
	memset(term, 0, sizeof(*term));

	term->c_iflag = INLCR   // \n -> \r (Translate NL to CR on input.)
			| ICRNL;  // \r -> \n (Translate carriage return to newline on input (unless IGNCR is set).)

	term->c_oflag = CR0 | NL0 | BS0 | TAB0 | VT0 | FF0
			| ONLCR	// \r -> \r\n (Map NL to CR-NL on output.)
			| ONOCR	// $)\r -> "" (Don't output CR at column 0.)
			;//| ONLRET;	// \r -> "" (Don't output CR.)

	term->c_cflag = CS8 | CREAD | CLOCAL;

	term->c_lflag = ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHONL | ECHOCTL;

	// Shell looks at those to learn how we generate signals
	term->c_cc[VEOF]  = 0x04; // Ctrl+D
	term->c_cc[VINTR] = 0x03; // Ctrl+C
	term->c_cc[VKILL] = 0x1c; // Ctrl+'\'
	term->c_cc[VSUSP] = 0x1a; // Ctrl+Z

	term->c_cc[VTIME] = 0;
	term->c_cc[VMIN]  = 1;

	term->c_ispeed = B115200;
	term->c_ospeed = B115200;
}

static ssize_t libtty_read_raw(libtty_common_t *tty, char *data, size_t size, unsigned mode)
{
	size_t vmin = tty->term.c_cc[VMIN];
	time_t vtime = (time_t)tty->term.c_cc[VTIME] * 100; // deciseconds to ms
	ssize_t len = 0;


	while (len < size) {
		if (fifo_is_empty(tty->rx_fifo)) {
			//DEBUG_CHAR('E');
			if (mode & O_NONBLOCK) {
				if (len == 0)
					return -EWOULDBLOCK;
				else
					break;
			} else if (vmin == 0 && vtime == 0) { // polling read
				break;
			} else { // read until at least vmin with optional initial/interchar timeout
				if (len < vmin) {
					while (fifo_is_empty(tty->rx_fifo)) {
						mutexLock(tty->mutex);
						int ret = condWait(tty->rx_waitq, tty->mutex, vtime);
						mutexUnlock(tty->mutex);
						if ((vtime > 0) && (ret == -ETIME))
							return len; // timer expired
					}
				}
				else
					break; // at least vmin chars present
			}
		}

		*data++ = fifo_pop_back(tty->rx_fifo);
		len += 1;
	}

	//DEBUG_CHAR('X');
	return len;

}

ssize_t libtty_read(libtty_common_t *tty, char *data, size_t size, unsigned mode)
{
	int ret = 0;

	//TODO: ICANON
#if 0
	if (CMP_FLAG(l, ICANON))
		ret = libtty_read_canonical(tty, data, size, mode);
	else
#endif
		ret = libtty_read_raw(tty, data, size, mode);

	return ret;
}

int libtty_putchar(libtty_common_t *tty, unsigned char c)
{
	//TODO: ICANON
	if (!fifo_is_full(tty->rx_fifo)) {
		fifo_push(tty->rx_fifo, c);
	} else {
		log_warn("RX OVERRUN!");
	}

	condSignal(tty->rx_waitq);
	return 0;
}

unsigned char libtty_getchar(libtty_common_t *tty)
{
	unsigned char ret = fifo_pop_back(tty->tx_fifo);
	if (!fifo_is_full(tty->rx_fifo)) {
		// TODO: watermark
		condSignal(tty->tx_waitq);
	}

	return ret;
}

int libtty_init(libtty_common_t* tty, libtty_callbacks_t* callbacks, unsigned int bufsize)
{
	memset(tty, 0, sizeof(*tty));
	tty->cb = *callbacks;

	tty->tx_fifo = malloc(sizeof(fifo_t) + bufsize * sizeof(tty->tx_fifo->data[0]));
	tty->rx_fifo = malloc(sizeof(fifo_t) + bufsize * sizeof(tty->tx_fifo->data[0]));
	if (tty->tx_fifo == NULL || tty->rx_fifo == NULL)
		return -1;

	if (condCreate(&tty->tx_waitq) != EOK)
		return -1;

	if (condCreate(&tty->rx_waitq) != EOK)
		return -1;

	if (mutexCreate(&tty->mutex) != EOK)
		return -1;

	fifo_init(tty->tx_fifo, bufsize);
	fifo_init(tty->rx_fifo, bufsize);

	termios_init(&tty->term);

	tty->ws.ws_row = 25;
	tty->ws.ws_col = 80;

	return 0;
}


ssize_t libtty_write(libtty_common_t *tty, const char *data, size_t size, unsigned mode)
{
	//TODO: ICANON
	//TODO: O_SYNC

	ssize_t len = 0;

	// short path
	if (fifo_is_full(tty->tx_fifo) && (mode & O_NONBLOCK))
		return -EWOULDBLOCK;

	mutexLock(tty->mutex);

	/* write contents of the buffer */
	while (len < size) {
		// TODO: maybe watermark fifo and signal automatically earlier?
		if (fifo_is_full(tty->tx_fifo)) {
			if (mode & O_NONBLOCK)
				break;

			// TODO: while
			CALLBACK(signal_txready);
			condWait(tty->tx_waitq, tty->mutex, 0);
		}

		fifo_push(tty->tx_fifo, *data++);
		len += 1;

#if 0
		if (tx_put(*((char *)data + i))) {
			if (!IS_COOL)
				break;
			else {
				*(uart.base + ucr1) |= 0x2000;
				condWait(uart.tx_cond, uart.lock, 0);
			}
		}
#endif
	}

	//DEBUG_CHAR('W');
	CALLBACK(signal_txready);
#if 0
	//TODO: test O_SYNC
	while ((mode & O_SYNC) && !fifo_is_empty(tty->tx_fifo))
		condWait(tty->tx_waitq, tty->mutex, 0);
#endif

	mutexUnlock(tty->mutex);

	if ((len == 0) && (mode & O_NONBLOCK))
		len = -EWOULDBLOCK;

	return len;
}

int libtty_txready(libtty_common_t* tty)
{
#if 0
	//DEBUG_CHAR('0' + fifo_count(tty->tx_fifo));
	if (fifo_is_empty(tty->tx_fifo))
		DEBUG_CHAR('E');
	else
		DEBUG_CHAR('F');
#endif

	return !fifo_is_empty(tty->tx_fifo);
}

int libtty_txfull(libtty_common_t* tty)
{
	return fifo_is_full(tty->tx_fifo);
}

int libtty_rxready(libtty_common_t* tty)
{
	return !fifo_is_empty(tty->rx_fifo);
}

int libtty_poll_status(libtty_common_t* tty)
{
	int revents = 0;

	if (libtty_rxready(tty))
		revents |= POLLIN|POLLRDNORM;
	if (!libtty_txfull(tty))
		revents |= POLLOUT|POLLWRNORM;

	return revents;
}

int libtty_ioctl(libtty_common_t* tty, unsigned int cmd, const void* in_arg, const void** out_arg)
{
	struct termios *termios_p = (struct termios *)in_arg;
	struct winsize *ws = (struct winsize*)in_arg;
	pid_t* pid = (pid_t*)in_arg;
	int ret = 0;

	*out_arg = NULL;

	//TODO
	//proc_semaphoreDown(&pty_common.mutex);

	switch (cmd) {
		case TIOCGWINSZ:
			log_ioctl("TIOCGWINSZ");
			*out_arg = (const void*) &tty->ws;
			break;
		case TIOCSWINSZ:
			log_ioctl("TIOCSWINSZ(col=%u, row=%u)", ws->ws_col, ws->ws_row);
			tty->ws.ws_row = ws->ws_row;
			tty->ws.ws_col = ws->ws_col;
			// libtty_signal_pgrp(tp, SIGWINCH);
			break;
		case TCSETS:
		case TCSETSW:
		case TCSETSF: {
			log_ioctl("TCSETS (%s)", ((termios_p->c_lflag & ICANON) ? "cooked" : "raw"));
			//TODO: SW SF

			/* need local copy to be able to change values */
			struct termios temp_term = *termios_p;
			//print_termios_flags(termios_p);
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

			/* all succeded, we can apply params now */
			tty->term.c_cflag = temp_term.c_cflag;
			// TODO: only supported params
			tty->term.c_iflag = temp_term.c_iflag;
			tty->term.c_oflag = temp_term.c_oflag;
			tty->term.c_lflag = temp_term.c_lflag;
			memcpy(&tty->term.c_cc, &temp_term.c_cc, sizeof(temp_term.c_cc));
			break;
		}
		case TCGETS:
			log_ioctl("TCGETS (%s)", ((tty->term.c_lflag & ICANON) ? "cooked" : "raw"));
			*out_arg = (const void*) &tty->term;
			break;
		case TIOCGPGRP:
			log_ioctl("TIOCGPGRP = %u", tty->pgrp);
			*out_arg = (const void*) &tty->pgrp;
			break;
		case TIOCSPGRP:
			log_ioctl("TIOCSPGRP(%u)", *pid);
			// FIXME: check permissions
			tty->pgrp = *pid;
			break;
		case TIOCNOTTY:
			log_ioctl("TIOCNOTTY");
			tty->pgrp = -1; /* process detached from the console */
			break;
#if 0
		case TIOCSCTTY: {
			// FIXME: check permissions
			thread_t *curr = proc_current();
			if (!curr || !curr->process)
				return -EINVAL;

			con->pgrp = proc_getpgid(curr->process);
			break;
		}
#endif
		default:
			//log_warn("unsupported ioctl: %u", cmd);
			log_warn("unsupported ioctl: 0x%x", cmd);
			ret = -EINVAL;
			break;
	}

	//proc_semaphoreUp(&pty_common.mutex);

	return ret;
}
