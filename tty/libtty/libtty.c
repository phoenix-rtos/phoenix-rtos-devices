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

#include <sys/ioctl.h>
#include <termios.h>
#include <string.h>
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

#define MARKER_MAGIC 0x10203040

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

int libtty_init(libtty_common_t* tty, libtty_callbacks_t* callbacks)
{
	memset(tty, 0, sizeof(*tty));
	tty->cb = *callbacks;

	termios_init(&tty->term);

	tty->ws.ws_row = 24;
	tty->ws.ws_col = 80;
	return 0;
}

#define CALLBACK(cb_name, ...) do {\
	if (tty->cb.cb_name != NULL)\
		log_warn("callback " #cb_name);\
		tty->cb.cb_name(tty->cb.arg, __VA_ARGS__);\
	} while (0)

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
			log_ioctl("TCSETS");
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

			tty->term = *termios_p; // FIXME
			break;
		}
		case TCGETS:
			log_ioctl("TCGETS (ibaud=%u, obaud=%u)", tty->term.c_ispeed, tty->term.c_ospeed);
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
			log_ioctl("unsupported: %u", cmd);
			//log_warn("unsupported ioctl: %02x", cmd);
			ret = -EINVAL;
			break;
	}

	//proc_semaphoreUp(&pty_common.mutex);

	return ret;
}
