/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * TTY abstraction layer - line discipline
 *
 * Copyright 2018 Phoenix Systems
 * Author: Marek Białowąs
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

/* Some of the code is inspired by FreeBSD implementation, thus we're retaining FreeBSD copyright notice */
/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2008 Ed Schouten <ed@FreeBSD.org>
 * All rights reserved.
 *
 * Portions of this software were developed under sponsorship from Snow
 * B.V., the Netherlands.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "libtty.h"
#include "libtty_disc.h"
#include "fifo.h"

#include <sys/threads.h>
#include <termios.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>

#include "ttydefaults.h"

/* DEBUG { */
#include <stdio.h> /* printf */

#define COL_RED    "\033[1;31m"
#define COL_CYAN   "\033[1;36m"
#define COL_YELLOW "\033[1;33m"
#define COL_NORMAL "\033[0m"


#define LOG_TAG "libtty-disc: "

/* clang-format off */
#define log_debug(fmt, ...)     do { if (0) printf(LOG_TAG fmt "\n", ##__VA_ARGS__); } while (0)
#define log_info(fmt, ...)      do { if (0) printf(COL_CYAN LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
#define log_warn(fmt, ...)      do { if (0) printf(COL_YELLOW LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
#define log_error(fmt, ...)     do { if (0) printf(COL_RED  LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
/* clang-format on */

/* } DEBUG */

#define CALLBACK(cb_name, ...) \
	do { \
		if (tty->cb.cb_name != NULL) \
			tty->cb.cb_name(tty->cb.arg, ##__VA_ARGS__); \
	} while (0)

#define DEBUG_CHAR(c) \
	do { \
		*(tty->debug + 16) = (c); \
	} while (0)

/* Control character should be printed using ^X notation. */
#define CTL_PRINT(c) ((c) == 0x7f || ((unsigned char)(c) < 0x20 && ((c) != CTAB && (c) != CNL)))
/* Control character should be processed on echo. */
#define CTL_ECHO(c) ((c) == CTAB || (c) == CNL || (c) == CCR)
/* Character is whitespace. */
#define CTL_WHITE(c) ((c) == ' ' || (c) == CTAB)
/* Character is alphanumeric. */
#define CTL_ALNUM(c) (((c) >= '0' && (c) <= '9') || \
	((c) >= 'a' && (c) <= 'z') || ((c) >= 'A' && (c) <= 'Z'))

/* writing chars to TX buffer without any futher processing */
static int tx_write_ifspace(libtty_common_t *tty, const char *data, size_t len)
{
	/* WARN: no locking */
	const char *data_end = data + len;

	while ((data < data_end) && !fifo_is_full(tty->tx_fifo))
		fifo_push(tty->tx_fifo, (uint8_t)*data++);

	CALLBACK(signal_txready);
	return len - (data_end - data);
}

static int libttydisc_echo(libtty_common_t *tty, char c)
{
	/*
	 * Only echo characters when ECHO is turned on, or ECHONL when
	 * the character is an unquoted newline.
	 */
	if (!CMP_FLAG(l, ECHO) && (!CMP_FLAG(l, ECHONL) || c != CNL))
		return 0;

	if (CMP_FLAG(o, OPOST) && CTL_ECHO(c)) {
		/*
		 * Only perform postprocessing when OPOST is turned on
		 * and the character is an unquoted BS/TB/NL/CR.
		 */
		return libttydisc_write_oproc(tty, c);
	}
	else if (CMP_FLAG(l, ECHOCTL) && CTL_PRINT(c)) {
		/*
		 * Only use ^X notation when ECHOCTL is turned on and
		 * we've got an quoted control character.
		 *
		 * Print backspaces when echoing an end-of-file.
		 */
		char ob[4] = "^?\b\b";

		/* Print ^X notation. */
		if (c != 0x7f)
			ob[1] = c + 'A' - 1;

		if (CMP_CC(VEOF, c)) {
			return tx_write_ifspace(tty, ob, 4);
		}
		else {
			return tx_write_ifspace(tty, ob, 2);
		}
	}
	else {
		/* Can just be printed. */
		tx_write_ifspace(tty, &c, 1);
	}

	return 0;
}

/* remove one char from RX buffer */
static int libttydisc_rubchar(libtty_common_t *tty)
{
	char c;

	if (fifo_is_empty(tty->rx_fifo))
		return -1;

	/* begining of line */
	c = fifo_peek_front(tty->rx_fifo);
	if (c == CNL || CMP_CC(VEOL, c) || CMP_CC(VEOF, c))
		return -1;

	fifo_pop_front(tty->rx_fifo);

	if (CMP_FLAG(l, ECHO)) {
		if (CMP_FLAG(l, ECHOE)) {
			if (CTL_PRINT(c)) {
				/* Remove ^X formatted chars. */
				if (CMP_FLAG(l, ECHOCTL)) {
					tx_write_ifspace(tty, "\b\b  \b\b", 6);
				}
			}
			else if (c == ' ') {
				/* Space character needs no rubbing. */
				tx_write_ifspace(tty, "\b", 1);
			}
			else {
				/* remove a regular character by punching a space over it. */
				tx_write_ifspace(tty, "\b \b", 3);
			}
		}
		else {
			/* Don't print spaces. */
			libttydisc_echo(tty, tty->term.c_cc[VERASE]);
		}
	}

	return 0;
}


static int libtty_putchar_helper(libtty_common_t *tty, unsigned char c, int *wake_reader, int lock)
{
	if (wake_reader != NULL) {
		*wake_reader = 0;
	}

	/* ISTRIP: removing the top bit */
	if (CMP_FLAG(i, ISTRIP)) {
		c &= ~0x80;
	}

	/* ISIG: signal processing */
	if (CMP_FLAG(l, ISIG)) {
		int signal = 0;
		if (CMP_CC(VINTR, c)) {
			signal = SIGINT;
		}
		else if (CMP_CC(VQUIT, c)) {
			signal = SIGQUIT;
		}
		else if (CMP_CC(VSUSP, c)) {
			signal = SIGTSTP;
		}

		if (signal != 0) {
			/* echo the character before signalling the processes */
			libttydisc_echo(tty, c);
			libtty_signal_pgrp(tty, signal);
			return 0;
		}
	}

	/* Skip input processing when we want to print it literally. */
	if (tty->t_flags & TF_LITERAL) {
		tty->t_flags &= ~TF_LITERAL;
		goto processed;
	}

	/* Special control characters that are implementation dependent. */
	if (CMP_FLAG(l, IEXTEN)) {
		/* Accept the next character as literal. */
		if (CMP_CC(VLNEXT, c)) {
			if (CMP_FLAG(l, ECHO)) {
				if (CMP_FLAG(l, ECHOE)) {
					tx_write_ifspace(tty, "^\b", 2);
				}
				else {
					libttydisc_echo(tty, c);
				}
			}
			tty->t_flags |= TF_LITERAL;
			return 0;
		}
	}

	/* INCRNL/INNLCR/IGNCR : conversion of CR and NL */
	switch (c) {
		case CCR:
			if (CMP_FLAG(i, IGNCR)) {
				return (0);
			}
			if (CMP_FLAG(i, ICRNL)) {
				c = CNL;
			}
			break;
		case CNL:
			if (CMP_FLAG(i, INLCR)) {
				c = CCR;
			}
			break;
	}

	/* ICANON: Canonical line editing. */
	if (CMP_FLAG(l, ICANON)) {
		if (CMP_CC(VERASE, c) || CMP_CC(VERASE2, c)) {
			libttydisc_rubchar(tty);
			return 0;
		}
		else if (CMP_CC(VKILL, c)) {
			while (libttydisc_rubchar(tty) == 0)
				;
			return 0;
#if 0
		}
		else if (CMP_FLAG(l, IEXTEN)) {
			if (CMP_CC(VWERASE, c)) {
				ttydisc_rubword(tp);
				return (0);
			}
			else if (CMP_CC(VREPRINT, c)) {
				ttydisc_reprint(tp);
				return (0);
			}
#endif
		}
	}


processed:
	if (lock != 0) {
		mutexLock(tty->rx_mutex);
	}
	if (fifo_is_full(tty->rx_fifo)) {
		log_warn("RX OVERRUN!");
		fifo_pop_back(tty->rx_fifo);
	}
	fifo_push(tty->rx_fifo, c);

	libttydisc_echo(tty, c);

	if (CMP_FLAG(l, ICANON)) {
		/* signal only when the line ends */
		if (libttydisc_is_breakchar(tty, c)) {
			tty->t_flags |= TF_HAVEBREAK;

			if (wake_reader != NULL) {
				*wake_reader = 1;
			}
			if (lock != 0) {
				condSignal(tty->rx_waitq);
			}
		}
	}
	else {
		if (wake_reader != NULL) {
			*wake_reader = 1;
		}
		if (lock != 0) {
			condSignal(tty->rx_waitq);
		}
	}
	if (lock != 0) {
		mutexUnlock(tty->rx_mutex);
	}

	return 0;
}


int libtty_putchar(libtty_common_t *tty, unsigned char c, int *wake_reader)
{
	return libtty_putchar_helper(tty, c, wake_reader, 1);
}


void libtty_putchar_lock(libtty_common_t *tty)
{
	mutexLock(tty->rx_mutex);
}


void libtty_wake_reader(libtty_common_t *tty)
{
	condSignal(tty->rx_waitq);
}


void libtty_putchar_unlock(libtty_common_t *tty)
{
	mutexUnlock(tty->rx_mutex);
}


int libtty_putchar_unlocked(libtty_common_t *tty, unsigned char c, int *wake_reader)
{
	return libtty_putchar_helper(tty, c, wake_reader, 0);
}


int libttydisc_write_oproc(libtty_common_t *tty, char c)
{
	int ret = 0;

#if 0
	if (!CMP_FLAG(o, OPOST))
		log_error("%s: OPOST is disabled!", __func__);
	if (!CTL_VALID(c))
		log_error("%s: not a valid control char: 0x%02x", __func__, c);
#endif

#define PRINT_NORMAL() tx_write_ifspace(tty, &c, 1)
	switch (c) {
		case CEOF:
			return PRINT_NORMAL();

		case CTAB:
			/* Tab expansion. */
			if (CMP_FLAG(o, TAB3)) {
				ret = tx_write_ifspace(tty, "        ", 8);
			}
			else {
				ret = PRINT_NORMAL();
			}
			return ret;

		case CNL:
			/* Newline conversion. */
			if (CMP_FLAG(o, ONLCR)) {
				/* Convert \n to \r\n. */
				ret = tx_write_ifspace(tty, "\r\n", 2);
			}
			else {
				ret = PRINT_NORMAL();
			}
			return ret;

		case CCR:
			/* Carriage return to newline conversion. */
			if (CMP_FLAG(o, OCRNL))
				c = CNL;
#if 0
			/* Omit carriage returns on column 0. */
			if (CMP_FLAG(o, ONOCR) && tp->t_column == 0)
				return (0);
#endif
			return PRINT_NORMAL();
	}

	/*
	 * Invisible control character. Print it, but don't
	 * increase the column count.
	 */
	return PRINT_NORMAL();
#undef PRINT_NORMAL
}

ssize_t libttydisc_read_canonical(libtty_common_t *tty, char *data, size_t size, unsigned mode, libtty_read_state_t *st)
{
	char byte = 0xff;
	size_t len = 0;

	if (st)
		st->timeout_ms = -1; /* default (finished) */

	/* check if we have break char in RX fifo */
	mutexLock(tty->rx_mutex);
	do {
		if (tty->t_flags & TF_HAVEBREAK)
			break;

		if (tty->t_flags & TF_CLOSING) {
			mutexUnlock(tty->rx_mutex);
			return -EBADF;
		}

		if (mode & O_NONBLOCK) {
			mutexUnlock(tty->rx_mutex);
			return -EWOULDBLOCK;
		}

		if (st) {               /* nonblocking */
			st->timeout_ms = 0; /* wait indefinitely */
			mutexUnlock(tty->rx_mutex);
			return 0; /* read will resume execution at a later time */
		}
		else {
			/* blocking wait for any of the chars from breakchars to be available in tty->rx_fifo */
			condWait(tty->rx_waitq, tty->rx_mutex, 0);
		}
	} while (1);

	while (len < size) {
		byte = (char)fifo_pop_back(tty->rx_fifo);
		if (CMP_CC(VEOF, byte))
			break; /* EOF - dropping and exiting */

		*data++ = byte;
		len += 1;

		if (libttydisc_is_breakchar(tty, byte))
			break; /* EOL - exiting after the byte was added */
	}

	if (libttydisc_is_breakchar(tty, byte)) { /* loop ended due to breakchar */
		/* check if we have another break char in the RX FIFO */
		tty->t_flags &= ~TF_HAVEBREAK;
		if (CMP_FLAG(l, ICANON)) {
			if (libttydisc_rx_have_breakchar(tty))
				tty->t_flags |= TF_HAVEBREAK;
		}
	}

	mutexUnlock(tty->rx_mutex);
	return len;
}

ssize_t libttydisc_read_raw(libtty_common_t *tty, char *data, size_t size, unsigned mode, libtty_read_state_t *st)
{
	size_t vmin = tty->term.c_cc[VMIN];
	time_t vtime = (time_t)tty->term.c_cc[VTIME] * 100; /* deciseconds to ms */
	time_t first_char_timeout = (vmin == 0) ? vtime : 0;
	ssize_t len = 0;

	if (st && st->timeout_ms >= 0) { /* continuing previous read */
		int we_wanted_to_sleep_ms = (st->prevlen == 0) ? first_char_timeout : vtime;
		if (fifo_is_empty(tty->rx_fifo)) {
			if (we_wanted_to_sleep_ms == 0) /* blocking read without timeout */
				return 0;
			else if (st->timeout_ms > 0) { /* no new data, wait some more time */
				return 0;
			}
			else { /* st->timeout == 0, timer expired */
				st->timeout_ms = -1;
				return st->prevlen; /* first- or interbyte timeout - return what we have */
			}
		}

		st->timeout_ms = -1; /* default (finished) */
		len = st->prevlen;
		data += st->prevlen;
	}

	while (len < size) {
		if (fifo_is_empty(tty->rx_fifo)) {
			if (mode & O_NONBLOCK) {
				if (len == 0)
					return -EWOULDBLOCK;
				else
					break;
			}
			else if (vmin == 0 && vtime == 0) { /* polling read */
				break;
			}
			else { /* read until at least vmin with optional initial/interchar timeout */
				if ((len == 0) || (len < vmin)) {
					if (st) { /* non-blocking wait */
						st->prevlen = len;
						st->timeout_ms = (len == 0) ? first_char_timeout : vtime;
						return 0;
					}
					else { /* blocking wait */
						mutexLock(tty->rx_mutex);
						while (fifo_is_empty(tty->rx_fifo)) {
							if (tty->t_flags & TF_CLOSING) {
								mutexUnlock(tty->rx_mutex);
								return len;
							}

							int ret = condWait(tty->rx_waitq, tty->rx_mutex, ((len == 0) ? first_char_timeout : vtime) * 1000);
							if (ret == -ETIME) {
								mutexUnlock(tty->rx_mutex);
								return len; /* timer expired */
							}
						}
						mutexUnlock(tty->rx_mutex);
					}
				}
				else
					break; /* at least vmin chars present */
			}
		}

		*data++ = fifo_pop_back(tty->rx_fifo);
		len += 1;
	}

	return len;
}
