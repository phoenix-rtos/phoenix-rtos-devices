/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * TTY abstraction layer - line discipline
 * THIS IS AN INTERNAL HEADER
 *
 * Copyright 2018 Phoenix Systems
 * Author: Marek Białowąs
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _LIBTTY_DISC_H_
#define _LIBTTY_DISC_H_

#include "libtty.h"
#include "fifo.h"

#include <stdint.h>
#include <termios.h>

/* termios comparison macro's. */
#define CMP_CC(v, c) (tty->term.c_cc[v] != _POSIX_VDISABLE && \
	tty->term.c_cc[v] == (c))
#define CMP_FLAG(field, opt) (tty->term.c_##field##flag & (opt))

/* Character is a control character. */
#define CTL_VALID(c) ((c) == 0x7f || (unsigned char)(c) < 0x20)


/* maximum amount of chars outputed by libttydisc_write_oproc */
#define LIBTTYDISC_WRITE_OPROC_MAXLEN 8

/* internal interface - line discipline */
int libttydisc_write_oproc(libtty_common_t *tty, char c);

ssize_t libttydisc_read_canonical(libtty_common_t *tty, char *data, size_t size, unsigned mode, libtty_read_state_t *st);
ssize_t libttydisc_read_raw(libtty_common_t *tty, char *data, size_t size, unsigned mode, libtty_read_state_t *st);


static inline int libttydisc_is_breakchar(libtty_common_t *tty, char c)
{
	for (char *breakc = tty->breakchars; *breakc; ++breakc)
		if (c == *breakc)
			return 1;

	return 0;
}

static inline int libttydisc_rx_have_breakchar(libtty_common_t *tty)
{
	char *breakc = tty->breakchars;
	while (*breakc) {
		if (fifo_has_char(tty->rx_fifo, *breakc))
			return 1;

		++breakc;
	}

	return 0;
}


#endif /* _LIBTTY_DISC_H_ */
