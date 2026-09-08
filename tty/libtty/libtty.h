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

#ifndef _LIBTTY_H_
#define _LIBTTY_H_

#include <stdbool.h>
#include <sys/threads.h>
#include <termios.h>
#include <unistd.h>

#include "ttydefaults.h"

typedef struct libtty_common_s libtty_common_t;
typedef struct libtty_callbacks_s libtty_callbacks_t;
typedef struct fifo_s fifo_t;
typedef struct libtty_read_state_s libtty_read_state_t;


/* NOTE: callbacks always called under the libtty_lock */
struct libtty_callbacks_s {
	void *arg; /* argument to be passed to each of the callbacks */

	/* HW configuration */
	void (*set_baudrate)(void *arg, int baudrate);
	void (*set_cflag)(void *arg, tcflag_t *cflag);

	/* Optional: */
	int (*get_halfduplex)(void *arg);
	void (*set_halfduplex)(void *arg, int enable);

	/* at least one character ready to be sent */
	void (*signal_txready)(void *arg);
};


struct libtty_common_s {
	libtty_callbacks_t cb;
	struct termios term;
	struct winsize ws;

	/*
	 * OS-LIMITATION: the kernel records only that a session holds a controlling
	 * terminal, not which one, and reports no exits - so both are revalidated
	 * only on the next ctty ioctl, until then pgrp may name a group nobody is
	 * left in.
	 */
	pid_t sid;  /* session owning this terminal, or -1 if unclaimed */
	pid_t pgrp; /* foreground process group, or <= 0 if none */

	fifo_t *tx_fifo;
	fifo_t *rx_fifo;

	handle_t tx_waitq;
	handle_t rx_waitq;

	handle_t lock;
	bool lockCreated;

	/* cached optimizations */
	char breakchars[4]; /* enough to hold \n, VEOF and VEOL. */
	unsigned int t_flags;
};


struct libtty_read_state_s {
	int timeout_ms;
	int prevlen;
};


static inline void libtty_read_state_init(libtty_read_state_t *st)
{
	st->timeout_ms = -1;
	st->prevlen = 0;
}


/* t_flags */
#define TF_HAVEBREAK 0x00001 /* There is a breakchar present in RX fifo */
#define TF_LITERAL   0x00200 /* Accept the next character literally. */
#define TF_BYPASS    0x04000 /* Optimized input path. */
#define TF_CLOSING   0x08000 /* TTY is being closed */


/*
 * bufsize: TX/RX buffer size - has to be power of 2 !
 * if lock != NULL, it will be used as the tty->lock, else new mutex will be created
 */
int libtty_init(libtty_common_t *tty, libtty_callbacks_t *callbacks, unsigned int bufsize, int speed, handle_t *lock);
int libtty_destroy(libtty_common_t *tty);
int libtty_close(libtty_common_t *tty);


static inline void libtty_lock(libtty_common_t *tty)
{
	mutexLock(tty->lock);
}


static inline void libtty_unlock(libtty_common_t *tty)
{
	mutexUnlock(tty->lock);
}


/* external (message) interface */


ssize_t libtty_read(libtty_common_t *tty, char *data, size_t size, unsigned mode);
ssize_t libtty_write(libtty_common_t *tty, const char *data, size_t size, unsigned mode);
int libtty_poll_status(libtty_common_t *tty);
/*
 * out_arg is assumed to point to a buffer of at least IOCPARM_LEN(cmd) size (assuming
 * the ioctl is not IOC_NESTED - if it is, the real size should be provided, probably
 * obtained with ((ioctl_in_t *)&msg.i.raw)->size).
 * HINT: use ioctl_unpackEx from <sys/ioctl.h> to obtain the sender_pid, the cmd number,
 * the in_arg buffer and the out_arg buffer from the received msg.
 */
int libtty_ioctl(libtty_common_t *tty, pid_t sender_pid, unsigned long cmd, const void *in_arg, void *out_arg);


/* non-blocking interface:
 *  - first invocation has to be done with initialized libtty_read_state_t st param
 *  - if the function returns 0 and st->timeout >= 0, the user needs to call function again:
 *      1) when caller receives signal_read_state_changed callback
 *      2) if st->timeout > 0 - when timeout expired
 *  - if st->timeout > 0 for every next function invocation the st->timeout should be decreased
 *    by the caller by amout of miliseconds which passed since the last call (until st->timeout == 0, do not set negative values here)
 */
ssize_t libtty_read_nonblock(libtty_common_t *tty, char *data, size_t size, unsigned mode, libtty_read_state_t *st);


/* protected by libtty_lock */
ssize_t _libtty_read_nonblock(libtty_common_t *tty, char *data, size_t size, unsigned mode, libtty_read_state_t *st);
ssize_t _libtty_read(libtty_common_t *tty, char *data, size_t size, unsigned mode);
ssize_t _libtty_write(libtty_common_t *tty, const char *data, size_t size, unsigned mode);
int _libtty_poll_status(libtty_common_t *tty);
int _libtty_ioctl(libtty_common_t *tty, pid_t sender_pid, unsigned long cmd, const void *in_arg, void *out_arg);
int _libtty_close(libtty_common_t *tty);


/* internal (HW) interface */


/* protected by libtty_lock */
/* writer/reader wake up is done outside of libtty if wake_{writer|reader} is not NULL */
int _libtty_putchar(libtty_common_t *tty, unsigned char c, int *wake_reader);
unsigned char _libtty_popchar(libtty_common_t *tty);
unsigned char _libtty_getchar(libtty_common_t *tty, int *wake_writer);
void _libtty_wake_reader(libtty_common_t *tty);
void _libtty_wake_writer(libtty_common_t *tty);
int _libtty_txready(libtty_common_t *tty); /* at least 1 character ready to be sent */
int _libtty_txfull(libtty_common_t *tty);  /* no more place in the TX buffer */
int _libtty_rxready(libtty_common_t *tty); /* at least 1 character ready to be read out */
void _libtty_signal_pgrp(libtty_common_t *tty, int signal);
void _libtty_hangup(libtty_common_t *tty); /* terminal disconnect: signal + dissociate the session */


static inline void libtty_set_mode_raw(libtty_common_t *tty)
{
	tty->term.c_iflag &= ~(IGNBRK | BRKINT | INLCR | IGNCR | ICRNL | ISTRIP);
	tty->term.c_oflag &= ~OPOST;
	tty->term.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
}


#endif /* _LIBTTY_H_ */
