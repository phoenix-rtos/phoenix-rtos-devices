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

#include <termios.h>

typedef struct libtty_common_s libtty_common_t;
typedef struct libtty_callbacks_s libtty_callbacks_t;

struct libtty_callbacks_s {
	void* arg; /* argument to be passed to each of the callbacks */

	void (*set_baudrate)(void* arg, speed_t baudrate);
	void (*set_cflag)(void* arg, tcflag_t* cflag);
};

struct libtty_common_s {
	libtty_callbacks_t cb;
	struct termios term;
	struct winsize ws;
	pid_t pgrp;
};



int libtty_init(libtty_common_t* tty, libtty_callbacks_t* callbacks);
int libtty_ioctl(libtty_common_t* tty, unsigned int cmd, const void* in_arg, const void** out_arg);

static inline int libtty_baudrate_to_int(speed_t baudrate)
{
	switch (baudrate) {
	case B0: 	return 0;
	case B300:	return 300;
	case B600:	return 600;
	case B1200:	return 1200;
	case B1800:	return 1800;
	case B2400:	return 2400;
	case B4800:	return 4800;
	case B9600:	return 9600;
	case B19200:	return 19200;
	case B38400:	return 38400;
	case B57600:	return 57600;
	case B115200:	return 115200;
	case B230400:	return 230400;
	case B460800:	return 460800;
	}

	return -1;
}

static inline speed_t libtty_int_to_baudrate(int baudrate) {
	switch (baudrate) {
	case 0: 	return B0;
	case 300:	return B300;
	case 600:	return B600;
	case 1200:	return B1200;
	case 1800:	return B1800;
	case 2400:	return B2400;
	case 4800:	return B4800;
	case 9600:	return B9600;
	case 19200:	return B19200;
	case 38400:	return B38400;
	case 57600:	return B57600;
	case 115200:	return B115200;
	case 230400:	return B230400;
	case 460800:	return B460800;
	}

	return -1;
}
