/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * stub implementation of /dev/tty
 * 
 * Copyright 2012-2015 Phoenix Systems
 * Author: Janusz Gralak, Pawel Kolodziej, Jacek Popko
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _PTY_H_
#define _PTY_H_

#include <proc/if.h>

#include <include/ioctl.h>
#include <include/termios.h>


#define SIZE_PTY 16
#define SIZE_PTY_BUFF 4096


typedef struct _pty_buf {
	semaphore_t mutex;
	thq_t waitq;

	char pbuff[SIZE_PTY_BUFF];

	unsigned int rbuffsz;
	unsigned int rb;
	unsigned int rp;

} pty_buf;


typedef struct _pty_virtual {
	pty_buf rw;
	pty_buf wr;
	u8 m_echo;
	struct winsize ws;
	int status;

} pty_con;


typedef struct _pty_t{
	page_t *page[SIZE_PTY] ;
	pty_con  *virtuals[SIZE_PTY];
	semaphore_t mutex;
} pty_t;


#define OPEN_MASTER 	0x0001
#define OPEN_SLAVE 		0x0002
#define CLOSE_MASTER 	0x0004
#define CLOSE_SLAVE 	0x0008

extern void _pty_init(void);

int pty_sput(pty_buf *cur,char *buff, unsigned int len, int minor);

int pty_sget(pty_buf *cur, char *buff, unsigned int len, int minor);

#endif
