/*
 * Phoenix-RTOS
 *
 * NRF91 multi driver main
 *
 * Copyright 2023 Phoenix Systems
 * Author: Aleksander Kaminski, Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/threads.h>
#include <sys/msg.h>
#include <sys/pwman.h>

#include "common.h"

#include "nrf91-multi.h"
#include "fs.h"
#include "tty.h"

#define THREADS_NO 3
#define THREADS_PRIORITY 1
#define STACKSZ 640


struct {
	char stack[THREADS_NO - 1][STACKSZ] __attribute__ ((aligned(8)));

	unsigned int port;
} common;


static ssize_t console_write(const char *str, size_t len, int mode)
{
	tty_consoleLog(str);
	return (ssize_t)len;
}


static ssize_t console_read(char *str, size_t bufflen, int mode)
{
	return -ENOSYS;
}


static void thread(void *arg)
{
	msg_t msg;
	unsigned long int rid;

	while (1) {
		while (msgRecv(common.port, &msg, &rid) < 0) {
		}

		priority(msg.priority);

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = 0;
				break;

			case mtRead:
				msg.o.io.err = console_read(msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtWrite:
				msg.o.io.err = console_write(msg.i.data, msg.i.size, msg.i.io.mode);
				break;

			case mtDevCtl:
				/* TODO: add implementation */
				msg.o.create.err = -EINVAL;
				break;

			case mtCreate:
				msg.o.create.err = -EINVAL;
				break;

			case mtLookup:
				msg.o.lookup.err = -EINVAL;
				break;

			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(common.port, &msg, rid);

		priority(THREADS_PRIORITY);
	}
}


int main(void)
{
	int i;
	oid_t oid;
	static const char welcome[] = "multidrv: Started\n";
	unsigned int ttyConsolePort;

	priority(THREADS_PRIORITY);

	portCreate(&ttyConsolePort);
	portCreate(&common.port);

	fs_init();
	/* Wait for the filesystem */
	while (lookup("/", NULL, &oid) < 0)
		usleep(10000);

	tty_init(&ttyConsolePort);

	/* it doesn't work! it crashes here, probably because of no dummyfs */
	portRegister(common.port, "/multi", &oid);

	console_write(welcome, sizeof(welcome) - 1, 0);

	for (i = 0; i < THREADS_NO - 1; ++i)
		beginthread(thread, THREADS_PRIORITY, common.stack[i], STACKSZ, (void *)i);

	thread((void *)i);

	return 0;
}
