/*
 * Phoenix-RTOS
 *
 * MCX N94x Multi driver server
 *
 * Copyright 2024 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <getopt.h>
#include <sys/msg.h>
#include <sys/threads.h>

#include "dev.h"
#include "uart.h"

#define THREAD_POOL     4
#define THREAD_STACKSZ  800
#define THREAD_PRIORITY 3

#ifndef UART_CONSOLE
#define UART_CONSOLE 4
#endif

static struct {
	unsigned char stack[THREAD_POOL - 1][THREAD_STACKSZ];
} common;


static void threadPool(void *arg)
{
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		int err = dev_msgReceive(&msg, &rid);
		switch (err) {
			case 0:
				dev_handle(&msg, rid);
				/* msgRespond handled in dev_handle() */
				break;

			case -EINTR:
			case -ENOMEM:
			case -EAGAIN:
				/* Temporary errors */
				break;

			default:
				/* TODO report? end? restart? */
				break;
		}
	}
}


static void usage(const char *p)
{
	printf("usage: %s [-t ttyno]\n", p);
}


int main(int argc, char *argv[])
{
	int ttyno = UART_CONSOLE;
	bool ttyinit = false;

	for (;;) {
		int c = getopt(argc, argv, "t:");
		if (c == -1) {
			break;
		}

		switch (c) {
			case 't':
				ttyno = atoi(optarg);
				ttyinit = true;
				break;

			default:
				usage(argv[0]);
				return EXIT_FAILURE;
		}
	}

	if (!ttyinit) {
		printf("%s: No TTY selected, fallback to default (uart%d)\n", argv[0], ttyno);
	}

	int err = uart_ttyInit(ttyno);
	if (err < 0) {
		fprintf(stderr, "%s: TTY%d init failed (%d), fallback to default (uart%d)\n",
			argv[0], ttyno, err, UART_CONSOLE);

		err = uart_ttyInit(UART_CONSOLE);
		if (err < 0) {
			fprintf(stderr, "%s: Default TTY #%d init failed (%d)\n", argv[0], UART_CONSOLE, err);

			/* Let's keep going, not the end of the world */
		}
	}

	/* Everything is already initialized by constructors */
	printf("%s: Initialized\n", argv[0]);

	for (int i = 0; i < THREAD_POOL - 1; ++i) {
		if (beginthread(threadPool, THREAD_PRIORITY, common.stack[i], THREAD_STACKSZ, (void *)i) < 0) {
			/* Not really that critical of the error, in the worst case we still have the main thread */
			fprintf(stderr, "%s: Thread pool initialization failed\n", argv[0]);
			break;
		}
	}

	threadPool((void *)THREAD_POOL);

	/* Never reached */
	return 0;
}
