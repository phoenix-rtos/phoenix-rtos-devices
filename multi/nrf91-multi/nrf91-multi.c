/*
 * Phoenix-RTOS
 *
 * nRF91 multi driver server
 *
 * Copyright 2023, 2024 Phoenix Systems
 * Author: Aleksander Kaminski, Damian Loewnau
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/threads.h>
#include <sys/msg.h>
#include <sys/pwman.h>

#include <libklog.h>

#include "common.h"

#include "fs.h"
#include "uart.h"

#define THREADS_CNT     3
#define THREAD_PRIORITY 1
#define THREAD_STACKSZ  640


static struct {
	char stack[THREADS_CNT - 1][THREAD_STACKSZ] __attribute__((aligned(8)));

	unsigned int port;
} multi_common;


static void multi_thread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;

	while (1) {
		while (msgRecv(multi_common.port, &msg, &rid) < 0) {
		}

		priority(msg.priority);

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = 0;
				break;

			case mtRead:
				msg.o.io.err = uart_read(msg.o.data, msg.o.size, msg.i.io.mode);
				break;

			case mtWrite:
				msg.o.io.err = uart_log(msg.i.data, msg.i.size, msg.i.io.mode);
				break;

			case mtDevCtl:
				/* TODO: add implementation */
				msg.o.io.err = -EINVAL;
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

		msgRespond(multi_common.port, &msg, rid);

		priority(THREAD_PRIORITY);
	}
}


int main(void)
{
	int i;
	oid_t oid;

	priority(THREAD_PRIORITY);

	portCreate(&multi_common.port);

	fs_init();
	uart_init();
	libklog_init(uart_log);

	/* Do this after klog init to keep shell from overtaking klog */
	uart_createConsoleDev();

	portRegister(multi_common.port, "/multi", &oid);

	/* 2 threads launched in for loop and another one in the main thread */
	for (i = 0; i < THREADS_CNT - 1; ++i) {
		beginthread(&multi_thread, THREAD_PRIORITY, multi_common.stack[i], THREAD_STACKSZ, (void *)i);
	}

	printf("multidrv: Started\n");

	multi_thread((void *)i);

	return 0;
}
