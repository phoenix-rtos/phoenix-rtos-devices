/*
 * Phoenix-RTOS
 *
 * i.MX RT multi driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski
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
#include <unistd.h> /* For usleep */
#include <sys/stat.h>
#include <sys/threads.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/pwman.h>
#include <sys/debug.h>

#include "common.h"

#include "uart.h"
#include "gpio.h"

#define THREADS_NO 4
#define THREADS_PRIORITY 2
#define STACKSZ 512


struct {
	char stack[THREADS_NO - 1][STACKSZ] __attribute__ ((aligned(8)));
} common;


static void dispatchMsg(msg_t *msg)
{
	id_t id;

	switch (msg->type) {
		case mtRead:
		case mtWrite:
			id = msg->i.io.oid.id;
			break;

		case mtGetAttr:
		case mtSetAttr:
			id = msg->i.attr.oid.id;
			break;

		case mtDevCtl:
			uart_handleMsg(msg, UART_CONSOLE - UART1 + id_uart1);
			return;

		default:
			return;
	}

	switch (id) {
		case id_console:
			uart_handleMsg(msg, UART_CONSOLE - UART1 + id_uart1);
			break;

		case id_uart1:
		case id_uart2:
		case id_uart3:
		case id_uart4:
		case id_uart5:
		case id_uart6:
		case id_uart7:
		case id_uart8:
			uart_handleMsg(msg, id);
			break;

		case id_gpio1:
		case id_gpio2:
		case id_gpio3:
		case id_gpio4:
		case id_gpio5:
		case id_gpio6:
		case id_gpio7:
		case id_gpio8:
		case id_gpio9:
			gpio_handleMsg(msg, id);
			break;
	}
}


static int mkFile(oid_t *dir, id_t id, char *name)
{
	msg_t msg;

	msg.type = mtCreate;
	msg.i.create.dir = *dir;
	msg.i.create.type = otDev;
	msg.i.create.mode = 0;
	msg.i.create.dev.port = multi_port;
	msg.i.create.dev.id = id;
	msg.i.data = name;
	msg.i.size = strlen(name) + 1;
	msg.o.data = NULL;
	msg.o.size = 0;

	if (msgSend(dir->port, &msg) < 0 || msg.o.create.err != EOK)
		return - 1;

	return 0;
}


static int createDevFiles(void)
{
	int err, i;
	oid_t dir;
	char name[6];

	while (lookup("/", NULL, &dir) < 0)
		usleep(100000);

	/* /dev */

	err = mkdir("/dev", 0);

	if (err < 0 && errno != EEXIST)
		return -1;

	if (lookup("/dev", NULL, &dir) < 0)
		return -1;

	/* UARTs */

#if UART1
	if (mkFile(&dir, id_uart1, "uart1") < 0)
		return -1;
#endif

#if UART2
	if (mkFile(&dir, id_uart2, "uart2") < 0)
		return -1;
#endif

#if UART3
	if (mkFile(&dir, id_uart3, "uart3") < 0)
		return -1;
#endif

#if UART4
	if (mkFile(&dir, id_uart4, "uart4") < 0)
		return -1;
#endif

#if UART5
	if (mkFile(&dir, id_uart5, "uart5") < 0)
		return -1;
#endif

#if UART6
	if (mkFile(&dir, id_uart6, "uart6") < 0)
		return -1;
#endif

#if UART7
	if (mkFile(&dir, id_uart7, "uart7") < 0)
		return -1;
#endif

#if UART8
	if (mkFile(&dir, id_uart8, "uart8") < 0)
		return -1;
#endif

	/* GPIOs */

	strcpy(name, "gpio1");
	for (i = 1; i <= GPIO_PORTS; ++i, ++name[4]) {
		if (mkFile(&dir, id_gpio1 + i - 1, name) < 0)
			return -1;
	}

	return 0;
}


static void thread(void *arg)
{
	msg_t msg;
	unsigned int rid;

	while (1) {
		while (msgRecv(multi_port, &msg, &rid) < 0)
			;

		switch (msg.type) {
			case mtRead:
			case mtWrite:
			case mtGetAttr:
			case mtSetAttr:
			case mtDevCtl:
				dispatchMsg(&msg);
				break;

			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;

			case mtCreate:
				msg.o.create.err = -ENOSYS;
				break;

			case mtTruncate:
			case mtDestroy:
			case mtLookup:
			case mtLink:
			case mtUnlink:
			case mtReaddir:
			default:
				msg.o.io.err = -ENOSYS;
				break;
		}

		msgRespond(multi_port, &msg, rid);
	}
}


int main(void)
{
	int i;

	portCreate(&multi_port);

	uart_init();
	gpio_init();

	for (i = 0; i < THREADS_NO - 1; ++i)
		beginthread(thread, THREADS_PRIORITY, common.stack[i], STACKSZ, (void *)i);

	if (createDevFiles() < 0) {
		printf("imxrt-multi: createSpecialFiles failed\n");
		return -1;
	}

	thread((void *)i);

	return 0;
}
