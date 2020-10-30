/*
 * Phoenix-RTOS
 *
 * STM32L4 UART driver
 *
 * Copyright 2017, 2018, 2020 Phoenix Systems
 * Author: Jan Sikorski, Aleksander Kaminski, Andrzej Glowinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/pwman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#include "lib/libuart.h"

#include "stm32-multi.h"
#include "config.h"
#include "common.h"
#include "gpio.h"
#include "uart.h"
#include "rcc.h"

#define UART1_POS 0
#define UART2_POS (UART1_POS + UART1)
#define UART3_POS (UART2_POS + UART2)
#define UART4_POS (UART3_POS + UART3)
#define UART5_POS (UART4_POS + UART4)

#define UART_CNT (UART1 + UART2 + UART3 + UART4 + UART5)

#define THREAD_POOL 3
#define THREAD_STACKSZ 512
#define THREAD_PRIO 1


struct {
	unsigned char poolstack[THREAD_POOL][THREAD_STACKSZ] __attribute__((aligned(8)));
	libuart_ctx_t ctx[UART_CNT];

	unsigned int port;
} uart_common;


static const int uartConfig[] = { UART1, UART2, UART3, UART4, UART5 };


static const int uartPos[] = { UART1_POS, UART2_POS, UART3_POS, UART4_POS, UART5_POS };


static libuart_ctx_t *uart_getCtx(id_t id)
{
	libuart_ctx_t *ctx = NULL;

	if (!id)
		id = usart1 + UART_CONSOLE;

	id -= 1;

	if (id >= usart1 && id <= uart5)
		ctx = &uart_common.ctx[uartPos[id - usart1]];

	return ctx;
}


static void uart_thread(void *arg)
{
	msg_t msg;
	unsigned long rid;
	libuart_ctx_t *ctx;
	unsigned long request;
	const void *in_data, *out_data = NULL;
	pid_t pid;
	int err;
	id_t id;

	while (1) {
		while (msgRecv(uart_common.port, &msg, &rid) < 0)
			;

		priority(msg.priority);

		switch (msg.type) {
		case mtOpen:
		case mtClose:
			if ((ctx = uart_getCtx(msg.i.io.oid.id)) == NULL) {
				msg.o.io.err = -EINVAL;
				break;
			}

			msg.o.io.err = EOK;
			break;

		case mtWrite:
			if ((ctx = uart_getCtx(msg.i.io.oid.id)) == NULL) {
				msg.o.io.err = -EINVAL;
				break;
			}
			msg.o.io.err = libuart_write(ctx, msg.i.data, msg.i.size, msg.i.io.mode);
			break;

		case mtRead:
			if ((ctx = uart_getCtx(msg.i.io.oid.id)) == NULL) {
				msg.o.io.err = -EINVAL;
				break;
			}
			msg.o.io.err = libuart_read(ctx, msg.o.data, msg.o.size, msg.i.io.mode);
			break;

		case mtGetAttr:
			if ((ctx = uart_getCtx(msg.i.attr.oid.id)) == NULL) {
				msg.o.attr.val = -EINVAL;
				break;
			}

			msg.o.attr.val = libuart_getAttr(ctx, msg.i.attr.type);
			break;

		case mtDevCtl:
			in_data = ioctl_unpack(&msg, &request, &id);
			if ((ctx = uart_getCtx(id)) == NULL) {
				err = -EINVAL;
			}
			else {
				pid = ioctl_getSenderPid(&msg);
				err = libuart_devCtl(ctx, pid, request, in_data, &out_data);
			}
			ioctl_setResponse(&msg, request, err, out_data);
			break;
		}

		msgRespond(uart_common.port, &msg, rid);

		priority(THREAD_PRIO);
	}
}


void uart_log(const char *str)
{
	libuart_write(uart_getCtx(0), str, strlen(str), 0);
}


int uart_init(void)
{
	unsigned int uart, i;
	char fname[] = "/dev/uartx";
	oid_t oid;

	portCreate(&uart_common.port);
	oid.port = uart_common.port;

	for (uart = usart1; uart <= uart5; ++uart) {
		if (!uartConfig[uart])
			continue;

		libuart_init(&uart_common.ctx[uartPos[uart]], uart);

		fname[sizeof(fname) - 2] = '0' + uart - usart1;
		oid.id = uart - usart1 + 1;
		portRegister(uart_common.port, fname, &oid);
	}

	oid.id = 0;
	portRegister(uart_common.port, "/dev/tty", &oid);

	for (i = 0; i < THREAD_POOL; ++i)
		beginthread(uart_thread, THREAD_PRIO, uart_common.poolstack[i], sizeof(uart_common.poolstack[i]), (void *)i);

	return EOK;
}
