/*
 * Phoenix-RTOS
 *
 * i.MX RT multi driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Aleksander Kaminski, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <libklog.h>
#include <paths.h>
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

#include <phoenix/ioctl.h>
#include <posix/utils.h>
#include <board_config.h>

#include "common.h"

#include "uart.h"
#include "gpio.h"
#include "spi.h"
#include "i2c.h"
#include "trng.h"
#include "cm4.h"

#if PSEUDODEV
#include <pseudodev.h>
#endif

#define MULTI_THREADS_NO 2
#define UART_THREADS_NO 2

#define STACKSZ 512


struct {
	uint32_t uart_port;
	char stack[MULTI_THREADS_NO + UART_THREADS_NO - 1][STACKSZ] __attribute__ ((aligned(8)));
} common;


static id_t multi_getID(msg_t *msg)
{
	id_t id = 0;

	switch (msg->type) {
		case mtOpen:
		case mtClose:
		case mtRead:
		case mtWrite:
			id = msg->i.io.oid.id;
			break;

		case mtGetAttr:
		case mtSetAttr:
			id = msg->i.attr.oid.id;
			break;

		default:
			break;
	}

	return id;
}


static void multi_handleError(msg_t *msg, int err)
{
	multi_o_t *omsg = (multi_o_t *)msg->o.raw;

	switch (msg->type) {
		case mtSetAttr:
		case mtGetAttr:
			msg->o.attr.err = err;
			break;

		case mtCreate:
			msg->o.create.err = err;
			break;

		case mtLookup:
			msg->o.lookup.err = err;
			break;

		case mtDevCtl:
			omsg->err = err;
			break;

		default:
			msg->o.io.err = err;
			break;
	}
}


#if PSEUDODEV
static inline int multi2pseudo(id_t id)
{
	switch (id) {
		default:
		case id_pseudoNull:
			return pseudo_idNull;
		case id_pseudoZero:
			return pseudo_idZero;
		case id_pseudoFull:
			return pseudo_idFull;
		case id_pseudoRandom:
			return pseudo_idRandom;
	}
}
#endif


static void multi_dispatchMsg(msg_t *msg)
{
	id_t id;
	multi_i_t *imsg;

	if (msg->type == mtDevCtl) {
		imsg = (multi_i_t *)msg->i.raw;
		id = imsg->id;
	}
	else {
		id = multi_getID(msg);
	}

	switch (id) {
		case id_gpio1:
		case id_gpio2:
		case id_gpio3:
		case id_gpio4:
		case id_gpio5:
		case id_gpio6:
		case id_gpio7:
		case id_gpio8:
		case id_gpio9:
#ifdef __CPU_IMXRT117X
		case id_gpio10:
		case id_gpio11:
		case id_gpio12:
		case id_gpio13:
#endif
			gpio_handleMsg(msg, id);
			break;

		case id_spi1:
		case id_spi2:
		case id_spi3:
		case id_spi4:
			spi_handleMsg(msg, id);
			break;

#ifdef __CPU_IMXRT117X
		case id_spi5:
		case id_spi6:
			spi_handleMsg(msg, id);
			break;

		case id_cm4_0:
		case id_cm4_1:
		case id_cm4_2:
		case id_cm4_3:
			cm4_handleMsg(msg);
			break;
#endif

#ifndef __CPU_IMXRT117X
		case id_i2c1:
		case id_i2c2:
		case id_i2c3:
		case id_i2c4:
			i2c_handleMsg(msg, id);
			break;
		case id_trng:
			trng_handleMsg(msg);
			break;
#endif

#if PSEUDODEV
		case id_pseudoNull:
		case id_pseudoZero:
		case id_pseudoFull:
		case id_pseudoRandom:
			if (pseudo_handleMsg(msg, multi2pseudo(id)) < 0) {
				multi_handleError(msg, -EPERM);
			}
			break;
#endif

		default:
			multi_handleError(msg, -ENODEV);
			break;
	}
}


static void uart_dispatchMsg(msg_t *msg)
{
	id_t id;
	ioctl_in_t *ioctl;

	if (msg->type == mtDevCtl) {
		ioctl = (ioctl_in_t *)msg->i.raw;
		id = ioctl->id;
	}
	else {
		id = multi_getID(msg);
	}

	switch (id) {
		case id_console:
			uart_handleMsg(msg, UART_CONSOLE - 1 + id_uart1);
			break;

		case id_uart1:
		case id_uart2:
		case id_uart3:
		case id_uart4:
		case id_uart5:
		case id_uart6:
		case id_uart7:
		case id_uart8:
		case id_uart9:
		case id_uart10:
		case id_uart11:
		case id_uart12:
			uart_handleMsg(msg, id);
			break;
	}
}


static int mkFile(oid_t *dir, id_t id, char *name, uint32_t port)
{
	msg_t msg;

	msg.type = mtCreate;
	msg.i.create.dir = *dir;
	msg.i.create.type = otDev;
	msg.i.create.mode = DEFFILEMODE;
	msg.i.create.dev.port = port;
	msg.i.create.dev.id = id;
	msg.i.data = name;
	msg.i.size = strlen(name) + 1;
	msg.o.data = NULL;
	msg.o.size = 0;

	if (msgSend(dir->port, &msg) < 0 || msg.o.create.err != EOK) {
		return -1;
	}

	return 0;
}


static int createDevFiles(void)
{
	int err, i;
	oid_t dir;
	char name[8];

	/* /dev */

	err = mkdir("/dev", 0);

	if (err < 0 && errno != EEXIST) {
		return -1;
	}

	if (lookup("/dev", NULL, &dir) < 0) {
		return -1;
	}

#if PSEUDODEV
	/* Pseudo devices */

	if (mkFile(&dir, id_pseudoNull, "null", multi_port) < 0) {
		return -1;
	}

	if (mkFile(&dir, id_pseudoZero, "zero", multi_port) < 0) {
		return -1;
	}

	if (mkFile(&dir, id_pseudoFull, "full", multi_port) < 0) {
		return -1;
	}

	if (mkFile(&dir, id_pseudoRandom, "urandom", multi_port) < 0) {
		return -1;
	}
#endif

	/* UARTs */

#if UART1
	if (mkFile(&dir, id_uart1, "uart1", common.uart_port) < 0) {
		return -1;
	}
#endif

#if UART2
	if (mkFile(&dir, id_uart2, "uart2", common.uart_port) < 0) {
		return -1;
	}
#endif

#if UART3
	if (mkFile(&dir, id_uart3, "uart3", common.uart_port) < 0) {
		return -1;
	}
#endif

#if UART4
	if (mkFile(&dir, id_uart4, "uart4", common.uart_port) < 0) {
		return -1;
	}
#endif

#if UART5
	if (mkFile(&dir, id_uart5, "uart5", common.uart_port) < 0) {
		return -1;
	}
#endif

#if UART6
	if (mkFile(&dir, id_uart6, "uart6", common.uart_port) < 0) {
		return -1;
	}
#endif

#if UART7
	if (mkFile(&dir, id_uart7, "uart7", common.uart_port) < 0) {
		return -1;
	}
#endif

#if UART8
	if (mkFile(&dir, id_uart8, "uart8", common.uart_port) < 0) {
		return -1;
	}
#endif

#if UART9
	if (mkFile(&dir, id_uart9, "uart9", common.uart_port) < 0) {
		return -1;
	}
#endif

#if UART10
	if (mkFile(&dir, id_uart10, "uart10", common.uart_port) < 0) {
		return -1;
	}
#endif

#if UART11
	if (mkFile(&dir, id_uart11, "uart11", common.uart_port) < 0) {
		return -1;
	}
#endif

#if UART12
	if (mkFile(&dir, id_uart12, "uart12", common.uart_port) < 0) {
		return -1;
	}
#endif

	/* GPIOs */
	for (i = 1; i <= GPIO_PORTS; ++i) {
		sprintf(name, "gpio%d", i);
		if (mkFile(&dir, id_gpio1 + i - 1, name, multi_port) < 0) {
			return -1;
		}
	}

	/* SPIs */

#if SPI1
	if (mkFile(&dir, id_spi1, "spi1", multi_port) < 0) {
		return -1;
	}
#endif

#if SPI2
	if (mkFile(&dir, id_spi2, "spi2", multi_port) < 0) {
		return -1;
	}
#endif

#if SPI3
	if (mkFile(&dir, id_spi3, "spi3", multi_port) < 0) {
		return -1;
	}
#endif

#if SPI4
	if (mkFile(&dir, id_spi4, "spi4", multi_port) < 0) {
		return -1;
	}
#endif

#ifdef __CPU_IMXRT117X

#if SPI5
	if (mkFile(&dir, id_spi5, "spi5", multi_port) < 0) {
		return -1;
	}
#endif

#if SPI6
	if (mkFile(&dir, id_spi6, "spi6", multi_port) < 0) {
		return -1;
	}
#endif

#if CM4
	strcpy(name, "cpuM40");
	for (i = 0; i < 4; ++i) {
		name[5] = '0' + i;
		if (mkFile(&dir, id_cm4_0 + i, name, multi_port) < 0) {
			return -1;
		}
	}
#endif

#endif


/* I2Cs */
#ifndef __CPU_IMXRT117X

#if I2C1
	if (mkFile(&dir, id_i2c1, "i2c1", multi_port) < 0) {
		return -1;
	}
#endif

#if I2C2
	if (mkFile(&dir, id_i2c2, "i2c2", multi_port) < 0) {
		return -1;
	}
#endif

#if I2C3
	if (mkFile(&dir, id_i2c3, "i2c3", multi_port) < 0) {
		return -1;
	}
#endif

#if I2C4
	if (mkFile(&dir, id_i2c4, "i2c4", multi_port) < 0) {
		return -1;
	}
#endif

#if TRNG
	if (mkFile(&dir, id_trng, "random", multi_port) < 0) {
		return -1;
	}

	/* TODO remove trng file - deprecated */
	if (mkFile(&dir, id_trng, "trng", multi_port) < 0) {
		return -1;
	}
#endif


#endif

	return 0;
}


static void multi_thread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;

	while (1) {
		while (msgRecv(multi_port, &msg, &rid) < 0) {
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
			case mtRead:
			case mtWrite:
			case mtGetAttr:
			case mtSetAttr:
			case mtDevCtl:
				multi_dispatchMsg(&msg);
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


static void uart_thread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;

	while (1) {
		while (msgRecv(common.uart_port, &msg, &rid) < 0) {
		}

		if (libklog_ctrlHandle(common.uart_port, &msg, rid) == 0) {
			/* msg has been handled by libklog */
			continue;
		}

		switch (msg.type) {
			case mtRead:
			case mtWrite:
			case mtGetAttr:
			case mtSetAttr:
			case mtDevCtl:
				uart_dispatchMsg(&msg);
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

		msgRespond(common.uart_port, &msg, rid);
	}
}


#if BUILTIN_DUMMYFS
extern int fs_init(void);
#endif


#if BUILTIN_POSIXSRV
extern int posixsrv_start(void);
#endif


int main(void)
{
	int i;
	oid_t oid;

	priority(IMXRT_MULTI_PRIO);

	portCreate(&common.uart_port);
	portCreate(&multi_port);

#if BUILTIN_DUMMYFS
	fs_init();
#else
	/* Wait for the filesystem */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(10 * 1000);
	}
#endif

	uart_init();
	gpio_init();
	spi_init();

	oid.port = common.uart_port;
	oid.id = id_console;
	create_dev(&oid, _PATH_CONSOLE);

	libklog_init(uart_klogCblk);
	oid_t kmsgctrl = { .port = common.uart_port, .id = id_kmsgctrl };
	libklog_ctrlRegister(&kmsgctrl);

#if TRNG
	trng_init();
#endif

#if CM4
	cm4_init();
#endif

#if BUILTIN_POSIXSRV
	posixsrv_start();
#endif

#if PSEUDODEV
	pseudo_init();
#endif

	for (i = 0; i < UART_THREADS_NO; ++i) {
		beginthread(uart_thread, IMXRT_MULTI_PRIO, common.stack[i], STACKSZ, (void *)i);
	}

	for (; i < (MULTI_THREADS_NO + UART_THREADS_NO - 1); ++i) {
		beginthread(multi_thread, IMXRT_MULTI_PRIO, common.stack[i], STACKSZ, (void *)i);
	}

	if (createDevFiles() < 0) {
		printf("imxrt-multi: createSpecialFiles failed\n");
		return EXIT_FAILURE;
	}

	multi_thread((void *)i);

	/* never reached */
	return EXIT_FAILURE;
}
