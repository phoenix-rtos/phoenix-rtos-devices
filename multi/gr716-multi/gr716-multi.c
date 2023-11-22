/*
 * Phoenix-RTOS
 *
 * GR716 multi driver main
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <board_config.h>
#include <errno.h>
#include <paths.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libklog.h>

#include <sys/debug.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <posix/utils.h>

#if PSEUDODEV
#include <pseudodev.h>
#endif

#include "adc.h"
#include "gpio.h"
#include "spi.h"
#include "uart.h"
#include "gr716-multi.h"


#define UART_THREADS_NO  2
#define MULTI_THREADS_NO 2
#define GR716_MULTI_PRIO 3

#define STACKSZ 2048

#define LOG(fmt, ...)       printf("gr716-multi: " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) fprintf(stderr, "gr716-multi: " fmt "\n", ##__VA_ARGS__)


static const char *multi_devs[] = {
#ifdef MULTI_GPIO
	"gpio0", "gpio1",
#endif

#ifdef MULTI_SPI
	"spi0", "spi1",
#endif
	"uart2", "uart3",

#ifdef MULTI_ADC
	"adc0",
#endif

#if PSEUDODEV
	"null", "zero", "full", "urandom",
#endif
	NULL
};

static struct {
	oid_t multiOid;
	oid_t uartOid;
	char uartStack[UART_THREADS_NO][STACKSZ] __attribute__((aligned(8)));
	char stack[MULTI_THREADS_NO - 1][STACKSZ] __attribute__((aligned(8)));
} multi_common;


static int multi_createDevs(void)
{
	int err;

	for (int i = 0; multi_devs[i] != NULL; i++) {
		err = create_dev(&multi_common.multiOid, multi_devs[i]);
		if (err < 0) {
			LOG_ERROR("Failed to create %s device file", multi_devs[i]);
			return err;
		}
	}

	return 0;
}


static id_t multi_getId(msg_t *msg)
{
	id_t id = 0;

	switch (msg->type) {
		case mtDevCtl:
			id = ((multi_i_t *)msg->i.raw)->id;
			break;
		case mtRead:
		case mtWrite:
			id = msg->i.io.oid.id;
			break;
		default:
			break;
	}

	return id;
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
	id_t id = multi_getId(msg);

	switch (id) {
#ifdef MULTI_GPIO
		case id_gpio0:
		case id_gpio1:
			gpio_handleMsg(msg, id);
			break;
#endif

#ifdef MULTI_SPI
		case id_spi0:
		case id_spi1:
			spi_handleMsg(msg, id);
			break;
#endif

#ifdef MULTI_ADC
		case id_adc0:
		case id_adc1:
		case id_adc2:
		case id_adc3:
		case id_adc4:
		case id_adc5:
		case id_adc6:
		case id_adc7:
			adc_handleMsg(msg, id);
			break;
#endif

#if PSEUDODEV
		case id_pseudoNull:
		case id_pseudoZero:
		case id_pseudoFull:
		case id_pseudoRandom:
			if (pseudo_handleMsg(msg, multi2pseudo(id)) < 0) {
				msg->o.io.err = -EPERM;
			}
			break;
#endif

		default:
			msg->o.io.err = -EINVAL;
			break;
	}
}


static void multi_thread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;

	(void)arg;

	while (1) {
		while (msgRecv(multi_common.multiOid.port, &msg, &rid) < 0) {
		}

		switch (msg.type) {
			case mtRead:
			case mtWrite:
			case mtDevCtl:
				multi_dispatchMsg(&msg);
				break;

			case mtOpen:
			case mtClose:
			case mtUnlink:
			case mtLink:
				msg.o.io.err = EOK;
				break;

			case mtCreate:
				msg.o.create.err = -ENOSYS;
				break;

			case mtSetAttr:
			case mtGetAttr:
				msg.o.attr.err = -ENOSYS;
				break;

			case mtTruncate:
			case mtDestroy:
			case mtLookup:
			case mtReaddir:
			default:
				msg.o.io.err = -ENOSYS;
				break;
		}

		msgRespond(multi_common.multiOid.port, &msg, rid);
	}
}


static void uart_dispatchMsg(msg_t *msg)
{
	id_t id = multi_getId(msg);

	switch (id) {
		case id_console:
			uart_handleMsg(msg, UART_CONSOLE_USER + id_uart0);
			break;

		case id_uart0:
		case id_uart1:
		case id_uart2:
		case id_uart3:
		case id_uart4:
		case id_uart5:
			uart_handleMsg(msg, id);
			break;

		default:
			msg->o.io.err = -EINVAL;
			break;
	}
}


static void uart_thread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;

	(void)arg;

	while (1) {
		while (msgRecv(multi_common.uartOid.port, &msg, &rid) < 0) {
		}

		if (libklog_ctrlHandle(multi_common.uartOid.port, &msg, rid) == 0) {
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

		msgRespond(multi_common.uartOid.port, &msg, rid);
	}
}


static void multi_cleanup(const char *msg)
{
	portDestroy(multi_common.multiOid.port);
	portDestroy(multi_common.uartOid.port);
	if (msg != NULL) {
		debug("gr716-multi: ");
		debug(msg);
	}
}


int main(void)
{
	oid_t oid;

	(void)priority(GR716_MULTI_PRIO);

	if (portCreate(&multi_common.uartOid.port) < 0) {
		debug("gr716-multi: Failed to create port\n");
		return EXIT_FAILURE;
	}

	if (portCreate(&multi_common.multiOid.port) < 0) {
		portDestroy(multi_common.uartOid.port);
		debug("gr716-multi: Failed to create port\n");
		return EXIT_FAILURE;
	}

	/* Wait for rootfs */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(100000);
	}

	if (uart_init() < 0) {
		multi_cleanup("Failed to initialize UART\n");
		return EXIT_FAILURE;
	}
#ifdef MULTI_SPI
	if (spi_init() < 0) {
		multi_cleanup("Failed to initialize SPI\n");
		return EXIT_FAILURE;
	}
#endif
#ifdef MULTI_GPIO
	if (gpio_init() < 0) {
		multi_cleanup("Failed to initialize GPIO\n");
		return EXIT_FAILURE;
	}
#endif
#ifdef MULTI_ADC
	if (adc_init() < 0) {
		multi_cleanup("Failed to initialize ADC\n");
		return EXIT_FAILURE;
	}
#endif

	oid.port = multi_common.uartOid.port;
	oid.id = id_console;

	if (create_dev(&oid, _PATH_CONSOLE) < 0) {
		multi_cleanup("Failed to create console file\n");
		return EXIT_FAILURE;
	}

	libklog_init(uart_klogClbk);
	oid_t kmsgctrl = { .port = oid.port, .id = id_kmsgctrl };
	libklog_ctrlRegister(&kmsgctrl);

#if PSEUDODEV
	pseudo_init();
#endif

	if (multi_createDevs() < 0) {
		multi_cleanup(NULL);
		return EXIT_FAILURE;
	}

	for (int i = 0; i < UART_THREADS_NO; i++) {
		beginthread(uart_thread, GR716_MULTI_PRIO, multi_common.uartStack[i], STACKSZ, NULL);
	}

	for (int i = 0; i < MULTI_THREADS_NO - 1; i++) {
		beginthread(multi_thread, GR716_MULTI_PRIO, multi_common.stack[i], STACKSZ, NULL);
	}

	LOG("initialized");

	multi_thread(NULL);

	return 0;
}
