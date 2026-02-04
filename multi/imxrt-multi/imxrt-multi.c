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

#include "common.h"

#include "uart.h"
#include "rtt.h"
#include "gpio.h"
#include "spi.h"
#include "i2c.h"
#include "trng.h"
#include "cm4.h"

#if PSEUDODEV
#include <pseudodev.h>
#endif

#if PCT2075
#include "pct2075.h"
#endif

#define MULTI_THREADS_NO 2
#define UART_THREADS_NO  2

#define STACKSZ 1024


struct {
	uint32_t uart_port;
	char stack[MULTI_THREADS_NO + UART_THREADS_NO - 1][STACKSZ] __attribute__((aligned(8)));
} common;


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


static void multi_dispatchMsg(msg_t *msg, msg_rid_t rid)
{
	id_t id = msg->oid.id;

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
			gpio_handleMsg(msg, rid, id);
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

#if CM4
		case id_cm4_0:
		case id_cm4_1:
		case id_cm4_2:
		case id_cm4_3:
			cm4_handleMsg(msg);
			break;
#endif
#endif

		case id_i2c1:
		case id_i2c2:
		case id_i2c3:
		case id_i2c4:
#ifdef __CPU_IMXRT117X
		case id_i2c5:
		case id_i2c6:
#endif
			i2c_handleMsg(msg, id);
			break;

#if !defined(__CPU_IMXRT117X) && TRNG
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
				msg->o.err = -EPERM;
			}
			break;
#endif

#if PCT2075
		case id_temp1:
			pct2075_handleMsg(msg);
			break;
#endif

		default:
			msg->o.err = -ENODEV;
			break;
	}
}


static void uart_dispatchMsg(msg_t *msg)
{
	id_t id = msg->oid.id;

	switch (id) {
		case id_console:
#if !ISEMPTY(RTT_CONSOLE_USER)
			rtt_handleMsg(msg, RTT_CONSOLE_USER + id_rtt0);
#elif !ISEMPTY(UART_CONSOLE_USER)
			uart_handleMsg(msg, UART_CONSOLE_USER - 1 + id_uart1);
#else
			/* TODO: Add support for no console */
			msg->o.err = -ENODEV;
#endif
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

#ifdef __CPU_IMXRT117X
		case id_uart9:
		case id_uart10:
		case id_uart11:
		case id_uart12:
			uart_handleMsg(msg, id);
			break;
#endif

		case id_rtt0:
		case id_rtt1:
			rtt_handleMsg(msg, id);
			break;

		default:
			msg->o.err = -ENODEV;
			break;
	}
}


static int mkFile(oid_t *dir, id_t id, char *name, uint32_t port)
{
	msg_t msg;

	msg.type = mtCreate;
	msg.oid = *dir;
	msg.i.create.type = otDev;
	msg.i.create.mode = DEFFILEMODE;
	msg.i.create.dev.port = port;
	msg.i.create.dev.id = id;
	msg.i.data = name;
	msg.i.size = strlen(name) + 1;
	msg.o.data = NULL;
	msg.o.size = 0;

	if ((msgSend(dir->port, &msg) < 0) || (msg.o.err != EOK)) {
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

#ifdef __CPU_IMXRT117X

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

#endif

#if RTT0
	if (mkFile(&dir, id_rtt0, "rtt0", common.uart_port) < 0) {
		return -1;
	}
#endif

#if RTT1
	if (mkFile(&dir, id_rtt1, "rtt1", common.uart_port) < 0) {
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


/* I2Cs */
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

#if I2C5
	if (mkFile(&dir, id_i2c5, "i2c5", multi_port) < 0) {
		return -1;
	}
#endif

#if I2C6
	if (mkFile(&dir, id_i2c6, "i2c6", multi_port) < 0) {
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

#if PCT2075
	if (mkFile(&dir, id_temp1, "temp1", multi_port) < 0) {
		return -1;
	}
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
				multi_dispatchMsg(&msg, rid);
				if (msg.o.err == EWOULDBLOCK) {
					/* msgRespond in driver */
					continue;
				}
				break;

			case mtCreate:
			case mtTruncate:
			case mtDestroy:
			case mtLookup:
			case mtLink:
			case mtUnlink:
			case mtReaddir:
			default:
				msg.o.err = -ENOSYS;
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
				msg.o.err = EOK;
				break;

			case mtCreate:
			case mtTruncate:
			case mtDestroy:
			case mtLookup:
			case mtLink:
			case mtUnlink:
			case mtReaddir:
			default:
				msg.o.err = -ENOSYS;
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


static void multi_cleanup(void)
{
	oid_t oid;
	if (common.uart_port != 0) {
		portDestroy(common.uart_port);
	}
	if (multi_port != 0) {
		portDestroy(multi_port);
	}
	if (lookup(_PATH_CONSOLE, &oid, NULL) >= 0) {
		remove(_PATH_CONSOLE);
	}
}


int main(void)
{
	int i;
	oid_t oid;

	(void)priority(IMXRT_MULTI_PRIO);

	common.uart_port = 0;
	multi_port = 0;

	if (portCreate(&common.uart_port) < 0) {
		debug("imxrt-multi: Failed to create UART port\n");
		return EXIT_FAILURE;
	}

	if (portCreate(&multi_port) < 0) {
		debug("imxrt-multi: Failed to create multi port\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}

#if BUILTIN_DUMMYFS
	if (fs_init() != EOK) {
		debug("imxrt-multi: Failed to initialize filesystem\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}
#else
	/* Wait for the filesystem */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(10 * 1000);
	}
#endif

	if (gpio_init() != EOK) {
		debug("imxrt-multi: Failed to initialize GPIO\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}

	if (uart_init() != EOK) {
		debug("imxrt-multi: Failed to initialize UART\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}

#if defined(RTT_ENABLED) && RTT_ENABLED
	if (rtt_init() != EOK) {
		debug("imxrt-multi: Failed to initialize RTT\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}
#endif

	if (spi_init() != EOK) {
		debug("imxrt-multi: Failed to initialize SPI\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}

	if (i2c_init() != EOK) {
		debug("imxrt-multi: Failed to initialize I2C\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}

	oid.port = common.uart_port;
	oid.id = id_console;
	if (create_dev(&oid, _PATH_CONSOLE)) {
		debug("imxrt-multi: Failed to create device file\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}

#if !ISEMPTY(RTT_CONSOLE_USER)
	if (libklog_init(rtt_klogCblk) != EOK) {
		debug("imxrt-multi: Failed to initialize klog\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}
#else
	if (libklog_init(uart_klogCblk) != EOK) {
		debug("imxrt-multi: Failed to initialize klog\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}
#endif
	oid_t kmsgctrl = { .port = common.uart_port, .id = id_kmsgctrl };
	if (libklog_ctrlRegister(&kmsgctrl) != EOK) {
		// FIXME: no way to remove file created by libklog_ctrlRegister
		debug("imxrt-multi: Failed to register klog ctrl\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}

#if TRNG
	if (trng_init() != EOK) {
		debug("imxrt-multi: Failed to initialize TRNG\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}
#endif

#if CM4
	cm4_init();
#endif

#if BUILTIN_POSIXSRV
	if (posixsrv_start() != EOK) {
		debug("imxrt-multi: Failed to start posixsrv\n");
		multi_cleanup();
		return EXIT_FAILURE;
	}
#endif

#if PSEUDODEV
	pseudo_init();
#endif

	for (i = 0; i < UART_THREADS_NO; ++i) {
		if (beginthread(uart_thread, IMXRT_MULTI_PRIO, common.stack[i], STACKSZ, (void *)i) < 0) {
			debug("imxrt-multi: Failed to start UART thread\n");
			multi_cleanup();
			return EXIT_FAILURE;
		}
	}

	for (; i < (MULTI_THREADS_NO + UART_THREADS_NO - 1); ++i) {
		if (beginthread(multi_thread, IMXRT_MULTI_PRIO, common.stack[i], STACKSZ, (void *)i) < 0) {
			debug("imxrt-multi: Failed to start MULTI thread\n");
			multi_cleanup();
			return EXIT_FAILURE;
		}
	}

	if (createDevFiles() < 0) {
		printf("imxrt-multi: createSpecialFiles failed\n");
		return EXIT_FAILURE;
	}

	multi_thread((void *)i);

	/* never reached */
	return EXIT_FAILURE;
}
