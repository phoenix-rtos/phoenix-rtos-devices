/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Lukasz Leczkowski, Andrzej Tlomak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <board_config.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <sys/debug.h>
#include <sys/minmax.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <posix/utils.h>
#include <phoenix/gaisler/ambapp.h>

#ifdef __CPU_GR765
#include <phoenix/arch/riscv64/riscv64.h>
#else
#include <phoenix/arch/sparcv8leon/sparcv8leon.h>
#endif

#include <libgrspw.h>

/* clang-format off */
#define TRACE(fmt, ...) do { if (0) { printf("%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } } while (0)
#define LOG(fmt, ...)       printf("spacewire: " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) fprintf(stderr, "spacewire: " fmt "\n", ##__VA_ARGS__)
/* clang-format on */

#define PAGE_ALIGN(addr) (((addr_t)(addr) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1))

#define SPW_THREADS_NO 4
#define SPW_STACKSZ    4096
#define SPW_PRIO       3


static struct {
	oid_t spwOid;
	char spwStack[SPW_THREADS_NO][SPW_STACKSZ];
} spw_common;


static void spw_dispatchMsg(msg_t *msg)
{
	id_t id = msg->oid.id;

	switch (id) {
		case id_spw0:
		case id_spw1:
		case id_spw2:
		case id_spw3:
		case id_spw4:
		case id_spw5:
			spw_handleMsg(msg, id);
			break;

		default:
			msg->o.err = -EINVAL;
			break;
	}
}


static void spw_thread(void *arg)
{
	(void)arg;

	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		while (msgRecv(spw_common.spwOid.port, &msg, &rid) < 0) {
		}

		switch (msg.type) {
			case mtDevCtl:
				spw_dispatchMsg(&msg);
				break;

			case mtRead:
			case mtWrite:
			case mtOpen:
			case mtClose:
				msg.o.err = EOK;
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(spw_common.spwOid.port, &msg, rid);
	}
}


int main(void)
{
	oid_t oid;

	if (portCreate(&spw_common.spwOid.port) < 0) {
		LOG_ERROR("Failed to create port\n");
		return EXIT_FAILURE;
	}

	/* Wait for rootfs */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(100000);
	}

	if (spw_init() < 0) {
		portDestroy(spw_common.spwOid.port);
		LOG_ERROR("Failed to initialize SpaceWire\n");
		return EXIT_FAILURE;
	}

	if (spw_createDevs(&spw_common.spwOid) < 0) {
		portDestroy(spw_common.spwOid.port);
		LOG_ERROR("Failed to create SpaceWire devices");
		return EXIT_FAILURE;
	}

	for (int i = 0; i < SPW_THREADS_NO - 1; i++) {
		beginthread(spw_thread, SPW_PRIO, spw_common.spwStack[i], SPW_STACKSZ, NULL);
	}

	LOG("initialized");
	priority(SPW_PRIO);
	spw_thread(NULL);

	return 0;
}
