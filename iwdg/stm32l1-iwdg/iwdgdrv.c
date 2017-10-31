/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 Independent Watchdog driver
 *
 * Copyright 2017 Phoenix Systems
 * Author: Adrian Kepka
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "iwdgdrv.h"
#include "proc/proc.h"


struct {
	volatile unsigned int *iwdg;
	volatile unsigned int *rcc;

	unsigned int port;
} iwdg_common;


enum { iwdg_kr = 0, iwdg_pr, iwdg_rlr, iwdg_sr };


enum { rcc_cr = 0, rcc_icscr, rcc_cfgr, rcc_cir, rcc_ahbrstr, rcc_apb2rstr, rcc_apb1rstr,
	rcc_ahbenr, rcc_apb2enr, rcc_apb1enr, rcc_ahblpenr, rcc_apb2lpenr, rcc_apb1lpenr, rcc_csr };


typedef enum { WWDG_PING } iwdgtype_t ;


typedef struct {
	iwdgtype_t type;
	int idx;
} __attribute__((packed)) iwdgdevctl_t;


static void iwdg_reload(void)
{
	*(iwdg_common.iwdg + iwdg_kr) = 0xaaaa;
}

static void iwdg_thread(void *arg)
{
	msghdr_t hdr;
	iwdgdevctl_t msg;

	for(;;) {
		int size = proc_recv(iwdg_common.port, &msg, sizeof(msg), &hdr);
		if (hdr.type == MSG_NOTIFY)
			continue;

		switch (hdr.op) {
			case MSG_DEVCTL:
				if (size != sizeof(iwdgdevctl_t)) {
					proc_respond(iwdg_common.port, EINVAL, NULL, 0);
					break;
				}

				switch (msg.type) {
					case WWDG_PING:
						iwdg_reload();
						proc_respond(iwdg_common.port, EOK, NULL, 0);
						break;
				}
				break;

			default:
				proc_respond(iwdg_common.port, EINVAL, NULL, 0);
				break;
		}
	}
}


int iwdg_ping(void)
{
	iwdgdevctl_t devctl;
	devctl.type = WWDG_PING;

	return proc_send(iwdg_common.port, MSG_DEVCTL, &devctl, sizeof(devctl), MSG_NORMAL, NULL, 0);
}


void iwdg_init(void)
{
	iwdg_common.iwdg = (void *)0x40003000;
	iwdg_common.rcc = (void *)0x40023800;

	/* Enable write access to IWDG */
	*(iwdg_common.iwdg + iwdg_kr) = 0x5555;

	/* Set prescaler to 256 */
	*(iwdg_common.iwdg + iwdg_pr) = 0x06;

	/* Set counter reload value to maximum to obtain ~30s */
	*(iwdg_common.iwdg + iwdg_rlr) = 0xfff;

	iwdg_reload();

	/* Enable watchdog */
	*(iwdg_common.iwdg + iwdg_kr) = 0xcccc;

	proc_portCreate(&iwdg_common.port);
	proc_portRegister(iwdg_common.port, "/iwdgdrv");

	proc_threadCreate(NULL, iwdg_thread, 0, 512, NULL, NULL);
}
