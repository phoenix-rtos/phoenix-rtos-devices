/*
 * Phoenix-RTOS
 *
 * USB Mass Storage class driver
 *
 * Device driver server
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "umass.h"
#include <usbprocdriver.h>
#include <board_config.h>


#ifndef UMASS_N_UMSG_THREADS
#define UMASS_N_UMSG_THREADS 1
#endif

#ifndef UMASS_UMSG_PRIO
#define UMASS_UMSG_PRIO 3
#endif


static void printHelp(const char *name)
{
	fprintf(stderr,
			"Usage: %s [opts]\n"
			"  -r   Mount as rootfs\n"
			"  -h   Print this help\n",
			name);
}


int main(int argc, char *argv[])
{
	int ret;
	char c;
	oid_t oid;
	umass_args_t umass_args = { .mount_root = false };
	usb_driver_t *driver = usb_registeredDriverPop();

	if (driver == NULL) {
		fprintf(stderr, "umass: no driver registered!");
		return 1;
	}

	if (argc > 1) {
		/* Process command line options */
		for (;;) {
			c = getopt(argc, argv, "hr");
			if (c == -1) {
				break;
			}
			switch (c) {
				case 'r':
					umass_args.mount_root = true;
					break;
				case 'h':
				default:
					printHelp(argv[0]);
					return 0;
			}
		}
	}

	if (!umass_args.mount_root) {
		/* Wait for root filesystem if not responsible for mounting it */
		while (lookup("/", NULL, &oid) < 0) {
			usleep(10000);
		}
	}

	ret = usb_driverProcRun(driver, UMASS_UMSG_PRIO, UMASS_N_UMSG_THREADS, &umass_args);

	return ret == 0 ? 0 : 1;
}
