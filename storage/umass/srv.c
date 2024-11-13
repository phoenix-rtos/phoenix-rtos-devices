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


int main(int argc, char *argv[])
{
	int ret;
	char c;
	umass_args_t umass_args = { .mount_root = false };
	usb_driver_t *driver = usb_registeredDriverTake();

	if (driver == NULL) {
		fprintf(stderr, "umass: no driver registered!");
		return 1;
	}

	/* Wait for console */
	while (write(1, "", 0) < 0) {
		usleep(50000);
	}

	if (argc > 1) {
		/* Process command line options */
		for (;;) {
			c = getopt(argc, argv, "r");
			if (c == -1) {
				break;
			}
			switch (c) {
				case 'r':
					umass_args.mount_root = true;
					break;
				default:
					return 0;
			}
		}
	}

	ret = usb_driverProcRun(driver, true, &umass_args);

	return ret == 0 ? 0 : 1;
}
