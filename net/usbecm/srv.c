/*
 * Phoenix-RTOS
 *
 * USB CDC ECM driver
 *
 * Device driver server
 *
 * Copyright 2026 Phoenix Systems
 * Author: Julian Uziembło
 *
 * %LICENSE%
 */

#include <stdio.h>

#include <usbprocdriver.h>


int main(int argc, char *argv[])
{
	usb_driver_t *driver = usb_registeredDriverPop();
	if (driver == NULL) {
		fprintf(stderr, "usbecm: no driver registered!\n");
		return 1;
	}

	usb_driverProcRun(driver, 3, 1, NULL);
}
