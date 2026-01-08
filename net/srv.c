/*
 * Phoenix-RTOS
 *
 * USB WLAN driver
 *
 * Device driver server
 *
 * Copyright 2025 Phoenix Systems
 * Author: Julian Uziembło
 *
 * %LICENSE%
 */

#include <stdio.h>

#include <usbprocdriver.h>


int main(int argc, char *argv[])
{
	int ret;
	usb_driver_t *driver = usb_registeredDriverPop();
	if (driver == NULL) {
		fprintf(stderr, "usbwlan: no driver registered!\n");
		return 1;
	}

	ret = usb_driverProcRun(driver, NULL);
	if (ret < 0) {
		fprintf(stderr, "usbwlan: failed to start server: %d\n", ret);
		return 1;
	}

	return 0;
}
