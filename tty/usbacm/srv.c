/*
 * Phoenix-RTOS
 *
 * USB CDC ACM driver
 *
 * Device driver server
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * %LICENSE%
 */


#include <usbdriver.h>
#include <usbprocdriver.h>


int main(int argc, char *argv[])
{
	int ret;
	usb_driver_t *driver = usb_registeredDriverTake();
	if (driver == NULL) {
		fprintf(stderr, "usbacm: no driver registered!");
		return 1;
	}

	ret = usb_driverProcRun(driver, false, NULL);
	if (ret < 0) {
		fprintf(stderr, "usbacm: failed to start server: %d\n", ret);
	}

	return ret == 0 ? 0 : 1;
}
