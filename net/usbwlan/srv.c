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

#include <board_config.h>

#ifndef USBWLAN_UMSG_PRIO
#define USBWLAN_UMSG_PRIO 4
#endif

#ifndef USBWLAN_N_UMSG_THREADS
#define USBWLAN_N_UMSG_THREADS 1
#endif


int main(int argc, char *argv[])
{
	usb_driver_t *driver = usb_registeredDriverPop();
	if (driver == NULL) {
		fprintf(stderr, "usbwlan: no driver registered!\n");
		return 1;
	}

	usb_driverProcRun(driver, USBWLAN_UMSG_PRIO, USBWLAN_N_UMSG_THREADS, NULL);
}
