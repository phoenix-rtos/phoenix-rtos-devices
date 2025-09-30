/*
 * Phoenix-RTOS
 *
 * Xilinx/AMD AXI I2C (axi_iic) BUS driver
 *
 * Copyright 2021, 2022, 2025 Phoenix Systems
 * Author: Marek Bialowas, Hubert Buczyński, Kamil Ber, Krzysztof Szostek
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/msg.h>
#include <posix/utils.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <i2c.h>
#include <i2c-msg.h>


int standard_proc(unsigned int dev_no)
{
	oid_t dev;
	// unsigned int dev_no;
	uint32_t port;
	char devname[sizeof("i2cX")];

	// if (argc != 2) {
	// 	print_usage(argv[0]);
	// 	return 1;
	// }

	// dev_no = atoi(argv[1]);
	if (i2c_init(dev_no) < 0) {
		printf("i2c: initialization failed\n");
		return 1;
	}

	if (portCreate(&port) != EOK) {
		printf("i2c: port allocation failed\n");
		return 2;
	}

	dev.port = port;
	dev.id = 0;

	snprintf(devname, sizeof(devname), "i2c%u", dev_no % 10);
	if (create_dev(&dev, devname) < 0) {
		printf("i2c: could not create device\n");
		return 3;
	}

	// printf("i2c: initialized\n");
	// thread((void *)(intptr_t)port);

	// printf("i2c: exiting\n");

	return 0;
}
