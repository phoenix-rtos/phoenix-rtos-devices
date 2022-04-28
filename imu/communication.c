/*
 * Phoenix-RTOS
 *
 * IMU I2C driver - i2c communication
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <time.h>

#include <sys/msg.h>
#include <posix/utils.h>

#include <i2c.h>

#include "communication.h"
#include "icm20xxx.h"
#include "ak099xx.h"


const imu_dev_t devicelist[] = {
	{ .type = type_imu, .family = icm20xxx, .model = icm20948, .name = "ICM20948", .devAddr = 0x68, .whoamiAddr = 0x00, .whoamiVal = 0xEA },
	{ .type = type_imu, .family = icm20xxx, .model = icm20948, .name = "ICM20948_1", .devAddr = 0x69, .whoamiAddr = 0x00, .whoamiVal = 0xEA },
	{ .type = type_magmeter, .family = ak099xx, .model = ak09916, .name = "AK09916", .devAddr = 0x0C, .whoamiAddr = 0x01, .whoamiVal = 0x09 }
};


int initialize_i2cbus(int bus_no)
{
	return i2c_init(bus_no);
}


const imu_dev_t *probeDevices(enum dev_types devtype)
{
	int i;
	uint8_t whoami;

	for (i = 0; i < sizeof(devicelist) / sizeof(imu_dev_t); i++) {
		whoami = 0;
		if (devicelist[i].type != devtype) {
			continue;
		}
		i2c_regRead(devicelist[i].devAddr, devicelist[i].whoamiAddr, &whoami, 1);
		if (whoami == devicelist[i].whoamiVal) {
			return &devicelist[i];
		}
	}
	return NULL;
}


int initDevice(const imu_dev_t *dev)
{
	int ret;

	switch (dev->family) {
		case icm20xxx:
			ret = icm20xxx_init(dev);
			break;

		case ak099xx:
			ret = ak099xx_init(dev);
			break;

		default:
			ret = -1;
	}

	if (ret >= 0) {
		printf("Device: %s initialized!\n", dev->name);
		return 0;
	}
	printf("Cannot initialize: %s\n", dev->name);
	return -1;
}


int getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen)
{
	int ret = 0;

	switch (dev->family) {
		case icm20xxx:
			ret = icm20xxx_getAllData(dev, buffer, buflen);
			break;

		case ak099xx:
			ret = ak099xx_getAllData(dev, buffer, buflen);
			break;

		default:
			ret = -ENODEV;
	}

	return ret;
}