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

#include <communication.h>
#include <icm20xxx.h>
#include <ak099xx.h>


const i2c_dev_t devicelist[] = {
	{ .type = type_imu, .family = icm20xxx, .model = icm20948, .name = "ICM20948", .addr = 0x68, .whoami_addr = 0x00, .whoami_val = 0xEA },
	{ .type = type_imu, .family = icm20xxx, .model = icm20948, .name = "ICM20948_1", .addr = 0x69, .whoami_addr = 0x00, .whoami_val = 0xEA },
	{ .type = type_magmeter, .family = ak099xx, .model = ak09916, .name = "AK09916", .addr = 0x0C, .whoami_addr = 0x01, .whoami_val = 0x09 }
};


int sendbyte(uint8_t dev_addr, uint8_t reg_addr, uint8_t val)
{
	return (i2c_regWrite(dev_addr, reg_addr, &val, 1) >= 0) ? 0 : -1;
}


int initialize_i2cbus(void)
{
	return i2c_init(2);
}


const i2c_dev_t *checkDevices(enum dev_types devtype)
{
	int i;
	uint8_t whoami = 0;

	for (i = 0; i < sizeof(devicelist) / sizeof(i2c_dev_t); i++) {
		if (devicelist[i].type != devtype) {
			continue;
		}
		i2c_regRead(devicelist[i].addr, devicelist[i].whoami_addr, &whoami, 1);
		if (whoami == devicelist[i].whoami_val) {
			return &devicelist[i];
		}
	}
	return NULL;
}


int initDevice(const i2c_dev_t *dev)
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


int getAllData(const i2c_dev_t *dev, float *buffer, uint8_t buflen)
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