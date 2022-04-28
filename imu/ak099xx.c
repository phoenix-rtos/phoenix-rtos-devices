/*
 * Phoenix-RTOS
 *
 * IMU I2C driver - ak099xx device driver
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <i2c.h>

#include "communication.h"
#include "ak099xx.h"

/* magnetometer addresses */
#define REG_STATUS     0x10
#define REG_DATA_ALL   0x11
#define REG_CNTL2      0x31
#define VAL_MODE_10HZ  0x02
#define VAL_MODE_20HZ  0x04
#define VAL_MODE_50HZ  0x06
#define VAL_MODE_100HZ 0x08
#define REG_ST1        0x10
#define MASK_ST1_DRDY  0x01
#define MASK_ST1_DOR   0x02
#define REG_ST2        0x18
#define MASK_ST2_HOFL  0x02


int ak099xx_init(const imu_dev_t *dev)
{
	if (i2c_regWrite(dev->devAddr, REG_CNTL2, VAL_MODE_100HZ) < 0) {
		printf("Failed to set ak099xx mode\n");
		return -EIO;
	}
	return EOK;
}


static float translateMag(uint8_t hbyte, uint8_t lbyte)
{
	int16_t val = 0;
	val |= hbyte;
	val <<= 8;
	val |= lbyte;
	return 0.15F * (float)val;
}


static int ak099xx_readAllData(const imu_dev_t *dev, uint8_t *buf, uint32_t len)
{
	int ret = 0;
	uint8_t status = 0;

	ret += i2c_regRead(dev->devAddr, REG_ST1, &status, 1);
	if (!(status & MASK_ST1_DRDY)) {
		return -EAGAIN;
	}
	if (ret < 0) {
		return -EIO;
	}

	/* read data */
	ret += i2c_regRead(dev->devAddr, REG_DATA_ALL, buf, AK0990XX_DATA_ALL_SIZE);
	/* read hall overflow */
	ret += i2c_regRead(dev->devAddr, REG_ST2, &status, 1);
	/* TODO: implement buffer size big enough for hall sensor overflow (HOFL) data */

	return (ret < 0) ? -EIO : EOK;
}


int ak099xx_getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen)
{
	uint8_t databuf[AK0990XX_DATA_ALL_SIZE];

	if (buflen < (AK0990XX_DATA_ALL_SIZE / sizeof(uint16_t))) {
		return -ENOMEM;
	}

	if (ak099xx_readAllData(dev, databuf, AK0990XX_DATA_ALL_SIZE) < 0) {
		return -EIO;
	}

	buffer[0] = translateMag(databuf[1], databuf[0]);
	buffer[1] = translateMag(databuf[3], databuf[2]);
	buffer[2] = translateMag(databuf[5], databuf[4]);

	return EOK;
}