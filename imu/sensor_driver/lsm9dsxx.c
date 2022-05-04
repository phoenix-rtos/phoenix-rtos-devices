/*
 * Phoenix-RTOS
 *
 * IMU I2C driver - lsmdsxx device driver
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
#include <unistd.h>

#include <i2c.h>

#include "communication.h"
#include "lsm9dsxx.h"

/* IMU CONTROL REGISTERS */

#define REG_CTRL_REG1_G             0x10
#define VAL_CTRL_REG1_G_ODR_G_952HZ 0xC0
#define VAL_CTRL_REG1_G_FS_G_245    0x00
#define VAL_CTRL_REG1_G_FS_G_500    0x08
#define VAL_CTRL_REG1_G_FS_G_2000   0x18

#define REG_CTRL_REG5_XL        0x1F
#define VAL_CTRL_REG5_XL_ENABLE 0x38

#define REG_CTRL_REG6_XL            0x20
#define VAL_CTRL_REG6_XL_ODR_XL_952 0xC0
#define MASK_CTRL_REG6_XL_FS        0x18
#define VAL_CTRL_REG6_XL_FS_2G      0x00 /* strange order, not a mistake! */
#define VAL_CTRL_REG6_XL_FS_4G      0x10 /* strange order, not a mistake! */
#define VAL_CTRL_REG6_XL_FS_8G      0x18 /* strange order, not a mistake! */
#define VAL_CTRL_REG6_XL_FS_16G     0x08 /* strange order, not a mistake! */

#define REG_CTRL_REG8 0x22

#define CTRL_REG4          0x1E
#define CTRL_REG4_G_ENABLE 0x38

#define REG_DATA_OUT_TEMP         0x15
#define REG_DATA_OUT_GYRO         0x18
#define REG_DATA_OUT_ACCL         0x28
#define DATA_OUT_TEMP_SIZE        2
#define DATA_OUT_GYR_SIZE         6
#define DATA_OUT_ACC_SIZE         6
#define LSM9DS1_IMU_DATA_ALL_SIZE 14


/* MAGNETOMETER REGISTERS */

/* magnetometer control registers */
#define REG_CTRL_REG1_M                  0x20
#define VAL_CTRL_REG1_M_XYHIGH_PERF_MODE 0x60
#define VAL_CTRL_REG1_M_ODR_80           0x1C
#define VAL_CTRL_REG1_M_FAST_ODR         0x02
#define VAL_CTRL_REG1_M_SELF_TEST        0x01

#define REG_CTRL_REG2_M       0x21
#define MASK_CTRL_REG2_M_FS   0x60
#define VAL_CTRL_REG2_M_FS_4  0x00
#define VAL_CTRL_REG2_M_FS_8  0x20
#define VAL_CTRL_REG2_M_FS_12 0x40
#define VAL_CTRL_REG2_M_FS_16 0x60

#define REG_CTRL_REG3_M               0x22
#define MASK_CTRL_REG2_M_MD           0x03
#define VAL_CTRL_REG3_M_MD_CONTINUOUS 0x00
#define VAL_CTRL_REG3_M_MD_SINGLE     0x01
#define VAL_CTRL_REG3_M_MD_PWR_DOWN   0x03

#define REG_CTRL_REG4_M                 0x23
#define VAL_CTRL_REG4_M_ZHIGH_PERF_MODE 0x08

/* magnetometer status registers */
#define REG_STATUS_REG_M       0x27
#define VAL_STATUS_REG_M_ZYXDA 0x08

/* magnetometer output registers */
#define REG_DATA_ALL              0x28
#define LSM9DS1_MAG_DATA_ALL_SIZE 6


static float translateMag(uint8_t hbyte, uint8_t lbyte)
{
	int16_t val = 0;
	val |= hbyte;
	val <<= 8;
	val |= lbyte;
	return 0.14F * (float)val; /* FIXME - add scale */
}


static float translateGyr(uint8_t hbyte, uint8_t lbyte)
{
	int16_t val = 0;
	val |= hbyte;
	val <<= 8;
	val |= lbyte;
	return 0.070F * DEG2RAD * (float)val; /* FIXME - add scale */
}

static float translateAcc(uint8_t hbyte, uint8_t lbyte)
{
	int16_t val = 0;
	val |= hbyte;
	val <<= 8;
	val |= lbyte;
	return 0.000244F * (float)val; /* FIXME - add scale */
}

static float translateTemp(uint8_t hbyte, uint8_t lbyte)
{
	int16_t val = 0;
	val |= hbyte;
	val <<= 8;
	val |= lbyte;
	return 25 + 0.0625 * (float)val; /* FIXME - add scale */
}

static int lsm9ds1_imu_init(const imu_dev_t *dev)
{
	int err;

	i2c_regWrite(dev->devAddr, REG_CTRL_REG8, 0x05);

	usleep(1000 * 10);

	err = (i2c_regWrite(dev->devAddr, REG_CTRL_REG1_G, (0 | VAL_CTRL_REG1_G_ODR_G_952HZ | VAL_CTRL_REG1_G_FS_G_2000)) < 0);
	err = err || (i2c_regWrite(dev->devAddr, REG_CTRL_REG6_XL, (0 | VAL_CTRL_REG6_XL_ODR_XL_952 | VAL_CTRL_REG6_XL_FS_8G)) < 0);
	usleep(1000 * 10);

	err = err || (i2c_regWrite(dev->devAddr, REG_CTRL_REG5_XL, (0 | VAL_CTRL_REG5_XL_ENABLE)) < 0);

	err = err || (i2c_regWrite(dev->devAddr, CTRL_REG4, (0 | CTRL_REG4_G_ENABLE)) < 0);
	usleep(1000 * 10);

	if (err) {
		printf("imu init failed\n");
		return -1;
	}

	return EOK;
}


/* initialize magnetometer device of lsm9ds1 sensor */
static int lsm9ds1_mag_init(const imu_dev_t *dev)
{
	int err;

	/* set CTRL_REG1_M */
	err = (i2c_regWrite(dev->devAddr, REG_CTRL_REG1_M, (0 | VAL_CTRL_REG1_M_XYHIGH_PERF_MODE | VAL_CTRL_REG1_M_ODR_80)) < 0);
	err = err || (i2c_regWrite(dev->devAddr, REG_CTRL_REG2_M, (0 | VAL_CTRL_REG2_M_FS_4)) < 0); /* this just sets all zeroes, but left for clarity */
	err = err || (i2c_regWrite(dev->devAddr, REG_CTRL_REG4_M, (0 | VAL_CTRL_REG4_M_ZHIGH_PERF_MODE)) < 0);
	usleep(10 * 1000);

	/* wakeup from power down */
	err = err || (i2c_regWrite(dev->devAddr, REG_CTRL_REG3_M, (0 | VAL_CTRL_REG3_M_MD_CONTINUOUS)) < 0);
	usleep(10 * 1000);

	if (err) {
		printf("mag init failed\n");
		return -1;
	}

	return EOK;
}


int lsm9ds1_imu_getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen)
{
	uint8_t databuf[LSM9DS1_IMU_DATA_ALL_SIZE];
	int buffpos = 0;

	if (buflen < (LSM9DS1_IMU_DATA_ALL_SIZE / sizeof(uint16_t))) {
		return -ENOMEM;
	}

	/* read accelerometer */

	if (i2c_regRead(dev->devAddr, REG_DATA_OUT_ACCL, databuf + buffpos, DATA_OUT_ACC_SIZE) != 0) {
		return -EIO;
	}

	buffpos += DATA_OUT_ACC_SIZE;
	/* read gyro */
	if (i2c_regRead(dev->devAddr, REG_DATA_OUT_GYRO, databuf + buffpos, DATA_OUT_GYR_SIZE) != 0) {
		return -EIO;
	}

	buffpos += DATA_OUT_GYR_SIZE;
	if (i2c_regRead(dev->devAddr, REG_DATA_OUT_TEMP, databuf + buffpos, DATA_OUT_TEMP_SIZE) != 0) {
		return -EIO;
	}

	/* minuses compensate for nonright-handness of imu coordinate system */
	buffer[0] = -translateAcc(databuf[1], databuf[0]);
	buffer[1] = translateAcc(databuf[3], databuf[2]);
	buffer[2] = translateAcc(databuf[5], databuf[4]);
	buffer[3] = -translateGyr(databuf[7], databuf[6]);
	buffer[4] = translateGyr(databuf[9], databuf[8]);
	buffer[5] = translateGyr(databuf[11], databuf[10]);
	buffer[6] = translateTemp(databuf[13], databuf[12]);

	return EOK;
}


int lsm9ds1_mag_getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen)
{
	uint8_t databuf[LSM9DS1_MAG_DATA_ALL_SIZE];
	int i;

	if (buflen < (LSM9DS1_MAG_DATA_ALL_SIZE / sizeof(uint16_t))) {
		return -ENOMEM;
	}

	/* read data from device */
	if (i2c_regRead(dev->devAddr, REG_DATA_ALL, databuf, LSM9DS1_MAG_DATA_ALL_SIZE) != 0) {
		return -EIO;
	}

	/* transform data into float representation */
	for (i = 0; i < (LSM9DS1_MAG_DATA_ALL_SIZE / sizeof(uint16_t)); i++) {
		buffer[i] = MGAUS2UTESLA * translateMag(databuf[i * 2 + 1], databuf[i * 2]);
	}
	//buffer[0] *= -1; /* lsm9ds1_mag X axis is flipped relative to lsm9ds1_imu accels/gyros */

	return EOK;
}


int lsm9dsxx_init(const imu_dev_t *dev)
{
	int ret = -1;
	switch (dev->model) {
		case lsm9ds1_imu:
			ret = lsm9ds1_imu_init(dev);
			break;
		case lsm9ds1_mag:
			ret = lsm9ds1_mag_init(dev);
			break;
	}
	return ret;
}


int lsm9dsxx_getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen)
{
	int ret = -1;
	switch (dev->model) {
		case lsm9ds1_imu:
			ret = lsm9ds1_imu_getAllData(dev, buffer, buflen);
			break;
		case lsm9ds1_mag:
			ret = lsm9ds1_mag_getAllData(dev, buffer, buflen);
			break;
	}
	return ret;
}
