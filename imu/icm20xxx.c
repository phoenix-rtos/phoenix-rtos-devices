/*
 * Phoenix-RTOS
 *
 * IMU I2C driver - icm20xxx device driver
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
#include "icm20xxx.h"

/* USER BANK SELECTION */
#define REG_BANK   0x7F
#define VAL_BANK_0 0x00
#define VAL_BANK_1 0x10
#define VAL_BANK_2 0x20
#define VAL_BANK_3 0x30

/* POWER MANAGEMENT */
#define REG_PWR_MIGMT_1 0x06
#define VAL_PWR_RUN     0x01
#define VAL_PWR_RESET   0x80

/* USER CONTROL */
#define REG_USER_CTRL             0x03
#define VAL_USER_CTRL_DMP_EN      0x80
#define VAL_USER_CTRL_FIFO_EN     0x40
#define VAL_USER_CTRL_I2C_MST_EN  0x20
#define VAL_USER_CTRL_I2C_IF_DIS  0x10
#define VAL_USER_CTRL_DMP_RST     0x08
#define VAL_USER_CTRL_SRAM_RST    0x04
#define VAL_USER_CTRL_I2C_MST_RST 0x02

/* ACCEL CONFIG */
#define REG_ACCEL_CONFIG   0x14
#define VAL_ACCEL_DLPCFG_6 0x30
#define VAL_ACCEL_DLPCFG_4 0x20
#define VAL_ACCEL_DLPCFG_2 0x10
#define VAL_ACCEL_FS_2G    0x00
#define VAL_ACCEL_FS_4G    0x02
#define VAL_ACCEL_FS_8G    0x04
#define VAL_ACCEL_FS_16G   0x06
#define VAL_ACCEL_DLPF     0x01
/* LSB for ACCEL sample rate div */
#define REG_ACCEL_SMPLRT_DIV_2 0x11
#define VAL_ACCEL_SMPLRT_DIV_2 0x07

/* GYRO CONFIG */
#define REG_GYR_CONFIG_1    0x01
#define VAL_GYRO_DLPCFG_6   0x30
#define VAL_GYRO_DLPCFG_4   0x20
#define VAL_GYRO_DLPCFG_2   0x10
#define VAL_GYRO_FS_DPS250  0x00
#define VAL_GYRO_FS_DPS500  0x02
#define VAL_GYRO_FS_DPS1000 0x04
#define VAL_GYRO_FS_DPS2000 0x06
#define VAL_GYRO_DLPF       0x01
/* LSB for GYRO sample rate div */
#define REG_GYR_SMPLRT_DIV_2 0x00
#define VAL_GYR_SMPLRT_DIV_2 0x07

/* data addresses */
#define REG_DATA_ALL 0x2D /* actually ACEL_XOUT_H address */

/* power modes of icm20xxx device */
enum pwr_modes { mode_sleep,
	mode_full };
/* accelerometer ranges for ixm20xxx family device */
enum scales_acc { g2,
	g4,
	g8,
	g16 };
/* gyroscope ranges for ixm20xxx device */
enum scales_gyr { dps250,
	dps500,
	dps1000,
	dps2000 };

/* icm20xxx device config structure */
typedef struct {
	enum pwr_modes pwr_mode;
	enum scales_acc scale_acc;
	enum scales_gyr scale_gyr;
} immu_t;


static immu_t imu_common = {
	.pwr_mode = mode_sleep,
	.scale_acc = g2,
	.scale_gyr = dps500
};


static int icm20xxx_reset(const imu_dev_t *dev)
{
	int ret = 0;

	ret += i2c_regWrite(dev->devAddr, REG_BANK, VAL_BANK_0);           /* switch to user bank 0 */
	ret += i2c_regWrite(dev->devAddr, REG_PWR_MIGMT_1, VAL_PWR_RESET); /* reset device */
	usleep(1000 * 10);

	return (ret < 0) ? -EIO : EOK;
}


static int icm20xxx_run_mode(const imu_dev_t *dev)
{
	int ret = 0;

	/* switch to user bank 0 and skip if it failed */
	ret += i2c_regWrite(dev->devAddr, REG_BANK, VAL_BANK_0);
	if (ret >= 0) {
		ret += i2c_regWrite(dev->devAddr, REG_PWR_MIGMT_1, VAL_PWR_RUN); /* put it in run mode */
		usleep(1000 * 10);

		/* set bypass mode for auxiliary I2C bus */
		ret += i2c_regWrite(dev->devAddr, REG_USER_CTRL, 0);
		ret += i2c_regWrite(dev->devAddr, 0x0f, 0 | (1 << 1));
	}

	if (ret >= 0) {
		imu_common.pwr_mode = mode_full;
		return EOK;
	}
	return -EIO;
}


static float translateAccel(short val)
{
	float fval = val / 32767.F;
	switch (imu_common.scale_acc) {
		case g2:
			return fval * 2.F;
		case g4:
			return fval * 4.F;
		case g16:
			return fval * 16.F;
		default:
			return fval * 8.F;
	}
}


static float translateGyr(short val)
{
	float fval = DEG2RAD * val / 32767.F;
	switch (imu_common.scale_gyr) {
		case dps250:
			return fval * 250;
		case dps500:
			return fval * 500;
		case dps2000:
			return fval * 2000;
		default:
			return fval * 1000;
	}
}


static float translateTemp(short val)
{
	return 21.F + val / 333.87F;
}


int icm20xxx_init(const imu_dev_t *dev)
{
	int ret = 0;

	ret += icm20xxx_reset(dev);
	ret += icm20xxx_run_mode(dev);
	if (ret < 0) {
		printf("Cannot reset and put in run\n");
		return -EIO;
	}
	ret = 0;

	/* Accellerometer and Gyro configs are in USER_BANK_2 */
	ret += i2c_regWrite(dev->devAddr, REG_BANK, VAL_BANK_2);
	if (ret >= 0) {
		/* set accel sample rate divider */
		ret += i2c_regWrite(dev->devAddr, REG_ACCEL_SMPLRT_DIV_2, VAL_ACCEL_SMPLRT_DIV_2);
		/* set low pass filter, accel scale and enable low pass */
		ret += i2c_regWrite(dev->devAddr, REG_ACCEL_CONFIG, 0 | VAL_ACCEL_FS_2G | VAL_ACCEL_DLPCFG_6 | VAL_ACCEL_DLPF);

		/* set gyro sample rate divider */
		ret += i2c_regWrite(dev->devAddr, REG_GYR_SMPLRT_DIV_2, VAL_GYR_SMPLRT_DIV_2);
		/* set low pass filter, gyro scale and enable low pass */
		ret += i2c_regWrite(dev->devAddr, REG_GYR_CONFIG_1, 0 | VAL_GYRO_FS_DPS500 | VAL_GYRO_DLPCFG_6 | VAL_GYRO_DLPF);
	}
	ret += i2c_regWrite(dev->devAddr, REG_BANK, VAL_BANK_0); /* switch to user bank 0 as default */

	if (ret < 0) {
		printf("Cannot setup accel/gyro config\n");
		return -EIO;
	}

	return EOK;
}


int icm20xxx_getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen)
{
	uint8_t databuf[ICM20XXX_DATA_ALL_SIZE];

	if (buflen < (ICM20XXX_DATA_ALL_SIZE / sizeof(uint16_t))) {
		return -ENOMEM;
	}

	/* read data from device */
	if (i2c_regRead(dev->devAddr, REG_DATA_ALL, databuf, ICM20XXX_DATA_ALL_SIZE) != 0) {
		return -EIO;
	}

	/* change accell values to floats */
	buffer[0] = translateAccel((databuf[0] << 8) | (databuf[1] & 0xFF));  /* acc_x */
	buffer[1] = translateAccel((databuf[2] << 8) | (databuf[3] & 0xFF));  /* acc_y */
	buffer[2] = translateAccel((databuf[4] << 8) | (databuf[5] & 0xFF));  /* acc_z */
	buffer[3] = translateGyr((databuf[6] << 8) | (databuf[7] & 0xFF));    /* gyr_x */
	buffer[4] = translateGyr((databuf[8] << 8) | (databuf[9] & 0xFF));    /* gyr_y */
	buffer[5] = translateGyr((databuf[10] << 8) | (databuf[11] & 0xFF));  /* gyr_z */
	buffer[6] = translateTemp((databuf[12] << 8) | (databuf[13] & 0xFF)); /* temp */

	return EOK;
}
