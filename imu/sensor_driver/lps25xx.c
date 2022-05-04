/*
 * Phoenix-RTOS
 *
 * IMU I2C driver - lps25xx device driver
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
#include "lps25xx.h"

#define REG_RES_CONF 0x10
#define VAL_AVGT_8   0x00
#define VAL_AVGT_32  0x04
#define VAL_AVGT_128 0x08
#define VAL_AVGT_512 0x0C
#define VAL_AVGP_8   0x00
#define VAL_AVGP_32  0x01
#define VAL_AVGP_128 0x02
#define VAL_AVGP_512 0x03

#define REG_CTRL_REG1      0x20
#define VAL_PD_ACTIVE_MODE 0x80
#define VAL_ODR_ONESHOT    0x00
#define VAL_ODR_1HZ        0x10
#define VAL_ODR_7HZ        0x20
#define VAL_ODR_12HZ       0x30
#define VAL_ODR_25HZ       0x40

#define VAL_AUTOINCREMENT_BIT 0x80

#define LPS25XX_DATA_OUT_ALL  0x28
#define LPS25XX_DATA_ALL_SIZE 5


int lps25xx_init(const imu_dev_t *dev)
{
	int err;

	/* all subdevices have common init, thus initing them here */
	err = (i2c_regWrite(dev->devAddr, REG_RES_CONF, (0 | VAL_AVGT_8 | VAL_AVGP_8)) < 0);
	err = err || (i2c_regWrite(dev->devAddr, REG_CTRL_REG1, (0 | VAL_PD_ACTIVE_MODE | VAL_ODR_7HZ)) < 0);
	usleep(1000 * 10);

	if (err) {
		printf("baro init failed\n");
		return -1;
	}

	return EOK;
}


int lps25xx_getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen)
{
	uint32_t readp;
	int16_t readt;
	uint8_t databuf[LPS25XX_DATA_ALL_SIZE];

	if (buflen < (LPS25XX_DATA_BARO_ALL_SIZE + LPS25XX_DATA_TEMP_ALL_SIZE)) {
		return -ENOMEM;
	}

	/* read data from device */
	if (i2c_regRead(dev->devAddr, LPS25XX_DATA_OUT_ALL | VAL_AUTOINCREMENT_BIT, databuf, LPS25XX_DATA_ALL_SIZE) != 0) {
		return -EIO;
	}

	/* barimeter data trimming */
	readp = 0 | (uint32_t)databuf[0] | ((uint32_t)databuf[1] << 8) | ((uint32_t)databuf[2] << 16);
	readp = readp & 0x7FFFFF; /* dismiss highest bit as it is unusable */
	buffer[0] = (float)readp / 4096.F;

	/* temperature data trimming */
	readt = 0 | databuf[3] | ((uint16_t)databuf[4] << 8);
	buffer[1] = 42.5F + (float)readt / 480.F;

	return 0;
}
