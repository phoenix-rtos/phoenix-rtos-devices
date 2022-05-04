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
#ifndef _PHOENIX_I2C_LSM9DSXX_H
#define _PHOENIX_I2C_LSM9DSXX_H

#include <stdint.h>
#include "communication.h"

/* size of all data accessible from lsmdsxx */
#define LSM9DSXX_DATA_IMU_ALL_SIZE 6
#define LSM9DSXX_DATA_MAG_ALL_SIZE 3

/* all supported members of lsmdsxx family */
enum lsm9ds_family { lsm9ds1_imu,
	lsm9ds1_mag };

/* initialization of device from lsmdsxx family */
int lsm9dsxx_init(const imu_dev_t *dev);

/* read all data from lsmdsxx family device */
int lsm9dsxx_getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen);

#endif
