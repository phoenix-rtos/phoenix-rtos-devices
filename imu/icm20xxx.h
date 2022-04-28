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
#ifndef _PHOENIX_I2C_ICM20XXX_H
#define _PHOENIX_I2C_ICM20XXX_H

#include <stdint.h>
#include "communication.h"

/* size of all data accessible from icm20xxx */
#define ICM20XXX_DATA_ALL_SIZE 14

/* all supported members of icm20xxx family */
enum icm_family { icm20948 };

/* initialization of device from icm20xxx family */
int icm20xxx_init(const imu_dev_t *dev);

/* read all data from icm20xxx family device */
int icm20xxx_getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen);

#endif
