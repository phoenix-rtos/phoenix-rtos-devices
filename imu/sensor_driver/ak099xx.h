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
#ifndef _PHOENIX_I2C_AK099XX_H
#define _PHOENIX_I2C_AK099XX_H

#include <stdint.h>
#include "communication.h"

/* size in bytes of all data accessible from ak099xx device */
#define AK0990XX_DATA_ALL_SIZE 6

/* all supported members of ak099xx magnetometers family */
enum ak099xx_family { ak09916 };

/* initialize device from ak099xx family */
int ak099xx_init(const imu_dev_t *dev);

/* read all data from magnetometer of ak099xx family */
int ak099xx_getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen);

#endif
