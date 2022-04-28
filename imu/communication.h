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
#ifndef _PHOENIX_I2C_IMU_COMMON_H
#define _PHOENIX_I2C_IMU_COMMON_H

#include <stdint.h>

/* earth gravitational acceleration in m/s^2 */
#define EARTH_G 9.80665F
/* 1 deg = DEG2RAD radians */
#define DEG2RAD 0.0174532925F

enum dev_types { type_imu,
	type_magmeter };

enum dev_families { icm20xxx,
	ak099xx,
	unknown };

typedef struct {
	enum dev_types type;
	enum dev_families family;
	int model;
	char name[16];
	uint8_t devAddr;
	uint8_t whoamiAddr;
	uint8_t whoamiVal;
} imu_dev_t;

/* i2c bus initialization wrapper */
int initialize_i2cbus(int bus_no);

/* checks if given devname from icm20xxx family is connected */
const imu_dev_t *probeDevices(enum dev_types devtype);

/* setups given device */
int initDevice(const imu_dev_t *dev);

/* gets all available data from accel and gyro */
int getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen);

#endif