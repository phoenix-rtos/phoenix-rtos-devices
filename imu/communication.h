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

enum dev_types { type_imu, type_magmeter };

enum dev_families { icm20xxx, ak099xx, unknown };

typedef struct {
	enum dev_types type;
	enum dev_families family;
	int model;
	char name[16];
	uint8_t addr;
	uint8_t whoami_addr;
	uint8_t whoami_val;
} i2c_dev_t;

/* i2c bus initialization wrapper */
int initialize_i2cbus(void);

/* writes byte to device */
int sendbyte(uint8_t dev_addr, uint8_t reg_addr, uint8_t val);

/* checks if given devname from icm20xxx family is connected */
const i2c_dev_t *checkDevices(enum dev_types devtype);

/* setups given device */
int initDevice(const i2c_dev_t *dev);

/* gets all available data from accel and gyro */
int getAllData(const i2c_dev_t *dev, float *buffer, uint8_t buflen);

#endif