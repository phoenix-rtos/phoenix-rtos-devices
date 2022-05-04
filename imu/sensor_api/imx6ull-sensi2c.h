/*
 * Phoenix-RTOS
 *
 * I2C sensors frontend header for imx6ull
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#ifndef _PHOENIX_ULL_SENSI2C_H
#define _PHOENIX_ULL_SENSI2C_H

/* available device types */
enum dev_types { type_imu, /* 7DoF (accel + gyro + temp) inertial measurement device */
	type_magmeter,         /* 3DoF magnetometer device */
	type_baro              /* 2DoF (pressure + temp) barometer device */
};

/* parametrization of information in 'msg.i.raw' table */
enum msg_fields { field_devtype = 0 };

/* 7DoF imu readings: accel, angular rate, imu-joined temperature */
struct sens_imu_t {
	float accel_x; /* acceleration x-axis in 'g' */
	float accel_y; /* acceleration y-axis in 'g' */
	float accel_z; /* acceleration z-axis in 'g' */
	float gyr_x;   /* angular rate x-axis in rad/s */
	float gyr_y;   /* angular rate y-axis in rad/s */
	float gyr_z;   /* angular rate z-axis in rad/s */
	float temp;    /* temperature in C */
};

/* 3DoF magnetometer readings in uT */
struct sens_mag_t {
	float mag_x; /* x-axis magnetic flux in uT */
	float mag_y; /* y-axis magnetic flux in uT */
	float mag_z; /* z-axis magnetic flux in uT */
};

/* 2DoF barometer readings: pressure, barometer-joined temperature */
struct sens_baro_t {
	float press;     /* pressure in hPa */
	float baro_temp; /* temperature in C */
};

/* Returns latest measurements of 7DoF IMU. Unavailable data is set to NaN */
int sensImu(struct sens_imu_t *imu_data);

/* Returns latest measurements of 3DoF magnetometer. Unavailable data is set to NaN */
int sensMag(struct sens_mag_t *mag_data);

/* Returns latest measurements of 2DoF barometer. Unavailable data is set to NaN */
int sensBaro(struct sens_baro_t *baro_data);

#endif