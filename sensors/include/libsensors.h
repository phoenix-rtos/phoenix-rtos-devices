/*
 * Phoenix-RTOS
 *
 * Sensors Library
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _LIBSENSORS_H_
#define _LIBSENSORS_H_

#include <stdint.h>
#include <sys/ioctl.h>


/* Available sensor types */

#define SENSOR_TYPE_ACCEL (1 << 0)
#define SENSOR_TYPE_BARO  (1 << 1)
#define SENSOR_TYPE_GPS   (1 << 2)
#define SENSOR_TYPE_GYRO  (1 << 3)
#define SENSOR_TYPE_MAG   (1 << 4)
#define SENSOR_TYPE_TEMP  (1 << 5)


typedef unsigned int sensor_type_t;


typedef struct {
	uint32_t devId;
	int16_t accelX; /* value [mm/s^2] */
	int16_t accelY; /* value [mm/s^2] */
	int16_t accelZ; /* value [mm/s^2] */
	uint8_t reserved[2];
} accel_data_t;


typedef struct {
	uint32_t devId;
	uint32_t pressure; /* pressure in [Pa] */
	uint32_t temp;     /* temperature value in Kelvin [K] */
} baro_data_t;


/* GPS position in WGS84 coordinates */
typedef struct {
	uint32_t devId;
	int32_t lat;           /* latitude in 1E-7 degrees */
	int32_t lon;           /* longitude in 1E-7 degrees */
	uint16_t hdop;         /* horizontal dilution of precision */
	uint16_t vdop;         /* vertical dilution of precision */
	int32_t alt;           /* altitude in 1E-3 [m] (millimetres) above MSL */
	int32_t altEllipsoid;  /* altitude in 1E-3 [m] (millimetres) above Ellipsoid */
	uint32_t groundSpeed;  /* GPS ground speed, [mm/s] */
	int32_t velNorth;      /* GPS North velocity, [mm/s] */
	int32_t velEast;       /* GPS East velocity, [mm/s] */
	int32_t velDown;       /* GPS Down velocity, [mm/s] */
	uint32_t eph;          /* GPS horizontal position accuracy 1E-3 [m] (millimetres)*/
	uint32_t epv;          /* GPS vertical position accuracy 1E-3 [m] (millimetres)*/
	uint16_t heading;      /* value in [mrad] */
	int16_t headingOffs;   /* value in [mrad] */
	uint16_t headingAccur; /* value in [mrad] */
	uint8_t satsNb;        /* number of used satellites */
	uint8_t fix;           /* fix quality */
} gps_data_t;


/* Gyroscope data */
typedef struct {
	uint32_t devId;
	int32_t gyroX; /* angular velocity value in [mrad/s] */
	int32_t gyroY; /* angular velocity value in [mrad/s] */
	int32_t gyroZ; /* angular velocity value in [mrad/s] */
} gyro_data_t;


/* Magnetometer data */
typedef struct {
	uint32_t devId;
	int16_t magX; /* value in 1E-7 [T] */
	int16_t magY; /* value in 1E-7 [T] */
	int16_t magZ; /* value in 1E-7 [T] */
	uint8_t reserved[2];
} mag_data_t;


/* Temperature value in [K] */
typedef struct {
	uint32_t devId;
	uint32_t temp; /* temperature value in Kelvin [K] */
} temp_data_t;


/* Event data gets from sensor manager */
typedef struct {
	sensor_type_t type;
	time_t timestamp;

	union {
		accel_data_t accels;
		baro_data_t baro;
		gps_data_t gps;
		gyro_data_t gyro;
		mag_data_t mag;
		temp_data_t temp;
	};
} sensor_event_t;


/* Structure of events passed to the client */
typedef struct {
	size_t size;
	sensor_event_t events[];
} sensors_data_t;


/* Options setting by the client _IOWR */
typedef struct {
	sensor_type_t types; /* in: sensor types defined by the client */
	size_t evtSz;        /* out: number of events for defined types */
} sensors_ops_t;


#define SENSORS_IOCTL_BASE 'S'
#define SMIOC_SENSORSSET   _IOWR(SENSORS_IOCTL_BASE, 1, sensors_ops_t) /* set sensor types and get events number */
#define SMIOC_SENSORSAVAIL _IOR(SENSORS_IOCTL_BASE, 2, sensor_type_t)  /* get available sensor types */

#endif
