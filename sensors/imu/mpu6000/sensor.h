#/*
 * Phoenix-RTOS
 *
 * Driver for MPU6000 imu
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

 #ifndef _SENSOR_MPU6000_H
 #define _SENSOR_MPU6000_H


 #include <libsensors/sensor.h>


int mpu6000_read(sensor_info_t *info, const sensor_event_t **evt);


int mpu6000_start(sensor_info_t *info);


int mpu6000_dealloc(sensor_info_t *info);


int mpu6000_alloc(sensor_info_t *info, const char *args);


 #endif