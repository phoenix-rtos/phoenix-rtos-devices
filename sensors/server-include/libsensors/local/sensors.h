/*
 * Phoenix-RTOS
 *
 * Sensor Manager local implementation
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _SENSORS_LOCAL_H
#define _SENSORS_LOCAL_H

#include <libsensors/sensor.h>


/**
 * Performs a read  operation on sensors specified by `info`.
 */
int locsensor_read(const sensor_info_t *info, const sensor_event_t **evt);


/**
 * Finds a sensor of specific type (e.g SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO for 6dof IMU).
 */
const sensor_info_t *locsensor_find(int types, int n);


/**
 * Deallocates all allocated local sensors
 */
void locsensor_dealloc(void);


/**
 * Allocates all local sensors from `args` which are auto-registered from
 * sensor libraries dependencies.
 *
 * Returns 0 on successful allocation of all sensors specified by "args".
 * Returns -1 on error.
 */
int locsensor_alloc(char *args);


/**
 * Function digests `event` from an `devId` sensor.
 * Function meant to be used as remapping from generic "sensor_register()"
 *
 * #TODO: This function is stub-implemented, no publishig is done!
 */
int locsensors_publish(unsigned int devId, const sensor_event_t *event);


/**
 * Function registers "drv" as local driver.
 * Function meant to be used as remapping from generic "sensor_register()"
 */
void locsensors_register(const sensor_drv_t *drv);


#endif
