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
 * Finds a driver specified by `name`. Returns NULL if not found.
 */
const sensor_drv_t *locsensors_find(const char *name);


/**
 * Function registers "drv" as local driver.
 *
 * Not meant to be used by application
 */
void locsensors_register(const sensor_drv_t *drv);


/**
 * Function digests `event` from an `devId` sensor.
 * Function meant to be used as remapping from generic "sensor_register()"
 *
 * #TODO: This function is stub-implemented, no publishig is done!
 */
int locsensors_publish(unsigned int devId, const sensor_event_t *event);


#endif
