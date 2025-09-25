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


#ifndef SENSORS_MAX
#define SENSORS_MAX 8
#endif

#define ERRSTR "locsensors"

#include <string.h>

#include <libsensors/sensor.h>
#include <libsensors/client.h>


struct {
	/* sensor auto-registration storage */
	const sensor_drv_t *drv[SENSORS_MAX];
	int ndrv;
} locsensors_common;


const sensor_drv_t *locsensors_find(const char *name)
{
	for (int i = 0; i < locsensors_common.ndrv; i++) {
		if (strcmp(locsensors_common.drv[i]->name, name) == 0) {
			return locsensors_common.drv[i];
		}
	}

	return NULL;
}


void locsensors_register(const sensor_drv_t *drv)
{
	if (locsensors_common.ndrv >= SENSORS_MAX) {
		printf("locsensors: no register space: %s\n", drv->name);
		return;
	}

	locsensors_common.drv[locsensors_common.ndrv++] = drv;
}


int locsensors_publish(unsigned int devId, const sensor_event_t *event)
{
	return 0;
}
