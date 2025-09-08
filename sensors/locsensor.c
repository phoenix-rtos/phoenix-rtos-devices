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

	/* sensor allocation storage */
	sensor_info_t dev[SENSORS_MAX];
	int ndev;
} locsensors_common;


/**
 * Searches for sensor with `name` through auto-registered sensors.
 *
 * Returns index of auto-registered driver structure in locsensors_common.drv[] array.
 * Returns -1 if no `name` sensor is not registered.
 */
static int sensor_nameSearch(const char *name)
{
	for (int i = 0; i < locsensors_common.ndrv; i++) {
		if (strcmp(locsensors_common.drv[i]->name, name) == 0) {
			return i;
		}
	}

	return -1;
}


int locsensor_read(const sensor_info_t *info, const sensor_event_t **evt)
{
	return locsensors_common.drv[info->id]->read(info, evt);
}


const sensor_info_t *locsensor_find(int types, int n)
{
	for (int i = 0; i < locsensors_common.ndev; i++) {
		if (locsensors_common.dev[i].types == types && n == 0) {
			return &locsensors_common.dev[i];
		}
	}

	return NULL;
}


void locsensor_dealloc(void)
{
	const sensor_drv_t *drv;
	sensor_info_t *info;

	for (int i = 0; i < locsensors_common.ndev; i++) {
		info = &locsensors_common.dev[i];
		drv = locsensors_common.drv[info->id];

		drv->dealloc(info);
	}
}


int locsensors_publish(unsigned int devId, const sensor_event_t *event)
{
	/* Empty implementation, just to match the "libsensors/sensor.h:sensors_publish" */
	return 0;
}


void locsensors_register(const sensor_drv_t *drv)
{
	if (locsensors_common.ndrv >= SENSORS_MAX) {
		printf("locsensors: no register space: %s\n", drv->name);
		return;
	}

	locsensors_common.drv[locsensors_common.ndrv++] = drv;
}


int locsensor_alloc(char *args)
{
	const char *sensorNames[SENSORS_MAX];
	const char *sensorArgs[SENSORS_MAX];
	char *ptr;
	int argc = 0;

	/* Get per-sensor tokens with "<name>:<arg1>:(...):<argN>" structure with sttrok() */
	ptr = strtok(args, " ");
	if (ptr == NULL) {
		fprintf(stderr, "%s: wrong arguments: %s\n", ERRSTR, args);
		return -1;
	}

	while (ptr != NULL && argc < locsensors_common.ndrv) {
		sensorNames[argc] = ptr;
		ptr = strtok(NULL, " ");
		argc++;
	}

	/* Check if all sensors from args are registered */
	for (int i = 0; i < argc; i++) {
		ptr = strchr(sensorNames[i], ':');
		if (ptr == NULL) {
			fprintf(stderr, "%s: no args for %s\n", ERRSTR, sensorNames[i]);
			return -1;
		}

		/**
		 * Split <name> and <arg1>:(...):<argN> parts.
		 * If there is only ':' after sensor name, sensorArgs will point to '\0'.
		 */
		*ptr = '\0';
		sensorArgs[i] = ptr + 1;

		/* Find sensor driver and prepare sensor device with it */
		sensor_info_t *info = &locsensors_common.dev[i];
		int drvIdx = sensor_nameSearch(sensorNames[i]);
		if (drvIdx < 0) {
			fprintf(stderr, "%s: unknown sensor %s\n", ERRSTR, sensorNames[i]);
			return -1;
		}
		info->id = drvIdx;
		info->drv = locsensors_common.drv[drvIdx]->name;
	}

	/* allocate valid sensors */
	for (int i = 0; i < argc; i++) {
		int err = 0;
		sensor_info_t *info = &locsensors_common.dev[i];
		const sensor_drv_t *drv = locsensors_common.drv[info->id];

		err = drv->alloc(info, sensorArgs[i]);
		if (err != 0) {
			fprintf(stderr, "%s: alloc failed: %s\n", ERRSTR, sensorNames[i]);
			for (int j = 0; j < i; ++j) {
				info = &locsensors_common.dev[j];
				drv = locsensors_common.drv[info->id];

				drv->dealloc(info);
			}
			return -1;
		}
	}

	locsensors_common.ndev = argc;

	return 0;
}
