/*
 * Phoenix-RTOS
 *
 * Generic functions for all sensor simulators
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "../sensors.h"


typedef struct {
	const char *name;
	sensor_type_t sensorTypes;
	unsigned int messageQueueLen;
	time_t timeHorizon;
} simsens_settings_t;


/* Starts simsensor. Function is meant to be while registering new sensor type. */
extern int simsens_start(sensor_info_t *info);


/* Allocates simsensor with specified settings. Meant to be used while registering new type. */
extern int simsens_alloc(sensor_info_t *info, const char *args, const simsens_settings_t *settings);
