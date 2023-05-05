/*
 * Phoenix-RTOS
 *
 * Simsensor for magnetometer sensors. Used in potential tests
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "../simsensor_common/simsensor_generic.h"

#define MAG_SIMESENS_NAME "mag_sim"
#define MAG_SIM_QLEN      10
#define MAG_TIME_HORIZON  100000


int magsim_alloc(sensor_info_t *info, const char *args)
{
	simsens_settings_t settings = {
		.name = MAG_SIMESENS_NAME,
		.messageQueueLen = MAG_SIM_QLEN,
		.sensorTypes = SENSOR_TYPE_MAG,
		.timeHorizon = MAG_TIME_HORIZON
	};

	return simsens_alloc(info, args, &settings);
}


void __attribute__((constructor)) magsim_register(void)
{
	static sensor_drv_t sensor = {
		.name = MAG_SIMESENS_NAME,
		.alloc = magsim_alloc,
		.start = simsens_start
	};

	sensors_register(&sensor);
}
