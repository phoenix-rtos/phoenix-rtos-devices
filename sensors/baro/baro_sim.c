/*
 * Phoenix-RTOS
 *
 * Simsensor for baro sensors. Used in potential tests
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "../simsensor_common/simsensor_generic.h"

#define BARO_SIMESENS_NAME "baro_sim"
#define BARO_SIM_QLEN      10
#define BARO_TIME_HORIZON  100000


int barosim_alloc(sensor_info_t *info, const char *args)
{
	simsens_settings_t settings = {
		.name = BARO_SIMESENS_NAME,
		.messageQueueLen = BARO_SIM_QLEN,
		.sensorTypes = SENSOR_TYPE_BARO,
		.timeHorizon = BARO_TIME_HORIZON
	};

	return simsens_alloc(info, args, &settings);
}


void __attribute__((constructor)) barosim_register(void)
{
	static sensor_drv_t sensor = {
		.name = BARO_SIMESENS_NAME,
		.alloc = barosim_alloc,
		.start = simsens_start
	};

	sensors_register(&sensor);
}
