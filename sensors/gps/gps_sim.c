/*
 * Phoenix-RTOS
 *
 * Simsensor for gps sensors. Used in potential tests
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "../simsensor_common/simsensor_generic.h"

#define GPS_SIMESENS_NAME "gps_sim"
#define GPS_SIM_QLEN      10
#define GPS_TIME_HORIZON  100000


int gpssim_alloc(sensor_info_t *info, const char *args)
{
	simsens_settings_t settings = {
		.name = GPS_SIMESENS_NAME,
		.messageQueueLen = GPS_SIM_QLEN,
		.sensorTypes = SENSOR_TYPE_GPS,
		.timeHorizon = GPS_TIME_HORIZON
	};

	return simsens_alloc(info, args, &settings);
}


void __attribute__((constructor)) gpssim_register(void)
{
	static sensor_drv_t sensor = {
		.name = GPS_SIMESENS_NAME,
		.alloc = gpssim_alloc,
		.start = simsens_start
	};

	sensors_register(&sensor);
}
