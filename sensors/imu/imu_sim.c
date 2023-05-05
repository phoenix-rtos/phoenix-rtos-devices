/*
 * Phoenix-RTOS
 *
 * Simsensor for imu sensors. Used in potential tests
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "../simsensor_common/simsensor_generic.h"

#define IMU_SIMESENS_NAME "imu_sim"
#define IMU_SIM_QLEN      10
#define IMU_TIME_HORIZON  100000


int imusim_alloc(sensor_info_t *info, const char *args)
{
	simsens_settings_t settings = {
		.name = IMU_SIMESENS_NAME,
		.messageQueueLen = IMU_SIM_QLEN,
		.sensorTypes = SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO,
		.timeHorizon = IMU_TIME_HORIZON
	};

	return simsens_alloc(info, args, &settings);
}


void __attribute__((constructor)) imusim_register(void)
{
	static sensor_drv_t sensor = {
		.name = IMU_SIMESENS_NAME,
		.alloc = imusim_alloc,
		.start = simsens_start
	};

	sensors_register(&sensor);
}
