#/*
 * Phoenix-RTOS
 *
 * MPU6000 Sensorhub registratioN
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <libsensors/sensor.h>
#include <libsensors/spi/spi.h>

#include <mpu6000/sensor.h>


void __attribute__((constructor)) mpu6000_register(void)
{
	static sensor_drv_t sensor = {
		.name = "mpu6000",
		.alloc = mpu6000_alloc,
		.start = mpu6000_start,
        .read = mpu6000_read
	};

	sensors_register(&sensor);
}
