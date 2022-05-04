/*
 * Phoenix-RTOS
 *
 * IMU I2C driver - lps25xx device driver
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#ifndef _PHOENIX_I2C_LPS25XX_H
#define _PHOENIX_I2C_LPS25XX_H

#include <stdint.h>
#include "communication.h"

/* size of all data accessible from lsmdsxx */
#define LPS25XX_DATA_BARO_ALL_SIZE 1
#define LPS25XX_DATA_TEMP_ALL_SIZE 1

/* all supported members of lps25xx family */
enum lps25xx_family { lps25hb_baro,
	lps25hb_temp };

/* initialization of device from lps25xx family */
int lps25xx_init(const imu_dev_t *dev);

/* read all data from lps25xx family device */
int lps25xx_getAllData(const imu_dev_t *dev, float *buffer, uint8_t buflen);

#endif
