/*
 * Phoenix-RTOS
 *
 * Driver for IMU lsm9dsxx
 * 
 * lsm9dsxx is preset to:
 *  - output data rate (ODR) to 952Hz 
 *  - accelerometer range to +-8G
 *  - gyroscope range to +-2000dps
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski, Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>
#include <spi.h>
#include <spi-msg.h>
#include <string.h>

#include "../sensors.h"

/* self-identification register of magnetometer */
#define REG_WHOAMI     0x0f
#define REG_VAL_WHOAMI 0x68

/* control register 1 */
#define REG_CTRL_REG1_G             0x10
#define VAL_CTRL_REG1_G_ODR_G_952HZ 0xc0
#define VAL_CTRL_REG1_G_FS_G_245    0x00
#define VAL_CTRL_REG1_G_FS_G_500    0x08
#define VAL_CTRL_REG1_G_FS_G_2000   0x18

/* control register 4 */
#define CTRL_REG4          0x1e
#define CTRL_REG4_G_ENABLE 0x38

/* control register 5 */
#define REG_CTRL_REG5_XL        0x1f
#define VAL_CTRL_REG5_XL_ENABLE 0x38

/* control register 6 */
#define REG_CTRL_REG6_XL            0x20
#define VAL_CTRL_REG6_XL_ODR_XL_952 0xc0
#define MASK_CTRL_REG6_XL_FS        0x18
#define VAL_CTRL_REG6_XL_FS_2G      0x00 /* strange order, not a mistake! */
#define VAL_CTRL_REG6_XL_FS_4G      0x10 /* strange order, not a mistake! */
#define VAL_CTRL_REG6_XL_FS_8G      0x18 /* strange order, not a mistake! */
#define VAL_CTRL_REG6_XL_FS_16G     0x08 /* strange order, not a mistake! */

/* control register 8 */
#define REG_CTRL_REG8 0x22

/* control register 9 */
#define REG_CTRL_REG9         0x23
#define VAL_CTRL_REK9_I2C_DIS 0x04

/* data storage addresses */
#define REG_DATA_OUT_TEMP 0x15
#define REG_DATA_OUT_GYRO 0x18
#define REG_DATA_OUT_ACCL 0x28
#define SPI_READ_BIT      0x80

/* sensors data sizes */
#define SENSOR_OUTPUT_SIZE 6
#define GYR_OVERFLOW       26820 /* if gyro returns more than this, the sensorhub result will overflow */

/* conversions */
#define GYR2000DPS_CONV_MRAD 1.221730475F /* convert gyroscope value (at scale 2000DPS) to mrad/s */
#define ACC8G_CONV_MS2       0.244F       /* convert accelerations (at 8G scale) [m/s^2] */


typedef struct {
	spimsg_ctx_t spiCtx;
	sensor_event_t evtAccel;
	sensor_event_t evtGyro;
	char stack[512] __attribute__((aligned(8)));
} lsm9dsxx_ctx_t;


static int16_t translateGyr(uint8_t hbyte, uint8_t lbyte)
{
	int16_t val = 0;
	val |= hbyte;
	val <<= 8;
	val |= lbyte;

	/* overflow handling */
	if (val > GYR_OVERFLOW) {
		return INT16_MAX;
	}
	if (val < -GYR_OVERFLOW) {
		return INT16_MIN;
	}
	return GYR2000DPS_CONV_MRAD * val; /* sensor value to [mrad/s] */
}


static int16_t translateAcc(uint8_t hbyte, uint8_t lbyte)
{
	int16_t val = 0;
	val |= hbyte;
	val <<= 8;
	val |= lbyte;
	return ACC8G_CONV_MS2 * val; /* sensor value to [mm/s^2] */
}


static int spiWriteReg(spimsg_ctx_t *spiCtx, uint8_t regAddr, uint8_t regVal)
{
	unsigned char cmd[2] = { (regAddr & 0x7F), regVal }; /* write bit set to regAddr */

	return spimsg_xfer(spiCtx, cmd, sizeof(cmd), NULL, 0, sizeof(cmd));
}


static int lsm9dsxx_whoamiCheck(spimsg_ctx_t *spiCtx)
{
	uint8_t cmd, val, err;

	cmd = REG_WHOAMI | SPI_READ_BIT;
	val = 0;
	err = spimsg_xfer(spiCtx, &cmd, 1, &val, 1, 1);
	if ((err < 0) | (val != REG_VAL_WHOAMI)) {
		return -1;
	}

	return 0;
}


static int lsm9dsxx_hwSetup(spimsg_ctx_t *spiCtx)
{
	if (lsm9dsxx_whoamiCheck(spiCtx) != 0) {
		printf("lsm9dsxx: cannot read/wrong WHOAMI returned!\n");
		return -1;
	}

	/* auto increment + SW reset */
	if (spiWriteReg(spiCtx, REG_CTRL_REG8, 0x05) < 0) {
		return -1;
	}
	usleep(1000 * 100);

	/* ranges and sampling of accelerometer and gyro */
	if (spiWriteReg(spiCtx, REG_CTRL_REG1_G, (VAL_CTRL_REG1_G_ODR_G_952HZ | VAL_CTRL_REG1_G_FS_G_2000)) < 0) {
		return -1;
	}
	if (spiWriteReg(spiCtx, REG_CTRL_REG6_XL, (VAL_CTRL_REG6_XL_ODR_XL_952 | VAL_CTRL_REG6_XL_FS_8G)) < 0) {
		return -1;
	}
	usleep(1000 * 100);

	/* enabling sensors */
	if (spiWriteReg(spiCtx, REG_CTRL_REG5_XL, (VAL_CTRL_REG5_XL_ENABLE)) < 0) {
		return -1;
	}
	if (spiWriteReg(spiCtx, CTRL_REG4, (CTRL_REG4_G_ENABLE)) < 0) {
		return -1;
	}
	usleep(1000 * 100);

	return 0;
}


static void lsm9dsxx_threadPublish(void *data)
{
	int err;
	time_t tstamp_gyro, tstamp_accl;
	sensor_info_t *info = (sensor_info_t *)data;
	lsm9dsxx_ctx_t *ctx = info->ctx;
	spimsg_ctx_t *spiCtx = &ctx->spiCtx;
	uint8_t ibuf[SENSOR_OUTPUT_SIZE] = { 0 };
	uint8_t obuf;

	while (1) {
		/* odr is set to 952, thus 1ms wait is satisfactory */
		usleep(1000);

		/* accel read */
		obuf = REG_DATA_OUT_ACCL | SPI_READ_BIT;
		err = spimsg_xfer(spiCtx, &obuf, 1, ibuf, SENSOR_OUTPUT_SIZE, 1);
		gettime(&tstamp_accl, NULL);

		if (err >= 0) {
			/* minus accounts for non right-handness of lsm9dsxx accelerometer frame of reference */
			ctx->evtAccel.accels.accelX = -translateAcc(ibuf[1], ibuf[0]);
			ctx->evtAccel.accels.accelY = translateAcc(ibuf[3], ibuf[2]);
			ctx->evtAccel.accels.accelZ = translateAcc(ibuf[5], ibuf[4]);
			ctx->evtAccel.timestamp = tstamp_accl;
			sensors_publish(info->id, &ctx->evtAccel);
		}

		/* gyroscope read */
		obuf = REG_DATA_OUT_GYRO | SPI_READ_BIT;
		err = spimsg_xfer(spiCtx, &obuf, 1, ibuf, SENSOR_OUTPUT_SIZE, 1);
		gettime(&tstamp_gyro, NULL);

		if (err >= 0) {
			/* minus accounts for non right-handness of lsm9dsxx gyroscope frame of reference */
			ctx->evtGyro.gyro.gyroX = -translateGyr(ibuf[1], ibuf[0]);
			ctx->evtGyro.gyro.gyroY = translateGyr(ibuf[3], ibuf[2]);
			ctx->evtGyro.gyro.gyroZ = translateGyr(ibuf[5], ibuf[4]);
			ctx->evtGyro.timestamp = tstamp_gyro;
			sensors_publish(info->id, &ctx->evtGyro);
		}
	}
}


static int lsm9dsxx_start(sensor_info_t *info)
{
	int err;
	lsm9dsxx_ctx_t *ctx = (lsm9dsxx_ctx_t *)info->ctx;

	err = beginthread(lsm9dsxx_threadPublish, 4, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("lsm9dsxx: launched sensor\n");
	}

	return err;
}


static int lsm9dsxx_alloc(sensor_info_t *info, const char *args)
{
	lsm9dsxx_ctx_t *ctx;
	int ntries = 10;

	/* sensor context allocation */
	ctx = malloc(sizeof(lsm9dsxx_ctx_t));
	if (ctx == NULL) {
		return -ENOMEM;
	}

	ctx->evtAccel.type = SENSOR_TYPE_ACCEL;
	ctx->evtAccel.accels.devId = info->id;
	ctx->evtGyro.type = SENSOR_TYPE_GYRO;
	ctx->evtGyro.gyro.devId = info->id;

	/* filling sensor info structure */
	info->ctx = ctx;
	info->types = SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO;

	ctx->spiCtx.mode = SPI_MODE3;
	ctx->spiCtx.speed = 6250000;

	/* open SPI context */
	while (lookup(args, NULL, &ctx->spiCtx.oid) < 0) {
		ntries--;
		if (ntries == 0) {
			printf("lsm9dsxx: Can`t open SPI device\n");
			free(ctx);
			return -ETIMEDOUT;
		}
		usleep(10 * 1000);
	}

	/* hardware setup of imu */
	if (lsm9dsxx_hwSetup(&ctx->spiCtx) < 0) {
		printf("lsm9dsxx: failed to setup device\n");
		spimsg_close(&ctx->spiCtx);
		free(ctx);
		return -1;
	}

	return EOK;
}


void __attribute__((constructor)) lsm9dsxx_register(void)
{
	static sensor_drv_t sensor = {
		.name = "lsm9dsxx",
		.alloc = lsm9dsxx_alloc,
		.start = lsm9dsxx_start
	};

	sensors_register(&sensor);
}
