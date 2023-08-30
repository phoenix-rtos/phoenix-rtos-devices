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
#include <sensors-spi.h>
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
#define VAL_CTRL_REG1_G_BW_MIN      0x00
#define VAL_CTRL_REG1_G_BW_LOW      0x01
#define VAL_CTRL_REG1_G_BW_HIGH     0x02
#define VAL_CTRL_REG1_G_BW_MAX      0x03

/* control register 2 */
#define REG_CTRL_REG2_G                      0x11
#define VAL_CTRL_REG2_G_OUT_SEL_LPF2_ENABLE  0x03
#define VAL_CTRL_REG2_G_OUT_SEL_LPF2_DISABLE 0x00

/* control register 4 */
#define CTRL_REG4          0x1e
#define CTRL_REG4_G_ENABLE 0x38

/* control register 5 */
#define REG_CTRL_REG5_XL        0x1f
#define VAL_CTRL_REG5_XL_ENABLE 0x38

/* control register 6 */
#define REG_CTRL_REG6_XL             0x20
#define VAL_CTRL_REG6_XL_ODR_XL_952  0xc0
#define MASK_CTRL_REG6_XL_FS         0x18
#define VAL_CTRL_REG6_XL_FS_2G       0x00 /* strange order, not a mistake! */
#define VAL_CTRL_REG6_XL_FS_4G       0x10 /* strange order, not a mistake! */
#define VAL_CTRL_REG6_XL_FS_8G       0x18 /* strange order, not a mistake! */
#define VAL_CTRL_REG6_XL_FS_16G      0x08 /* strange order, not a mistake! */
#define VAL_CTRL_REG6_XL_BW_SCAL_ODR 0x04
#define VAL_CTRL_REG6_XL_BW_XL_105HZ 0x02
#define VAL_CTRL_REG6_XL_BW_XL_50HZ  0x03

/* control register 7 */
#define REG_CTRL_REG7_XL           0x21
#define VAL_CTRL_REG7_XL_HR_ENABLE 0x80
#define VAL_CTRL_REG7_XL_DCF_50    0x00
#define VAL_CTRL_REG7_XL_DCF_100   0x20
#define VAL_CTRL_REG7_XL_DCF_9     0x40
#define VAL_CTRL_REG7_XL_DCF_400   0x60
#define VAL_CTRL_REG7_XL_FDS       0x04

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
#define GYR2000DPS_CONV_MRAD 1.221730475f /* convert gyroscope value (at scale 2000DPS) to mrad/s */
#define ACC8G_CONV_MG        0.244F       /* convert accelerations (at 8G scale) [m/s^2] */
#define MG_CONV_MMS2         9.80665f     /* convert milli G to [mm/s^2] */


typedef struct {
	spimsg_ctx_t spiCtx;
	oid_t spiSS;
	sensor_event_t evtAccel;
	sensor_event_t evtGyro;
	char stack[512] __attribute__((aligned(8)));
} lsm9dsxx_ctx_t;


static int32_t translateGyr(uint8_t hbyte, uint8_t lbyte)
{
	/* MISRA incompliant - casting u16 to s16 with no regard to sign */
	int16_t val = ((uint16_t)hbyte << 8) | (uint16_t)lbyte;

	return GYR2000DPS_CONV_MRAD * val; /* sensor value to [mrad/s] */
}


static int32_t translateAcc(uint8_t hbyte, uint8_t lbyte)
{
	/* MISRA incompliant - casting u16 to s16 with no regard to sign */
	int16_t val = ((uint16_t)hbyte << 8) | (uint16_t)lbyte;

	return ACC8G_CONV_MG * MG_CONV_MMS2 * val; /* sensor value to [mm/s^2] */
}


static int spiWriteReg(lsm9dsxx_ctx_t *ctx, uint8_t regAddr, uint8_t regVal)
{
	unsigned char cmd[2] = { (regAddr & 0x7F), regVal }; /* write bit set to regAddr */

	return sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, cmd, sizeof(cmd), NULL, 0, sizeof(cmd));
}


static int lsm9dsxx_whoamiCheck(lsm9dsxx_ctx_t *ctx)
{
	uint8_t cmd, val;
	int err;

	cmd = REG_WHOAMI | SPI_READ_BIT;
	val = 0;
	err = sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &cmd, sizeof(cmd), &val, sizeof(val), sizeof(cmd));
	if ((err < 0) || (val != REG_VAL_WHOAMI)) {
		return -1;
	}

	return 0;
}


static int lsm9dsxx_hwSetup(lsm9dsxx_ctx_t *ctx)
{
	if (lsm9dsxx_whoamiCheck(ctx) != 0) {
		printf("lsm9dsxx: cannot read/wrong WHOAMI returned!\n");
		return -1;
	}

	/* auto increment + SW reset */
	if (spiWriteReg(ctx, REG_CTRL_REG8, 0x05) < 0) {
		return -1;
	}
	usleep(1000 * 100);

	/* ranges and sampling of accelerometer and gyro, accelerometer LPF */
	if (spiWriteReg(ctx, REG_CTRL_REG1_G, (VAL_CTRL_REG1_G_ODR_G_952HZ | VAL_CTRL_REG1_G_FS_G_2000 | VAL_CTRL_REG1_G_BW_MAX)) < 0) {
		return -1;
	}
	if (spiWriteReg(ctx, REG_CTRL_REG6_XL, (VAL_CTRL_REG6_XL_ODR_XL_952 | VAL_CTRL_REG6_XL_FS_8G | VAL_CTRL_REG6_XL_BW_SCAL_ODR | VAL_CTRL_REG6_XL_BW_XL_50HZ)) < 0) {
		return -1;
	}
	if (spiWriteReg(ctx, REG_CTRL_REG2_G, VAL_CTRL_REG2_G_OUT_SEL_LPF2_DISABLE) < 0) {
		return -1;
	}
	usleep(1000 * 100);

	/* enabling sensors */
	if (spiWriteReg(ctx, REG_CTRL_REG5_XL, (VAL_CTRL_REG5_XL_ENABLE)) < 0) {
		return -1;
	}
	if (spiWriteReg(ctx, CTRL_REG4, (CTRL_REG4_G_ENABLE)) < 0) {
		return -1;
	}
	usleep(1000 * 100);

	return 0;
}


static void lsm9dsxx_threadPublish(void *data)
{
	static uint32_t dAngleX = 0, dAngleY = 0, dAngleZ = 0;
	static int32_t lastGyroX = 0, lastGyroY = 0, lastGyroZ = 0;
	static time_t lastGyroTime;

	int err;
	time_t tstamp_gyro, tstamp_accl;
	float step;
	sensor_info_t *info = (sensor_info_t *)data;
	lsm9dsxx_ctx_t *ctx = info->ctx;
	uint8_t ibuf[SENSOR_OUTPUT_SIZE] = { 0 };
	uint8_t obuf;

	gettime(&lastGyroTime, NULL);

	/*
	 * TODO: Temperature read not implemented!
	 * If gyro read is extended temperature may be read at once (temp. registers are close to gyro registers).
	 */
	ctx->evtAccel.accels.temp = 0;
	ctx->evtGyro.gyro.temp = 0;

	while (1) {
		/* odr is set to 952, thus 1ms wait is satisfactory */
		usleep(1000);

		/* accel read */
		obuf = REG_DATA_OUT_ACCL | SPI_READ_BIT;
		err = sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &obuf, sizeof(obuf), ibuf, sizeof(ibuf), sizeof(obuf));
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
		err = sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &obuf, sizeof(obuf), ibuf, sizeof(ibuf), sizeof(obuf));
		gettime(&tstamp_gyro, NULL);

		if (err >= 0) {
			/* minus accounts for non right-handness of lsm9dsxx gyroscope frame of reference */
			ctx->evtGyro.gyro.gyroX = -translateGyr(ibuf[1], ibuf[0]);
			ctx->evtGyro.gyro.gyroY = translateGyr(ibuf[3], ibuf[2]);
			ctx->evtGyro.gyro.gyroZ = translateGyr(ibuf[5], ibuf[4]);
			ctx->evtGyro.timestamp = tstamp_gyro;

			/* Integration of current measurement */
			step = (tstamp_gyro - lastGyroTime) / 2000.f; /* dividing by (1000 * 2) for correct unit and avg. of current and last measurement */
			dAngleX += (uint32_t)((ctx->evtGyro.gyro.gyroX + lastGyroX) * step);
			dAngleY += (uint32_t)((ctx->evtGyro.gyro.gyroY + lastGyroY) * step);
			dAngleZ += (uint32_t)((ctx->evtGyro.gyro.gyroZ + lastGyroZ) * step);

			ctx->evtGyro.gyro.dAngleX = dAngleX;
			ctx->evtGyro.gyro.dAngleY = dAngleY;
			ctx->evtGyro.gyro.dAngleZ = dAngleZ;

			lastGyroTime = tstamp_gyro;
			lastGyroX = ctx->evtGyro.gyro.gyroX;
			lastGyroY = ctx->evtGyro.gyro.gyroY;
			lastGyroZ = ctx->evtGyro.gyro.gyroZ;

			sensors_publish(info->id, &ctx->evtGyro);
		}
	}
}


static int lsm9dsxx_start(sensor_info_t *info)
{
	int err;
	lsm9dsxx_ctx_t *ctx = (lsm9dsxx_ctx_t *)info->ctx;

	err = beginthread(lsm9dsxx_threadPublish, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("lsm9dsxx: launched sensor\n");
	}

	return err;
}


static int lsm9dsxx_alloc(sensor_info_t *info, const char *args)
{
	lsm9dsxx_ctx_t *ctx;
	char *ss;
	int err;

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
	ctx->spiCtx.speed = 10000000;

	ss = strchr(args, ':');
	if (ss != NULL) {
		*(ss++) = '\0';
	}

	/* initialize SPI device communication */
	err = sensorsspi_open(args, ss, &ctx->spiCtx.oid, &ctx->spiSS);
	if (err < 0) {
		printf("lsm9dsxx: Can`t initialize SPI device\n");
		free(ctx);
		return err;
	}

	/* hardware setup of imu */
	if (lsm9dsxx_hwSetup(ctx) < 0) {
		printf("lsm9dsxx: failed to setup device\n");
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
