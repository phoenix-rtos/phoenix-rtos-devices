/*
 * Phoenix-RTOS
 *
 * Driver for MPU6000 imu
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
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

/* self-identification register */
#define REG_WHOAMI 0x75
#define VAL_WHOAMI 0x68

#define SPI_READ_BIT 0x80

#define REG_USER_CTRL            0x6a
#define VAL_USER_CTRL_I2C_IF_DIS 0x10 /* disable I2C bit */

/* Sample rate divider accepts all values 0-255. Providing only div=1 macro */
#define REG_SMPRT_DIV   0x19
#define VAL_SMPRT_DIV_1 0x00 /* sample rate divider = 1 */

#define REG_CONFIG                0x1a
#define VAL_CONFIG_DLPF_CFG_NONE  0x07 /* no LPF, 8KHz gyro sampling */
#define VAL_CONFIG_DLPF_CFG_256HZ 0x00 /* LPF on, 8KHz gyro sampling */
#define VAL_CONFIG_DLPF_CFG_188HZ 0x01 /* LPF on, 1KHz gyro sampling */
#define VAL_CONFIG_DLPF_CFG_98HZ  0x02 /* LPF on, 1KHz gyro sampling */
#define VAL_CONFIG_DLPF_CFG_42HZ  0x03 /* LPF on, 1KHz gyro sampling */
#define VAL_CONFIG_DLPF_CFG_20HZ  0x04 /* LPF on, 1KHz gyro sampling */
#define VAL_CONFIG_DLPF_CFG_10HZ  0x05 /* LPF on, 1KHz gyro sampling */
#define VAL_CONFIG_DLPF_CFG_5HZ   0x06 /* LPF on, 1KHz gyro sampling */

#define REG_GYRO_CONFIG                0x1B
#define VAL_GYRO_CONFIG_FS_SEL_250DPS  0x00
#define VAL_GYRO_CONFIG_FS_SEL_500DPS  0x08
#define VAL_GYRO_CONFIG_FS_SEL_1000DPS 0x10
#define VAL_GYRO_CONFIG_FS_SEL_2000DPS 0x18

#define REG_ACCEL_CONFIG             0x1c
#define VAL_ACCEL_CONFIG_AFS_SEL_2G  0x00
#define VAL_ACCEL_CONFIG_AFS_SEL_4G  0x08
#define VAL_ACCEL_CONFIG_AFS_SEL_8G  0x10
#define VAL_ACCEL_CONFIG_AFS_SEL_16G 0x18

#define REG_SIGNAL_PATH_RESET             0x68
#define VAL_SIGNAL_PATH_RESET_TEMP_RESET  0x01
#define VAL_SIGNAL_PATH_RESET_ACCEL_RESET 0x02
#define VAL_SIGNAL_PATH_RESET_GYRO_RESET  0x04

#define REG_PWR_MGMT_1              0x6b
#define VAL_PWR_MGMT_1_DEVICE_RESET 0x80
#define VAL_PWR_MGMT_1_SLEEP        0x64
#define VAL_PWR_MGMT_1_CLKSEL_PLL_Z 0x03

#define REG_DATA_OUT_ALL   0x3b
#define SENSOR_OUTPUT_SIZE 14

/* conversions */
#define GYR2000DPS_CONV_MRAD 1.064225152f     /* convert gyroscope value (at scale 2000DPS) to mrad/s */
#define ACC8G_CONV_MG        0.24414f         /* convert accelerations (at 8G scale) [m/s^2] */
#define MG_CONV_MMS2         9.80665f         /* convert milli G to [mm/s^2] */
#define TEMP_SCALER          340              /* temperature register scaler */
#define TEMP_OFFSET          (36530 + 273150) /* sensor offset + celsius2kelvin offset */

typedef struct {
	spimsg_ctx_t spiCtx;
	oid_t spiSS;
	sensor_event_t evtAccel;
	sensor_event_t evtGyro;
	uint8_t lpfSel;
	char stack[512] __attribute__((aligned(8)));
} mpu6000_ctx_t;


static int32_t translateGyr(uint8_t hbyte, uint8_t lbyte)
{
	int16_t val;

	/* MISRA incompliant - casting u16 to s16 with no regard to sign */
	val = ((uint16_t)hbyte << 8) | (uint16_t)lbyte;

	return GYR2000DPS_CONV_MRAD * val; /* sensor value to [mrad/s] */
}


static int32_t translateAcc(uint8_t hbyte, uint8_t lbyte)
{
	/* MISRA incompliant - casting u16 to s16 with no regard to sign */
	int16_t val = ((uint16_t)hbyte << 8) | (uint16_t)lbyte;

	return ACC8G_CONV_MG * MG_CONV_MMS2 * val; /* sensor value to [mm/s^2] */
}


static uint32_t translateTemp(uint8_t hbyte, uint8_t lbyte)
{
	/* MISRA incompliant - casting u16 to s16 with no regard to sign */
	int16_t val = ((uint16_t)hbyte << 8) | (uint16_t)lbyte;

	return (1000 * (int32_t)val) / TEMP_SCALER + TEMP_OFFSET; /* sensor value to [millikelvins] */
}


static int spiWriteReg(mpu6000_ctx_t *ctx, uint8_t regAddr, uint8_t regVal)
{
	unsigned char cmd[2] = { (regAddr & 0x7F), regVal }; /* write bit set to regAddr */

	return sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, cmd, sizeof(cmd), NULL, 0, sizeof(cmd));
}


static int mpu6000_whoamiCheck(mpu6000_ctx_t *ctx)
{
	uint8_t cmd, val;
	int err;

	cmd = REG_WHOAMI | SPI_READ_BIT;
	val = 0;
	err = sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &cmd, sizeof(cmd), &val, sizeof(val), sizeof(cmd));
	if ((err < 0) || (val != VAL_WHOAMI)) {
		return -1;
	}

	return 0;
}


static int mpu6000_hwSetup(mpu6000_ctx_t *ctx)
{
	if (mpu6000_whoamiCheck(ctx) != 0) {
		printf("mpu6000: cannot read/wrong WHOAMI returned!\n");
		return -1;
	}

	/* Reset procedure according to "MPU-6000/MPU-6050 Register Map and Descriptions", section 4.28 */
	if (spiWriteReg(ctx, REG_PWR_MGMT_1, VAL_PWR_MGMT_1_DEVICE_RESET) < 0) {
		return -1;
	}
	usleep(100 * 1000);
	if (spiWriteReg(ctx, REG_SIGNAL_PATH_RESET, VAL_SIGNAL_PATH_RESET_ACCEL_RESET | VAL_SIGNAL_PATH_RESET_GYRO_RESET | VAL_SIGNAL_PATH_RESET_TEMP_RESET) < 0) {
		return -1;
	}
	usleep(100 * 1000);

	/* Wake up the device. Writing 0 unsets DEVICE_RESET, SLEEP, CYCLE, TEMP_DIS bits */
	if (spiWriteReg(ctx, REG_PWR_MGMT_1, (0 | VAL_PWR_MGMT_1_CLKSEL_PLL_Z)) < 0) {
		return -1;
	}

	/* Disabling I2C */
	if (spiWriteReg(ctx, REG_USER_CTRL, VAL_USER_CTRL_I2C_IF_DIS) < 0) {
		return -1;
	}

	/* Sampling rate and LPF config */
	if (spiWriteReg(ctx, REG_CONFIG, ctx->lpfSel) < 0) {
		return -1;
	}

	if (spiWriteReg(ctx, REG_SMPRT_DIV, VAL_SMPRT_DIV_1) < 0) {
		return -1;
	}

	/* Sensor ranges setup */
	if (spiWriteReg(ctx, REG_GYRO_CONFIG, VAL_GYRO_CONFIG_FS_SEL_2000DPS) < 0) {
		return -1;
	}

	if (spiWriteReg(ctx, REG_ACCEL_CONFIG, VAL_ACCEL_CONFIG_AFS_SEL_8G) < 0) {
		return -1;
	}

	return 0;
}


static void mpu6000_threadPublish(void *data)
{
	static uint32_t dAngleX = 0, dAngleY = 0, dAngleZ = 0;
	static int32_t lastGyroX = 0, lastGyroY = 0, lastGyroZ = 0;
	static time_t tStampLast;

	const uint8_t obuf = REG_DATA_OUT_ALL | SPI_READ_BIT;
	sensor_info_t *info = (sensor_info_t *)data;
	mpu6000_ctx_t *ctx = info->ctx;
	uint8_t ibuf[SENSOR_OUTPUT_SIZE] = { 0 };
	time_t tStamp;
	float step;
	int err;

	gettime(&tStampLast, NULL);

	/* TODO: SPI speed bottleneck: MPU6000 SPI accepts <1MHz CLK writes, but <20MHz CLK reads; runtime SPI CLK setup unavailable */
	ctx->spiCtx.speed = 10000000;

	while (1) {
		/* odr is set to 952, thus 1ms wait is satisfactory */
		usleep(1000);

		/* data read */
		err = sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &obuf, sizeof(obuf), ibuf, sizeof(ibuf), sizeof(obuf));
		gettime(&tStamp, NULL);

		if (err < 0) {
			continue;
		}

		/* Common package: gyro and accel utilize the same temperature reading */
		ctx->evtAccel.accels.temp = translateTemp(ibuf[6], ibuf[7]);
		ctx->evtGyro.gyro.temp = ctx->evtAccel.accels.temp;

		ctx->evtAccel.accels.accelX = translateAcc(ibuf[0], ibuf[1]);
		ctx->evtAccel.accels.accelY = translateAcc(ibuf[2], ibuf[3]);
		ctx->evtAccel.accels.accelZ = translateAcc(ibuf[4], ibuf[5]);
		ctx->evtAccel.timestamp = tStamp;

		ctx->evtGyro.gyro.gyroX = translateGyr(ibuf[8], ibuf[9]);
		ctx->evtGyro.gyro.gyroY = translateGyr(ibuf[10], ibuf[11]);
		ctx->evtGyro.gyro.gyroZ = translateGyr(ibuf[12], ibuf[13]);
		ctx->evtGyro.timestamp = tStamp;

		/* Integration of current measurement */
		step = (tStamp - tStampLast) / 2000.f; /* dividing by (1000 * 2) for correct unit and avg. of current and last measurement */
		dAngleX += (uint32_t)((ctx->evtGyro.gyro.gyroX + lastGyroX) * step);
		dAngleY += (uint32_t)((ctx->evtGyro.gyro.gyroY + lastGyroY) * step);
		dAngleZ += (uint32_t)((ctx->evtGyro.gyro.gyroZ + lastGyroZ) * step);

		ctx->evtGyro.gyro.dAngleX = dAngleX;
		ctx->evtGyro.gyro.dAngleY = dAngleY;
		ctx->evtGyro.gyro.dAngleZ = dAngleZ;

		tStampLast = tStamp;
		lastGyroX = ctx->evtGyro.gyro.gyroX;
		lastGyroY = ctx->evtGyro.gyro.gyroY;
		lastGyroZ = ctx->evtGyro.gyro.gyroZ;

		sensors_publish(info->id, &ctx->evtGyro);
		sensors_publish(info->id, &ctx->evtAccel);
	}
}


static int mpu6000_start(sensor_info_t *info)
{
	int err;
	mpu6000_ctx_t *ctx = (mpu6000_ctx_t *)info->ctx;

	err = beginthread(mpu6000_threadPublish, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("mpu6000: launched sensor\n");
	}

	return err;
}


static int mpu6000_alloc(sensor_info_t *info, const char *args)
{
	mpu6000_ctx_t *ctx;
	char *ss, *lpfChr;
	int err;
	unsigned long lpfSel = 0;

	/* sensor context allocation */
	ctx = malloc(sizeof(mpu6000_ctx_t));
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
	ctx->spiCtx.speed = 1000000;

	ss = strchr(args, ':');
	if (ss != NULL) {
		*(ss++) = '\0';

		lpfChr = strchr(ss, ':');
		if (lpfChr != NULL) {
			*(lpfChr++) = '\0';

			errno = EOK;
			lpfSel = strtoul(lpfChr, NULL, 10);
			if (lpfSel > 7 || (lpfSel == 0 && errno != EOK)) {
				fprintf(stderr, "mpu6000: failed to read lpfSel\n");
				free(ctx);
				return -1;
			}
		}
	}
	ctx->lpfSel = lpfSel;

	/* initialize SPI device communication */
	err = sensorsspi_open(args, ss, &ctx->spiCtx.oid, &ctx->spiSS);
	if (err < 0) {
		printf("mpu6000: Can`t initialize SPI device\n");
		free(ctx);
		return err;
	}

	/* hardware setup of imu */
	if (mpu6000_hwSetup(ctx) < 0) {
		printf("mpu6000: failed to setup device\n");
		free(ctx);
		return -1;
	}

	return EOK;
}


void __attribute__((constructor)) mpu6000_register(void)
{
	static sensor_drv_t sensor = {
		.name = "mpu6000",
		.alloc = mpu6000_alloc,
		.start = mpu6000_start
	};

	sensors_register(&sensor);
}
