/*
 * Phoenix-RTOS
 *
 * Driver for lis2mdl magnetometer
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

/* self-identification register of magnetometer */
#define REG_WHOAMI 0x4f
#define VAL_WHOAMI 0x40

#define SPI_READ_BIT 0x80

#define REG_CFG_REG_A               0x60
#define VAL_CFG_REG_A_COMP_TEMP_EN  0x80
#define VAL_CFG_REG_A_ODR_10HZ      0x00
#define VAL_CFG_REG_A_ODR_20HZ      0x04
#define VAL_CFG_REG_A_ODR_50HZ      0x08
#define VAL_CFG_REG_A_ODR_100HZ     0x0c
#define VAL_CFG_REG_A_MD_CONTINUOUS 0x00
#define VAL_CFG_REG_A_MD_SINGLE     0x01
#define VAL_CFG_REG_A_MD_IDLE       0x03

#define REG_CFG_REG_B         0x61
#define VAL_CFG_REG_B_LPF_OFF 0x00
#define VAL_CFG_REG_B_LPF_ON  0x01

#define REG_CFG_REG_C         0x62
#define VAL_CFG_REG_C_4WSPI   0x04
#define VAL_CFG_REG_C_I2C_DIS 0x20

#define REG_DATA_OUT       0x68
#define SENSOR_OUTPUT_SIZE 6

#define MAG_CONV_MGAUSS 1.5F  /* conversion from sensor value to milligauss (equal to 10^-7 T) */
#define MAG_OVERFLOW    21843 /* sensorhub will overflow if lis2mdl returns abs(raw) > MAG_OVERFLOW */


typedef struct {
	spimsg_ctx_t spiCtx;
	oid_t spiSS;
	sensor_event_t evt;
	char stack[512] __attribute__((aligned(8)));
} lis2mdl_ctx_t;


static int16_t translateMag(uint8_t hbyte, uint8_t lbyte)
{
	int16_t val;

	/* MISRA incompliant - casting u16 to s16 with no regard to sign */
	val = ((uint16_t)hbyte << 8) | (uint16_t)lbyte;

	if (val > MAG_OVERFLOW) {
		return MAG_OVERFLOW * MAG_CONV_MGAUSS;
	}
	else if (val < -MAG_OVERFLOW) {
		return -MAG_OVERFLOW * MAG_CONV_MGAUSS;
	}
	else {
		/* conversion to milligaus (10^-7 T), as it is already sensor manager storage scale */
		return val * MAG_CONV_MGAUSS;
	}
}


static int spiWriteReg(lis2mdl_ctx_t *ctx, uint8_t regAddr, uint8_t regVal)
{
	unsigned char cmd[2] = { regAddr, regVal };

	return sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, cmd, sizeof(cmd), NULL, 0, sizeof(cmd));
}


static int lis2mdl_whoamiCheck(lis2mdl_ctx_t *ctx)
{
	uint8_t cmd, val;
	int err;

	cmd = REG_WHOAMI | SPI_READ_BIT;
	val = 0;
	err = sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &cmd, sizeof(cmd), &val, sizeof(val), sizeof(cmd));
	if ((err < 0) || (val != VAL_WHOAMI)) {
		fprintf(stderr, "whoami %x %x\n", val, VAL_WHOAMI);
		return -1;
	}

	return 0;
}


static int lis2mdl_hwSetup(lis2mdl_ctx_t *ctx)
{
	/* LIS2MDL has 3-wire SPI on power-up. Enable 4-wire SPI */
	if (spiWriteReg(ctx, REG_CFG_REG_C, VAL_CFG_REG_C_I2C_DIS | VAL_CFG_REG_C_4WSPI) < 0) {
		return -1;
	}
	usleep(1000);

	if (lis2mdl_whoamiCheck(ctx) != 0) {
		printf("lis2mdl: cannot read/wrong WHOAMI returned!\n");
		return -1;
	}

	if (spiWriteReg(ctx, REG_CFG_REG_A, VAL_CFG_REG_A_ODR_100HZ | VAL_CFG_REG_A_MD_CONTINUOUS | VAL_CFG_REG_A_COMP_TEMP_EN) < 0) {
		return -1;
	}

	if (spiWriteReg(ctx, REG_CFG_REG_B, VAL_CFG_REG_B_LPF_ON) < 0) {
		return -1;
	}

	return 0;
}


static void lis2mdl_threadPublish(void *data)
{
	int err;
	sensor_info_t *info = (sensor_info_t *)data;
	lis2mdl_ctx_t *ctx = info->ctx;
	uint8_t ibuf[SENSOR_OUTPUT_SIZE] = { 0 };
	const uint8_t obuf = REG_DATA_OUT | SPI_READ_BIT;

	while (1) {
		usleep(100 * 1000);

		err = sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &obuf, sizeof(obuf), ibuf, sizeof(ibuf), sizeof(obuf));

		if (err >= 0) {
			gettime(&(ctx->evt.timestamp), NULL);
			ctx->evt.mag.magX = translateMag(ibuf[1], ibuf[0]);
			ctx->evt.mag.magY = -translateMag(ibuf[3], ibuf[2]); /* minus accounts for non right-handness of measurement */
			ctx->evt.mag.magZ = translateMag(ibuf[5], ibuf[4]);
			sensors_publish(info->id, &ctx->evt);
		}
	}
}


static int lis2mdl_start(sensor_info_t *info)
{
	int err;
	lis2mdl_ctx_t *ctx = (lis2mdl_ctx_t *)info->ctx;

	err = beginthread(lis2mdl_threadPublish, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("lis2mdl: launched sensor\n");
	}

	return err;
}


static int lis2mdl_alloc(sensor_info_t *info, const char *args)
{
	lis2mdl_ctx_t *ctx;
	char *ss;
	int err;

	/* sensor context allocation */
	ctx = malloc(sizeof(lis2mdl_ctx_t));
	if (ctx == NULL) {
		return -ENOMEM;
	}

	ctx->evt.type = SENSOR_TYPE_MAG;
	ctx->evt.accels.devId = info->id;

	/* filling sensor info structure */
	info->ctx = ctx;
	info->types = SENSOR_TYPE_MAG;

	ctx->spiCtx.mode = SPI_MODE3;
	ctx->spiCtx.speed = 10000000;

	ss = strchr(args, ':');
	if (ss != NULL) {
		*(ss++) = '\0';
	}

	/* initialize SPI device communication */
	err = sensorsspi_open(args, ss, &ctx->spiCtx.oid, &ctx->spiSS);
	if (err < 0) {
		printf("lis2mdl: Can`t initialize SPI device\n");
		free(ctx);
		return err;
	}

	/* hardware setup of imu */
	if (lis2mdl_hwSetup(ctx) < 0) {
		printf("lis2mdl: failed to setup device\n");
		free(ctx);
		return -1;
	}

	return EOK;
}


void __attribute__((constructor)) lis2mdl_mag_register(void)
{
	static sensor_drv_t sensor = {
		.name = "lis2mdl",
		.alloc = lis2mdl_alloc,
		.start = lis2mdl_start
	};

	sensors_register(&sensor);
}
