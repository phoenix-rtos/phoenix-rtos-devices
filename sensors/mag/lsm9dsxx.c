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
#define REG_VAL_WHOAMI 0x3D

/* control register 1 */
#define REG_CTRL_REG1_M                  0x20
#define VAL_CTRL_REG1_M_XYHIGH_PERF_MODE 0x60
#define VAL_CTRL_REG1_M_ODR_80           0x1c
#define VAL_CTRL_REG1_M_FAST_ODR         0x02
#define VAL_CTRL_REG1_M_SELF_TEST        0x01

/* control register 2 */
#define REG_CTRL_REG2_M       0x21
#define MASK_CTRL_REG2_M_FS   0x60
#define VAL_CTRL_REG2_M_FS_4  0x00
#define VAL_CTRL_REG2_M_FS_8  0x20
#define VAL_CTRL_REG2_M_FS_12 0x40
#define VAL_CTRL_REG2_M_FS_16 0x60

/* control register 3 */
#define REG_CTRL_REG3_M               0x22
#define MASK_CTRL_REG2_M_MD           0x03
#define VAL_CTRL_REG3_M_MD_CONTINUOUS 0x00
#define VAL_CTRL_REG3_M_MD_SINGLE     0x01
#define VAL_CTRL_REG3_M_MD_PWR_DOWN   0x03
#define VAL_CTRL_REG3_M_SIM_SPIRW     0x04
#define VAL_CTRL_REG3_M_I2CDSBL       0x80

/* control register 4 */
#define REG_CTRL_REG4_M                 0x23
#define VAL_CTRL_REG4_M_ZHIGH_PERF_MODE 0x08

/* status registers */
#define REG_STATUS_REG_M       0x27
#define VAL_STATUS_REG_M_ZYXDA 0x08

/* data storage addresses */
#define REG_DATA_OUT         0x28
#define SPI_READ_BIT         0x80
#define SPI_AUTOADDRINCR_BIT 0x40

/* sensors data sizes */
#define SENSOR_OUTPUT_SIZE 6

/* conversions */
#define MAG4G_CONV_MGAUSS 0.14F        /* conversion from sensor (at +-4 gauss scale) value to milligauss */
#define MGAUSS_2_UTSL     1.221730475F /* milligauss to microtesla convertion parameter */
#define UTSL_2_MAGSHUB    10.F         /* microtesla to sensorhub magnetic flux storage scale of 10^-7 T */


typedef struct {
	spimsg_ctx_t spiCtx;
	sensor_event_t evt;
	char stack[512] __attribute__((aligned(8)));
} lsm9dsxxMag_ctx_t;


static int16_t translateMag(uint8_t hbyte, uint8_t lbyte)
{
	int16_t val = 0;
	val |= hbyte;
	val <<= 8;
	val |= lbyte;
	return val * MAG4G_CONV_MGAUSS * MGAUSS_2_UTSL * UTSL_2_MAGSHUB; /* value -> milligauss -> microtesla -> sensorhub storage scale*/
}


static int spiWriteReg(spimsg_ctx_t *spiCtx, uint8_t regAddr, uint8_t regVal)
{
	unsigned char cmd[2] = { (regAddr & 0x7F), regVal }; /* write bit set to regAddr */

	return spimsg_xfer(spiCtx, cmd, sizeof(cmd), NULL, 0, sizeof(cmd));
}


static int lsm9dsxxMag_whoamiCheck(spimsg_ctx_t *spiCtx)
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


static int lsm9dsxxMag_hwSetup(spimsg_ctx_t *spiCtx)
{
	if (lsm9dsxxMag_whoamiCheck(spiCtx) != 0) {
		printf("lsm9dsxxMag: cannot read/wrong WHOAMI returned!\n");
		return -1;
	}

	/* output data rate to 80Hz + high performance mode to X and Y axis */
	if (spiWriteReg(spiCtx, REG_CTRL_REG1_M, (VAL_CTRL_REG1_M_XYHIGH_PERF_MODE | VAL_CTRL_REG1_M_ODR_80)) < 0) {
		return -1;
	}
	/* scale +-4 gauss (same as default, written just for clarity) */
	if (spiWriteReg(spiCtx, REG_CTRL_REG2_M, VAL_CTRL_REG2_M_FS_4) < 0) {
		return -1;
	}
	/* high performance mode for Z axis */
	if (spiWriteReg(spiCtx, REG_CTRL_REG4_M, VAL_CTRL_REG4_M_ZHIGH_PERF_MODE) < 0) {
		return -1;
	}
	usleep(10 * 1000);

	/* continuous measurement mode */
	if (spiWriteReg(spiCtx, REG_CTRL_REG3_M, (VAL_CTRL_REG3_M_MD_CONTINUOUS | VAL_CTRL_REG3_M_SIM_SPIRW | VAL_CTRL_REG3_M_I2CDSBL)) < 0) {
		return -1;
	}
	usleep(100 * 1000);

	return 0;
}


static void lsm9dsxx_threadPublish(void *data)
{
	int err;
	time_t tstamp;
	sensor_info_t *info = (sensor_info_t *)data;
	lsm9dsxxMag_ctx_t *ctx = info->ctx;
	spimsg_ctx_t *spiCtx = &ctx->spiCtx;
	uint8_t ibuf[SENSOR_OUTPUT_SIZE] = { 0 };
	uint8_t obuf;

	while (1) {
		usleep(12500); /* 80Hz sampling period */

		obuf = REG_DATA_OUT | SPI_READ_BIT | SPI_AUTOADDRINCR_BIT;
		err = spimsg_xfer(spiCtx, &obuf, 1, ibuf, SENSOR_OUTPUT_SIZE, 1);
		gettime(&tstamp, NULL);

		if (err >= 0) {
			/* minus accounts for non right-handness of lsm9dsxx accelerometer frame of reference */
			ctx->evt.mag.magX = translateMag(ibuf[1], ibuf[0]);
			ctx->evt.mag.magY = translateMag(ibuf[3], ibuf[2]);
			ctx->evt.mag.magZ = translateMag(ibuf[5], ibuf[4]);
			ctx->evt.timestamp = tstamp;
			/* TEST */
			printf("%d %d %d\n", ctx->evt.mag.magX, ctx->evt.mag.magY, ctx->evt.mag.magZ);
			sensors_publish(info->id, &ctx->evt);
		}
	}
}


static int lsm9dsxxMag_start(sensor_info_t *info)
{
	int err;
	lsm9dsxxMag_ctx_t *ctx = (lsm9dsxxMag_ctx_t *)info->ctx;

	err = beginthread(lsm9dsxx_threadPublish, 4, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("lsm9dsxxMag: launched sensor\n");
	}

	return err;
}


static int lsm9dsxxMag_alloc(sensor_info_t *info, const char *args)
{
	lsm9dsxxMag_ctx_t *ctx;
	int ntries = 10;

	/* sensor context allocation */
	ctx = malloc(sizeof(lsm9dsxxMag_ctx_t));
	if (ctx == NULL) {
		return -ENOMEM;
	}

	ctx->evt.type = SENSOR_TYPE_MAG;
	ctx->evt.accels.devId = info->id;

	/* filling sensor info structure */
	info->ctx = ctx;
	info->types = SENSOR_TYPE_MAG;

	ctx->spiCtx.mode = SPI_MODE3;
	ctx->spiCtx.speed = 6250000;

	/* open SPI context */
	while (lookup(args, NULL, &ctx->spiCtx.oid) < 0) {
		ntries--;
		if (ntries == 0) {
			printf("lsm9dsxxMag: Can`t open SPI device\n");
			free(ctx);
			return -ETIMEDOUT;
		}
		usleep(10 * 1000);
	}

	/* hardware setup of imu */
	if (lsm9dsxxMag_hwSetup(&ctx->spiCtx) < 0) {
		printf("lsm9dsxxMag: failed to setup device\n");
		spimsg_close(&ctx->spiCtx);
		free(ctx);
		return -1;
	}

	return EOK;
}


void __attribute__((constructor)) lsm9dsxxMag_register(void)
{
	static sensor_drv_t sensor = {
		.name = "lsm9dsxxMag",
		.alloc = lsm9dsxxMag_alloc,
		.start = lsm9dsxxMag_start
	};

	sensors_register(&sensor);
}
