/*
 * Phoenix-RTOS
 *
 * Driver for IMU lps25xx (accelerometer, gyroscope)
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/threads.h>
#include <spi.h>
#include <sensors-spi.h>

#include "../sensors.h"

/* self identification register */
#define REG_WHOAMI     0x0f
#define REG_VAL_WHOAMI 0xbd

/* resolution configuration register */
#define REG_RES_CONF 0x10
#define VAL_AVGT_8   0x00
#define VAL_AVGT_16  0x04
#define VAL_AVGT_32  0x08
#define VAL_AVGT_64  0x0c
#define VAL_AVGP_8   0x00
#define VAL_AVGP_32  0x01
#define VAL_AVGP_128 0x02
#define VAL_AVGP_512 0x03

/* control register 1 */
#define REG_CTRL_REG1      0x20
#define VAL_PD_ACTIVE_MODE 0x80
#define VAL_ODR_ONESHOT    0x00
#define VAL_ODR_1HZ        0x10
#define VAL_ODR_7HZ        0x20
#define VAL_ODR_12HZ       0x30
#define VAL_ODR_25HZ       0x40

/* control register 2 */
#define REG_CTRL_REG2 0x21
#define VAL_BOOT      0x80

/* data storage addresses */
#define REG_DATA_OUT         0x28
#define SPI_READ_BIT         0x80
#define SPI_AUTOADDRINCR_BIT 0x40

/* sensors data sizes */
#define SENSOR_OUTPUT_SIZE 5

/* conversions */
#define SENSOR_CONV_PASCALS 0.024414062F /* conversion from sensor value to pascals; = (100 * 1/4096) */
#define SENSOR_CONV_KELV    0.002083333F /*conversion from sensor value to kelvin */
#define SENSOR_KELV_OFFS    315.65F      /* temperature measurement offset + kelvin to celsius offset */


typedef struct {
	spimsg_ctx_t spiCtx;
	oid_t spiSS;
	sensor_event_t evtBaro;
	char stack[512] __attribute__((aligned(8)));
} lps25xx_ctx_t;


static uint32_t translatePress(uint8_t lbyte, uint8_t mbyte, uint8_t hbyte)
{
	uint32_t val;

	val = lbyte | (mbyte << 8) | (hbyte << 16);
	/* dismiss highest bit as it is unusable */
	val &= 0x7FFFFF;

	return val * SENSOR_CONV_PASCALS;
}


static uint32_t translateTemp(uint8_t hbyte, uint8_t lbyte)
{
	int16_t val;

	val = lbyte | (hbyte << 8);

	return (uint32_t)((float)val * SENSOR_CONV_KELV + SENSOR_KELV_OFFS);
}


static int spiWriteReg(lps25xx_ctx_t *ctx, uint8_t regAddr, uint8_t regVal)
{
	unsigned char cmd[2] = { regAddr, regVal };

	return sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, cmd, sizeof(cmd), NULL, 0, sizeof(cmd));
}


static int lps25xx_whoamiCheck(lps25xx_ctx_t *ctx)
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


static int lps25xx_hwSetup(lps25xx_ctx_t *ctx)
{
	if (lps25xx_whoamiCheck(ctx) != 0) {
		printf("lps25xx: cannot read/wrong WHOAMI returned!\n");
		return -1;
	}

	/* Boot process: refresh the content of the internal registers stored in the flash memory block */
	if (spiWriteReg(ctx, REG_CTRL_REG2, VAL_BOOT) < 0) {
		return -1;
	}
	usleep(1000 * 10); /* The boot process takes 2.2 msec. Waiting more for safety */

	/* Activate the device (VAL_PD_ACTIVE_MODE), and set output data rate (ODR) to highest */
	if (spiWriteReg(ctx, REG_CTRL_REG1, (VAL_PD_ACTIVE_MODE | VAL_ODR_25HZ)) < 0) {
		return -1;
	}
	usleep(1000 * 100); /* Arbitrary wait */

	/* Internal averaging configuration. Lowest noise settings */
	if (spiWriteReg(ctx, REG_RES_CONF, VAL_AVGT_64 | VAL_AVGP_512) < 0) {
		return -1;
	}
	usleep(1000 * 100); /* Arbitrary wait */

	return 0;
}


static void lps25xx_publishthr(void *data)
{
	int err;
	sensor_info_t *info = (sensor_info_t *)data;
	lps25xx_ctx_t *ctx = info->ctx;
	uint8_t ibuf[SENSOR_OUTPUT_SIZE] = { 0 };
	uint8_t obuf;

	while (1) {
		/* ODR set to 25Hz */
		usleep(40 * 1000);

		obuf = REG_DATA_OUT | SPI_READ_BIT | SPI_AUTOADDRINCR_BIT;
		err = sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &obuf, sizeof(obuf), ibuf, sizeof(ibuf), sizeof(obuf));

		if (err >= 0) {
			gettime(&(ctx->evtBaro.timestamp), NULL);
			ctx->evtBaro.baro.pressure = translatePress(ibuf[0], ibuf[1], ibuf[2]);
			ctx->evtBaro.baro.temp = translateTemp(ibuf[4], ibuf[3]);

			sensors_publish(info->id, &ctx->evtBaro);
		}
	}
}


static int lps25xx_start(sensor_info_t *info)
{
	int err;
	lps25xx_ctx_t *ctx = (lps25xx_ctx_t *)info->ctx;

	err = beginthread(lps25xx_publishthr, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("lps25xx: launched sensor\n");
	}

	return err;
}


static int lps25xx_alloc(sensor_info_t *info, const char *args)
{
	lps25xx_ctx_t *ctx;
	char *ss;
	int err;

	/* sensor context allocation */
	ctx = malloc(sizeof(lps25xx_ctx_t));
	if (ctx == NULL) {
		return -ENOMEM;
	}

	ctx->evtBaro.type = SENSOR_TYPE_BARO;
	ctx->evtBaro.baro.devId = info->id;

	/* filling sensor info structure */
	info->ctx = ctx;
	info->types = SENSOR_TYPE_BARO;

	ctx->spiCtx.mode = SPI_MODE3;
	ctx->spiCtx.speed = 10000000;

	ss = strchr(args, ':');
	if (ss != NULL) {
		*(ss++) = '\0';
	}

	/* initialize SPI device communication */
	err = sensorsspi_open(args, ss, &ctx->spiCtx.oid, &ctx->spiSS);
	if (err < 0) {
		printf("lps25xx: Can`t initialize SPI device\n");
		free(ctx);
		return err;
	}

	/* hardware setup of Barometer */
	if (lps25xx_hwSetup(ctx) < 0) {
		printf("lps25xx: failed to setup device\n");
		free(ctx);
		return -1;
	}

	return EOK;
}


void __attribute__((constructor)) lps25xx_register(void)
{
	static sensor_drv_t sensor = {
		.name = "lps25xx",
		.alloc = lps25xx_alloc,
		.start = lps25xx_start
	};

	sensors_register(&sensor);
}
