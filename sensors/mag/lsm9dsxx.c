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
#include <string.h>

#include <libsensors/sensor.h>
#include <libsensors/bus.h>

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
#define MAG4G_CONV_MGAUSS 0.14F /* conversion from sensor (at +-4 gauss scale) value to milligauss (equal to 10^-7 T) */


typedef struct {
	sensor_event_t evt;
	char stack[512] __attribute__((aligned(8)));
} lsm9dsxx_ctx_t;


static int16_t translateMag(uint8_t hbyte, uint8_t lbyte)
{
	int16_t val;

	val = (hbyte << 8) | lbyte;

	/* conversion to milligaus (10^-7 T), as it is already sensor manager storage scale */
	return val * MAG4G_CONV_MGAUSS;
}


static int spiWriteReg(sensor_bus_t *bus, uint8_t regAddr, uint8_t regVal)
{
	unsigned char cmd[2] = { regAddr, regVal };

	return bus->ops.bus_xfer(bus, cmd, sizeof(cmd), NULL, 0, sizeof(cmd));
}


static int lsm9dsxx_whoamiCheck(sensor_bus_t *bus)
{
	uint8_t cmd, val;
	int err;

	cmd = REG_WHOAMI | SPI_READ_BIT;
	val = 0;
	err = bus->ops.bus_xfer(bus, &cmd, sizeof(cmd), &val, sizeof(val), sizeof(cmd));
	if ((err < 0) || (val != REG_VAL_WHOAMI)) {
		return -1;
	}

	return 0;
}


static int lsm9dsxx_hwSetup(sensor_bus_t *bus)
{
	if (lsm9dsxx_whoamiCheck(bus) != 0) {
		printf("lsm9dsxx_mag: cannot read/wrong WHOAMI returned!\n");
		return -1;
	}

	/* output data rate to 80Hz + high performance mode to X and Y axis */
	if (spiWriteReg(bus, REG_CTRL_REG1_M, (VAL_CTRL_REG1_M_XYHIGH_PERF_MODE | VAL_CTRL_REG1_M_ODR_80)) < 0) {
		return -1;
	}
	/* scale +-4 gauss (same as default, written just for clarity) */
	if (spiWriteReg(bus, REG_CTRL_REG2_M, VAL_CTRL_REG2_M_FS_4) < 0) {
		return -1;
	}
	/* high performance mode for Z axis */
	if (spiWriteReg(bus, REG_CTRL_REG4_M, VAL_CTRL_REG4_M_ZHIGH_PERF_MODE) < 0) {
		return -1;
	}
	usleep(10 * 1000);

	/* continuous measurement mode */
	if (spiWriteReg(bus, REG_CTRL_REG3_M, (VAL_CTRL_REG3_M_MD_CONTINUOUS)) < 0) {
		return -1;
	}
	usleep(100 * 1000);

	return 0;
}


static int lsm9dsxx_read(const sensor_info_t *info, const sensor_event_t **evt)
{
	int err;
	lsm9dsxx_ctx_t *ctx = info->ctx;
	uint8_t ibuf[SENSOR_OUTPUT_SIZE] = { 0 };
	uint8_t obuf;

	obuf = REG_DATA_OUT | SPI_READ_BIT | SPI_AUTOADDRINCR_BIT;
	err = info->bus.ops.bus_xfer(&info->bus, &obuf, sizeof(obuf), ibuf, sizeof(ibuf), sizeof(obuf));

	if (err < 0) {
		return -1;
	}

	gettime(&(ctx->evt.timestamp), NULL);
	ctx->evt.mag.magX = translateMag(ibuf[1], ibuf[0]);
	ctx->evt.mag.magY = translateMag(ibuf[3], ibuf[2]);
	ctx->evt.mag.magZ = translateMag(ibuf[5], ibuf[4]);

	if (evt != NULL) {
		*evt = &ctx->evt;
	}

	return 1;
}


static void lsm9dsxx_threadPublish(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	lsm9dsxx_ctx_t *ctx = info->ctx;

	while (1) {
		usleep(1000 * 1000 / 64);

		if (lsm9dsxx_read(info, NULL) < 0) {
			continue;
		}

		sensors_publish(info->id, &ctx->evt);
	}
}


static int lsm9dsxx_start(sensor_info_t *info)
{
	int err;
	lsm9dsxx_ctx_t *ctx = (lsm9dsxx_ctx_t *)info->ctx;

	err = beginthread(lsm9dsxx_threadPublish, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("lsm9dsxx_mag: launched sensor\n");
	}

	return err;
}


static int lsm9dsxx_dealloc(sensor_info_t *info)
{
	free((lsm9dsxx_ctx_t *)info->ctx);

	return 0;
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

	ctx->evt.type = SENSOR_TYPE_MAG;
	ctx->evt.accels.devId = info->id;

	/* filling sensor info structure */
	info->ctx = ctx;
	info->types = SENSOR_TYPE_MAG;

	ss = strchr(args, ':');
	if (ss != NULL) {
		*(ss++) = '\0';
	}

	err = sensor_bus_genericSpiSetup(&info->bus, args, ss, (int)10e7, SPI_MODE3);
	if (err < 0) {
		printf("lsm9dsxx_mag: failed spi setup: %d\n", err);
	}

	/* hardware setup of imu */
	if (lsm9dsxx_hwSetup(&info->bus) < 0) {
		printf("lsm9dsxx_mag: failed to setup device\n");
		info->bus.ops.bus_close(&info->bus);
		free(ctx);
		return -1;
	}

	return EOK;
}


void __attribute__((constructor)) lsm9dsxx_mag_register(void)
{
	static sensor_drv_t sensor = {
		.name = "lsm9dsxx_mag",
		.alloc = lsm9dsxx_alloc,
		.dealloc = lsm9dsxx_dealloc,
		.start = lsm9dsxx_start,
		.read = lsm9dsxx_read
	};

	sensors_register(&sensor);
}
