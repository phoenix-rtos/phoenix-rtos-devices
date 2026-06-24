/*
 * Phoenix-RTOS
 *
 * Driver for barometer lsp22xx
 *
 * Copyright 2026 Phoenix Systems
 * Author: Mateusz Niewiadomski
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

#include <libsensors/sensor.h>
#include <libsensors/bus.h>

/* self identification register */
#define REG_WHOAMI        0x0f
#define REG_VAL_WHOAMI_HH 0xb3
#define REG_VAL_WHOAMI_DF 0xb4

/* control register 1 - LPS22HH */
#define HH_REG_CTRL_REG1 0x10
#define HH_VAL_ODR_200   0x70
#define HH_VAL_ODR_100   0x60
#define HH_VAL_ODR_75    0x50
#define HH_VAL_ODR_50    0x40
#define HH_VAL_ODR_25    0x30
#define HH_VAL_EN_LPFP   0x08
#define HH_VAL_LPFP_CFG  0x04
#define HH_VAL_BDU       0x02

/* control register 2 - LPS22HH */
#define HH_REG_CTRL_REG2    0x11
#define HH_VAL_BOOT         0x80
#define HH_VAL_LOW_NOISE_EN 0x02
#define HH_IF_ADD_INC       0x10

/* control register 3 - LPS22HH */
#define HH_REG_CTRL_REG3 0x12

/* control register 1 - LPS22DF */
#define DF_REG_CTRL_REG1 0x10
#define DF_VAL_ODR_200   0x40
#define DF_VAL_ODR_100   0x38
#define DF_VAL_ODR_75    0x30
#define DF_VAL_ODR_50    0x28
#define DF_VAL_ODR_25    0x20
#define DF_VAL_AVG_4     0x00
#define DF_VAL_AVG_8     0x01
#define DF_VAL_AVG_16    0x02
#define DF_VAL_AVG_32    0x03
#define DF_VAL_AVG_64    0x04
#define DF_VAL_AVG_128   0x06
#define DF_VAL_AVG_512   0x07


/* control register 2 - LPS22DF */
#define DF_REG_CTRL_REG2 0x11
#define DF_VAL_BOOT      0x80
#define DF_VAL_LPFP_CFG  0x20
#define DF_VAL_EN_LPFP   0x10
#define DF_VAL_BDU       0x08

/* control register 3 - LPS22DF */
#define DF_REG_CTRL_REG3 0x12
#define DF_IF_ADD_INC    0x01


/* data storage addresses */
#define REG_DATA_OUT 0x28
#define SPI_READ_BIT 0x80

/* sensors data sizes */
#define SENSOR_OUTPUT_SIZE 5

/* conversions */
#define SENSOR_CONV_PASCALS 0.024414062F /* conversion from sensor value to pascals; = (100 * 1/4096) */
#define SENSOR_CONV_KELV    0.01F        /* conversion from sensor value to kelvin */
#define SENSOR_KELV_OFFS    273.15F      /* temperature measurement offset + kelvin to celsius offset */

#define LPS22XX_BOOT_DELAY_US   (10 * 1000)
#define LPS22XX_CONFIG_DELAY_US (100 * 1000)


typedef struct {
	sensor_event_t evtBaro;
	char stack[512] __attribute__((aligned(8)));
} lps22xx_ctx_t;


static struct {
	int whoAmI;
} common = {
	.whoAmI = -1
};


static uint32_t translatePress(uint8_t lbyte, uint8_t mbyte, uint8_t hbyte)
{
	/**
	 * lps22xx pressure measurement is stored (in chip)
	 * as signed integer as it can have negative value
	 * if RPDS_L/RPDS_H registers are set.
	 *
	 * This driver does not set RPDS_L/RPDS_H so the reading
	 * will be positive all the time.
	 */
	uint32_t val = 0;

	val = lbyte | (mbyte << 8) | (hbyte << 16);

	return (uint32_t)((float)val * SENSOR_CONV_PASCALS);
}


static uint32_t translateTemp(uint8_t hbyte, uint8_t lbyte)
{
	/**
	 * lps22xx temperature is stored as signed integer
	 * as it is in degrees Celsius.
	 */
	int16_t val;

	val = lbyte | (hbyte << 8);

	return (uint32_t)((float)val * SENSOR_CONV_KELV + SENSOR_KELV_OFFS);
}


static int spiWriteReg(sensor_bus_t *bus, uint8_t regAddr, uint8_t regVal)
{
	unsigned char cmd[2] = { regAddr, regVal };

	return bus->ops.bus_xfer(bus, cmd, sizeof(cmd), NULL, 0, sizeof(cmd));
}


static int lps22xx_whoamiCheck(sensor_bus_t *bus)
{
	uint8_t cmd, val;
	int err;

	cmd = REG_WHOAMI | SPI_READ_BIT;
	val = 0;
	err = bus->ops.bus_xfer(bus, &cmd, sizeof(cmd), &val, sizeof(val), sizeof(cmd));
	if (err < 0) {
		fprintf(stderr, "whoami xfer: %d\n", err);
		return -1;
	}

	switch (val) {
		case REG_VAL_WHOAMI_HH:
		case REG_VAL_WHOAMI_DF:
			common.whoAmI = val;
			break;

		default:
			fprintf(stderr, "whoami: %x\n", val);
			return -1;
	}

	return 0;
}


static int lps22x_setupHH(sensor_bus_t *bus)
{
	/* Boot process: refresh the content of the internal registers stored in the flash memory block */
	if (spiWriteReg(bus, HH_REG_CTRL_REG2, HH_VAL_BOOT) < 0) {
		return -1;
	}
	usleep(LPS22XX_BOOT_DELAY_US); /* The boot process takes 2.2 msec. Waiting more for safety */

	if (spiWriteReg(bus, HH_REG_CTRL_REG1, (HH_VAL_ODR_75 | HH_VAL_BDU | HH_VAL_EN_LPFP)) < 0) {
		return -1;
	}
	usleep(LPS22XX_CONFIG_DELAY_US); /* Arbitrary wait */

	if (spiWriteReg(bus, HH_REG_CTRL_REG2, (HH_IF_ADD_INC | HH_VAL_LOW_NOISE_EN)) < 0) {
		return -1;
	}
	usleep(LPS22XX_CONFIG_DELAY_US); /* Arbitrary wait */

	return 0;
}


static int lps22x_setupDF(sensor_bus_t *bus)
{
	/* Boot process: refresh the content of the internal registers stored in the flash memory block */
	if (spiWriteReg(bus, DF_REG_CTRL_REG2, DF_VAL_BOOT) < 0) {
		return -1;
	}
	usleep(LPS22XX_BOOT_DELAY_US); /* The boot process takes 2.2 msec. Waiting more for safety */

	if (spiWriteReg(bus, DF_REG_CTRL_REG1, (DF_VAL_ODR_75 | DF_VAL_AVG_128)) < 0) {
		return -1;
	}
	usleep(LPS22XX_CONFIG_DELAY_US); /* Arbitrary wait */

	if (spiWriteReg(bus, DF_REG_CTRL_REG2, (DF_VAL_EN_LPFP | DF_VAL_BDU)) < 0) {
		return -1;
	}
	usleep(LPS22XX_CONFIG_DELAY_US); /* Arbitrary wait */

	if (spiWriteReg(bus, DF_REG_CTRL_REG3, DF_IF_ADD_INC) < 0) {
		return -1;
	}
	usleep(LPS22XX_CONFIG_DELAY_US); /* Arbitrary wait */

	return 0;
}


static int lps22xx_hwSetup(sensor_bus_t *bus)
{
	if (lps22xx_whoamiCheck(bus) != 0) {
		printf("lps22xx: cannot read/wrong WHOAMI returned!\n");
		return -1;
	}

	switch (common.whoAmI) {
		case REG_VAL_WHOAMI_HH:
			return lps22x_setupHH(bus);

		case REG_VAL_WHOAMI_DF:
			return lps22x_setupDF(bus);

		default:
			return -1;
	}

	return -1;
}


static int lps22xx_read(const sensor_info_t *info, const sensor_event_t **evt)
{
	int err;
	lps22xx_ctx_t *ctx = info->ctx;
	uint8_t ibuf[SENSOR_OUTPUT_SIZE] = { 0 };
	uint8_t obuf;

	obuf = REG_DATA_OUT | SPI_READ_BIT;
	err = info->bus.ops.bus_xfer(&info->bus, &obuf, sizeof(obuf), ibuf, sizeof(ibuf), sizeof(obuf));

	if (err < 0) {
		return -1;
	}

	gettime(&(ctx->evtBaro.timestamp), NULL);
	ctx->evtBaro.baro.pressure = translatePress(ibuf[0], ibuf[1], ibuf[2]);
	ctx->evtBaro.baro.temp = translateTemp(ibuf[4], ibuf[3]);

	if (evt != NULL) {
		*evt = &ctx->evtBaro;
	}

	return 1;
}


static void lps22xx_publishthr(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	lps22xx_ctx_t *ctx = info->ctx;

	while (1) {
		/* Almost 75 Hz */
		usleep(15 * 1000);

		if (lps22xx_read(info, NULL) < 0) {
			continue;
		}

		sensors_publish(info->id, &ctx->evtBaro);
	}
}


static int lps22xx_start(sensor_info_t *info)
{
	int err;
	lps22xx_ctx_t *ctx = (lps22xx_ctx_t *)info->ctx;

	err = beginthread(lps22xx_publishthr, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("lps22xx: launched sensor\n");
	}

	return err;
}


static int lps22xx_dealloc(sensor_info_t *info)
{
	free((lps22xx_ctx_t *)info->ctx);

	return 0;
}


static int lps22xx_alloc(sensor_info_t *info, const char *args)
{
	lps22xx_ctx_t *ctx;
	char *ss;
	int err;

	/* sensor context allocation */
	ctx = malloc(sizeof(lps22xx_ctx_t));
	if (ctx == NULL) {
		return -ENOMEM;
	}

	ctx->evtBaro.type = SENSOR_TYPE_BARO;
	ctx->evtBaro.baro.devId = info->id;

	/* filling sensor info structure */
	info->ctx = ctx;
	info->types = SENSOR_TYPE_BARO;

	ss = strchr(args, ':');
	if (ss != NULL) {
		*(ss++) = '\0';
	}

	err = sensor_bus_genericSpiSetup(&info->bus, args, ss, (int)1e7, SPI_MODE3);
	if (err < 0) {
		printf("lps22xx: failed spi setup: %d\n", err);
	}

	/* hardware setup of Barometer */
	if (lps22xx_hwSetup(&info->bus) < 0) {
		printf("lps22xx: failed to setup device\n");
		free(ctx);
		return -1;
	}

	return EOK;
}


void __attribute__((constructor)) lps22xx_register(void)
{
	static sensor_drv_t sensor = {
		.name = "lps22xx",
		.alloc = lps22xx_alloc,
		.dealloc = lps22xx_dealloc,
		.start = lps22xx_start,
		.read = lps22xx_read
	};

	sensors_register(&sensor);
}
