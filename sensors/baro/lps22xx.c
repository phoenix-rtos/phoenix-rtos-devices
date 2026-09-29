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
#define REG_WHOAMI         0x0f
#define LPS22HB_WHOAMI_VAL 0xB1
#define LPS22HH_WHOAMI_VAL 0xB3

/* control register 1 */
#define REG_CTRL_REG1 0x10
#define VAL_ODR_200   0x70
#define VAL_ODR_100   0x60
#define VAL_ODR_75    0x50
#define VAL_ODR_50    0x40
#define VAL_ODR_25    0x30
#define VAL_EN_LPFP   0x08
#define VAL_LPFP_CFG  0x04
#define VAL_BDU       0x02

/* control register 2 */
#define REG_CTRL_REG2            0x11
#define VAL_SWRESET              0x04
#define VAL_BOOT                 0x80
#define VAL_LOW_NOISE_EN         0x02
#define IF_ADD_INC               0x10
#define LPS22HB_REG_RES_CONF     0x1a
#define LPS22HB_VAL_LC_EN        0x01
#define LPS22HH_VAL_LOW_NOISE_EN 0x02

/* control register 3 */
#define REG_CTRL_REG3 0x12

/* data storage addresses */
#define REG_DATA_OUT 0x28
#define SPI_READ_BIT 0x80

/* sensors data sizes */
#define SENSOR_OUTPUT_SIZE 5

/* conversions */
#define SENSOR_CONV_CENTIPASCALS (1e4F / 4096.0F) /* conversion from sensor value to centipascals */
#define SENSOR_CONV_KELV         0.01F            /* conversion from sensor value to kelvin */
#define SENSOR_KELV_OFFS         273.15F          /* temperature measurement offset + kelvin to celsius offset */

#define LPS22XX_BOOT_DELAY_US   (10 * 1000)
#define LPS22XX_CONFIG_DELAY_US (100 * 1000)


typedef enum {
	lps22_type_unknown = 0,
	lps22_type_hb,
	lps22_type_hh
} lps22_type_t;


typedef struct {
	sensor_event_t evtBaro;
	char stack[512] __attribute__((aligned(8)));
} lps22xx_ctx_t;


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

	return (uint32_t)((float)val * SENSOR_CONV_CENTIPASCALS);
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


static int lps22xx_readReg(sensor_bus_t *bus, uint8_t regAddr, uint8_t *regVal)
{
	uint8_t cmd = regAddr | SPI_READ_BIT;

	return bus->ops.bus_xfer(bus, &cmd, sizeof(cmd), regVal, sizeof(*regVal), sizeof(cmd));
}


static int spiWriteReg(sensor_bus_t *bus, uint8_t regAddr, uint8_t regVal)
{
	unsigned char cmd[2] = { regAddr, regVal };

	return bus->ops.bus_xfer(bus, cmd, sizeof(cmd), NULL, 0, sizeof(cmd));
}


static lps22_type_t lps22xx_whoamiCheck(sensor_bus_t *bus)
{
	uint8_t val = 0;
	int err;

	err = lps22xx_readReg(bus, REG_WHOAMI, &val);
	if (err < 0) {
		fprintf(stderr, "lps22xx: WHOAMI read failed: %d\n", err);
		return lps22_type_unknown;
	}

	switch (val) {
		case LPS22HB_WHOAMI_VAL:
			return lps22_type_hb;

		case LPS22HH_WHOAMI_VAL:
			return lps22_type_hh;

		default:
			fprintf(stderr, "whoami: %x\n", (unsigned int)val);
			return lps22_type_unknown;
	}
}


static int lps22xx_hwSetup(sensor_bus_t *bus)
{
	lps22_type_t type;
	uint8_t val;
	int err, i;

	type = lps22xx_whoamiCheck(bus);
	if (type == lps22_type_unknown) {
		printf("lps22xx: WHOAMI fail\n");
		return -1;
	}

	if (spiWriteReg(bus, REG_CTRL_REG2, VAL_SWRESET) < 0) {
		return -1;
	}

	for (i = 0; i < 10; i++) {
		usleep(LPS22XX_BOOT_DELAY_US);
		err = lps22xx_readReg(bus, REG_CTRL_REG2, &val);

		if (err < 0) {
			return -1;
		}

		if ((val & VAL_SWRESET) == 0) {
			break;
		}
	}

	if (i >= 10) {
		printf("lps22xx: SWRESET timeout\n");
		return -1;
	}

	/* Configure low-noise mode while ODR is still powered down. */
	switch (type) {
		case lps22_type_hb:
			/*
			 * HB: CTRL_REG2 bit 1 is reserved.
			 * Enable address increment; leave FIFO disabled.
			 */
			if (spiWriteReg(bus, REG_CTRL_REG2, IF_ADD_INC) < 0) {
				return -1;
			}

			/*
			 * Low-noise mode requires LC_EN = 0.
			 * Preserve RES_CONF bit 1, which must not be modified.
			 */
			err = lps22xx_readReg(bus, LPS22HB_REG_RES_CONF, &val);
			if (err < 0) {
				return -1;
			}

			val &= (uint8_t)~LPS22HB_VAL_LC_EN;
			if (spiWriteReg(bus, LPS22HB_REG_RES_CONF, val) < 0) {
				return -1;
			}
			break;

		case lps22_type_hh:
			if (spiWriteReg(bus, REG_CTRL_REG2, IF_ADD_INC | LPS22HH_VAL_LOW_NOISE_EN) < 0) {
				return -1;
			}
			break;

		default:
			return -1;
	}
	usleep(LPS22XX_CONFIG_DELAY_US);

	/* Both variants: 75 Hz, BDU enabled, pressure LPF at ODR/9. */
	if (spiWriteReg(bus, REG_CTRL_REG1, VAL_ODR_75 | VAL_BDU | VAL_EN_LPFP) < 0) {
		return -1;
	}
	usleep(LPS22XX_CONFIG_DELAY_US);

	return 0;
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
