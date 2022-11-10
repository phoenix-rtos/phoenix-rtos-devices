/*
 * Phoenix-RTOS
 *
 * Driver for ms5611 (barometer & temperature sensor)
 *
 * Copyright 2022 Phoenix Systems
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
#include <sensors-spi.h>

#include "../sensors.h"

/* MS5611 COMMANDS */

/* Reset command */
#define CMD_RESET 0x1e

/* Conversion request command and oversampling (OSR) rate flags */
#define CMD_CVRT_PRESS 0x40
#define CMD_CVRT_TEMP  0x50
#define CVRT_OSR_256   0x00
#define CVRT_OSR_512   0x02
#define CVRT_OSR_1024  0x04
#define CVRT_OSR_2048  0x06
#define CVRT_OSR_4096  0x08

/* Read ADC conversion result command */
#define CMD_READ_ADC 0x00

/* Read PROM command and PROM address specifiers */
#define CMD_READ_PROM 0xa0
#define PROM_ADDR_C1  0x02 /* C1: Pressure sensitivity */
#define PROM_ADDR_C2  0x04 /* C2: Pressure offset */
#define PROM_ADDR_C3  0x06 /* C3: Temperature coefficient of pressure sensitivity */
#define PROM_ADDR_C4  0x08 /* C4: Temperature coefficient of pressure offset */
#define PROM_ADDR_C5  0x0A /* C5: Reference temperature */
#define PROM_ADDR_C6  0x0C /* C6: Temperature coefficient of the temperature */

/* MS5611 DELAYS */

/* OSR dependent minimum wait (microseconds) for conversion. Datasheet values ceil-ed to full milliseconds */
#define OSR_256_SLEEP  1000
#define OSR_512_SLEEP  2000
#define OSR_1024_SLEEP 3000
#define OSR_2048_SLEEP 5000
#define OSR_4096_SLEEP 10000

/* Device reset duration */
#define RESET_SLEEP 3000 /* Datasheet says 2800. It seems some data may be lost with such sleep */


typedef struct {
	spimsg_ctx_t spiCtx;
	oid_t spiSS;
	sensor_event_t evtBaro;
	uint16_t promC[7]; /* Prom memory. Assigning one too many indexes to use 1-6 indexing as in datasheet */
	char stack[512] __attribute__((aligned(8)));
} ms5611_ctx_t;


static void ms5611_publishthr(void *data)
{
	/* TODO: data acquisition thread and calculations based on PROM coefficients */

	return;
}


static int ms5611_hwSetup(ms5611_ctx_t *ctx)
{
	static const uint8_t promAddr[6] = { PROM_ADDR_C1, PROM_ADDR_C2, PROM_ADDR_C3, PROM_ADDR_C4, PROM_ADDR_C5, PROM_ADDR_C6 };

	unsigned int i;
	uint8_t cmd;

	/* Reset ms5611 sequence */
	cmd = CMD_RESET;
	if (sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &cmd, sizeof(cmd), NULL, 0, 0) < 0) {
		fprintf(stderr, "ms5611: failed to reset device\n");
		return -1;
	}
	usleep(RESET_SLEEP);

	/* PROM reading */
	for (i = 0; i < sizeof(promAddr); i++) {
		cmd = CMD_READ_PROM | promAddr[i];
		/* data received from MS5611 is written to incremented index in ctx to match datasheet (so that C1 corresponds to promC[1] and so on) */
		if (sensorsspi_xfer(&ctx->spiCtx, &ctx->spiSS, &cmd, sizeof(cmd), &ctx->promC[i + 1], sizeof(ctx->promC[i + 1]), sizeof(cmd)) < 0) {
			fprintf(stderr, "ms5611: failed read PROM C%i\n", i + 1);
			return -1;
		}
		ctx->promC[i + 1] = (ctx->promC[i + 1] << 8) | (ctx->promC[i + 1] >> 8); /* swap higher and lower bytes */
	}

	return 0;
}


static int ms5611_start(sensor_info_t *info)
{
	int err;
	ms5611_ctx_t *ctx = (ms5611_ctx_t *)info->ctx;

	err = beginthread(ms5611_publishthr, 4, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf("ms5611: launched sensor\n");
	}

	return err;
}


static int ms5611_alloc(sensor_info_t *info, const char *args)
{
	ms5611_ctx_t *ctx;
	char *ss;
	int err;

	/* sensor context allocation */
	ctx = malloc(sizeof(ms5611_ctx_t));
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
		printf("ms5611: Can`t initialize SPI device\n");
		free(ctx);
		return err;
	}

	/* hardware setup of Barometer */
	if (ms5611_hwSetup(ctx) < 0) {
		printf("ms5611: failed to setup device\n");
		free(ctx);
		return -1;
	}

	return EOK;
}


void __attribute__((constructor)) ms5611_register(void)
{
	static sensor_drv_t sensor = {
		.name = "ms5611",
		.alloc = ms5611_alloc,
		.start = ms5611_start
	};

	sensors_register(&sensor);
}
