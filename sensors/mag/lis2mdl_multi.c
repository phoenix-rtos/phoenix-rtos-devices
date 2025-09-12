/*
 * Phoenix-RTOS
 *
 * Driver for lis2mdl_multi magnetometer
 *
 * Copyright 2023, 2025 Phoenix Systems
 * Author: Mateusz Niewiadomski, Maciej Latocha
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
#include <libsensors/spi/spi.h>
#include <sys/msg.h>
#include <libmulti/libspi.h>
#include <stm32l4-multi.h>

#define DRIVER_NAME "lis2mdl_multi"

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

#define REG_HARD_OFFSETS 0x45
#define REG_DATA_OUT     0x68

#define MAG_CONV_MGAUSS 1.5F  /* conversion from sensor value to milligauss (equal to 10^-7 T) */
#define MAG_OVERFLOW    21843 /* sensorhub will overflow if lis2mdl returns abs(raw) > MAG_OVERFLOW */


typedef struct {
	spimsg_ctx_t spiCtx;
	oid_t spiSS;
	sensor_event_t evt;
	char stack[512] __attribute__((aligned(8)));
} lis2mdl_ctx_t;


static void gpioSet(oid_t dev, char port, char pin, int value)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = dev
	};
	multi_i_t *imsg = (multi_i_t *)msg.i.raw;
	imsg->type = gpio_set;
	imsg->gpio_set.port = port;
	imsg->gpio_set.mask = 1 << pin;
	imsg->gpio_set.state = (value & 0x1) << pin;

	msgSend(dev.port, &msg);
}


static int spiRead(oid_t dev, int spi, unsigned int addr, uint8_t *buf, size_t size)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = dev
	};
	multi_i_t *imsg = (multi_i_t *)msg.i.raw;
	msg.o.size = size;
	msg.o.data = buf;

	imsg->type = spi_get;
	imsg->spi_rw.spi = spi;
	imsg->spi_rw.addr = addr | SPI_READ_BIT;
	imsg->spi_rw.flags = 1 << SPI_ADDRSHIFT;
	imsg->spi_rw.cmd = spi_cmd;

	msgSend(dev.port, &msg);
	return msg.o.err;
}

static int spiWrite(oid_t dev, int spi, uint8_t addr, uint8_t buf)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = dev
	};
	multi_i_t *imsg = (multi_i_t *)msg.i.raw;
	msg.i.size = 1;
	msg.i.data = &buf;

	imsg->type = spi_set;
	imsg->spi_rw.spi = spi;
	imsg->spi_rw.addr = addr;
	imsg->spi_rw.flags = 1 << SPI_ADDRSHIFT;
	imsg->spi_rw.cmd = spi_cmd;

	msgSend(dev.port, &msg);
	return msg.o.err;
}


static int spiConfig(oid_t dev, char spi, char mode, char bdiv, int enable)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = dev
	};
	multi_i_t *imsg = (multi_i_t *)msg.i.raw;

	imsg->type = spi_def;
	imsg->spi_def.spi = spi;
	imsg->spi_def.mode = mode;
	imsg->spi_def.bdiv = bdiv;
	imsg->spi_def.enable = enable;

	msgSend(dev.port, &msg);
	return msg.o.err;
}

static int gpioSetAltMode(oid_t oid, int gpio, char pin)
{
	msg_t msg = {
		.type = mtDevCtl,
		.oid = oid,
	};
	multi_i_t *i = (multi_i_t *)msg.i.raw;
	i->type = gpio_def;
	i->gpio_def.port = gpio;
	i->gpio_def.mode = 2;
	i->gpio_def.pin = pin;
	i->gpio_def.af = 5;
	i->gpio_def.ospeed = gpio_ospeed_low;
	int err = msgSend(oid.port, &msg);
	if (err < 0 || msg.o.err) {
		printf(DRIVER_NAME ": failed to set alt mode for gpio %u\n", (uint32_t)pin);
		return err;
	}
	return 0;
}

static int32_t clamp_s32(int32_t v, int32_t min, int32_t max)
{
	return (v < min) ? min : (v > max) ? max :
										 v;
}

static void lis2mdl_threadPublish(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	lis2mdl_ctx_t *ctx = info->ctx;
	const oid_t device = ctx->spiSS;

	int16_t offsets[3];
	int16_t read[3];

	while (1) {
		usleep(100000);

		int err = spiRead(device, spi5, REG_HARD_OFFSETS, (uint8_t *)offsets, sizeof(offsets));
		if (err < 0) {
			printf(DRIVER_NAME ": failed to read sensors offsets\n");
			continue;
		}

		err = spiRead(device, spi5, REG_DATA_OUT, (uint8_t *)read, sizeof(read));
		if (err < 0) {
			printf(DRIVER_NAME ": failed to read sensors data\n");
			continue;
		}

		read[0] -= offsets[0];
		read[1] -= offsets[1];
		read[2] -= offsets[2];
		ctx->evt.mag.magX = clamp_s32(read[0] * MAG_CONV_MGAUSS, -MAG_OVERFLOW, MAG_OVERFLOW);
		ctx->evt.mag.magY = clamp_s32(read[1] * MAG_CONV_MGAUSS, -MAG_OVERFLOW, MAG_OVERFLOW);
		ctx->evt.mag.magZ = clamp_s32(read[2] * MAG_CONV_MGAUSS, -MAG_OVERFLOW, MAG_OVERFLOW);
		gettime(&(ctx->evt.timestamp), NULL);
		sensors_publish(info->id, &ctx->evt);

		// printf( DRIVER_NAME ": x %d\t\ty %d\t\tz %d\n", ctx->evt.mag.magX, ctx->evt.mag.magY, ctx->evt.mag.magZ );
	}
}

static int lis2mdl_start(sensor_info_t *info)
{
	int err;
	lis2mdl_ctx_t *ctx = (lis2mdl_ctx_t *)info->ctx;

	err = beginthread(lis2mdl_threadPublish, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info);
	if (err >= 0) {
		printf(DRIVER_NAME ": launched sensor\n");
	}

	return err;
}

static int lis2mdl_alloc(sensor_info_t *info, const char *args)
{
	lis2mdl_ctx_t *ctx = malloc(sizeof(lis2mdl_ctx_t));
	if (!ctx) {
		return -ENOMEM;
	}

	info->ctx = ctx;
	info->types = SENSOR_TYPE_MAG;
	int err = -1;
	const int tries = 15;
	for (int i = tries; i-- && err < 0;) {
		usleep(100000);
		err = lookup("/dev/multi", NULL, &ctx->spiSS);
	}

	const oid_t device = ctx->spiSS;

	if (err < 0) {
		printf(DRIVER_NAME ": failed to open multi after %d tries\n", tries);
		return err;
	}

	gpioSetAltMode(device, gpioa, 3);
	gpioSetAltMode(device, gpioe, 15);
	gpioSetAltMode(device, gpiog, 1);
	gpioSetAltMode(device, gpiog, 2);

	const char csPort = gpioa, csPin = 3;
	int res = spiConfig(device, spi5, SPI_MODE3, spi_bdiv_128, 1);
	if (res < 0) {
		printf(DRIVER_NAME ": failed to config SPI 5\n");
		gpioSet(device, csPort, csPin, 1);
		return res;
	}

	gpioSet(device, csPort, csPin, 0);

	uint8_t val = VAL_CFG_REG_C_I2C_DIS | VAL_CFG_REG_C_4WSPI;
	err = spiWrite(device, spi5, REG_CFG_REG_C, val);
	if (err < 0) {
		printf(DRIVER_NAME ": failed to set REG_C flags\n");
		return err;
	}

	uint8_t whoami = 0;
	err = spiRead(device, spi5, REG_WHOAMI, &whoami, 1);
	if (err < 0) {
		printf(DRIVER_NAME ": failed to read whoami\n");
		return err;
	}
	if (whoami != VAL_WHOAMI) {
		printf(DRIVER_NAME ": whoami value mismatch, expected %u, got %u\n", VAL_WHOAMI, whoami);
		return -EINVAL;
	}

	val = VAL_CFG_REG_A_ODR_100HZ | VAL_CFG_REG_A_MD_CONTINUOUS | VAL_CFG_REG_A_COMP_TEMP_EN;
	err = spiWrite(device, spi5, REG_CFG_REG_A, val);
	if (err < 0) {
		printf(DRIVER_NAME ": failed to set REG_A flags\n");
		return err;
	}

	err = spiWrite(device, spi5, REG_CFG_REG_B, VAL_CFG_REG_B_LPF_ON);
	if (err < 0) {
		printf(DRIVER_NAME ": failed to enable low pass filter\n");
		return err;
	}

	return EOK;
}

void __attribute__((constructor)) lis2mdl_mag_register(void)
{
	static sensor_drv_t sensor = {
		.name = DRIVER_NAME,
		.alloc = lis2mdl_alloc,
		.start = lis2mdl_start
	};

	sensors_register(&sensor);
}
