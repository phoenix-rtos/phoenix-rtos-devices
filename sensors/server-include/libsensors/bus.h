/*
 * Phoenix-RTOS
 *
 * Sensor bus operands and management
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef SENSOR_BUS_H
#define SENSOR_BUS_H

#include <stddef.h>


/**
 * Forward declarations of implementation-defined context structures.
 * These has to be defined by library user.
 */
struct sensor_busctx_spi_t;
struct sensor_busctx_i2c_t;


enum sensor_bus_type { bus_spi,
	bus_i2c };


typedef struct _sensor_bus_t {
	enum sensor_bus_type bustype; /* bus type identifier for the device for correct initialisation procedure */
	struct {
		/**
		 * bus transaction
		 *
		 * out - output data buffer
		 * olen - length of output data buffer
		 * in - input data buffer
		 * ilen - input data buffer length
		 * iskip - data skip size during read
		 *
		 * Returns 0 on success, -1 on failure.
		 */
		int (*bus_xfer)(const struct _sensor_bus_t *bus, const void *out, size_t olen, void *in, size_t ilen, size_t iskip);

		/**
		 * bus configuration function.
		 *
		 * For SPI this sets the clock speed in Hz and SPI mode character.
		 * For I2C this only sets speed in Hz, mode is unused.
		 *
		 * Returns 0 on success, -1 on bus donfiguration failure.
		 */
		int (*bus_cfg)(struct _sensor_bus_t *bus, int speed, char mode);
		int (*bus_close)(struct _sensor_bus_t *bus);

		/**
		 * bus open function accepts dev/subdev parameters.
		 *
		 * For SPI these are the name, or virtual path to spi device (e.g "/dev/spi1"), and subdev is the `ss` number.
		 * For I2C these are the name, or virtual path to i2c device (e.g "/dev/i2c1"), and subdev is unused.
		 */
		int (*bus_open)(struct _sensor_bus_t *bus, const char *dev, const char *subdev);
	} ops;
	union {
		struct sensor_busctx_spi_t *spi;
		struct sensor_busctx_i2c_t *i2c;
	} ctx;
} sensor_bus_t;


/**
 * Generic SPI setup wrapper
 *
 * Uses operands from `bus` to open bus and configure speed and mode.
 * More exotic setup scenario may self-implement this wrapper if needed.
 *
 * Returns:
 * 0 - on success.
 * (-1) - if bus type is not SPI.
 * (-2) - bus open fail.
 * (-3) - bus configure fail
 */
extern int sensor_bus_genericSpiSetup(sensor_bus_t *bus, const char *spiDev, const char *ss, int speed, char mode);


#endif
