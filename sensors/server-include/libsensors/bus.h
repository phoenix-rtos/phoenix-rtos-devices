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
struct sensor_busctx_uart_t;


enum sensor_bus_type { bus_spi,
	bus_i2c,
	bus_uart };


/**
 * UART frame format used as `mode` in `bus_cfg`.
 * Values fit in `char`.
 * The most common format (8N1) yields when initialized with "0" by accident.
 */
enum sensor_uart_mode {
	bus_uart_8n1 = 0,
	bus_uart_8n2,
	bus_uart_8e1,
	bus_uart_8e2,
	bus_uart_8o1,
	bus_uart_8o2,
	bus_uart_7e1,
	bus_uart_7o1,
};


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
		 * Returns non-negative value on success, negative on failure.
		 * For SPI and I2C returns 0 on success.
		 * For UART returns number of bytes stored in `in`.
		 */
		int (*bus_xfer)(const struct _sensor_bus_t *bus, const void *out, size_t olen, void *in, size_t ilen, size_t iskip);

		/**
		 * bus configuration function.
		 *
		 * For SPI this sets the clock speed in Hz and SPI mode character.
		 * For I2C this only sets speed in Hz, mode is unused.
		 * For UART:
		 *  - speed sets baudrate (numeric value, not `speed_t`)
		 *  - mode is frame format as `enum sensor_uart_mode` value
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
		 * For UART these are the name, or virtual path to uart/tty device, and subdev is unused.
		 */
		int (*bus_open)(struct _sensor_bus_t *bus, const char *dev, const char *subdev);
	} ops;
	union {
		struct sensor_busctx_spi_t *spi;
		struct sensor_busctx_i2c_t *i2c;
		struct sensor_busctx_uart_t *uart;
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


/**
 * Generic UART setup wrapper
 *
 * Uses operands from `bus` to open bus and configure baudrate and frame format.
 * More exotic setup scenario may self-implement this wrapper if needed.
 *
 * Returns:
 * 0 - on success.
 * (-1) - if bus type is not UART.
 * (-2) - bus open fail.
 * (-3) - bus configure fail
 */
extern int sensor_bus_genericUartSetup(sensor_bus_t *bus, const char *uartDev, int baudrate, enum sensor_uart_mode mode);


#endif
