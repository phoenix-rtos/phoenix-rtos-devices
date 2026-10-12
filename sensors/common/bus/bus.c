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

#include <libsensors/bus.h>


extern int sensor_bus_genericSpiSetup(sensor_bus_t *bus, const char *spiDev, const char *ss, int speed, char mode)
{
	if ((bus == NULL) || (bus->bustype != bus_spi)) {
		return -1;
	}

	if (bus->ops.bus_open(bus, spiDev, ss) < 0) {
		return -2;
	}

	if (bus->ops.bus_cfg(bus, speed, mode) < 0) {
		bus->ops.bus_close(bus);
		return -3;
	}

	return 0;
}


extern int sensor_bus_genericUartSetup(sensor_bus_t *bus, const char *uartDev, int baudrate, enum sensor_uart_mode mode)
{
	if ((bus == NULL) || (bus->bustype != bus_uart)) {
		return -1;
	}

	if (bus->ops.bus_open(bus, uartDev, NULL) < 0) {
		return -2;
	}

	if (bus->ops.bus_cfg(bus, baudrate, (char)mode) < 0) {
		bus->ops.bus_close(bus);
		return -3;
	}

	return 0;
}
