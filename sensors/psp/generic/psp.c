/*
 * Phoenix-RTOS
 *
 * generic/stub platform support package (PSP)
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <libsensors/sensor.h>
#include <libsensors/bus.h>


void bsp_busDealloc(struct _sensor_bus_t *bus)
{
	return;
}


int psp_busAlloc(sensor_bus_t *bus, enum sensor_bus_type type)
{
	return -1;
}


void psp_busDone(void)
{
	return;
}


int psp_busInit(void)
{
	return -1;
}
