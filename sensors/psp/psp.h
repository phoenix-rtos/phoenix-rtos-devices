

#ifndef _SENSORHUB_PSP_H_
#define _SENSORHUB_PSP_H_

#include <libsensors/sensor.h>
#include <libsensors/bus.h>


void bsp_busDealloc(sensor_bus_t *bus);


int psp_busAlloc(sensor_bus_t *bus, enum sensor_bus_type type);


void psp_busDone(void);


int psp_busInit(void);


#endif
