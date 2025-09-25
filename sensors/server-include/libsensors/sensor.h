/*
 * Phoenix-RTOS
 *
 * Sensor Manager
 *
 * Copyright 2022-2025 Phoenix Systems
 * Author: Hubert Buczynski, Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _LIBSENSORS_SENSOR_H_
#define _LIBSENSORS_SENSOR_H_

#include <libsensors/client.h>
#include <libsensors/bus.h>

#include <sys/rb.h>
#include <posix/idtree.h>


#define THREAD_PRIORITY_MSGSRV 3 /* priority of messaging server of sensorhub */
#define THREAD_PRIORITY_SENSOR 3 /* priority of sensor driver threads */


/* Info structure is filled by a sensor driver */
typedef struct {
	unsigned int id;     /* device identifier */
	sensor_type_t types; /* supported sensor types */
	const char *drv;     /* device driver name */
	void *ctx;           /* internal context for a specific sensor */
	sensor_bus_t bus;    /* bus operands of that sensor */

	idnode_t node;
} sensor_info_t;


/* Basic operations on sensors */
typedef struct {
	char name[16];                                       /* device name */
	int (*alloc)(sensor_info_t *info, const char *args); /* alloc sensor and initialize driver */
	int (*dealloc)(sensor_info_t *info);                 /* deallocate sensor */
	int (*start)(sensor_info_t *info);                   /* start measurement thread */

	/**
	 * Reads data from the device. Pointer to read data is stored in evt.
	 * Returns number of read data samples, or -1 on error.
	 */
	int (*read)(const sensor_info_t *info, const sensor_event_t **evt);

	rbnode_t node;
} sensor_drv_t;


/* Function used by sensor drivers to publish sensor values */
extern int sensors_publish(unsigned int devId, const sensor_event_t *event);


/* Sensor's drivers constructor registers itself in a sensor manager */
extern void sensors_register(const sensor_drv_t *ops);


#endif
