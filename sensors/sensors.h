/*
 * Phoenix-RTOS
 *
 * Sensor Manager
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <libsensors.h>

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

	idnode_t node;
} sensor_info_t;


/* Basic operations on sensors */
typedef struct {
	char name[16];                                       /* device name */
	int (*alloc)(sensor_info_t *info, const char *args); /* alloc sensor and initialize driver */
	int (*start)(sensor_info_t *info);                   /* start measurement thread */
	rbnode_t node;
} sensor_drv_t;


/* Function used by sensor drivers to publish sensor values */
extern int sensors_publish(unsigned int devId, const sensor_event_t *event);


/* Sensor's drivers constructor registers itself in a sensor manager */
extern void sensors_register(const sensor_drv_t *ops);


#endif
