/*
 * Phoenix-RTOS
 *
 * IMU I2C driver - main logic
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include <sys/msg.h>
#include <posix/utils.h>
#include <sys/threads.h>

#include "communication.h"

struct data_context_t {
	const imu_dev_t *imu;
	const imu_dev_t *mag;

	float imu_data[7];
	float mag_data[3];

	handle_t data_lock;
};

struct data_context_t data_context;

void daq(void *arg)
{
	int i = 0;
	for (;;) {
		/* TODO: add variable sampling frequency */
		for (i = 0; i < 10; i++) {
			mutexLock(data_context.data_lock);
			getAllData(data_context.imu, data_context.imu_data, sizeof(data_context.imu_data) / sizeof(float));
			mutexUnlock(data_context.data_lock);
			usleep(1000 * 2);
		}
		mutexLock(data_context.data_lock);
		getAllData(data_context.mag, data_context.mag_data, sizeof(data_context.mag_data) / sizeof(float));
		mutexUnlock(data_context.data_lock);
		usleep(1000);
	}

	endthread();
}

static void server(void *arg)
{
	uint32_t port = (uint32_t)arg;
	msg_t msg;
	unsigned long int rid;
	int err;
	unsigned int i, i_mag;
	float *val = (float *)msg.o.raw;

	memset(&msg, 0, sizeof(msg));

	for (;;) {
		if ((err = msgRecv(port, &msg, &rid)) < 0) {
			printf("imu: msgRecv returned error: %s\n", strerror(-err));
			if (err == -EINTR)
				continue;
			else
				break; /* EINVAL (invalid/closed port) or ENOMEM - fatal error */
		}

		switch (msg.type) {
			case mtRead:
				i = i_mag = 0;
				mutexLock(data_context.data_lock);
				for (; i < sizeof(data_context.imu_data) / sizeof(float); i++) {
					val[i] = data_context.imu_data[i];
				}
				for (; i_mag < sizeof(data_context.mag_data); i++, i_mag++) {
					val[i] = data_context.mag_data[i_mag];
				}
				mutexUnlock(data_context.data_lock);
				break;
			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}

int main(int argc, char **argv)
{
	const size_t stacksz = 1024;
	int bus_no = 2;
	char devname[] = "imu_driver", *stack;
	uint32_t port;
	oid_t dev;

	/* by default init bus 2 */
	if (argc >= 2) {
		bus_no = atoi(argv[1]);
	}
	/* i2c init wrapper */
	if (initialize_i2cbus(bus_no) != 0) {
		printf("%s: cannot initialize I2C bus!\n", devname);
		return 1;
	}
	/* Find and init IMU */
	data_context.imu = probeDevices(type_imu);
	if (data_context.imu == NULL) {
		printf("%s: IMU not found!\n", devname);
		return 2;
	}
	if (initDevice(data_context.imu) < 0) {
		return 2;
	}

	/* Find and init magnetometer */
	data_context.mag = probeDevices(type_magmeter);
	if (data_context.mag == NULL) {
		printf("%s: magnetometer not found!\n", devname);
		return 2;
	}
	initDevice(data_context.mag);

	/* Prepare port */
	if (portCreate(&port) != EOK) {
		return 3;
	}
	dev.port = port;
	dev.id = 0;
	printf("%s: running on port %u\n", devname, port);
	if (create_dev(&dev, devname) < 0) {
		printf("%s: could not create device\n", devname);
		return 4;
	}

	/* prepare and start data acquisition thread */
	mutexCreate(&data_context.data_lock);
	if ((stack = malloc(stacksz)) == NULL) {
		printf("%s: cannot allocate memory for DAQ!\n", devname);
		return 1;
	}
	beginthread(daq, 4, stack, stacksz, (void *)&data_context);

	/* start messaging server */
	server((void *)port);

	return 0;
}
