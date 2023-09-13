/*
 * Phoenix-RTOS
 *
 * NCT75 i2c Temperature Sensor driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Marek Bialowas
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <sys/msg.h>
#include <posix/utils.h>

#include <i2c.h>


static const int32_t INVALID_TEMP = -1000 * 1000;

static const uint8_t DEV_ADDR = 0x48u;      /* NCT75 */
static const uint8_t TEMP_REG_ADDR = 0x00u; /* stored temperature value - 16bit */


/* returns temperature in miliCelsius */
static int32_t getTemp(void)
{
	uint8_t temp_raw[2];
	int16_t temp_enc;

	if (i2c_regRead(DEV_ADDR, TEMP_REG_ADDR, temp_raw, sizeof(temp_raw)) < 0)
		return INVALID_TEMP;

	/* 12-bit left-adjusted, 2s complement, every bit is 0.0625 C */
	temp_enc = (temp_raw[0] << 8) | temp_raw[1];
	return (temp_enc >> 5) * 125;
}


static void thread(void *arg)
{
	uint32_t port = (uint32_t)arg;
	msg_t msg;
	msg_rid_t rid;
	int err;

	for (;;) {
		if ((err = msgRecv(port, &msg, &rid)) < 0) {
			printf("i2c: msgRecv returned error: %s\n", strerror(-err));
			if (err == -EINTR)
				continue;
			else
				break; /* EINVAL (invalid/closed port) or ENOMEM - fatal error */
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;
			case mtWrite:
				msg.o.io.err = -EINVAL;
				break;
			case mtRead:
				/* don't support partial reads, signal EOF */
				if (msg.i.io.offs > 0) {
					msg.o.io.err = 0; /* EOF */
				}
				else {
					msg.o.io.err = snprintf(msg.o.data, msg.o.size, "%d\n", getTemp());
				}
				break;
			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void print_usage(const char *progname)
{
	printf("Usage: %s [i2c_bus_no <1,4>] [temp_device_no]\n", progname);
}


int main(int argc, char **argv)
{
	unsigned int i2c_bus_no, dev_no;
	uint32_t port;
	oid_t dev;
	char devname[sizeof("tempX")];

	if (argc != 3) {
		print_usage(argv[0]);
		return 1;
	}

	i2c_bus_no = atoi(argv[1]);
	if (i2c_init(i2c_bus_no) < 0) {
		printf("temp: i2c initialization failed\n");
		return 1;
	}

	dev_no = atoi(argv[2]);
	if (portCreate(&port) != EOK)
		return 2;

	dev.port = port;
	dev.id = 0;

	snprintf(devname, sizeof(devname), "temp%u", dev_no % 10);
	if (create_dev(&dev, devname) < 0) {
		printf("temp: could not create device\n");
		return 3;
	}

	puts("temp: initialized");
	thread((void *)port);

	printf("temp: exiting\n");
	return 0;
}
