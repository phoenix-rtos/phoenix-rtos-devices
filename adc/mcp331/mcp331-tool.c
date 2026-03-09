/*
 * Phoenix-RTOS
 *
 * MCP33131/21/11-XX driver tool
 *
 * Copyright 2026 Phoenix Systems
 * Author: Michal Woyke
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/msg.h>

#include "mcp331-api.h"


static int recalibrate(uint32_t port)
{
	msg_t msg = { 0 };
	mcp331_i_devctl_t ioctl = { 0 };

	msg.type = mtDevCtl;
	msg.oid.port = port;
	ioctl.cmd = mcp331_cmd__recalibrate;
	memcpy(msg.i.raw, &ioctl, sizeof(mcp331_i_devctl_t));

	if (msgSend(port, &msg) < 0) {
		puts("Failed to send msg");
		return -1;
	}

	if (msg.o.err < 0) {
		printf("Failed to recalibrate: %s\n", strerror(msg.o.err));
		return -1;
	}

	return 0;
}


static int readDev(uint32_t port, uint8_t *data, size_t size)
{
	msg_t msg = { 0 };

	msg.type = mtRead;
	msg.oid.port = port;
	msg.o.size = size;
	msg.o.data = data;

	if (msgSend(port, &msg) < 0) {
		puts("Failed to send msg");
		return -1;
	}

	if (msg.o.err < 0) {
		printf("Failed to read: %s\n", strerror(msg.o.err));
		return -1;
	}

	return 0;
}


static void usage(const char *progname)
{
	printf("Usage: %s [OPTIONS] dev_path\n"
		   "\t-h             This help message\n"
		   "\t-r             Recalibrate\n"
		   "\t-g             Get sample\n",
			progname);
}


int main(int argc, char **argv)
{
	oid_t oid;

	bool doSample = false, doRecalibrate = false;
	while (true) {
		int c = getopt(argc, argv, "grh");
		if (c == -1) {
			break;
		}

		switch (c) {
			case 'g':
				doSample = true;
				break;

			case 'r':
				doRecalibrate = true;
				break;

			case 'h':
				usage(argv[0]);
				return 0;

			default:
				usage(argv[0]);
				return 1;
		}
	}

	if (argc != optind + 1) {
		usage(argv[0]);
		return 2;
	}

	if (lookup(argv[optind], NULL, &oid) < 0) {
		printf("%s not found\n", argv[optind]);
		return 3;
	}

	if (doRecalibrate) {
		recalibrate(oid.port);
	}

	if (doSample) {
		uint8_t data[2];
		if (readDev(oid.port, data, sizeof(data)) == 0) {
			printf("Voltage: %d\n", ((uint16_t)data[0] << 8) | data[1]);
		}
	}

	return 0;
}
