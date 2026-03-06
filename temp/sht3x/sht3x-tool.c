/*
 * Phoenix-RTOS
 *
 * SHT3x driver tool
 *
 * Copyright 2026 Phoenix Systems
 * Author: Michal Woyke
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <sys/msg.h>

#include "sht3x-api.h"

#define STATUS_BIT_ALERT   (1 << 15)
#define STATUS_BIT_HEATER  (1 << 13)
#define STATUS_BIT_RH      (1 << 11)
#define STATUS_BIT_TEMP    (1 << 10)
#define STATUS_BIT_RESET   (1 << 4)
#define STATUS_BIT_CMD     (1 << 1)
#define STATUS_BIT_CHCKSUM (1 << 0)

#define IS_BIT_SET(_val, bit) (((_val) & (bit)) != 0)


static int measConfig(uint32_t port, uint8_t repeatability, uint8_t freq)
{
	msg_t msg = { 0 };

	msg.type = mtDevCtl;
	msg.oid.port = port;

	sht3x_i_devctl_t ctl = {
		.type = sht3x_typeMeasConfig,
		.measConfig = {
			.repeatability = repeatability,
			.freq = freq,
		},
	};

	memcpy(msg.i.raw, &ctl, sizeof(sht3x_i_devctl_t));

	if (msgSend(port, &msg) < 0) {
		puts("Failed to send msg\n");
		return -1;
	}

	if (msg.o.err < 0) {
		printf("Failed to config device: %s\n", strerror(msg.o.err));
		return -1;
	}

	return 0;
}


static int sendCmd(uint32_t port, uint16_t cmd)
{
	msg_t msg = { 0 };
	sht3x_i_devctl_t ctl = { 0 };

	msg.type = mtDevCtl;
	msg.oid.port = port;
	ctl.type = sht3x_typeCmd;
	ctl.cmd = cmd;
	memcpy(msg.i.raw, &ctl, sizeof(sht3x_i_devctl_t));

	if (msgSend(port, &msg) < 0) {
		puts("Failed to send msg\n");
		return -1;
	}

	if (msg.o.err < 0) {
		printf("Failed to send 0x%x cmd: %s\n", cmd, strerror(msg.o.err));
		return -1;
	}

	return 0;
}


static int getStatus(uint32_t port, uint16_t *status)
{
	msg_t msg = { 0 };
	sht3x_i_devctl_t ctl = { 0 };

	uint8_t data[3];

	msg.type = mtDevCtl;
	msg.oid.port = port;

	ctl.type = sht3x_typeCmd;
	ctl.cmd = sht3x_cmdGetStatus;
	memcpy(msg.i.raw, &ctl, sizeof(sht3x_i_devctl_t));

	msg.o.size = sizeof(data);
	msg.o.data = data;

	if (msgSend(port, &msg) < 0) {
		puts("Failed to send msg\n");
		return -1;
	}

	if (msg.o.err < 0) {
		printf("Failed to get status: %s\n", strerror(msg.o.err));
		return -1;
	}

	*status = data[0] << 8 | data[1];

	return 0;
}


static int readDev(uint32_t port, char *data, size_t size)
{
	msg_t msg = { 0 };

	msg.type = mtRead;
	msg.oid.port = port;
	msg.o.size = size;
	msg.o.data = data;

	if (msgSend(port, &msg) < 0) {
		puts("Failed to send msg\n");
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
	printf("Usage: %s dev_path [OPTIONS]\n"
		   "\t-h             This help message\n"
		   "\t-r             Reset device\n"
		   "\t-c <R>,<F>     Configure periodic data acquisition mode\n"
		   "\t   <R> = repeatability id <0,2>\n"
		   "\t   <F> = frequency id <0,4>\n"
		   "\t-b             Stop periodic data acquisition mode\n"
		   "\t-l             Clear status\n"
		   "\t-n             Turn on heater\n"
		   "\t-f             Turn off heater\n"
		   "\t-g             Read measurements\n"
		   "\t-s             Read status\n",
			progname);
}


int main(int argc, char **argv)
{
	oid_t oid;

	if (argc < 2 || getopt(2, argv, "h") == 'h') {
		usage(argv[0]);
		return 1;
	}

	if (lookup(argv[1], NULL, &oid) < 0) {
		printf("Device %s not found\n", argv[1]);
		usage(argv[0]);
		return 1;
	}

	while (true) {
		int c = getopt(argc - 1, &argv[1], "hbfc:gnlrs");
		if (c == -1) {
			break;
		}

		switch (c) {
			case 'h':
				usage(argv[0]);
				return 0;

			case 'r':
				(void)sendCmd(oid.port, sht3x_cmdSoftReset);
				break;

			case 'c': {
				/* repeatability, frequency */
				char *endptr, *curropt = optarg;

				int repeatability = strtoul(curropt, &endptr, 0);
				if (*endptr != ',') {
					puts("Invalid subfield format (1)\n");
					break;
				}
				curropt = endptr + 1; /* skip ',' */

				int freq = strtoul(curropt, &endptr, 0);
				if (*endptr != '\0') {
					puts("Invalid subfield format (2)\n");
					break;
				}

				(void)measConfig(oid.port, repeatability, freq);
			} break;

			case 'b':
				(void)sendCmd(oid.port, sht3x_cmdBreak);
				break;

			case 'l':
				(void)sendCmd(oid.port, sht3x_cmdClearStatus);
				break;

			case 'n':
				(void)sendCmd(oid.port, sht3x_cmdHeaterOn);
				break;

			case 'f':
				(void)sendCmd(oid.port, sht3x_cmdHeaterOff);
				break;

			case 'g': {
				char str[32];
				if (readDev(oid.port, str, sizeof(str)) < 0) {
					break;
				}

				char *val = strtok(str, ",");
				printf("Temperature: %s\n", val);

				val = strtok(NULL, "\n");
				printf("Relative Humidity: %s\n", val);
			} break;

			case 's': {
				uint16_t status = 0;

				if (getStatus(oid.port, &status) < 0) {
					break;
				}
				printf("Status: %x\n"
					   "Alert %d | Heater %d | Arh %d | At %d | Reset %d | Acmd %d | Achksum %d\n",
						status,
						IS_BIT_SET(status, STATUS_BIT_ALERT),
						IS_BIT_SET(status, STATUS_BIT_HEATER),
						IS_BIT_SET(status, STATUS_BIT_RH),
						IS_BIT_SET(status, STATUS_BIT_TEMP),
						IS_BIT_SET(status, STATUS_BIT_RESET),
						IS_BIT_SET(status, STATUS_BIT_CMD),
						IS_BIT_SET(status, STATUS_BIT_CHCKSUM));
			} break;

			default:
				usage(argv[0]);
				return 1;
		}
	}

	return 0;
}
