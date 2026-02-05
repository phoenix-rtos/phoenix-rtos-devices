/*
 * Phoenix-RTOS
 *
 * AD7779 driver tool
 *
 * Copyright 2023 Phoenix Systems
 * Author: Marek Bialowas
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <sys/msg.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include "sht31-api.h"

#define DEV_PATH "/dev/sht31-0"

static struct {
	int verbose;
} tool_common;


#define log_verbose(fmt, ...) \
	do { \
		if (tool_common.verbose > 0) \
			printf(fmt, ##__VA_ARGS__); \
	} while (0)


static int sht31Config(uint32_t port, uint8_t repeatability, uint8_t freq)
{
	msg_t msg = { 0 };

	msg.type = mtDevCtl;
	msg.oid.port = port;

	sht31_i_devctl_t ioctl = {
		.type = sht31_typeMeasConfig,
		.measConfig = {
			.repeatability = repeatability,
			.freq = freq,
		},
	};

	memcpy(msg.i.raw, &ioctl, sizeof(sht31_i_devctl_t));

	log_verbose("config\n");
	if ((msgSend(port, &msg) < 0) || (msg.o.err != 0)) {
		printf("sht31-tool: failed to config device: %d\n", msg.o.err);
		return 1;
	}

	return 0;
}


static int sht31sendCmd(uint32_t port, uint16_t cmd)
{
	msg_t msg = { 0 };
	sht31_i_devctl_t ioctl = { 0 };

	msg.type = mtDevCtl;
	msg.oid.port = port;
	ioctl.type = sht31_typeCmd;
	ioctl.cmd = cmd;
	memcpy(msg.i.raw, &ioctl, sizeof(sht31_i_devctl_t));

	if ((msgSend(port, &msg) < 0) || (msg.o.err != 0)) {
		printf("sht31-tool: failed to send %u cmd: %d\n", cmd, msg.o.err);
		return 1;
	}

	return 0;
}


static int sht31GetStatus(uint32_t port, uint16_t *status)
{
	msg_t msg = { 0 };
	sht31_i_devctl_t ioctl = { 0 };

	uint8_t data[3];

	msg.type = mtDevCtl;
	msg.oid.port = port;

	ioctl.type = sht31_typeCmd;
	ioctl.cmd = sht31_cmdGetStatus;
	memcpy(msg.i.raw, &ioctl, sizeof(sht31_i_devctl_t));

	msg.o.size = 3;
	msg.o.data = data;

	log_verbose("status\n");
	if ((msgSend(port, &msg) < 0) || (msg.o.err != 0)) {
		printf("sht31-tool: failed to reset device: %d\n", msg.o.err);
		return 1;
	}

	*status = data[0] << 8 | data[1];

	return 0;
}

static uint32_t sht31Read(uint32_t port, char *data, size_t size)
{
	msg_t msg = { 0 };

	msg.type = mtRead;
	msg.oid.port = port;
	msg.o.size = size;
	msg.o.data = data;

	if ((msgSend(port, &msg) < 0) || (msg.o.err != 0)) {
		printf("MSG failed to read adc intr: %s", strerror(msg.o.err));
		return -1;
	}
	printf("Data: %s", data);


	return 0;
}


static void print_usage(const char *progname)
{
	printf("Usage: %s [OPTIONS]\n", progname);
	puts("\t-h             This help message");
	puts("\t-v             Increase verbosity");
	puts("\t-r             Reset ADC");
	puts("\t-c             Show current configuration");
	puts("\t-s             Estimate current samplerate");
	puts("\t-a             Estimate average signal per channel");
	puts("\t-i [int_cnt]   Limit reading samples to [int_cnt] interrupts (default: infinite until ^C)");
	puts("\t-d [file.pcm]  Dump samples to file");
}


#define STATUS_BIT_ALERT   1 << 15
#define STATUS_BIT_HEATER  1 << 13
#define STATUS_BIT_RH      1 << 11
#define STATUS_BIT_TEMP    1 << 10
#define STATUS_BIT_RESET   1 << 4
#define STATUS_BIT_CMD     1 << 1
#define STATUS_BIT_CHCKSUM 1 << 0


#define IS_BIT_SET(_val, bit) (((_val) & (bit)) != 0)

int main(int argc, char **argv)
{
	oid_t oid;

	int ret = 0;

	if (lookup(DEV_PATH, NULL, &oid) < 0) {
		printf("temp device: " DEV_PATH " not found\n");
		return 1;
	}

	while (true) {
		int c = getopt(argc, argv, "bfc:gnlrsv");
		if (c == -1) {
			break;
		}

		switch (c) {
			case 'v':
				tool_common.verbose += 1;
				break;

			case 'h':
				print_usage(argv[0]);
				return 0;

			case 'r':
				sht31sendCmd(oid.port, sht31_cmdSoftReset);
				break;

			case 'b':
				sht31sendCmd(oid.port, sht31_cmdBreak);
				break;

			case 'g': {
				char str[32];
				sht31Read(oid.port, str, sizeof(str));

				char *val = strtok(str, ",");
				printf("Temperature: %s\n", val);

				val = strtok(NULL, "\n");
				printf("Relative Humidity: %s\n", val);

				break;
			}

			case 'l':
				sht31sendCmd(oid.port, sht31_cmdClearStatus);
				break;

			case 'n':
				sht31sendCmd(oid.port, sht31_cmdHeaterOn);
				break;

			case 'f':
				sht31sendCmd(oid.port, sht31_cmdHeaterOff);
				break;

			case 'c': {
				/* no,thresh,hyst */
				char *endptr, *curropt = optarg;

				int repeatability = strtoul(curropt, &endptr, 10);
				if (*endptr != ',') {
					puts("set-Ulvl: invalid subfield format (1)");
					break;
				}
				curropt = endptr + 1; /* skip ',' */

				int freq = strtoul(curropt, &endptr, 10);
				if (*endptr != '\0') {
					printf("set-Ulvl: invalid subfield format (3) %c %s", *endptr, endptr);
					break;
				}

				sht31Config(oid.port, repeatability, freq);
				break;
			}

			case 's': {
				uint16_t status = 0;

				sht31GetStatus(oid.port, &status);
				printf("sht31-tool: status: %x\n"
					   "Alert %d | Heater %d | Arh %d | At %d | Reset %d | Acmd %d | Achksum %d\n",
						status,
						IS_BIT_SET(status, STATUS_BIT_ALERT),
						IS_BIT_SET(status, STATUS_BIT_HEATER),
						IS_BIT_SET(status, STATUS_BIT_RH),
						IS_BIT_SET(status, STATUS_BIT_TEMP),
						IS_BIT_SET(status, STATUS_BIT_RESET),
						IS_BIT_SET(status, STATUS_BIT_CMD),
						IS_BIT_SET(status, STATUS_BIT_CHCKSUM));
				break;
			}

			default:
				print_usage(argv[0]);
				return 1;
		}
	}


	return ret;
}
