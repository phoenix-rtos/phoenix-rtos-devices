/*
 * Phoenix-RTOS
 *
 * I2C bus scanner utility
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <i2c.h>


#define I2C_MIN_ADDR 0x08
#define I2C_MAX_ADDR 0x77
#define I2C_DEV_NO   0


static void usage(const char *prog)
{
	printf("Usage: %s [device_number]\n", prog);
	printf("  device_number - I2C bus to scan (default: 0)\n");
	printf("\nScans I2C bus for connected devices.\n");
}


int main(int argc, char *argv[])
{
	unsigned int i2cDev = I2C_DEV_NO;
	uint8_t addr;
	uint8_t byte;
	int ret;
	int found = 0;

	if (argc > 1) {
		if (strcmp(argv[1], "-h") == 0) {
			usage(argv[0]);
			return 0;
		}
		i2cDev = (unsigned int)strtoul(argv[1], NULL, 0);
	}

	printf("Scanning I2C bus %u for connected devices...\n", i2cDev);
	printf("Found devices at addresses:\n");

	/* Initialize I2C */
	ret = i2c_init(i2cDev);
	if (ret < 0) {
		printf("Failed to initialize I2C device %u: %s\n", i2cDev, strerror(-ret));
		return EXIT_FAILURE;
	}

	for (addr = I2C_MIN_ADDR; addr <= I2C_MAX_ADDR; addr++) {

		ret = i2c_busRead(addr, &byte, 1);
		if (ret == 0) {
			/* Device found */
			printf("  0x%02X\n", addr);
			found++;
		}
		else if (ret != -EIO) {
			/* Timeout/other error */
			printf("Error accessing address 0x%02X: %s\n", addr, strerror(-ret));
		}
		else {
			/* No device at this address */
		}
	}

	printf("Scan complete: found %d device(s)\n", found);

	return EXIT_SUCCESS;
}
