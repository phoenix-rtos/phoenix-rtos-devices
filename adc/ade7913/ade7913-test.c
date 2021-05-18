/*
 * Phoenix-RTOS
 *
 * i.MX RT1170 ADE7913 test application
 *
 * Copyright 2021 Phoenix Systems
 * Author: Marcin Baran
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/msg.h>

#include "ade7913.h"


int main(int argc, char **argv)
{
	int i;
	oid_t ade7913_spi;
	ade7913_burst_reg_t sample;

	int devcnt = 1;

	if (argc >= 2)
		devcnt = atoi(argv[1]);

	printf("Starting ADE7913 test (%d devices)\n", devcnt);

	if (devcnt > 4 || devcnt <= 0) {
		printf("Incorrect ADE7913 device count (4 max)\n");
		return -1;
	}

	while (lookup("/dev/spi1", NULL, &ade7913_spi) < 0)
		usleep(5000);

	printf("SPI1 initialized\n");

	for (i = 0; i < devcnt; ++i) {
		printf("Configuring ADE7913 device nr %d\n", i);

		while (ade7913_init(&ade7913_spi, i, i < devcnt - 1 ? 1 : 0) < 0) {
			printf("Failed to initialize ade7913 nr %d\n", i);
			usleep(500000);
		}

		if (ade7913_lock(&ade7913_spi, i) < 0)
			printf("Could not lock ADE7913 nr %d\n", i);
		if (ade7913_enable(&ade7913_spi, i) < 0)
			printf("Could not enable ADE7913 nr %d\n", i);
	}

	printf("Reading ADE7913 registers in burst mode:\n");

	while (1) {
		for (i = 0; i < devcnt; ++i) {
			if (ade7913_sample_regs_read(&ade7913_spi, i, &sample) < 0) {
				printf("Failed reading sample registers from device %d\n", i);
				continue;
			}

			printf("IWV[%d]: %x\n", i, sample.iwv);
			printf("V1WV[%d]: %x\n", i, sample.v1wv);
			printf("V2WV[%d]: %x\n", i, sample.v2wv);
			printf("ADC_CRC[%d]: %x\n", i, sample.adc_crc);
			printf("CNT_SNAPSHOT[%d]: %x\n", i, sample.cnt_snapshot);
			printf("STATUS0[%d]: %x\n", i, sample.status0);
		}

		printf("\n");
		usleep(500000);
	}

	return 0;
}