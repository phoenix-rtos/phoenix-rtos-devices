/*
 * Phoenix-RTOS
 *
 * i.MX RT1170 ADE7913 test application
 *
 * Copyright 2021-2022 Phoenix Systems
 * Author: Marcin Baran, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/msg.h>
#include <imxrt-multi.h>

#include "ade7913.h"


#define ADE7913_DEFAULT_LIST "1230"


static int lpspi_config(oid_t *device, int spi)
{
	msg_t msg;
	multi_i_t *imsg = (multi_i_t *)msg.i.raw;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg->id = id_spi1 + spi;
	imsg->spi.type = spi_config;
	imsg->spi.config.cs = 0;
	imsg->spi.config.mode = spi_mode_3;
	imsg->spi.config.endian = spi_msb;
	imsg->spi.config.sckDiv = 0;
	imsg->spi.config.prescaler = 1;

	return msgSend(device->port, &msg);
}


int main(int argc, char **argv)
{
	int i, devnum, devcnt, isclkout;
	const char *order;
	oid_t ade7913_spi;
	ade7913_burst_reg_t sample;

	if (argc != 2) {
		printf("No device list given. Using default: %s\n", ADE7913_DEFAULT_LIST);
		devcnt = strlen(ADE7913_DEFAULT_LIST);
		order = ADE7913_DEFAULT_LIST;
	}
	else {
		order = argv[1];
		devcnt = strlen(argv[1]);

		for (i = 0; i < devcnt; ++i) {
			devnum = (int)(order[i] - '0');
			if (devnum >= devcnt || devnum < 0) {
				printf("Wrong order format provided\n");
			}
		}

		printf("Device order: %s\n", order);
	}

	if (devcnt > 4) {
		printf("Incorrect ADE7913 device count (4 max)\n");
		return 1;
	}

	while (lookup("/dev/spi1", NULL, &ade7913_spi) < 0) {
		usleep(100 * 1000);
	}

	if (lpspi_config(&ade7913_spi, 0) < 0) {
		printf("Could not initialize SPI1\n");
		return 2;
	}

	printf("SPI1 initialized\n");

	/* Start init from device with xtal */
	for (i = 0; i < devcnt; ++i) {
		devnum = (int)(order[i] - '0');
		/* the last ADC is DREADY and the other CLOCK clock output (daisy chain) */
		isclkout = (i != devcnt - 1);

		usleep(500 * 1000);
		printf("Configuring ADE7913 device no. %d\n", devnum);

		while (ade7913_init(&ade7913_spi, devnum, isclkout) < 0) {
			printf("Failed to initialize ADE7913 no. %d\n", devnum);
			usleep(500 * 1000);
		}

		if (ade7913_enable(&ade7913_spi, devnum) < 0) {
			printf("Could not enable ADE7913 no. %d\n", devnum);
			return 3;
		}

		if (ade7913_lock(&ade7913_spi, devnum) < 0) {
			printf("Could not lock ADE7913 no. %d\n", devnum);
			return 4;
		}

		/* Wait for next ADC to start */
		usleep(50 * 1000);
	}

	printf("Reading ADE7913 registers in burst mode:\n");

	while (1) {
		for (i = 0; i < devcnt; ++i) {
			devnum = (int)(order[i] - '0');

			if (ade7913_sample_regs_read(&ade7913_spi, devnum, &sample) < 0) {
				printf("Failed reading sample registers from device %d\n", devnum);
				break;
			}

			printf("IWV[%d]: 0x%x\n", devnum, sample.iwv);
			printf("V1WV[%d]: 0x%x\n", devnum, sample.v1wv);
			printf("V2WV[%d]: 0x%x\n", devnum, sample.v2wv);
			printf("ADC_CRC[%d]: 0x%x\n", devnum, sample.adc_crc);
			printf("CNT_SNAPSHOT[%d]: 0x%x\n", devnum, sample.cnt_snapshot);
			printf("STATUS0[%d]: 0x%x\n", devnum, sample.status0);
		}

		printf("\n");
		usleep(500 * 1000);
	}

	return 5;
}
