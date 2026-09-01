/*
 * Phoenix-RTOS
 *
 * TDA4VM MCU SPI driver demo app
 *
 * Copyright 2026 Phoenix Systems
 * Author: Rafal Mikielis
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/platform.h>
#include <phoenix/arch/armv7r/tda4vm/tda4vm.h>

#include <spi-tda4vm.h>


#define BUFFER_SIZE        128U
#define MCU_MCSPI_DEVCOUNT 3U


static char txBuff[BUFFER_SIZE];
static char rxBuff[BUFFER_SIZE];

static struct {
	uint8_t devnum;
	uint8_t chnum;
	uint8_t devtype;
} mcu_mcspi[MCU_MCSPI_DEVCOUNT] = {
	{},
	{
		.devnum = MCU_MCSPI1,
		.chnum = MCU_MCSPI_CH0,
		.devtype = MCSPI_DEVICE_SLAVE,
	},
	{},
};

static struct mcspi_modulctrl modulDefconfig[MCU_MCSPI_DEVCOUNT] = {
	{},
	{
		.single = 1U, /* single channel mode */
		.pin34 = 0U,  /* CS pin used */
		.ms = 1U,     /* slave */
	},
	{},
};

static struct mcspi_chconf channelDefconfig[MCU_MCSPI_DEVCOUNT] = {
	{},
	{
		.pha = 0U,
		.pol = 1U,
		.clkd = 0U, /* SPICLK divisor (only master mode) */
		.epol = 1U,
		.dpe0 = 0,
		.dpe1 = 1,
		.is = 1,
	},
	{},
};

static struct mcspi_xferReq xferConfig[MCU_MCSPI_DEVCOUNT] = {
	{},
	{
		.devnum = 1U,
		.channel = 0U,
		.rxBuff = &rxBuff[0],
		.txBuff = &txBuff[0],
		.xferType = MCSPI_XFER_RX,
	},
	{},
};


static void printRxBuff(int wordSize, int wordCount)
{
	printf("RX: ");

	for (int i = 0; i < wordCount; i++) {
		if (wordSize <= 8U) {
			printf("%02x ", ((uint8_t *)rxBuff)[i]);
		}
		else if (wordSize <= 16U) {
			printf("%04x ", ((uint16_t *)rxBuff)[i]);
		}
		else {
			printf("%08x ", ((uint32_t *)rxBuff)[i]);
		}
	}
	printf("\n");
}

/* NOTE: MCU SPI driver is now only capable to act as a Slave to HLOS (Linux) in MAIN domain.
 *       MCSPI1 device is routed to MAIN domain - no GPIO configuration needed.
 *       MCSPI0 and MCSPI2 devices are not functional with this driver.
 */
int main(int argc, char **argv)
{
	int ret, opt;
	uint32_t wordSize, wordCount, devnum, xferType;

	struct mcspi_modulctrl *modctrl;
	struct mcspi_chconf *chconf;
	struct mcspi_xferReq *xfer;

	if (argc < 5) {
		fprintf(stderr, "Wrong argument count...\n");
		return -1;
	}

	/* default values */
	wordSize = 8U;
	wordCount = 4U;
	devnum = 1;
	xferType = MCSPI_XFER_RX;

	/* Process options */
	while ((opt = getopt(argc, argv, "s:c:d:t:h:")) != -1) {
		switch (opt) {
			case 's':
				wordSize = (uint32_t)strtoul(optarg, NULL, 10);
				break;

			case 'c':
				wordCount = (uint32_t)strtoul(optarg, NULL, 10);
				break;

			case 'd':
				devnum = (uint32_t)strtoul(optarg, NULL, 10);
				if (devnum > MCU_MCSPI2) {
					fprintf(stderr, "Wrong device number...\n");
					return -1;
				}
				break;
			case 't':
				xferType = (uint32_t)strtoul(optarg, NULL, 10);
				if (xferType < MCSPI_XFER_TXRX || xferType > MCSPI_XFER_TX) {
					fprintf(stderr, "Wrong transfer type...\n");
					return -1;
				}
				break;
			case 'h':
			default:
				printf("Command line arguments:\n");
				printf(" -s <size>:   size of the SPI word in bits (8-32)\n");
				printf(" -c <count>:  count of the SPI words to transfer\n");
				printf(" -d <device>: MCSPI device to use (0-2)\n");
				printf(" -t <type>:   transfer type (0 - txrx, 1 - rx only, 2 - tx only)\n");
				return -1;
		}
	}

	modctrl = &modulDefconfig[devnum];
	chconf = &channelDefconfig[devnum];
	xfer = &xferConfig[devnum];

	/* prepare tx data */
	for (int i = 0; i < BUFFER_SIZE; i++) {
		txBuff[i] = i + 1;
	}

	ret = mcspi_init(devnum, mcu_mcspi[devnum].chnum, mcu_mcspi[devnum].devtype, modctrl, chconf);
	if (ret != EOK) {
		printf("MCSPI dev initialisation failed %s\n", strerror(ret));
	}

	xfer->devnum = devnum;
	xfer->channel = mcu_mcspi[devnum].chnum;
	xfer->wordCount = wordCount;
	xfer->wordSize = wordSize;
	xfer->xferType = xferType;

	ret = mcspi_xferStart(xfer);
	if (ret != EOK) {
		printf("MCSPI xfer initialisation failed %s\n", strerror(ret));
	}

	printf("MCSPI xfer succes\n");

	printRxBuff(wordSize, wordCount);

	mcspi_deinit(devnum);

	return 0;
}
