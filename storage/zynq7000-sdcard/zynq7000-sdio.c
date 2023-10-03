/*
 * Phoenix-RTOS
 *
 * Zynq-7000 SDIO peripheral initialization
 *
 * Copyright 2023 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "zynq7000-sdio.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <sys/platform.h>

#include <phoenix/arch/zynq7000.h>
#include <board_config.h>

/* This platform has 2 SDIO hosts, but a slot was only implemented on one */
#define IMPLEMENTED_SLOTS 1

#define SDIO0_ADDR 0xE0100000
#define SDIO0_IRQ  56
#define SDIO1_ADDR 0xE0101000
#define SDIO1_IRQ  79

/* Set ref clock for SDIO peripheral
 * Set IO PLL as source clock and set divider:
 * IO_PLL / 20 :  1000 MHz / 20 = 50 MHz
 * 50 MHz is enough because we never use SD clock faster than that
 */
static int sdio_setDevClk(int dev, int state)
{
	platformctl_t ctl;

	ctl.action = pctl_get;
	ctl.type = pctl_devclock;
	ctl.devclock.dev = pctl_ctrl_sdio_clk;

	if (platformctl(&ctl) < 0) {
		return -EIO;
	}

	ctl.action = pctl_set;
	switch (dev) {
		case 0:
			ctl.devclock.clkact0 = state;
			break;

		case 1:
			ctl.devclock.clkact1 = state;
			break;

		default:
			return -EINVAL;
	}
	ctl.devclock.srcsel = 0;
	ctl.devclock.divisor0 = 20;

	return platformctl(&ctl);
}


/* Activate AMBA clocks for the SDIO peripheral
 */
static int sdio_setAmbaClk(int dev, unsigned int state)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_ambaclock;

	ctl.ambaclock.dev = (dev == 0) ? pctl_amba_sdi0_clk : pctl_amba_sdi1_clk;
	ctl.ambaclock.state = state;

	return platformctl(&ctl);
}


/* Power on both SDIO peripherals. */
static int sdio_poweron(void)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_devreset;
	pctl.devreset.dev = pctl_ctrl_sdio_rst;
	pctl.devreset.state = 0;
	int err = platformctl(&pctl);
	if (err < 0) {
		return err;
	}

	return err;
}

/* Set pin functions for SDIO peripheral within platform's I/O mux. */
static int sdio_setPin(int dev, uint32_t pin)
{
	platformctl_t ctl;

	/* Pin should not be configured by the driver */
	if (pin < 0) {
		return EOK;
	}

	if (dev != 0) {
		/* Currently only pins for SDIO0 are supported */
		return -EINVAL;
	}

	ctl.action = pctl_set;
	ctl.type = pctl_mio;

	/* Select pin */
	ctl.mio.pin = pin;
	ctl.mio.disableRcvr = 1;
	ctl.mio.pullup = 0;

	if ((pin >= 40) && (pin <= 45)) {
		/* Route through the MIO mux levels to get the right function */
		ctl.mio.ioType = 1;
		ctl.mio.speed = 0x1;
		ctl.mio.l0 = 0;
		ctl.mio.l1 = 0;
		ctl.mio.l2 = 0;
		ctl.mio.l3 = 0b100;
	}
	else if ((pin == SD_CARD_CD) || (pin == SD_CARD_WP)) {
		ctl.mio.ioType = 1;
		ctl.mio.speed = 0;
		ctl.mio.l0 = 0;
		ctl.mio.l1 = 0;
		ctl.mio.l2 = 0;
		ctl.mio.l3 = 0;
	}
	else {
		return -EINVAL;
	}

	ctl.mio.triEnable = 0;

	return platformctl(&ctl);
}


/* Set WP (write protect) and CD (card detect) pins for peripheral
 * These are handled differently from other pins, as they are not routed directly
 * but are set as GPIO pins and we need to inform the peripheral which pins
 * were selected.
 */
static int sdio_setSdWpCdPins(int dev, int wpPin, int cdPin)
{
	platformctl_t ctl;

	if (wpPin < 0) {
		wpPin = 0;
	}

	if (cdPin < 0) {
		cdPin = 0;
	}

	ctl.action = pctl_set;
	ctl.type = pctl_sdwpcd;
	ctl.SDWpCd.cdPin = cdPin;
	ctl.SDWpCd.wpPin = wpPin;
	ctl.SDWpCd.dev = dev;

	return platformctl(&ctl);
}


/* Initialize all pins on the peripheral.
 */
static int sdio_initPins(int dev)
{
	int res;
	static const int pins[] = {
		SD_CARD_CLK,
		SD_CARD_CMD,
		SD_CARD_D0,
		SD_CARD_D1,
		SD_CARD_D2,
		SD_CARD_D3,
		SD_CARD_CD,
		SD_CARD_WP,
	};

	for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
		/* Pin should not be configured by the driver */
		if (pins[i] < 0) {
			continue;
		}

		res = sdio_setPin(dev, pins[i]);
		if (res < 0) {
			return res;
		}
	}

	return sdio_setSdWpCdPins(dev, SD_CARD_WP, SD_CARD_CD);
}


int sdio_platformConfigure(unsigned int slot, sdio_platformInfo_t *infoOut)
{
	int res;

	if (slot > 0) {
		/* Currently this code only supports SDIO0 */
		return -ENOENT;
	}

	res = sdio_poweron();
	if (res < 0) {
		return -EIO;
	}

	res = sdio_setDevClk(slot, 1);
	if (res < 0) {
		return -EIO;
	}

	res = sdio_setAmbaClk(slot, 1);
	if (res < 0) {
		return -EIO;
	}

	res = sdio_initPins(slot);
	if (res < 0) {
		return -EIO;
	}

	infoOut->refclkFrequency = 50UL * 1000 * 1000;
	infoOut->regBankPhys = SDIO0_ADDR;
	infoOut->interruptNum = SDIO0_IRQ;
	infoOut->isCDPinSupported = SD_CARD_CD >= 0;
	infoOut->isWPPinSupported = SD_CARD_WP >= 0;
	return 0;
}
