/*
 * Phoenix-RTOS
 *
 * usbclient - usb device controller driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Kamil Amanowicz, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/platform.h>
#include <phoenix/arch/imxrt.h>

#include "../imx-usbc/phy.h"


#define USB_BASE_ADDR       0x402E0000
#define OCRAM2_BASE_ADRR    0x20210000


static uint8_t counter_OCRAM2 = 0;

/* TODO: It should be moved into imx-usbc. */
void *usbclient_allocBuff(uint32_t size)
{
	return  (void *)(OCRAM2_BASE_ADRR + size * counter_OCRAM2++);
}

/* TODO: It should be moved into imx-usbc. */
void usbclient_buffDestory(void *addrs, uint32_t size)
{
	counter_OCRAM2 = 0;
}


static int setClock(int dev, unsigned int state)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.dev = dev;
	pctl.devclock.state = state;

	return platformctl(&pctl);
}


void phy_setClock(void)
{
	setClock(pctl_clk_usboh3, clk_state_run);
}


void *phy_getBase(uint32_t size)
{
	return (void *)USB_BASE_ADDR;
}


uint32_t phy_getIrq(void)
{
	return usb_otg1_irq;
}
