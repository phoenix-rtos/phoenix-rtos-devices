/*
 * Phoenix-RTOS
 *
 * usbclient - usb device controller driver
 *
 * Copyright 2019-2021 Phoenix Systems
 * Author: Kamil Amanowicz, Hubert Buczynski, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/platform.h>
#include <phoenix/arch/imxrt.h>
#include <sys/mman.h>

#include "../imx-usbc/phy.h"


#define USB_BASE_ADDR       0x402E0000


/* Reserve DTCM Memory range. */
/* NOTICE: Verify if syspage map setup is not overlaping with uncached map
 * defined below, uncached map should be aligned to 0x800 both base and
 * end addresses
 */
#define UNCACHED_BASE_ADDR 0x20020000
#define UNCACHED_END_ADDR  0x20028000


static void *uncached_ptr = (void *)UNCACHED_END_ADDR;


/* TODO: It should be moved into imx-usbc. */
void *usbclient_allocBuff(uint32_t size)
{
	/* TODO: reimplement this stopgap using mmap() with separate, special
	 * uncached map and forced alignment to 0x800 instead of only PAGE_SIZE
	 * alignment
	 */

	if ((uintptr_t)uncached_ptr & 0x7ff)
		return MAP_FAILED;

	size = (size + 0x7ff) & ~0x7ff;

	if ((uintptr_t)uncached_ptr < size)
		return MAP_FAILED;

	if ((uintptr_t)uncached_ptr - size < UNCACHED_BASE_ADDR)
		return MAP_FAILED;

	uncached_ptr -= size;

	return uncached_ptr;
}

/* TODO: It should be moved into imx-usbc. */
void usbclient_buffDestory(void *addrs, uint32_t size)
{
	/* TODO: reimplement this stopgap with munmap() */
	uncached_ptr = (void *)UNCACHED_END_ADDR;
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
