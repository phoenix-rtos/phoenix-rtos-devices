/*
 * Phoenix-RTOS
 *
 * usbclient - usb device controller driver
 *
 * Copyright 2020-2021 Phoenix Systems
 * Author: Hubert Buczynski, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/mman.h>

#include "../imx-usbc/phy.h"

#define USB_BASE_ADDR       0x02184000


/* TODO: It should be moved into imx-usbc. */
void *usbclient_allocBuff(uint32_t size)
{
	return mmap(NULL, size, PROT_WRITE | PROT_READ, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
}


/* TODO: It should be moved into imx-usbc. */
void usbclient_buffDestory(void *addrs, uint32_t size)
{
	munmap(addrs, size);
}


void phy_setClock(void)
{

}


void *phy_getBase(uint32_t size)
{
	return mmap(NULL, size, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, USB_BASE_ADDR);
}


uint32_t phy_getIrq(void)
{
	return 75;
}


void phy_reset(void)
{

}


void phy_init(void)
{

}
