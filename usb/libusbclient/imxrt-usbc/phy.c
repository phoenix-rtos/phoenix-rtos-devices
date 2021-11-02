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

#include <phoenix/arch/imxrt.h>
#include <sys/platform.h>
#include <sys/mman.h>

#include "../imx-usbc/phy.h"


#define USB_CORE_BASE_ADDR     0x402E0000
#define USB_PHY_BASE_ADDR      0x400D9000


enum {
	/* Power-Down Register */
	usbphy_pwd, usbphy_pwd_set, usbphy_pwd_clr, usbphy_pwd_tog,
	/* Transmitter & Receiver Control Registers */
	usbphy_tx, usbphy_tx_set, usbphy_tx_clr, usbphy_tx_tog,
	usbphy_rx, usbphy_rx_set, usbphy_rx_clr, usbphy_rx_tog,
	/* General Control Register */
	usbphy_ctrl, usbphy_ctrl_set, usbphy_ctrl_clr, usbphy_ctrl_tog,
	/* USB Status & Debug Registers */
	usbphy_status,
	usbphy_debug = 20, usbphy_debug_set, usbphy_debug_clr, usbphy_debug_tog,
	/* UTMI Status & Debug Registers */
	usbphy_debug0_status, usbphy_debug1 = 28,
	usbphy_debug1_set, usbphy_debug1_clr, usbphy_debug1_tog,
	/* UTMI RTL */
	usbphy_version
};


/* Reserve DTCM Memory range. */
/* NOTICE: Verify if syspage map setup is not overlaping with uncached map
 * defined below, uncached map should be aligned to 0x800 both base and
 * end addresses
 */
#define UNCACHED_BASE_ADDR 0x20050000
#define UNCACHED_END_ADDR  0x20058000


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
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_devclock,
		.devclock = {
			.dev = dev,
			.state = state
		}
	};

	return platformctl(&pctl);
}


void phy_setClock(void)
{
	setClock(pctl_clk_iomuxc, clk_state_run_wait);
	setClock(pctl_clk_usboh3, clk_state_run_wait);
}


void *phy_getBase(uint32_t size)
{
	return (void *)USB_CORE_BASE_ADDR;
}


uint32_t phy_getIrq(void)
{
	return usb_otg1_irq;
}


static void phy_initPad(void)
{
	/* USB_OTG1_ID is an instance of anatop */
	platformctl_t set_ioisel = {
		.action = pctl_set,
		.type = pctl_ioisel,
		.ioisel = {
			.isel = pctl_isel_anatop_usb_otg1_id,
			.daisy = 0
		}
	};

	/* GPIO_AD_B0_01 is configured as USB_OTG1_ID (input) ALT3 */
	platformctl_t set_mux = {
		.action = pctl_set,
		.type = pctl_iomux,
		.iomux = {
			.mux = pctl_mux_gpio_ad_b0_01,
			.sion = 0, .mode = 3
		}
	};

	/* IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_01 = 0x10b0 */
	platformctl_t set_pad = {
		.action = pctl_set,
		.type = pctl_iopad,
		.iopad = {
			.pad = pctl_mux_gpio_ad_b0_01,
			.hys = 0, .pus = 0, .pue = 0, .pke = 1,
			.ode = 0, .speed = 2, .dse = 6, .sre = 0
		}
	};

	platformctl(&set_ioisel);
	platformctl(&set_mux);
	platformctl(&set_pad);
}


void phy_reset(void)
{
	volatile uint32_t *usbphy = (uint32_t *)USB_PHY_BASE_ADDR;

	/* Reset PHY */
	*(usbphy + usbphy_ctrl_set) = (1 << 31);

	/* Enable clock and release the PHY from reset */
	*(usbphy + usbphy_ctrl_clr) = (1 << 31) | (1 << 30);

	/* Power up the PHY */
	*(usbphy + usbphy_pwd) = 0;
}


void phy_init(void)
{
	phy_setClock();
	phy_initPad();
	phy_reset();
}
