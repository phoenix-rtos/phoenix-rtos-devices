/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * STM32 USB physical layer controller
 *
 * Copyright 2025 Phoenix Systems
 * Author: Olaf Czerwinski, Radoslaw Szewczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <phoenix/arch/armv8m/stm32/n6/stm32n6.h>
#include <sys/platform.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>

#include "phy.h"


static struct {
	volatile uint32_t *otg_base;
	volatile uint32_t *phyc_base;
	volatile uint32_t *rcc_base;
} common;


volatile uint32_t *phy_getOtgBase(void)
{
	return common.otg_base;
}


uint32_t phy_getIrq(void)
{
	return PHY_IRQ;
}


static int phy_setIpClock(unsigned int ipclk, unsigned int setting)
{
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_ipclk,
		.ipclk = {
			.ipclk = ipclk,
			.setting = setting }
	};
	return platformctl(&pctl);
}


static int phy_enableDevClock(unsigned int dev)
{
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_devclk,
		.devclk = {
			.dev = dev,
			.state = 1,
			.lpState = 0 }
	};
	return platformctl(&pctl);
}


static int phy_mapRegs(void)
{

	uint32_t phyc_page = PHY_ADDR_USBPHYC & ~(0xFFF);
	uint32_t phyc_off = PHY_ADDR_USBPHYC & 0xFFF;
	void *ptr, *rcc_ptr;

	if (common.otg_base && common.phyc_base && common.rcc_base) {
		return 0;
	}

	common.otg_base = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)PHY_ADDR_OTG);

	if (common.otg_base == MAP_FAILED) {
		return -1;
	}

	ptr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)phyc_page);
	if (ptr == MAP_FAILED)
		return -1;
	common.phyc_base = (volatile uint32_t *)((uintptr_t)ptr + phyc_off);

	rcc_ptr = mmap(NULL, 0x2000, PROT_READ | PROT_WRITE,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)RCC_BASE_ADDR);

	if (rcc_ptr == MAP_FAILED) {
		return -1;
	}

	common.rcc_base = (volatile uint32_t *)rcc_ptr;

	return 0;
}


static void phy_hwResetSequence(void)
{
	/* Bits: 27(PHY), 26(Core), 23(PhyCtrl) */
	uint32_t rst_mask = (1 << 27) | (1 << 26) | (1 << 23);

	/* ASSERT RESET */
	common.rcc_base[RCC_AHB5RSTSR] = rst_mask;
	usleep(2000);

	/* DEASSERT PHY CTRL (Mostek) */
	common.rcc_base[RCC_AHB5RSTCR] = (1 << 23);
	usleep(1000);

	/* CONFIG PHY PLL */
	uint32_t val = common.phyc_base[USBPHYC_CR];
	val &= ~(0x7 << 4);
	val |= USBPHYC_CR_FSEL_24MHZ;

	/**
	 * Turn on PLL (Bit 2 CMN = 0)
	 * Default: 1 (PLL OFF)
	 */
	val &= ~(1 << 2);

	common.phyc_base[USBPHYC_CR] = val;

	/* DEASSERT PHY RESET */
	common.rcc_base[RCC_AHB5RSTCR] = (1 << 27);

	/* Wait for Lock PLL*/
	usleep(500);

	/* DEASSERT CORE RESET */
	common.rcc_base[RCC_AHB5RSTCR] = (1 << 26);
	usleep(10000);
}


int phy_init(void)
{
	/* Clocks */
	platformctl_t pctl_risup1 = {
		.action = pctl_set,
		.type = pctl_risup,
		.risup = {
			.index = pctl_risup_otg1_hs,
			.privileged = -1,
			.secure = 0,
			.lock = 0 }
	};

	platformctl(&pctl_risup1);

	phy_setIpClock(pctl_ipclk_otgphy1sel, 3);
	phy_setIpClock(pctl_ipclk_otgphy1ckrefsel, 1);
	phy_enableDevClock(pctl_otg1);
	phy_enableDevClock(pctl_otgphy1);

	if (phy_mapRegs() < 0)
		return -1;

	/* Reset hardware + PLL Enable */
	phy_hwResetSequence();

	/* Soft Reset Core */
	common.otg_base[GRSTCTL] |= (1 << 0);
	while (common.otg_base[GRSTCTL] & (1 << 0))
		;
	usleep(10000);

	/* Config: Force Device, TRDT=9, PHY LowPower=1 (Full Speed for Start) */
	common.otg_base[GUSBCFG] = (1 << 30) | (0x9 << 10) | (1 << 15);

	/**
	 * VBUS HACK (Bit 23)
	 * Force internal VBUS Valid signal high to allow D+ Pull-Up activation without physical VBUS detection
	 */
	uint32_t gccfg = common.otg_base[GCCFG];
	gccfg |= (1 << 23);
	common.otg_base[GCCFG] = gccfg;
	/* Override physical VBUS pin state to force B-Session Valid (BVALOVAL + BVALOEN) */
	common.otg_base[GOTGCTL] |= (1 << 7) | (1 << 6);

	/* AHB Config (Burst INCR4) */
	common.otg_base[GAHBCFG] |= (1 << 0) | (3 << 1);
	common.otg_base[GINTMSK] = 0;

	/* Device Config (Full Speed) */
	common.otg_base[DCFG] |= 0x3;


	return 0;
}
