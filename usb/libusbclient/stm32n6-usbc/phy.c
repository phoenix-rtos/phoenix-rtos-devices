/*
 * Phoenix-RTOS
 *
 * libusbclient (STM32 N6)
 *
 * Physical layer configuration
 *
 * Copyright 2026 Phoenix Systems
 * Author: Olaf Czerwinski, Radosław Szewczyk, Rafał Mikielis
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
#include "client.h"

#define USB_REG(x)     *(common.otg_base + x)
#define USB_PHY_REG(x) *(common.phyc_base + x)
#define RCC_REG(x)     *(common.rcc_base + x)

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
	return otg1_irq;
}


static int setClock(int dev, unsigned int state)
{
	platformctl_t pctl_ip = {
		.action = pctl_set,
		.type = pctl_ipclk,
		.ipclk = {
			.ipclk = dev,
			.setting = state }
	};

	return platformctl(&pctl_ip);
}


int phy_setClock(void)
{
	int ret;
	/* Look at RCC_CCIPR6 for options */
	ret = setClock(pctl_ipclk_otgphy1sel, 3); /* hse_div2_osc_ck */
	if (ret < 0) {
		fprintf(stderr, "[USB PHY]: setting clock failed\n");
		return -EIO;
	}

	ret = setClock(pctl_ipclk_otgphy1ckrefsel, 1); /* hse_div2_osc_ck */
	if (ret < 0) {
		fprintf(stderr, "[USB PHY]: setting clock failed\n");
		return -EIO;
	}

	return EOK;
}


int phy_mapRegs(void)
{
	uint32_t phycPage = PHY_ADDR_USBPHYC & ~(0xFFFUL);
	uint32_t phycOff = PHY_ADDR_USBPHYC & 0xFFFUL;
	void *ptr;

	if ((common.otg_base != NULL) && (common.phyc_base != NULL) && (common.rcc_base != NULL)) {
		return EOK;
	}

	common.otg_base = mmap(NULL, 0x2000, PROT_READ | PROT_WRITE,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)PHY_ADDR_OTG);

	if (common.otg_base == MAP_FAILED) {
		return -ENOMEM;
	}

	ptr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)phycPage);

	if (ptr == MAP_FAILED) {
		munmap((void *)common.otg_base, 0x2000);
		return -ENOMEM;
	}

	common.phyc_base = (volatile uint32_t *)((uintptr_t)ptr + phycOff);

	common.rcc_base = mmap(NULL, 0x2000, PROT_READ | PROT_WRITE,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)RCC_BASE_ADDR);

	if (common.rcc_base == MAP_FAILED) {
		munmap((void *)common.otg_base, 0x2000);
		munmap(ptr, 0x1000);
		return -ENOMEM;
	}

	return EOK;
}


void phy_unmapRegs(void)
{
	(void)munmap((void *)common.otg_base, 0x2000);
	(void)munmap((void *)common.phyc_base, 0x1000);
	(void)munmap((void *)common.rcc_base, 0x2000);
}


static int phy_otg1_clk_enable(void)
{
	/* (RCC_BASE_ADDR + RCC_PLL_STRIDE) |= (1 << 26) */
	platformctl_t pctl_otg1_set = {
		.action = pctl_set,
		.type = pctl_devclk,
		.devclk = {
			.dev = pctl_otg1,
			.state = 1,
			.lpState = 1 }
	};
	return platformctl(&pctl_otg1_set);
}


static int phy_otg1phy_clk_enable(void)
{
	/* (RCC_BASE_ADDR + RCC_PLL_STRIDE) |= (1 << 27) */
	platformctl_t pctl_otgphy1_set = {
		.action = pctl_set,
		.type = pctl_devclk,
		.devclk = {
			.dev = pctl_otgphy1,
			.state = 1,
			.lpState = 1 }
	};
	return platformctl(&pctl_otgphy1_set);
}


int phy_clk_reset(void)
{
	int ret;

	/* set clock for OTGPHY1 and OTG1*/
	ret = phy_setClock();
	if (ret < 0) {
		return -EIO;
	}

	ret = phy_otg1_clk_enable();
	if (ret < 0) {
		return -EIO;
	}

	ret = phy_otg1phy_clk_enable();
	if (ret < 0) {
		return -EIO;
	}

	/* Reset modules: Bits: 27(PHY), 26(Core), 23(PhyCtrl) - RM0486 Rev 2; page: 3743/12 */
	RCC_REG(rcc_ahb5rstsr) |= (1UL << 23);
	RCC_REG(rcc_ahb5rstsr) |= (1UL << 26);
	RCC_REG(rcc_ahb5rstsr) |= (1UL << 27);

	/* Setting hse_div2_osc_ck = hse_osc_ck/2 is done in plo */

	/* Deassert PhyCtrl reset */
	RCC_REG(rcc_ahb5rstcr) = (1 << 23);

	/* Delay before accessing USBCPHY_CR */
	usleep(1000);

	/* Select OTHPHY1 frequency */
	USB_PHY_REG(usbphyc_cr) &= ~(0x7U << 4);
	USB_PHY_REG(usbphyc_cr) |= USBPHYC_CR_FSEL_24MHZ;

	/* Deassert OTGPHY1 reset */
	RCC_REG(rcc_ahb5rstcr) = (1UL << 27);

	/* Wait befre releasing OTG1 reset */
	usleep(1000);

	/* Deassert OTG1 reset  */
	RCC_REG(rcc_ahb5rstcr) = (1UL << 26);

	__asm__ volatile("dsb");

	return EOK;
}


/**
 * VBUS HACK (Bit 23)
 * Force internal VBUS Valid signal high to allow D+ Pull-Up activation without physical VBUS detection
 */
static void vbusHack(void)
{
	/* Override physical VBUS pin state to force B-Session Valid ([6] BVALOEN + [7] BVALOVAL) */
	USB_REG(otg_gotgctl) |= (1U << 6);
	USB_REG(otg_gotgctl) |= (1U << 7);

	USB_REG(otg_gccfg) |= (1 << 23);
}


static void phy_start(void)
{
	/* Enable interrupts */
	USB_REG(otg_gahbcfg) |= (1U);

	/* Ungate clocks */
	USB_REG(otg_pcgcctl) &= ~(0x3U);

	/* Clear soft reset */
	USB_REG(otg_dctl) &= ~(1U << 1);
}


int phy_clear_config(void)
{
	int cnt, ret, epSet;

	/* clear TX FIFO sizes */
	USB_REG(otg_dieptxf0) = 0UL;
	for (cnt = 1; cnt < ENDPOINTS_NUMBER; cnt++) {
		USB_REG(otg_dieptxfx + (cnt - 1)) = 0UL;
	}

	/* Disable pull-downs in device mode */
	USB_REG(otg_gccfg) &= ~(1UL << 25);

	vbusHack();

	/* clear clk gating options */
	USB_REG(otg_pcgcctl) = 0U;

	/* Program device speed (FS) */
	USB_REG(otg_dcfg) &= ~(3U);
	USB_REG(otg_dcfg) |= 1U;

	ret = clbc_flushTxFifo(16U);
	if (ret < 0) {
		fprintf(stderr, "flushing TX FIFOs failed\n");
		return -EBUSY;
	}

	ret = clbc_flushRxFifo();
	if (ret < 0) {
		fprintf(stderr, "flushing RX FIFOs failed\n");
		return -EBUSY;
	}

	/* clear pending interrupts */
	USB_REG(otg_diepmsk) = 0UL;
	USB_REG(otg_doepmsk) = 0UL;
	USB_REG(otg_daintmsk) = 0UL;

	for (cnt = 0; cnt < ENDPOINTS_NUMBER; cnt++) {

		epSet = cnt * EP_STRIDE;
		/* IEPs clearing */
		if (((USB_REG(otg_diepctl0 + epSet) >> DIEPCTL_EPENA) & 1U) == 1U) {
			if (cnt == 0U) {
				USB_REG(otg_diepctl0) |= (1UL << DIEPCTL_SNAK);
			}
			else {
				USB_REG(otg_diepctl0 + epSet) |= ((1UL << DIEPCTL_SNAK) | (1UL << DIEPCTL_EPDIS));
			}
		}
		else {
			USB_REG(otg_diepctl0 + epSet) = 0UL;
		}
		USB_REG(otg_dieptsiz0 + epSet) = 0UL;
		USB_REG(otg_diepint0 + epSet) = DIEPINTxWrMsk;

		/* OEPs clearing */
		if (((USB_REG(otg_doepctl0 + epSet) >> DOEPCTL_EPENA) & 1U) == 1U) {
			if (cnt == 0U) {
				USB_REG(otg_doepctl0) |= (1UL << DOEPCTL_SNAK);
			}
			else {
				USB_REG(otg_doepctl0 + epSet) |= ((1UL << DOEPCTL_SNAK) | (1UL << DOEPCTL_EPDIS));
			}
		}
		else {
			USB_REG(otg_doepctl0 + epSet) = 0UL;
		}
		USB_REG(otg_doeptsiz0 + epSet) = 0UL;
		USB_REG(otg_doepint0 + epSet) = DOEPINTxWrMsk;
	}

	/* Clear interrupts */
	USB_REG(otg_gintsts) |= GINTSTSWrMsk;

	/* unmask Rx FIFO non-empty */
	USB_REG(otg_gintmsk) |= (1U << GINTSTS_RXFLVL);

	/* OTG_GINTMSK unmask: USB reset, Enumeration Done, Early suspend, USB suspend, SOF...*/
	USB_REG(otg_gintmsk) |= ((1UL << GINTSTS_USBRST) | (1UL << GINTSTS_ENUMDNE) |
			(1UL << GINTSTS_ESUSP) | (1UL << GINTSTS_USBSUSP) |
			(1UL << GINTSTS_SOF) | (1UL << GINTSTS_OEPINT) |
			(1UL << GINTSTS_IEPINT) | (1UL << GINTSTS_OTGINT));

	/* setting USB turnaround time to min val. 0x9 */
	USB_REG(otg_gusbcfg) &= ~(0xFUL << GUSBCFG_TRDT);
	USB_REG(otg_gusbcfg) |= (0x9UL << GUSBCFG_TRDT);

	/* set USB PLL clock to 48MHz in low-power for FS */
#if DEVICE_SPEED == USB_SPEED_FS
	USB_REG(otg_gusbcfg) |= (0x1UL << GUSBCFG_PHYLPC);
#endif

	/* non-zero-length status OUT handshake set to STALL */
	USB_REG(otg_dcfg) |= (1U << DCFG_NZLSOHSK);

	/* setting Periodic Frame Interval to 95% */
	USB_REG(otg_dcfg) |= (3UL << DCFG_PFIVL);

	/* Set soft reset */
	USB_REG(otg_dctl) |= (1U << 1);

	return EOK;
}


void phy_config(usb_dc_t *dc)
{
	/* FIFO RAM: OTG_GRXFSIZ	-  Set Rx FIFO */
	USB_REG(otg_grxfsiz) = RX_FIFO_DEPTH;

	/* FIFO RAM: OTG_DIEPTXF0  - EP0 */
	USB_REG(otg_dieptxf0) = (TX0FD << 16) | (TX0FSA);

	/* FIFO RAM: OTG_DIEPTXF1 - EP1 (CDC Control IN)*/
	USB_REG(otg_dieptxfx) = (TX1FD << 16) | (TX1FSA);

	/* FIFO RAM: OTG_DIEPTXF2 - EP2 (CDC Bulk IN)*/
	USB_REG(otg_dieptxfx + 0x1U) = (TX2FD << 16) | (TX2FSA);

	phy_start();
}


int phy_usbss_init(void)
{
	int ret;

	/* Disable interrupts */
	USB_REG(otg_gahbcfg) &= ~(1U);

	/* Soft reset USBSS */
	ret = clbc_resetUSBSS();
	if (ret < 0) {
		fprintf(stderr, "resetting USB SS failed\n");
		return -EBUSY;
	}

	/* set device mode */
	ret = clbc_setDevMode();
	if (ret < 0) {
		fprintf(stderr, "setting device mode failed\n");
		return -EBUSY;
	}

	return EOK;
}


static uint32_t getPLLSourceFreq(uint32_t PLLsource)
{
	uint32_t pllinputfreq = 0;
	uint32_t rccReady, divider, msiFreq;

	switch (PLLsource) {
		case 0U:
			/* HSI */
			rccReady = ((RCC_REG(rcc_sr) >> 3) & 1);
			if (rccReady != 0) {
				divider = ((RCC_REG(rcc_hsicfgr) >> 7) & 3);
				pllinputfreq = (64000000UL >> divider);
			}
			break;

		case 1U:
			/* MSI */
			rccReady = ((RCC_REG(rcc_sr) >> 2) & 1);
			if (rccReady != 0) {
				msiFreq = ((RCC_REG(rcc_msicfgr) >> 9) & 1);
				/* 4MHz */
				if (msiFreq == 0) {
					pllinputfreq = 4000000UL;
				}
				else {
					pllinputfreq = 16000000UL;
				}
			}
			break;

		case 2U:
			/* HSE */
			rccReady = ((RCC_REG(rcc_sr) >> 4) & 1);
			if (rccReady != 0) {
				pllinputfreq = 48000000UL;
			}
			break;

		case 3U:
			/* CKIN */
			pllinputfreq = 12288000UL;
			break;

		default:
			break;
	}

	return pllinputfreq;
}


static uint32_t calcPLLFreq(uint32_t pllInputFreq, uint32_t M, uint32_t N, uint32_t FRACN, uint32_t p1, uint32_t p2)
{
	float freq;

	freq = ((float)pllInputFreq * ((float)N + ((float)FRACN / (float)0x1000000))) / (float)M;
	freq = freq / (float)p1;
	freq = freq / (float)p2;

	return (uint32_t)freq;
}


static int isBypassEnabledPLL1(int pllNum)
{
	return ((RCC_REG(rcc_pll1cfgr1 + (pllNum * RCC_PLL_STRIDE)) >> 27) & 1U);
}


static uint32_t getPLLXFreq(uint8_t pllNum)
{
	uint32_t plloutputfreq = 0U;
	uint32_t divm;
	uint32_t isPllEnabled, source, pllinputfreq;
	uint32_t valN, valFRACN, valP1, valP2;
	int pllSet = pllNum * RCC_PLL_STRIDE;

	uint32_t isPllReady = ((RCC_REG(rcc_sr) >> (8 + pllNum)) & 1);

	if (isPllReady != 0U) {
		isPllEnabled = ((RCC_REG(rcc_pll1cfgr3 + pllSet) >> 30) & 1);
		if (isPllEnabled != 0U) {
			source = (RCC_REG(rcc_pll1cfgr1 + pllSet) >> 27) & 7UL;
			pllinputfreq = getPLLSourceFreq(source);

			if (pllinputfreq != 0U) {
				divm = ((RCC_REG(rcc_pll1cfgr1 + pllSet) >> 20) & 0x3F);

				if (divm != 0) {
					valN = ((RCC_REG(rcc_pll1cfgr1 + pllSet) >> 8) & 0xFFF);
					valFRACN = ((RCC_REG(rcc_pll1cfgr2 + pllSet) >> 0) & 0xFFFFFF);
					valP1 = ((RCC_REG(rcc_pll1cfgr2 + pllSet) >> 27) & 0x7);
					valP2 = ((RCC_REG(rcc_pll1cfgr2 + pllSet) >> 24) & 0x7);
					pllinputfreq = calcPLLFreq(pllinputfreq, divm, valN, valFRACN, valP1, valP2);
				}
			}
		}
	}
	else if (isBypassEnabledPLL1(pllNum) != 0) {
		source = ((RCC_REG(rcc_pll1cfgr1 + pllSet) >> 28) & 0x7);
		plloutputfreq = getPLLSourceFreq(source);
	}
	else {
		/* Nothing to do */
	}

	return plloutputfreq;
}


static uint32_t getSysClockFrequency(void)
{
	uint32_t frequency = 0;
	uint32_t source = ((RCC_REG(rcc_ic2cfgr) >> 28) & 3);
	uint32_t divider = (((RCC_REG(rcc_ic2cfgr) >> 16) & 0xFF) + 1UL);
	uint32_t sysClkSource = (RCC_REG(rcc_cfgr1) >> 28) & 3U;
	uint32_t hsiDivider, msiVal, msifreqsel, hseVal;

	switch (sysClkSource) {
		case 0U:
			/* CLKSOURCE_STATUS_HSI */
			hsiDivider = ((RCC_REG(rcc_hsicfgr) >> 7) & 3);
			frequency = (64000000UL >> hsiDivider);
			break;

		case 1U:
			/* CLKSOURCE_STATUS_MSI */
			msiVal = 4000000UL;
			msifreqsel = ((RCC_REG(rcc_msicfgr) >> 9) & 1);
			frequency = (msifreqsel ? (msiVal << 2) : (msiVal));
			break;

		case 2U:
			/* CLKSOURCE_STATUS_HSE */
			hseVal = 48000000UL;
			frequency = hseVal;
			break;

		case 3U:
			/* PLL_X (X in range 1-4, but corresponding source values are 0-3) */
			frequency = getPLLXFreq(source);
			frequency = frequency / divider;
			break;

		default:
			break;
	}

	return frequency;
}


uint32_t phy_getHclkFreq(void)
{
	uint32_t sysClockFreq = getSysClockFrequency();
	uint32_t ahbPrescaler = ((RCC_REG(rcc_cfgr2) >> 20) & 0x7U);

	return (sysClockFreq >> ahbPrescaler);
}
