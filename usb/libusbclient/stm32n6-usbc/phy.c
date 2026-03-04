/*
 * Phoenix-RTOS
 *
 * libusbclient (STM32 N6)
 *
 * Physical layer configuration
 *
 * Copyright 2025 Phoenix Systems
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

#define USB_REG(x)     common.otg_base[x]
#define USB_PHY_REG(x) common.phyc_base[x]
#define RCC_REG(x)     common.rcc_base[x]

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
		printf("[USB PHY]: setting clock failed\n");
		return -1;
	}

	ret = setClock(pctl_ipclk_otgphy1ckrefsel, 1); /* hse_div2_osc_ck */
	if (ret < 0) {
		printf("[USB PHY]: setting clock failed\n");
		return -1;
	}

	return 0;
}


int phy_mapRegs(void)
{
	uint32_t phycPage = PHY_ADDR_USBPHYC & ~(0xFFFUL);
	uint32_t phycOff = PHY_ADDR_USBPHYC & 0xFFFUL;
	void *ptr;

	if ((common.otg_base != NULL) && (common.phyc_base != NULL) && (common.rcc_base != NULL)) {
		return 0;
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

	return 0;
}


static int phy_otg1_clk_enable(void)
{
	//(RCC_BASE_ADDR + RCC_AHB5ENSR) |= (1 << 26);
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
	//(RCC_BASE_ADDR + RCC_AHB5ENSR) |= (1 << 27);
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

	/* set clock for OTGPHY1 and OTGPHY1*/
	ret = phy_setClock();
	if (ret < 0) {
		return -1;
	}

	ret = phy_otg1_clk_enable();
	if (ret < 0) {
		return -1;
	}

	ret = phy_otg1phy_clk_enable();
	if (ret < 0) {
		return -1;
	}

	/* Reset modules: Bits: 27(PHY), 26(Core), 23(PhyCtrl) - RM0486 Rev 2; page: 3743/12 */
	RCC_REG(RCC_AHB5RSTSR) |= (1UL << 23);
	RCC_REG(RCC_AHB5RSTSR) |= (1UL << 26);
	RCC_REG(RCC_AHB5RSTSR) |= (1UL << 27);

	/* Set hse_div2_osc_ck = hse_osc_ck/2 */
	RCC_REG(RCC_HSECFGR) |= (1U << 6);

	/* Deassert PhyCtrl reset */
	RCC_REG(RCC_AHB5RSTCR) = (1 << 23);

	/* Delay before accessing USBCPHY_CR */
	usleep(1000);

	/* Select OTHPHY1 frequency */
	USB_PHY_REG(USBPHYC_CR) &= ~(0x7U << 4);
	USB_PHY_REG(USBPHYC_CR) |= USBPHYC_CR_FSEL_24MHZ;

	/* Deassert OTGPHY1 reset */
	RCC_REG(RCC_AHB5RSTCR) = (1UL << 27);

	/* Wait befre releasing OTG1 reset */
	usleep(1000);

	/* Deassert OTG1 reset  */
	RCC_REG(RCC_AHB5RSTCR) = (1UL << 26);

	__asm__ volatile("dsb");

	return 0;
}


/**
 * VBUS HACK (Bit 23)
 * Force internal VBUS Valid signal high to allow D+ Pull-Up activation without physical VBUS detection
 */
static void vbusHack(void)
{
	/* Override physical VBUS pin state to force B-Session Valid ([6] BVALOEN + [7] BVALOVAL) */
	USB_REG(GOTGCTL) |= (1U << 6);
	USB_REG(GOTGCTL) |= (1U << 7);

	USB_REG(GCCFG) |= (1 << 23);
}


static void phy_start(void)
{
	/* Enable interrupts */
	USB_REG(GAHBCFG) |= (1U);

	/* Ungate clocks */
	USB_REG(PCGCCTL) &= ~(0x3U);

	/* Clear soft reset */
	USB_REG(DCTL) &= ~(1U << 1);
}


int phy_clear_config(void)
{
	int cnt, ret;

	/* clear TX FIFO sizes */
	USB_REG(DIEPTXF0) = 0UL;
	for (cnt = 1; cnt < ENDPOINTS_NUMBER; cnt++) {
		USB_REG(DIEPTXFx + (cnt - 1)) = 0UL;
	}

	/* Disable pull-downs in device mode */
	USB_REG(GCCFG) &= ~(1UL << 25);

	vbusHack();

	/* clear clk gating options */
	USB_REG(PCGCCTL) = 0U;

	/* Program device speed (FS) */
	USB_REG(DCFG) &= ~(3U);
	USB_REG(DCFG) |= 1U;

	ret = clbc_flushTxFifo(16U);
	if (ret < 0) {
		printf("flushing TX FIFOs failed\n");
		return -1;
	}

	ret = clbc_flushRxFifo();
	if (ret < 0) {
		printf("flushing RX FIFOs failed\n");
		return -1;
	}

	/* clear pending interrupts */
	USB_REG(DIEPMSK) = 0UL;
	USB_REG(DOEPMSK) = 0UL;
	USB_REG(DAINTMSK) = 0UL;

	for (cnt = 0; cnt < ENDPOINTS_NUMBER; cnt++) {

		/* IEPs clearing */
		if (((USB_REG(DIEPCTL0 + cnt * EP_STRIDE) >> DIEPCTL_EPENA) & 1U) == 1U) {
			if (cnt == 0U) {
				USB_REG(DIEPCTL0) |= (1UL << DIEPCTL_SNAK);
			}
			else {
				USB_REG(DIEPCTL0 + cnt * EP_STRIDE) |= ((1UL << DIEPCTL_SNAK) | (1UL << DIEPCTL_EPDIS));
			}
		}
		else {
			USB_REG(DIEPCTL0 + cnt * EP_STRIDE) = 0UL;
		}
		USB_REG(DIEPTSIZ0 + cnt * EP_STRIDE) = 0UL;
		USB_REG(DIEPINT0 + cnt * EP_STRIDE) = DIEPINTxWrMsk;

		/* OEPs clearing */
		if (((USB_REG(DOEPCTL0 + cnt * EP_STRIDE) >> DOEPCTL_EPENA) & 1U) == 1U) {
			if (cnt == 0U) {
				USB_REG(DOEPCTL0) |= (1UL << DOEPCTL_SNAK);
			}
			else {
				USB_REG(DOEPCTL0 + cnt * EP_STRIDE) |= ((1UL << DOEPCTL_SNAK) | (1UL << DOEPCTL_EPDIS));
			}
		}
		else {
			USB_REG(DOEPCTL0 + cnt * EP_STRIDE) = 0UL;
		}
		USB_REG(DOEPTSIZ0 + cnt * EP_STRIDE) = 0UL;
		USB_REG(DOEPINT0 + cnt * EP_STRIDE) = DOEPINTxWrMsk;
	}

	/* Clear interrupts */
	USB_REG(GINTSTS) |= GINTSTSWrMsk;

	/* unmask Rx FIFO non-empty */
	USB_REG(GINTMSK) |= (1U << GINTSTS_RXFLVL);

	/* OTG_GINTMSK unmask: USB reset, Enumeration Done, Early suspend, USB suspend, SOF...*/
	USB_REG(GINTMSK) |= ((1UL << GINTSTS_USBRST) | (1UL << GINTSTS_ENUMDNE) |
			(1UL << GINTSTS_ESUSP) | (1UL << GINTSTS_USBSUSP) |
			(1UL << GINTSTS_SOF) | (1UL << GINTSTS_OEPINT) |
			(1UL << GINTSTS_IEPINT) | (1UL << GINTSTS_OTGINT));

	/* setting USB turnaround time to min val. 0x9 */
	USB_REG(GUSBCFG) &= ~(0xFUL << GUSBCFG_TRDT);
	USB_REG(GUSBCFG) |= (0x9UL << GUSBCFG_TRDT);

	/* set USB PLL clock to 48MHz in low-power for FS */
#if DEVICE_SPEED == USB_SPEED_FS
	USB_REG(GUSBCFG) |= (0x1UL << GUSBCFG_PHYLPC);
#endif

	/* non-zero-length status OUT handshake set to STALL */
	USB_REG(DCFG) |= (1U << DCFG_NZLSOHSK);

	/* setting Periodic Frame Interval to 95% */
	USB_REG(DCFG) |= (3UL << DCFG_PFIVL);

	/* Set soft reset */
	USB_REG(DCTL) |= (1U << 1);

	return 0;
}


void phy_config(usb_dc_t *dc)
{
	/* FIFO RAM: OTG_GRXFSIZ	-  Set Rx FIFO */
	USB_REG(GRXFSIZ) = RX_FIFO_DEPTH;

	/* FIFO RAM: OTG_DIEPTXF0  - EP0 */
	USB_REG(DIEPTXF0) = (TX0FD << 16) | (TX0FSA);

	/* FIFO RAM: OTG_DIEPTXF1 - EP1 (CDC Control IN)*/
	USB_REG(DIEPTXFx) = (TX1FD << 16) | (TX1FSA);

	/* FIFO RAM: OTG_DIEPTXF2 - EP2 (CDC Bulk IN)*/
	USB_REG(DIEPTXFx + 0x1U) = (TX2FD << 16) | (TX2FSA);

	phy_start();
}


int phy_usbss_init(void)
{
	int ret;

	/* Disable interrupts */
	USB_REG(GAHBCFG) &= ~(1U);

	/* Soft reset USBSS */
	ret = clbc_resetUSBSS();
	if (ret < 0) {
		printf("resetting USB SS failed\n");
		return -1;
	}

	/* set device mode */
	ret = clbc_setDevMode();
	if (ret < 0) {
		printf("setting device mode failed\n");
		return -1;
	}

	return 0;
}


static uint32_t getPLLSourceFreq(uint32_t PLLsource)
{
	uint32_t pllinputfreq = 0;
	uint32_t rccReady, divider, msiFreq;

	switch (PLLsource) {
		case 0U:
			/* HSI */
			rccReady = ((RCC_REG(RCC_SR) >> 3) & 1);
			if (rccReady != 0) {
				divider = ((RCC_REG(RCC_HSICFGR) >> 7) & 3);
				pllinputfreq = (64000000UL >> divider);
			}
			break;

		case (1UL << 28):
			/* MSI */
			rccReady = ((RCC_REG(RCC_SR) >> 2) & 1);
			if (rccReady != 0) {
				msiFreq = ((RCC_REG(RCC_MSICFGR) >> 9) & 1);
				/* 4MHz */
				if (msiFreq == 0) {
					pllinputfreq = 4000000UL;
				}
				else {
					pllinputfreq = 16000000UL;
				}
			}
			break;

		case (2UL << 28):
			/* HSE */
			rccReady = ((RCC_REG(RCC_SR) >> 4) & 1);
			if (rccReady != 0) {
				pllinputfreq = 48000000UL;
			}
			break;

		case (3UL << 28):
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
	return ((RCC_REG(RCC_PLL1CFGR1 + (pllNum * RCC_PLL_STRIDE)) >> 27) & 1U);
}


static uint32_t getPLLXFreq(uint8_t pllNum)
{
	uint32_t plloutputfreq = 0U;
	uint32_t divm;
	uint32_t isPllEnabled, source, pllinputfreq;
	uint32_t valN, valFRACN, valP1, valP2;

	uint32_t isPllReady = ((RCC_REG(RCC_SR) >> (8 + pllNum)) & 1);

	if (isPllReady != 0U) {
		isPllEnabled = ((RCC_REG(RCC_PLL1CFGR3 + (pllNum * RCC_PLL_STRIDE)) >> 30) & 1);
		if (isPllEnabled != 0U) {
			source = (RCC_REG(RCC_PLL1CFGR1 + (pllNum * RCC_PLL_STRIDE)) >> 27) & 7UL;
			pllinputfreq = getPLLSourceFreq(source);

			if (pllinputfreq != 0U) {
				divm = ((RCC_REG(RCC_PLL1CFGR1 + (pllNum * RCC_PLL_STRIDE)) >> 20) & 0x3F);

				if (divm != 0) {
					valN = ((RCC_REG(RCC_PLL1CFGR1 + (pllNum * RCC_PLL_STRIDE)) >> 8) & 0xFFF);
					valFRACN = ((RCC_REG(RCC_PLL1CFGR2 + (pllNum * RCC_PLL_STRIDE)) >> 0) & 0xFFFFFF);
					valP1 = ((RCC_REG(RCC_PLL1CFGR2 + (pllNum * RCC_PLL_STRIDE)) >> 27) & 0x7);
					valP2 = ((RCC_REG(RCC_PLL1CFGR2 + (pllNum * RCC_PLL_STRIDE)) >> 24) & 0x7);
					pllinputfreq = calcPLLFreq(pllinputfreq, divm, valN, valFRACN, valP1, valP2);
				}
			}
		}
	}
	else if (isBypassEnabledPLL1(pllNum) != 0) {
		source = ((RCC_REG(RCC_PLL1CFGR1 + (pllNum * RCC_PLL_STRIDE)) >> 28) & 0x7);
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
	uint32_t source = ((RCC_REG(RCC_IC2CFGR) >> 28) & 3);
	uint32_t divider = (((RCC_REG(RCC_IC2CFGR) >> 16) & 0xFF) + 1UL);
	uint32_t sysClkSource = (RCC_REG(RCC_CFGR1) >> 28) & 3;
	uint32_t hsiDivider, msiVal, msifreqsel, hseVal;

	switch (sysClkSource) {
		case 0:
			/* CLKSOURCE_STATUS_HSI */
			hsiDivider = ((RCC_REG(RCC_HSICFGR) >> 7) & 3);
			frequency = (64000000UL >> hsiDivider);
			break;

		case (1 << 28):
			/* CLKSOURCE_STATUS_MSI */
			msiVal = 4000000UL;
			msifreqsel = ((RCC_REG(RCC_MSICFGR) >> 9) & 1);
			frequency = (msifreqsel ? (msiVal << 2) : (msiVal));
			break;

		case (2 << 28):
			/* CLKSOURCE_STATUS_HSE */
			hseVal = 48000000UL;
			frequency = hseVal;
			break;

		case (3 << 28):
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
	uint32_t ahbPrescaler = ((RCC_REG(RCC_CFGR2) >> 20) & 0x7U);

	return (sysClockFreq >> ahbPrescaler);
}
