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


void phy_setClock(void)
{
	/* Look at RCC_CCIPR6 for options */
	setClock(pctl_ipclk_otgphy1sel, 3); /* hse_div2_osc_ck */
	// *(volatile uint32_t *)(RCC_BASE_ADDR + RCC_AHB5ENSR) |= (1 << 26);
	platformctl_t pctl_otg1_set = {
		.action = pctl_set,
		.type = pctl_devclk,
		.devclk = {
			.dev = pctl_otg1,
			.state = 1,
			.lpState = 0 }
	};
	platformctl(&pctl_otg1_set);

	setClock(pctl_ipclk_otgphy1ckrefsel, 1); /* hse_div2_osc_ck */
	// *(volatile uint32_t *)(RCC_BASE_ADDR + RCC_AHB5ENSR) |= (1 << 27);
	platformctl_t pctl_otgphy1_set = {
		.action = pctl_set,
		.type = pctl_devclk,
		.devclk = {
			.dev = pctl_otgphy1,
			.state = 1,
			.lpState = 0 }
	};
	platformctl(&pctl_otgphy1_set);
}


static int phy_mapRegs(void)
{
	uint32_t phycPage = PHY_ADDR_USBPHYC & ~(0xFFF);
	uint32_t phycOff = PHY_ADDR_USBPHYC & 0xFFF;
	void *ptr;
	void *rccPtr;

	if (common.otg_base && common.phyc_base && common.rcc_base) {
		return 0;
	}

	common.otg_base = mmap(NULL, 0x2000, PROT_READ | PROT_WRITE,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)PHY_ADDR_OTG);

	if (common.otg_base == MAP_FAILED) {
		return ENOMEM;
	}

	ptr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)phycPage);

	if (ptr == MAP_FAILED) {
		munmap((void *)common.otg_base, 0x2000);
		return ENOMEM;
	}

	common.phyc_base = (volatile uint32_t *)((uintptr_t)ptr + phycOff);


	rccPtr = mmap(NULL, 0x2000, PROT_READ | PROT_WRITE,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)RCC_BASE_ADDR);

	if (rccPtr == MAP_FAILED) {
		munmap((void *)common.otg_base, 0x1000);
		munmap(ptr, 0x1000);
		return ENOMEM;
	}

	common.rcc_base = (volatile uint32_t *)rccPtr;

	return 0;
}


void phy_reset(void)
{
	/* Bits: 27(PHY), 26(Core), 23(PhyCtrl) - RM0486 Rev 2; page: 3743/12 */
	uint32_t rstMask = (1 << 27) | (1 << 26) | (1 << 23);

	common.rcc_base[RCC_AHB5RSTSR] = rstMask;

	phy_setClock();

	/* Deassert PhyCtrl */
	common.rcc_base[RCC_AHB5RSTCR] = (1 << 23);

	/* Select OTHPHY1 frequency */
	common.phyc_base[USBPHYC_CR] &= ~(0x7 << 4);
	common.phyc_base[USBPHYC_CR] |= USBPHYC_CR_FSEL_24MHZ;

	/* DEASSERT PHY RESET */
	common.rcc_base[RCC_AHB5RSTCR] = (1 << 27);

	/* Wait 50ms for Lock PLL*/
	usleep(100);

	/* DEASSERT CORE RESET */
	common.rcc_base[RCC_AHB5RSTCR] = (1 << 26);
}


void phy_enable(void)
{
	/* TODO: For now make it accessible without security */
	platformctl_t pctl_otg1_hs = {
		.action = pctl_set,
		.type = pctl_risup,
		.risup = {
			.index = pctl_risup_otg1_hs,
			.privileged = -1,
			.secure = 1,
			.lock = 0 }
	};
	platformctl(&pctl_otg1_hs);
}


/**
 * VBUS HACK (Bit 23)
 * Force internal VBUS Valid signal high to allow D+ Pull-Up activation without physical VBUS detection
 */
static void vbusHack(void)
{
	common.otg_base[GCCFG] |= (1 << 23);
	common.otg_base[GCCFG] |= (1 << 24);
	/* Override physical VBUS pin state to force B-Session Valid (BVALOVAL + BVALOEN) */
	common.otg_base[GOTGCTL] |= (1 << 7) | (1 << 6);
}


void phy_config(void)
{
	/* Core config */
	/* 1. Program the following fields in the OTG_GAHBCFG register:
	– Global interrupt mask bit GINTMSK = 1
	– Rx FIFO non-empty (RXFLVL bit in OTG_GINTSTS)
	– Periodic Tx FIFO empty level Note: Only accessible in host mode
	2. Program the following fields in the OTG_GUSBCFG register:
	– OTG timeout calibration field
	– USB turnaround time field
	3. The software must unmask the following bits in the OTG_GINTMSK register:
	OTG interrupt mask
	Mode mismatch interrupt mask
	4. The software can read the CMOD bit in OTG_GINTSTS to determine whether the OTG
	controller is operating in host or device mode. */

	/* Enable interrupt */
	common.otg_base[GAHBCFG] |= 1;
	// *(usbotg + usbotg_gahbcfg) |= 1;

	/* Rx FIFO non-empty - propably they mean to unmask it*/
	for (uint8_t i = 0U; i < 15U; i++) {
		/** TODO: SET FOR OTHER ENDPOINTS */
		common.otg_base[DIEPTXF0] = 0U;
		break;
	}


	common.otg_base[GCCFG] &= ~(1 << 25);

	common.otg_base[DIEPMSK] = 0U;
	common.otg_base[DOEPMSK] = 0U;
	common.otg_base[DAINTMSK] = 0U;

	for (uint8_t i = 0U; i < 9U; i++) {
		if ((common.otg_base[DIEPCTL0 + EP_STRIDE * i] >> 31) & 1) {
			if (i == 0U) {
				common.otg_base[DIEPCTL0 + EP_STRIDE * i] = (1 << 27);
			}
			else {
				common.otg_base[DIEPCTL0 + EP_STRIDE * i] = ((1 << 27) | (1 << 30));
			}
		}
		else {
			common.otg_base[DIEPCTL0 + EP_STRIDE * i] = 0U;
		}

		common.otg_base[DIEPTSIZ0 + EP_STRIDE * i] = 0U;
		common.otg_base[DIEPINT0 + EP_STRIDE * i] = 0xFB7FU;
	}

	for (uint8_t i = 0U; i < 9U; i++) {
		if ((common.otg_base[DOEPCTL0 + EP_STRIDE * i] >> 31) & 1) {
			if (i == 0U) {
				common.otg_base[DOEPCTL0 + EP_STRIDE * i] = (1 << 27);
			}
			else {
				common.otg_base[DOEPCTL0 + EP_STRIDE * i] = ((1 << 27) | (1 << 30));
			}
		}
		else {
			common.otg_base[DOEPCTL0 + EP_STRIDE * i] = 0U;
		}

		common.otg_base[DOEPTSIZ0 + EP_STRIDE * i] = 0U;
		common.otg_base[DOEPINT0 + EP_STRIDE * i] = 0xFB7FU;
	}

	common.otg_base[DIEPMSK] &= ~(1 << 8);

	common.otg_base[GINTSTS] = 0xBFFFFFFFU;


	// common.otg_base[GINTSTS] |= (1 << 4);
	common.otg_base[GINTMSK] |= (1 << 4);
	// *(usbotg + usbotg_gintsts) |= (1 << 4);

	/* Force device mode */
	/* *(usbotg + usbotg_gusbcfg) |= (1 << 30); */

	/* OTG timeout calibration field (N/A for now) and USB turnaround time */
	/* [30]=force device mode, [15]=clock selection, [13:10]USB turnaroun time (9 or 6) */

	common.otg_base[GUSBCFG] |= (1 << 30) | (1 << 15) | (6 << 10);

	/* Clear interrupts */
	common.otg_base[GINTSTS] |= GINTSTSWrMsk;

	/* Unmask OTG interrupt and Mode mismatch interrupt */
	common.otg_base[GINTMSK] |= 3;

	if ((common.otg_base[GINTSTS] & 1) != 0) {
		printf(" [USB PHY]: mode set to device, but is host\n");
	}

	/* Device config */
	/* 1. Program the following fields in the OTG_DCFG register:
	– Device speed
	– Non-zero-length status OUT handshake
	– Periodic Frame Interval
	2. Program the Device threshold control register. This is required only if you are using
	DMA mode and you are planning to enable thresholding. - N/A for now
	3. Clear the DCTL.SDIS bit. The core issues a connect after this bit is cleared.
	4. Program the OTG_GINTMSK register to unmask the following interrupts:
	– USB reset
	– Enumeration done
	– Early suspend
	– USB suspend
	– SOF

	---- !!!! I guess this part is the begining of enumeration provess ----

	5. Wait for the USBRST interrupt in OTG_GINTSTS. It indicates that a reset has been
	detected on the USB that lasts for about 10 ms on receiving this interrupt.

	---- !!!! I guess the enumeration process with the whole endpoint setup goes there

	6. Wait for the ENUMDNE interrupt in OTG_GINTSTS. This interrupt indicates the end of
	reset on the USB. On receiving this interrupt, the application must read the OTG_DSTS
	register to determine the enumeration speed and perform the steps listed in Endpoint
	initialization on enumeration completion. */

	/* Program device speed, non-zero-length status OUT handshake and periodic Frame Interval */
	common.otg_base[DCFG] &= ~((1 << 2) | 3); /* [1:0] speed, 00=Hs, 01=Fs */
	common.otg_base[DCFG] |= 1;               /* [1:0] speed, 00=Hs, 01=Fs */

	/* Clear the DCTL.SDIS bit. The core issues a connect after this bit is cleared.!!!!! */
	uint32_t timeout = 0;
	common.otg_base[DCTL] &= ~2;

	/* 4. OTG_GINTMSK unmask: USB reset, Enumeration Done, Early suspend, USB suspend, SOF ... and OEPINT, IEPINT .. WKUPIN*/
	common.otg_base[GINTMSK] |= (1 << 12) | (1 << 13) | (1 << 10) | (1 << 11) | (1 << 3) | (1 << 19) | (1 << 18) | (1 << 31);

	/* issue mentioned in errata */
	vbusHack();
}


int phy_init(void)
{
	int res = phy_mapRegs();
	if (res < 0) {
		return res;
	}

	phy_reset();
	phy_config();

	return 0;
}


static uint32_t getPLLSourceFreq(uint32_t PLLsource)
{
	uint32_t pllinputfreq = 0U;

	switch (PLLsource) {
		case 0U:
			/* HSI */
			uint32_t isRccHsiReady = ((common.rcc_base[RCC_SR] >> 3) & 1);
			if (isRccHsiReady != 0) {
				uint32_t divider = ((common.rcc_base[RCC_HSICFGR] >> 7) & 3);
				pllinputfreq = (64000000UL >> divider);
			}
			break;

		case (1 << 28):
			/* MSI */
			uint32_t isRccMsiReady = ((common.rcc_base[RCC_SR] >> 2) & 1);
			if (isRccMsiReady != 0) {
				uint32_t msiFreq = ((common.rcc_base[RCC_MSICFGR] >> 9) & 1);
				/* 4MHz */
				if (msiFreq == 0) {
					pllinputfreq = 4000000UL;
				}
				else {
					pllinputfreq = 16000000UL;
				}
			}
			break;

		case (2 << 28):
			/* HSE */
			uint32_t isRccHseReady = ((common.rcc_base[RCC_SR] >> 4) & 1);
			if (isRccHseReady != 0) {
				pllinputfreq = 48000000UL;
			}
			break;

		case (3 << 28):
			/* CKIN */
			pllinputfreq = 12288000U;
			break;

		default:
			break;
	}
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
	return ((common.rcc_base[RCC_PLL1CFGR1 + (pllNum * RCC_PLL_STRIDE)] >> 27) & 1);
}


static uint32_t getPLLXFreq(uint8_t pllNum)
{
	uint32_t plloutputfreq = 0U;
	uint32_t divm;

	uint32_t isPllReady = ((common.rcc_base[RCC_SR] >> (8 + pllNum)) & 1);

	if (isPllReady != 0U) {
		uint32_t isPllEnabled = ((common.rcc_base[RCC_PLL1CFGR3 + (pllNum * RCC_PLL_STRIDE)] >> 30) & 1);
		if (isPllEnabled != 0U) {
			uint32_t source = (common.rcc_base[RCC_PLL1CFGR1 + (pllNum * RCC_PLL_STRIDE)] & (7 << 27));
			uint32_t pllinputfreq = getPLLSourceFreq(source);

			if (pllinputfreq != 0U) {
				divm = ((common.rcc_base[RCC_PLL1CFGR1 + (pllNum * RCC_PLL_STRIDE)] >> 20) & 0x3F);

				if (divm != 0) {
					uint32_t valN = ((common.rcc_base[RCC_PLL1CFGR1 + (pllNum * RCC_PLL_STRIDE)] >> 8) & 0xFFF);
					uint32_t valFRACN = ((common.rcc_base[RCC_PLL1CFGR2 + (pllNum * RCC_PLL_STRIDE)] >> 0) & 0xFFFFFF);
					uint32_t valP1 = ((common.rcc_base[RCC_PLL1CFGR2 + (pllNum * RCC_PLL_STRIDE)] >> 27) & 0x7);
					uint32_t valP2 = ((common.rcc_base[RCC_PLL1CFGR2 + (pllNum * RCC_PLL_STRIDE)] >> 24) & 0x7);
					pllinputfreq = calcPLLFreq(pllinputfreq, divm, valN, valFRACN, valP1, valP2);
				}
			}
		}
	}
	else if (isBypassEnabledPLL1(pllNum) != 0) {
		uint32_t source = ((common.rcc_base[RCC_PLL1CFGR1 + (pllNum * RCC_PLL_STRIDE)] >> 28) & 0x7);
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

	uint32_t source = ((common.rcc_base[RCC_IC2CFGR] >> 28) & 3);
	uint32_t divider = (((common.rcc_base[RCC_IC2CFGR] >> 16) & 0xFF) + 1UL);

	uint32_t sysClkSource = (common.rcc_base[RCC_CFGR1] & (3 << 28));

	switch (sysClkSource) {
		case 0:
			/* CLKSOURCE_STATUS_HSI */
			uint32_t hsiDevider = ((common.rcc_base[RCC_HSICFGR] >> 7) & 3);
			frequency = (64000000UL >> hsiDevider);
			break;

		case (1 << 28):
			/* CLKSOURCE_STATUS_MSI */
			uint32_t msiVal = 4000000UL;
			uint32_t msifreqsel = ((common.rcc_base[RCC_MSICFGR] >> 9) & 1);
			frequency = (msifreqsel ? (msiVal << 2) : (msiVal));
			break;

		case (2 << 28):
			/* CLKSOURCE_STATUS_HSE */
			uint32_t hseVal = 48000000UL;
			frequency = hseVal;
			break;

		case (3 << 28):
			/* PLL_X (X in range 1-4, but coresponding source values are 0-3) */
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
	// uint32_t GetSysClockFreq =1;
	// uint32_t GetAHBPrescaler = 1;
	// uint32_t res =  ((GetSysClockFreq >> (((GetAHBPrescaler) & (0x7UL << 20UL))) >> (20UL)));
	uint32_t sysClockFreq = getSysClockFrequency();
	uint32_t ahbPrescaler = ((common.rcc_base[RCC_CFGR2] >> 20) & 7);

	return (sysClockFreq >> ahbPrescaler);
}
