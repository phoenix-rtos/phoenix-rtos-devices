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
	/* volatile uint32_t *rcc_base = (uint32_t *)RCC_BASE; */

	/* Look at RCC_CCIPR6 for options */
	setClock(pctl_ipclk_otgphy1sel, 3); /* hse_div2_osc_ck */
	/* *(rcc_base + rcc_ahb5ensr) |= (1 << 26); */
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
	/* *(rcc_base + rcc_ahb5ensr) |= (1 << 27); */
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
	uint32_t phyc_page = PHY_ADDR_USBPHYC & ~(0xFFF);
	uint32_t phyc_off = PHY_ADDR_USBPHYC & 0xFFF;
	void *ptr;
	void *rcc_ptr;

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

	if (ptr == MAP_FAILED) {
		munmap((void *)common.otg_base, 0x1000);
		return -1;
	}

	common.phyc_base = (volatile uint32_t *)((uintptr_t)ptr + phyc_off);


	rcc_ptr = mmap(NULL, 0x2000, PROT_READ | PROT_WRITE,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)RCC_BASE_ADDR);

	if (rcc_ptr == MAP_FAILED) {
		munmap((void *)common.otg_base, 0x1000);
		munmap(ptr, 0x1000);
		return -1;
	}

	common.rcc_base = (volatile uint32_t *)rcc_ptr;

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
	uint32_t gccfg = common.otg_base[GCCFG];
	gccfg |= (1 << 23);
	common.otg_base[GCCFG] = gccfg;
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

	/* Rx FIFO non-empty */
	common.otg_base[GINTSTS] |= (1 << 4);
	// *(usbotg + usbotg_gintsts) |= (1 << 4);

	/* Force device mode */
	/* *(usbotg + usbotg_gusbcfg) |= (1 << 30); */

	/* OTG timeout calibration field (N/A for now) and USB turnaround time */
	/* [30]=force device mode, [15]=clock selection, [13:10]USB turnaroun time (9 or 6) */

	common.otg_base[GUSBCFG] |= (1 << 30) | (1 << 15) | (6 << 10);

	/* Clear interrupts */
	common.otg_base[GINTSTS] |= OTG_GINTSTS_DEVICE_MASK;

	/* Unmask OTG interrupt and Mode mismatch interrupt */
	// common.otg_base[GINTMSK] &= ~3U;

	if ((common.otg_base[GINTSTS] & 1) != 0) {
		printf(" Olaf: mode set to device, but is host\n");
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

	/* 4. OTG_GINTMSK unmask: USB reset, Enumeration Done, Early suspend, USB suspend, SOF */
	common.otg_base[GINTMSK] |= (1 << 12) | (1 << 13) | (1 << 10) | (1 << 11) | (1 << 3);

	/* 5. Wait for USBRST interrupt in OTG)GINTSTS */
	while (((common.otg_base[GINTSTS] >> 12) & 1) == 0) {
		usleep(50);
		timeout++;
		if (timeout > 0x0FFFFFFF) {
			printf(" Olaf: timeout waiting for USBRST in init\n");
			break;
		}
	}

	/* TODO RADEK: odnsnik do erraty (na stronie stm) */
	vbusHack();
	/* now we can get usb reset, when the wire is pluged in*/
	/* the rest of the code should be in controller.c etc */


	/* Endpoint initialization on USB reset */
	/* 1. Set the NAK bit for all OUT endpoints
	– SNAK = 1 in OTG_DOEPCTLx (for all OUT endpoints)
	2. Unmask the following interrupt bits
	– INEP0 = 1 in OTG_DAINTMSK (control 0 IN endpoint)
	– OUTEP0 = 1 in OTG_DAINTMSK (control 0 OUT endpoint)
	– STUPM = 1 in OTG_DOEPMSK
	– XFRCM = 1 in OTG_DOEPMSK
	– XFRCM = 1 in OTG_DIEPMSK
	– TOM = 1 in OTG_DIEPMSK
	3. Set up the data FIFO RAM for each of the FIFOs
	– Program the OTG_GRXFSIZ register, to be able to receive control OUT data and
	setup data. If thresholding is not enabled, at a minimum, this must be equal to 1
	max packet size of control endpoint 0 + 2 words (for the status of the control OUT
	data packet) + 10 words (for setup packets).
	– Program the OTG_DIEPTXF0 register (depending on the FIFO number chosen) to
	be able to transmit control IN data. At a minimum, this must be equal to 1 max
	packet size of control endpoint 0.
	4. Program the following fields in the endpoint-specific registers for control OUT endpoint
	0 to receive a SETUP packet
	– STUPCNT = 3 in OTG_DOEPTSIZ0 (to receive up to 3 back-to-back SETUP
	packets)
	5. For USB OTG in DMA mode, the OTG_DOEPDMA0 register must have a valid
	memory address to store any SETUP packets received.
	At this point, all initialization required to receive SETUP packets is done. */


	/* Endpoint initialization on enumeration completion */
	/* 1. On the Enumeration Done interrupt (ENUMDNE in OTG_GINTSTS), read the
	OTG_DSTS register to determine the enumeration speed.
	2. Program the MPSIZ field in OTG_DIEPCTL0 to set the maximum packet size. This
	step configures control endpoint 0. The maximum packet size for a control endpoint
	depends on the enumeration speed.
	3. For USB OTG in DMA mode, program the OTG_DOEPCTL0 register to enable control
	OUT endpoint 0, to receive a SETUP packet. - N/A for now */

	// return;
	timeout = 0;
	while (((common.otg_base[GINTSTS] >> 13) & 1) == 0) {
		usleep(50);
		timeout++;
		if (timeout > 0x0FFFFFFF) {
			printf(" Olaf: timeout waiting for ENUMDNE in init\n");
			break;
		}
	}

	uint32_t enum_speed = (common.otg_base[DSTS] >> 1) & 3;
	printf(" Olaf: enum speed(0=Hs, 1=Fs, 3=Res): %d\n", enum_speed);
}


int phy_init(void)
{
	if (phy_mapRegs() < 0) {
		return -1;
	}

	phy_reset();
	phy_config();


	/* Config: Force Device, TRDT=9, PHY LowPower=1 (Full Speed for Start) */
	// common.otg_base[GUSBCFG] = (1 << 30) | (0x9 << 10) | (1 << 15);


	/* AHB Config (Burst INCR4) */
	// common.otg_base[GAHBCFG] |= (1 << 0) | (3 << 1)| (1 << 5);
	// common.otg_base[GINTMSK] = 0;

	/* Device Config (Full Speed) */
	// common.otg_base[DCFG] |= 0x3;


	return 0;
}
