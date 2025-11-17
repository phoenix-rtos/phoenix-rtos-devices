/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * STM32 USB physical layer controller
 *
 * Copyright 2025 Phoenix Systems
 * Author: Olaf Czerwinski
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

#include "phy.h"


/* USB OTG */

#define USB0_BASE_ADDR ((void *)0x58040000) // OTG1_HS
#define USB0_PHY_BASE_ADDR ((void *)0x5803FC00) // OTG1PHYCTL
#define USB1_BASE_ADDR ((void *)0x58080000)
#define USB1_PHY_BASE_ADDR ((void *)0x580C0000)

#define USB0_IRQ otg1_irq
#define USB1_IRQ otg2_irq

#define RCC_BASE    ((void *)0x56028000)

#define OTG_GINTSTS_DEVICE_MASK 0xD8FCFCDC

enum {
	rcc_ahb5rstr = 136, rcc_ahb5enr = 152, rcc_ahb5rstsr = 648, rcc_ahb5ensr = 664, rcc_ahb5rstcr = 1160, rcc_ahb5encr = 1176
};

/* USB OTG controller */
enum {
	/* Core global control and status */
	usbotg_gotgctl = 0, usbotg_gotgint, usbotg_gahbcfg, usbotg_gusbcfg, usbotg_grstctl, usbotg_gintsts, usbotg_gintmsk, usbotg_grxstsr, usbotg_grxstsp, usbotg_grxfsiz, usbotg_dieptxf0, usbotg_hnptxsts, usbotg_gccfg = 14, usbotg_cid, usbotg_glpmcfg = 21, usbotg_hptxfsiz = 64, usbotg_dieptxf1, usbotg_dieptxf2, usbotg_dieptxf3, usbotg_dieptxf4, usbotg_dieptxf5, usbotg_dieptxf6, usbotg_dieptxf7, usbotg_dieptxf8,
	usbotg_dcfg = 512, usbotg_dctl, usbotg_dsts
};

/* USB PHY */
enum {
	usbphyc_cr = 0,
};

static void* currentUsbclientBuffPtr = NULL;

static int phy_getMemInfo(void **usbBuffPtr_out, void **usbBuffBasePtr_out, size_t *usbMemSz_out)
{
	void *usbBuffPtr = NULL;
	void *usbBuffBasePtr = NULL;
	size_t usbMemSz = 0;
	meminfo_t mi;
	mi.page.mapsz = -1;
	mi.entry.kmapsz = -1;
	mi.entry.mapsz = -1;
	mi.maps.mapsz = 0;
	mi.maps.map = NULL;
	meminfo(&mi);

	mi.maps.map = malloc(mi.maps.mapsz * sizeof(mapinfo_t));
	if (mi.maps.map == NULL) {
		return -ENOMEM;
	}

	meminfo(&mi);
	for (int i = 0; i < mi.maps.mapsz; i++) {
		if (strcmp(mi.maps.map[i].name, "usbdma") == 0) {
			usbBuffBasePtr = (void *)mi.maps.map[i].vstart;
			usbBuffPtr = (void *)mi.maps.map[i].vend;
			usbMemSz = mi.maps.map[i].vend - mi.maps.map[i].vstart;
			break;
		}
	}

	printf("usbclient_allocBuff: alloc: %d, free: %d, boot: %d, sz: %d, usbclientBuffPtr: 0x%p, currentUsbclientBuffPtr: 0x%p, usbMemSz: 0x%x\n", mi.page.alloc, mi.page.free, mi.page.boot, mi.page.sz, (void *)usbBuffPtr, (void *)usbBuffBasePtr, usbMemSz);

	free(mi.maps.map);
	if (usbBuffBasePtr == NULL) {
		return -ENODEV;
	}

	*usbBuffPtr_out = usbBuffPtr;
	*usbBuffBasePtr_out = usbBuffBasePtr;
	*usbMemSz_out = usbMemSz;
	return EOK;
}

/* 0x30020000 - 0x3003FFFF DTCM - FLEXMEM extension
0x30000000 - 0x3001FFFF DTCM - Base line */
void *usbclient_allocBuff(uint32_t size)
{

	void *usbclientBuffPtr = NULL;
	void *usbclientBuffBasePtr = NULL;
	size_t usbclientMemSz = 0;

	int ret = phy_getMemInfo(&usbclientBuffPtr, &usbclientBuffBasePtr, &usbclientMemSz);
	if (ret != 0)
		return MAP_FAILED;


	if (currentUsbclientBuffPtr == NULL) {
		currentUsbclientBuffPtr = usbclientBuffPtr;
	}

	size = (size + 0x7ff) & ~0x7ff;
	currentUsbclientBuffPtr -= size;

	if ((uintptr_t)currentUsbclientBuffPtr < size)
		return MAP_FAILED;

	if ((uintptr_t)currentUsbclientBuffPtr - size < (uintptr_t)usbclientBuffBasePtr)
		return MAP_FAILED;

	return currentUsbclientBuffPtr;

}


void usbclient_buffDestroy(void *addrs, uint32_t size)
{
	/* TODO: reimplement this stopgap with munmap() */
	currentUsbclientBuffPtr = NULL;
}


void *phy_getBase(uint32_t size)
{
	return USB0_BASE_ADDR;
}


uint32_t phy_getIrq(void)
{
	return USB0_IRQ;
}

static int setClock(int dev, unsigned int state)
{

	platformctl_t pctl_ip = {
		.action = pctl_set,
		.type = pctl_ipclk,
		.ipclk = {
			.ipclk = dev,
			.setting = state
		}
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
			.lpState = 0
		}
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
			.lpState = 0
		}
	};
	platformctl(&pctl_otgphy1_set);

}

void phy_reset(void)
{
//  TODO: Olaf:
// 	RM0486 Rev 2 3743/12
// RM0486 USB subsystem (USBSS)
// 3753
// Figure 973. USB2 OTG high-speed Port 1 resets
// There are three resets for the USB2 OTG high-speed Port 1:
// • rst_n_otg1, controlled by the OTG1RST bit in RCC_AHB5RSTR register.
// • rst_n_otg1phyctl, controlled by the OTG1PHYCTLRST bit in RCC_AHB5RSTR
// register.
// • rst_n_otgphy1, controlled by the OTGPHY1RST bit in RCC_AHB5RSTR register.
// When configuring the USB2 OTG high-speed Port 1, the reset sequence is:
// 1. rst_n_otgphy1, rst_n_otg1 and rst_n_otg1phyctl resets are asserted - done
// 2. clocks coming from RCC and HSE are enabled - done
// 3. rst_n_otg1phyctl reset is released - done
// 4. the frequency of OTGPHY1 reference clock is selected in OTG1PHYCTL_CR register - done
// 5. OTGPHY1 trimming bits are programmed (if needed) - n/a
// 6. rst_n_otgphy1 is released
// 7. wait 50 μs (OTGPHY1 PLL lock time and start of generation of phyclock0) - done
// 8. rst_n_otg1 is released - done

	volatile uint32_t *rcc_base = (uint32_t *)RCC_BASE;
	uint32_t rcc_test;
	/* 1. rst_n_otgphy1, rst_n_otg1 and rst_n_otg1phyctl resets are asserted  */
	rcc_test = *(rcc_base + rcc_ahb5rstsr);
	*(rcc_base + rcc_ahb5rstsr) |= (1 << 27) | (1 << 26) | (1 << 23);
	if(rcc_test == *(rcc_base + rcc_ahb5rstsr)) {
		printf("Olaf: ERROR: value unchanged: 0x%08x\n", rcc_test);
	}
	/* 2. clocks coming from RCC and HSE are enabled */
	phy_setClock();

	/* 3. rst_n_otg1phyctl reset is released */
	*(rcc_base + rcc_ahb5rstcr) |= (1 << 23);

	/* 4. the frequency of OTGPHY1 reference clock is selected in OTG1PHYCTL_CR register */
	volatile uint32_t *usbphy_base = (uint32_t *)USB0_PHY_BASE_ADDR;
	*(usbphy_base + usbphyc_cr) |= (2 << 4); /* 24 MHz */

	/* 6. rst_n_otgphy1 is released */
	*(rcc_base + rcc_ahb5rstcr) |= (1 << 27);

	/* 7. wait 50 μs (OTGPHY1 PLL lock time and start of generation of phyclock0) */
	usleep(100);

	/* 8. rst_n_otg1 is released */
	*(rcc_base + rcc_ahb5rstcr) |= (1 << 26);
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
			.lock = 0
		}
	};
	platformctl(&pctl_otg1_hs);
}

void phy_config(void) {
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


	volatile uint32_t *usbotg = (uint32_t *)USB0_BASE_ADDR;
	/* Enable interrupt */
	*(usbotg + usbotg_gahbcfg) |= 1;

	/* Rx FIFO non-empty */
	*(usbotg + usbotg_gintsts) |= (1 << 4);

	/* Force device mode */
	/* *(usbotg + usbotg_gusbcfg) |= (1 << 30); */

	/* OTG timeout calibration field (N/A for now) and USB turnaround time */
	/* [30]=force device mode, [15]=clock selection, [13:10]USB turnaroun time (9 or 6) */
	*(usbotg + usbotg_gusbcfg) |= (1 << 30) | (1 << 15) | (6 << 10);

	/* Clear interrupts */
	*(usbotg + usbotg_gintsts) |= OTG_GINTSTS_DEVICE_MASK;

	/* Unmask OTG interrupt and Mode mismatch interrupt */
	*(usbotg + usbotg_gintmsk) &= ~3U;

	if((*(usbotg + usbotg_gintsts) & 1) != 0) {
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
	6. Wait for the ENUMDNE interrupt in OTG_GINTSTS. This interrupt indicates the end of
	reset on the USB. On receiving this interrupt, the application must read the OTG_DSTS
	register to determine the enumeration speed and perform the steps listed in Endpoint
	initialization on enumeration completion. */

	/* Program device speed, non-zero-length status OUT handshake and periodic Frame Interval */
	*(usbotg + usbotg_dcfg) &= ~((1 << 2) | 3); /* [1:0] speed, 00=Hs, 01=Fs */
	*(usbotg + usbotg_dcfg) |= 1; /* [1:0] speed, 00=Hs, 01=Fs */

	/* Clear the DCTL.SDIS bit. The core issues a connect after this bit is cleared.!!!!! */
	uint32_t timeout = 0;
	*(usbotg + usbotg_dctl) &= ~2;

	while(((*(usbotg + usbotg_gintsts) >> 12) & 1) == 0) {
		usleep(50);
		timeout++;
		if(timeout > 0x0FFFFFFF) {
			printf(" Olaf: timeout waiting for USBRST in init\n");
			break;
		}
	}

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

	timeout = 0;
	while(((*(usbotg + usbotg_gintsts) >> 13) & 1) == 0) {
		usleep(50);
		timeout++;
		if(timeout > 0x0FFFFFFF) {
			printf(" Olaf: timeout waiting for ENUMDNE in init\n");
			break;
		}
	}

	uint32_t enum_speed = (*(usbotg + usbotg_dsts) >> 1) & 3;
	printf(" Olaf: enum speed(0=Hs, 1=Fs, 3=Res): %d\n", enum_speed);



}

void phy_init(void)
{

	/* phy_enable(); */
	phy_reset();
	phy_config();


#if 0
	/* Enable interrupt */
	*(usbotg + usbotg_gahbcfg) |= 1;

	/* Rx FIFO non-empty */
	*(usbotg + usbotg_gintsts) |= (1 << 4);

	/* Force device mode */
	*(usbotg + usbotg_gusbcfg) |= (1 << 30);

	/* OTG timeout calibration field and USB turnaround time */
	*(usbotg + usbotg_gusbcfg) |= (1 << 30);
#endif


}
