/*
 * Phoenix-RTOS
 *
 * ehci/phy-imxrt1176.c
 *
 * Copyright 2022 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <sys/platform.h>
#include <phoenix/arch/imxrt1170.h>
#include <usbhost.h>
#include <hcd.h>
#include <board_config.h>


enum { phy_pwd = 0, phy_pwd_set, phy_pwd_clr, phy_pwd_tog, phy_tx, phy_tx_set,
	phy_tx_clr, phy_tx_tog, phy_rx, phy_rx_set, phy_rx_clr, phy_rx_tog, phy_ctrl,
	phy_ctrl_set, phy_ctrl_clr, phy_ctrl_tog, phy_status, phy_debug = 20,
	phy_debug_set, phy_debug_clr, phy_debug_tog, phy_debug0_status, phy_debug1 = 28,
	phy_debug1_set, phy_debug1_clr, phy_debug1_tog, phy_version, phy_pll_sic = 40,
	phy_pll_sic_set, phy_pll_sic_clr, phy_pll_sic_tog, phy_vbus_detect = 48,
	phy_vbus_detect_set, phy_vbus_detect_clr, phy_vbus_detect_tog, phy_vbus_detect_stat,
	phy_chrg_detect = 56, phy_chrg_detect_set, phy_chrg_detect_clr, phy_chrg_detect_tog,
	phy_chrg_detect_stat, phy_anactrl = 64, phy_anactrl_set, phy_anactrl_clr,
	phy_anactrl_tog, phy_loopback, phy_loopback_set, phy_loopback_clr, phy_loopback_tog,
	phy_loopback_hsfscnt, phy_loopback_hsfscnt_set, phy_loopback_hsfscnt_clr,
	phy_loopback_hsfscnt_tog, phy_trim_override_en, phy_trim_override_en_set,
	phy_trim_override_en_clr, phy_trim_override_en_tog };


static const hcd_info_t imxrt_info[] = {
#ifdef USB_EHCI_OTG1_ENABLED
	{
		.type = "ehci",
		.hcdaddr = 0x40430000,
		.phyaddr = 0x40434000,
		.clk = pctl_lpcg_usb,
		.irq = usb_otg1_irq,
	},
#endif
#ifdef USB_EHCI_OTG2_ENABLED
	{
		.type = "ehci",
		.hcdaddr = 0x4042c000,
		.phyaddr = 0x40438000,
		.clk = pctl_lpcg_usb,
		.irq = usb_otg2_irq,
	}
#endif
};


static int setClock(int dev, unsigned int state)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_lpcg;
	pctl.lpcg.op = pctl_lpcg_op_direct;
	pctl.lpcg.dev = dev;
	pctl.lpcg.state = (state != 0);

	return platformctl(&pctl);
}


int hcd_getInfo(const hcd_info_t **hcinfo)
{
	*hcinfo = imxrt_info;

	return sizeof(imxrt_info) / sizeof(hcd_info_t);
}


void phy_dumpRegisters(hcd_t *hcd, FILE *stream)
{
	fprintf(stream, "%22s: %08x", "phy_pwd", *(hcd->phybase + phy_pwd));
	fprintf(stream, "%22s: %08x\n", "phy_tx", *(hcd->phybase + phy_tx));
	fprintf(stream, "%22s: %08x", "phy_rx", *(hcd->phybase + phy_rx));
	fprintf(stream, "%22s: %08x\n", "phy_ctrl", *(hcd->phybase + phy_ctrl));
	fprintf(stream, "%22s: %08x", "phy_status", *(hcd->phybase + phy_status));
	fprintf(stream, "%22s: %08x\n", "phy_debug", *(hcd->phybase + phy_debug));
	fprintf(stream, "%22s: %08x", "phy_debug0_status", *(hcd->phybase + phy_debug0_status));
	fprintf(stream, "%22s: %08x\n", "phy_debug1", *(hcd->phybase + phy_debug1));
	fprintf(stream, "%22s: %08x", "phy_version", *(hcd->phybase + phy_version));
	fprintf(stream, "%22s: %08x\n", "phy_pll_sic", *(hcd->phybase + phy_pll_sic));
	fprintf(stream, "%22s: %08x", "phy_vbus_detect", *(hcd->phybase + phy_vbus_detect));
	fprintf(stream, "%22s: %08x\n", "phy_chrg_detect", *(hcd->phybase + phy_chrg_detect));
	fprintf(stream, "%22s: %08x", "phy_anactrl", *(hcd->phybase + phy_anactrl));
	fprintf(stream, "%22s: %08x\n", "phy_loopback", *(hcd->phybase + phy_loopback));
	fprintf(stream, "%22s: %08x", "phy_loopback_hsfscnt", *(hcd->phybase + phy_loopback_hsfscnt));
	fprintf(stream, "%22s: %08x\n", "phy_trim_override_en", *(hcd->phybase + phy_trim_override_en));
}


static void phy_start(hcd_t *hcd)
{
	uint32_t tmp;

	/* Release PHY soft reset */
	*(hcd->phybase + phy_ctrl) &= ~(1u << 31);

	/* Enable regulator */
	*(hcd->phybase + phy_pll_sic) |= (3 << 12) | (1 << 21);

	/* 24 MHz input clock (480MHz / 20) */
	tmp = *(hcd->phybase + phy_pll_sic) & ~(7 << 22);
	*(hcd->phybase + phy_pll_sic) = tmp | (3 << 22);

	/* Clear bypass bit */
	*(hcd->phybase + phy_pll_sic) &= ~(1 << 16);

	/* Enable usb clocks */
	*(hcd->phybase + phy_pll_sic) |= (1 << 6);

	/* Enable clock */
	*(hcd->phybase + phy_ctrl) &= ~(1 << 30);

	/* Wait until the PLL locks */
	while ((*(hcd->phybase + phy_pll_sic) & (1u << 31)) == 0)
		;
}


void phy_config(hcd_t *hcd)
{
	*(hcd->phybase + phy_ctrl) |= 7 << 14;
	*(hcd->phybase + phy_ctrl) |= (1 << 11) | 1;
	*(hcd->phybase + phy_pwd) = 0;
}


void phy_reset(hcd_t *hcd)
{
	*(hcd->phybase + phy_ctrl) |= (1u << 31);
	*(hcd->phybase + phy_ctrl) &= ~(1 << 30);
	*(hcd->phybase + phy_ctrl) &= ~(1u << 31);
	*(hcd->phybase + phy_pwd) = 0;
}


void phy_enableHighSpeedDisconnect(hcd_t *hcd, int enable)
{
	if (enable) {
		*(hcd->phybase + phy_ctrl) |= 0x2;
	}
	else {
		*(hcd->phybase + phy_ctrl) &= ~0x2;
	}
}


int phy_init(hcd_t *hcd)
{
	int res;

	if (sizeof(imxrt_info) == 0) {
		return -ENODEV;
	}

	res = setClock(hcd->info->clk, 1);
	if (res < 0) {
		return res;
	}

	/* NOMMU architecture, mmap not needed */
	hcd->phybase = (void *)hcd->info->phyaddr;
	hcd->base = (void *)hcd->info->hcdaddr;

	phy_start(hcd);
	phy_config(hcd);

	return EOK;
}
