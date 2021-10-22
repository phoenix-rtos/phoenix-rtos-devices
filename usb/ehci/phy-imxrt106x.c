/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ehci/phy-imxrt106x.c
 *
 * Copyright 2018 Phoenix Systems
 * Copyright 2007 Pawel Pisarczyk
 * Author: Jan Sikorski, Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#include <sys/mman.h>
#include <sys/platform.h>
#include <phoenix/arch/imxrt.h>
#include <stdio.h>
#include <stdint.h>
#include <usbhost.h>
#include <hcd.h>


enum { phy_pwd, phy_pwd_set, phy_pwd_clr, phy_pwd_tog, phy_tx, phy_tx_set,
	phy_tx_clr, phy_tx_tog, phy_rx, phy_rx_set, phy_rx_clr, phy_rx_tog,
	phy_ctrl, phy_ctrl_set, phy_ctrl_clr, phy_ctrl_tog, phy_status,
	phy_debug = phy_status + 4, phy_debug_set, phy_debug_clr, phy_debug_tog,
	phy_debug0_status, phy_debug1 = phy_debug0_status + 4, phy_debug1_set,
	phy_debug1_clr, phy_debug1_tog, phy_version };


/* NOTE: This should be later implemented using device tree */
static const hcd_info_t imxrt_info[] = {
	{
		.type = "ehci",
		.hcdaddr = 0x402e0200,
		.phyaddr = 0x400da000,
		.clk = pctl_clk_usboh3,
		.irq = 128
	}
};


int hcd_getInfo(const hcd_info_t **hcinfo)
{
	*hcinfo = imxrt_info;

	return sizeof(imxrt_info) / sizeof(hcd_info_t);
}


void phy_dumpRegisters(hcd_t *hcd, FILE *stream)
{
	fprintf(stream, "%18s: %08x", "phy_pwd", *(hcd->phybase + phy_pwd));
	fprintf(stream, "%18s: %08x\n", "phy_tx", *(hcd->phybase + phy_tx));
	fprintf(stream, "%18s: %08x", "phy_rx", *(hcd->phybase + phy_rx));
	fprintf(stream, "%18s: %08x\n", "phy_ctrl", *(hcd->phybase + phy_ctrl));
	fprintf(stream, "%18s: %08x", "phy_status", *(hcd->phybase + phy_status));
	fprintf(stream, "%18s: %08x\n", "phy_debug", *(hcd->phybase + phy_debug));
	fprintf(stream, "%18s: %08x", "phy_debug0_status", *(hcd->phybase + phy_debug0_status));
	fprintf(stream, "%18s: %08x\n", "phy_debug1", *(hcd->phybase + phy_debug1));
	fprintf(stream, "%18s: %08x\n\n", "phy_version", *(hcd->phybase + phy_version));
}


void phy_config(hcd_t *hcd)
{
	*(hcd->phybase + phy_ctrl) |= 7 << 14;
	*(hcd->phybase + phy_ctrl) |= 1 << 11 | 1;
}


void phy_reset(hcd_t *hcd)
{
	*(hcd->phybase + phy_ctrl) |= (1 << 31);
	*(hcd->phybase + phy_ctrl) &= ~(1 << 30);
	*(hcd->phybase + phy_ctrl) &= ~(1 << 31);
	*(hcd->phybase + phy_pwd) = 0;
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

void phy_enableHighSpeedDisconnect(hcd_t *hcd, int enable)
{
	if (enable)
		*(hcd->phybase + phy_ctrl) |= 0x2;
	else
		*(hcd->phybase + phy_ctrl) &= ~0x2;
}


int phy_init(hcd_t *hcd)
{
	/* No mmapping, since we are on NOMMU architecture */
	hcd->phybase = (volatile int *)hcd->info->phyaddr;
	hcd->base = (volatile int *)hcd->info->hcdaddr;

	setClock(hcd->info->clk, clk_state_run);
	phy_reset(hcd);
	phy_config(hcd);

	return 0;
}
