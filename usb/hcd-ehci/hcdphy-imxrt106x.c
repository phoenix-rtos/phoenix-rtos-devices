/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ehci/hcdphy-imxrt106x.c
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


enum { hcdphy_pwd, hcdphy_pwd_set, hcdphy_pwd_clr, hcdphy_pwd_tog, hcdphy_tx, hcdphy_tx_set,
	hcdphy_tx_clr, hcdphy_tx_tog, hcdphy_rx, hcdphy_rx_set, hcdphy_rx_clr, hcdphy_rx_tog,
	hcdphy_ctrl, hcdphy_ctrl_set, hcdphy_ctrl_clr, hcdphy_ctrl_tog, hcdphy_status,
	hcdphy_debug = hcdphy_status + 4, hcdphy_debug_set, hcdphy_debug_clr, hcdphy_debug_tog,
	hcdphy_debug0_status, hcdphy_debug1 = hcdphy_debug0_status + 4, hcdphy_debug1_set,
	hcdphy_debug1_clr, hcdphy_debug1_tog, hcdphy_version };


/* NOTE: This should be later implemented using device tree */
static const hcd_info_t imxrt_info[] = {
	{ .type = "ehci",
	  .hcdaddr = 0x402e0000,
	  .phyaddr = 0x400da000,
	  .clk = pctl_clk_usboh3,
	  .irq = 128 },
};


int hcd_getInfo(const hcd_info_t **hcinfo)
{
	*hcinfo = imxrt_info;

	return sizeof(imxrt_info) / sizeof(hcd_info_t);
}


void hcdphy_dumpRegisters(hcd_t *hcd, FILE *stream)
{
	fprintf(stream, "%18s: %08x", "hcdphy_pwd", *(hcd->phybase + hcdphy_pwd));
	fprintf(stream, "%18s: %08x\n", "hcdphy_tx", *(hcd->phybase + hcdphy_tx));
	fprintf(stream, "%18s: %08x", "hcdphy_rx", *(hcd->phybase + hcdphy_rx));
	fprintf(stream, "%18s: %08x\n", "hcdphy_ctrl", *(hcd->phybase + hcdphy_ctrl));
	fprintf(stream, "%18s: %08x", "hcdphy_status", *(hcd->phybase + hcdphy_status));
	fprintf(stream, "%18s: %08x\n", "hcdphy_debug", *(hcd->phybase + hcdphy_debug));
	fprintf(stream, "%18s: %08x", "hcdphy_debug0_status", *(hcd->phybase + hcdphy_debug0_status));
	fprintf(stream, "%18s: %08x\n", "hcdphy_debug1", *(hcd->phybase + hcdphy_debug1));
	fprintf(stream, "%18s: %08x\n\n", "hcdphy_version", *(hcd->phybase + hcdphy_version));
}


void hcdphy_config(hcd_t *hcd)
{
	*(hcd->phybase + hcdphy_ctrl) |= 7 << 14;
	*(hcd->phybase + hcdphy_ctrl) |= 2;
	*(hcd->phybase + hcdphy_ctrl) |= 1 << 11 | 1;
}


void hcdphy_reset(hcd_t *hcd)
{
	*(hcd->phybase + hcdphy_ctrl) |= (1 << 31);
	*(hcd->phybase + hcdphy_ctrl) &= ~(1 << 30);
	*(hcd->phybase + hcdphy_ctrl) &= ~(1 << 31);
	*(hcd->phybase + hcdphy_pwd) = 0;
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


void hcdphy_init(hcd_t *hcd)
{
	/* No mmapping, since we are on NOMMU architecture */
	hcd->phybase = (volatile int *) hcd->info->phyaddr;
	hcd->base = (volatile int *) hcd->info->hcdaddr;

	setClock(hcd->info->clk, clk_state_run);
	hcdphy_reset(hcd);
	hcdphy_config(hcd);
}
