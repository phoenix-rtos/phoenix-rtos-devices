
/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ehci/phy.c
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
#include <phoenix/arch/imx6ull.h>
#include <stdio.h>

#include <hcd.h>

enum { hcdphy_pwd, hcdphy_pwd_set, hcdphy_pwd_clr, hcdphy_pwd_tog, hcdphy_tx, hcdphy_tx_set,
	hcdphy_tx_clr, hcdphy_tx_tog, hcdphy_rx, hcdphy_rx_set, hcdphy_rx_clr, hcdphy_rx_tog,
	hcdphy_ctrl, hcdphy_ctrl_set, hcdphy_ctrl_clr, hcdphy_ctrl_tog, hcdphy_status,
	hcdphy_debug = hcdphy_status + 4, hcdphy_debug_set, hcdphy_debug_clr, hcdphy_debug_tog,
	hcdphy_debug0_status, hcdphy_debug1 = hcdphy_debug0_status + 4, hcdphy_debug1_set,
	hcdphy_debug1_clr, hcdphy_debug1_tog, hcdphy_version };


/* NOTE: This should be obtained using device tree */
static const hcd_info_t imx6ull_info[] = {
	{ .type = "ehci",
	  .hcdaddr = 0x02184080,
	  .phyaddr = 0x020c9400,
	  .clk = pctl_clk_usboh3,
	  .irq = 128,  },
};

int hcd_getInfo(const hcd_info_t **info)
{
	info = imx6ull_info;

	return sizeof(imx6ull_info) / sizeof(hcd_info_t);
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
	*(hcd->phybase + hcdphy_ctrl) |= 3 << 14;
	*(hcd->phybase + hcdphy_ctrl) |= 2;
	*(hcd->phybase + hcdphy_ctrl) |= 1 << 11;
}

void phy_initPad(void)
{
	platformctl_t set_mux = {
		.action = pctl_set,
		.type = pctl_iomux,
		.iomux = { .mux = pctl_mux_sd1_d1, .sion = 0, .mode = 8 },
	};

	platformctl_t set_pad = {
		.action = pctl_set,
		.type = pctl_iopad,
		.iopad = { .pad = pctl_pad_sd1_d1, .hys = 0, .pus = 0, .pue = 0,
			.pke = 0, .ode = 0, .speed = 2, .dse = 4, .sre = 0 },
	};

	platformctl(&set_mux);
	platformctl(&set_pad);
}


void hcdphy_reset(hcd_t *hcd)
{
	*(hcd->phybase + hcdphy_ctrl) |= (1 << 31);
	*(hcd->phybase + hcdphy_ctrl) &= ~(1 << 30);
	*(hcd->phybase + hcdphy_ctrl) &= ~(1 << 31);
	*(hcd->phybase + hcdphy_pwd) = 0;
}


void hcdphy_initClock(hcd_t *hcd)
{
	platformctl_t ctl = (platformctl_t) {
		.action = pctl_set,
		.type = pctl_devclock,
		.devclock = {
			.dev = hcd->info->clk,
			.state = 3,
		}
	};

	platformctl(&ctl);
}


void hcdphy_disableClock(hcd_t *hcd)
{
	platformctl_t ctl = (platformctl_t) {
		.action = pctl_set,
		.type = pctl_devclock,
		.devclock = {
			.dev = hcd->info->clk,
			.state = 0,
		}
	};

	platformctl(&ctl);
}


void hcdphy_init(hcd_t *hcd)
{
	hcd->phybase = mmap(NULL, 2 * _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, hcd->info->phyaddr);
	hcd->base = mmap(NULL, 4 * _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, hcd->info->hcdaddr);

	hcdphy_initClock(hcd);
	hcdphy_initPad(hcd);
	hcdphy_reset(hcd);
	hcdphy_config(hcd);
}
