
/*
 * Phoenix-RTOS
 *
 * EHCI USB Physical Layer for imx6ull
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

#include <errno.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <phoenix/arch/imx6ull.h>
#include <stdio.h>

#include <hcd.h>

enum { phy_pwd, phy_pwd_set, phy_pwd_clr, phy_pwd_tog, phy_tx, phy_tx_set,
	phy_tx_clr, phy_tx_tog, phy_rx, phy_rx_set, phy_rx_clr, phy_rx_tog,
	phy_ctrl, phy_ctrl_set, phy_ctrl_clr, phy_ctrl_tog, phy_status,
	phy_debug = phy_status + 4, phy_debug_set, phy_debug_clr, phy_debug_tog,
	phy_debug0_status, phy_debug1 = phy_debug0_status + 4, phy_debug1_set,
	phy_debug1_clr, phy_debug1_tog, phy_version };


/* NOTE: This should be obtained using device tree */
static const hcd_info_t imx6ull_info[] = {
	{
		.type = "ehci",
		.hcdaddr = 0x02184200,
		.phyaddr = 0x020ca000,
		.clk = pctl_clk_usboh3,
		.irq = 74
	}
};


int hcd_getInfo(const hcd_info_t **info)
{
	*info = imx6ull_info;

	return sizeof(imx6ull_info) / sizeof(*imx6ull_info);
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
	*(hcd->phybase + phy_ctrl) |= 3 << 14;
	*(hcd->phybase + phy_ctrl) |= 2;
	*(hcd->phybase + phy_ctrl) |= 1 << 11;
}


void phy_reset(hcd_t *hcd)
{
	*(hcd->phybase + phy_ctrl) |= (1 << 31);
	*(hcd->phybase + phy_ctrl) &= ~(1 << 30);
	*(hcd->phybase + phy_ctrl) &= ~(1 << 31);
	*(hcd->phybase + phy_pwd) = 0;
}


void phy_initClock(hcd_t *hcd)
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


void phy_disableClock(hcd_t *hcd)
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

void phy_enableHighSpeedDisconnect(hcd_t *hcd, int enable)
{
	if (enable)
		*(hcd->phybase + phy_ctrl) |= 0x2;
	else
		*(hcd->phybase + phy_ctrl) &= ~0x2;
}

int phy_init(hcd_t *hcd)
{
	off_t offs;

	offs = hcd->info->phyaddr % _PAGE_SIZE;
	hcd->phybase = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, hcd->info->phyaddr - offs);
	if (hcd->phybase == MAP_FAILED)
		return -ENOMEM;
	hcd->phybase += (offs / sizeof(int));

	offs = hcd->info->hcdaddr % _PAGE_SIZE;
	hcd->base = mmap(NULL, 2 * _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, hcd->info->hcdaddr - offs);
	if (hcd->base == MAP_FAILED) {
		munmap((void *)hcd->phybase, _PAGE_SIZE);
		return -ENOMEM;
	}
	hcd->base += (offs / sizeof(int));

	phy_initClock(hcd);
	phy_reset(hcd);
	phy_config(hcd);

	return 0;
}
