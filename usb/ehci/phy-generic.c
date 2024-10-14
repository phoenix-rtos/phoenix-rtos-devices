/*
 * Phoenix-RTOS
 *
 * EHCI USB Physical Layer for ia32
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
#include <phoenix/arch/ia32/ia32.h>
#include <stdio.h>
#include <sys/platform.h>

#include <hcd.h>


hcd_info_t hcd_info[] = { {
	.type = "ehci",
	.hcdaddr = 0,
	.phyaddr = 0,
	.clk = -1,
	.irq = 11,
} };

// 0x00:0x04:0x0

int hcd_getInfo(const hcd_info_t **info)
{
	platformctl_t pctl;
	int err;

	/* TODO: Make this enumeration generic. Requirements: class = 0x0c03, BAR0 non-null
	  On QEMU, there's another 0x0c03 device, but it has a null BAR0.
	  Since it has lower vid:did, it will be found first based only on class lookup*/
	pctl.action = pctl_get;
	pctl.type = pctl_pci;
	pctl.pci.id.vendor = 0x8086;
	pctl.pci.id.device = 0x24cd;
	pctl.pci.id.subvendor = PCI_ANY;
	pctl.pci.id.subdevice = PCI_ANY;
	pctl.pci.dev.bus = 0;
	pctl.pci.dev.dev = 0;
	pctl.pci.dev.func = 0;
	pctl.pci.caps = NULL;

	pctl.pci.id.cl = 0x0c03;
	err = platformctl(&pctl);
	if (err < 0) {
		return err;
	}

	hcd_info[0].hcdaddr = pctl.pci.dev.resources[0].base;

	*info = hcd_info;

	return sizeof(hcd_info) / sizeof(*hcd_info);
}


void phy_dumpRegisters(hcd_t *hcd, FILE *stream)
{
}


void phy_config(hcd_t *hcd)
{
	platformctl_t pctl;
	int err;

	pctl.action = pctl_set;
	pctl.type = pctl_busmaster;
	pctl.busmaster.dev.bus = 0x0;
	pctl.busmaster.dev.dev = 0x4;
	pctl.busmaster.dev.func = 0x0;
	pctl.busmaster.enable = 1;

	err = platformctl(&pctl);
	if (err < 0) {
		fprintf(stderr, "pctl_busmaster failed: %d\n", err);
	}

	pctl.action = pctl_set;
	pctl.type = pctl_memoryspace;
	pctl.busmaster.dev.bus = 0x0;
	pctl.busmaster.dev.dev = 0x4;
	pctl.busmaster.dev.func = 0x0;
	pctl.busmaster.enable = 1;

	err = platformctl(&pctl);
	if (err < 0) {
		fprintf(stderr, "pctl_memoryspace failed: %d\n", err);
	}
}


void phy_reset(hcd_t *hcd)
{
}


void phy_initClock(hcd_t *hcd)
{
}


void phy_disableClock(hcd_t *hcd)
{
}


void phy_enableHighSpeedDisconnect(hcd_t *hcd, int enable)
{
}


int phy_init(hcd_t *hcd)
{
	off_t offs;

	hcd->phybase = NULL;

	printf("%x\n", hcd->info->hcdaddr);

	offs = hcd->info->hcdaddr % _PAGE_SIZE;
	hcd->base = mmap(NULL, 2 * _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, hcd->info->hcdaddr - offs);
	if (hcd->base == MAP_FAILED) {
		return -ENOMEM;
	}
	hcd->base += (offs / sizeof(int));

	phy_initClock(hcd);
	phy_reset(hcd);
	phy_config(hcd);

	return 0;
}
