/*
 * Phoenix-RTOS
 *
 * EHCI USB Physical Layer for ia32
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <phoenix/arch/ia32/ia32.h>
#include <stdio.h>
#include <sys/platform.h>

#include <hcd.h>


#define EHCI_MAP_SIZE (0x1000)

#define EHCI_HCD_CLASS  (0x0c03) /* Serial bus controller / USB controller */
#define EHCI_HCD_PROGIF (0x20)   /* EHCI */

#define EHCI_HCCPARAMS_EECP (0x8)


int hcd_getInfo(const hcd_info_t **info)
{
	platformctl_t pctl;
	int i, err;

	hcd_info_t *hcd_info = NULL;

	pctl.action = pctl_get;
	pctl.type = pctl_pci;
	pctl.pci.id.cl = EHCI_HCD_CLASS;
	pctl.pci.id.progif = EHCI_HCD_PROGIF;
	pctl.pci.dev.bus = 0;
	pctl.pci.dev.dev = 0;
	pctl.pci.dev.func = 0;
	pctl.pci.caps = NULL;

	i = 0;
	for (;;) {
		pctl.pci.id.vendor = PCI_VENDOR_INTEL;
		pctl.pci.id.device = PCI_ANY;
		pctl.pci.id.subvendor = PCI_ANY;
		pctl.pci.id.subdevice = PCI_ANY;

		err = platformctl(&pctl);

		if (err == -ENODEV) {
			break;
		}

		if (err < 0) {
			fprintf(stderr, "phy: pctl_get failed\n");
			return err;
		}

		if (pctl.pci.dev.progif != EHCI_HCD_PROGIF) {
			pctl.pci.dev.func++;
			continue;
		}

		hcd_info = realloc(hcd_info, sizeof(hcd_info_t) * (i + 1));
		if (hcd_info == NULL) {
			return -ENOMEM;
		}
		memset((void *)(hcd_info + i), 0, sizeof(hcd_info_t));

		sprintf(hcd_info[i].type, "ehci");
		hcd_info[i].hcdaddr = pctl.pci.dev.resources[0].base;
		hcd_info[i].irq = pctl.pci.dev.irq;
		hcd_info[i].pci_devId.bus = pctl.pci.dev.bus;
		hcd_info[i].pci_devId.dev = pctl.pci.dev.dev;
		hcd_info[i].pci_devId.func = pctl.pci.dev.func;

		i++;
		pctl.pci.dev.func++;
	}

	*info = hcd_info;

	return i;
}


static void phy_config(hcd_t *hcd)
{
	platformctl_t pctl;
	int err;
	uint8_t eecp;
	short bus = hcd->info->pci_devId.bus;
	short dev = hcd->info->pci_devId.dev;
	short func = hcd->info->pci_devId.func;

	pctl.action = pctl_set;
	pctl.type = pctl_pcicfg;
	pctl.pcicfg.dev.bus = bus;
	pctl.pcicfg.dev.dev = dev;
	pctl.pcicfg.dev.func = func;
	pctl.pcicfg.cfg = pci_cfg_busmaster;
	pctl.pcicfg.enable = 1;

	err = platformctl(&pctl);
	if (err < 0) {
		fprintf(stderr, "phy: setting busmaster failed: %d\n", err);
	}

	pctl.pcicfg.cfg = pci_cfg_memoryspace;
	pctl.pcicfg.enable = 1;

	err = platformctl(&pctl);
	if (err < 0) {
		fprintf(stderr, "phy: setting memoryspace failed: %d\n", err);
	}

	pctl.pcicfg.cfg = pci_cfg_interruptdisable;
	pctl.pcicfg.enable = 0;

	err = platformctl(&pctl);
	if (err < 0) {
		fprintf(stderr, "phy: enabling interrupts failed: %d\n", err);
	}

	eecp = (*(uint32_t *)((char *)hcd->base + EHCI_HCCPARAMS_EECP) >> 8) & 0xFF;

	if (eecp >= 0x40) {
		/* Take ownership of the controller from BIOS */
		pctl.action = pctl_set;
		pctl.type = pctl_usbownership;
		pctl.usbownership.dev.bus = bus;
		pctl.usbownership.dev.dev = dev;
		pctl.usbownership.dev.func = func;
		pctl.usbownership.osOwned = 1;
		pctl.usbownership.eecp = eecp;

		err = platformctl(&pctl);
		if (err < 0) {
			fprintf(stderr, "phy: taking controller ownership from BIOS failed: %d\n", err);
		}
	}
}


void phy_enableHighSpeedDisconnect(hcd_t *hcd, int enable)
{
}


int phy_init(hcd_t *hcd)
{
	off_t offs;

	hcd->phybase = NULL;

	offs = hcd->info->hcdaddr % _PAGE_SIZE;
	hcd->base = mmap(NULL, EHCI_MAP_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, hcd->info->hcdaddr - offs);
	if (hcd->base == MAP_FAILED) {
		return -ENOMEM;
	}
	hcd->base += (offs / sizeof(int));

	phy_config(hcd);

	return 0;
}
