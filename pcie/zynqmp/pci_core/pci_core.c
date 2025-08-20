/*
 * Phoenix-RTOS
 *
 * ZynqMP PCI Express driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala, Mikolaj Matalowski
 *
 * %LICENSE%
 */

#include <errno.h>
#include <sys/interrupt.h>
#include <sys/threads.h>

#include "pci_core.h"
#include "pci_link.h"
#include "../safe_printf.h"

void pci_enumerateRoot(uint32_t *pcie, pci_dev_t *root, uint8_t bus, int depth)
{
	pci_dev_t *last = NULL;

	/* Iterate over all devices connected to the certain bus */
	for (uint8_t dev = 0; dev < (bus ? 32 : 1); ++dev) {
		/**
		 * In case there is no device under certain identifier the bridge
		 * returns all "ones" on read
		 */
		uint16_t vendor_id = ecamRead16((uintptr_t)pcie, bus, dev, 0, PCI_VENDOR_ID);
		if (vendor_id == 0xffff) {
			continue;
		}
		uint16_t product_id = ecamRead16((uintptr_t)pcie, bus, dev, 0, PCI_VENDOR_ID + 2);

		/* Add device to the map */
		pci_dev_t *base_function = (pci_dev_t *)calloc(1, sizeof(pci_dev_t));
		if (NULL == base_function) {
			safe_printf("pcie: Failed to allocate memory for PCI dev during enumeration, level :%d\n", depth);
			return;
		}

		if (NULL == root->child) {
			root->child = base_function;
		}

		base_function->vid = vendor_id;
		base_function->pid = product_id;
		base_function->bus_no = bus;
		base_function->dev_no = dev;
		base_function->func_no = 0;
		last = base_function;

		/* Scan first function of device */
		pci_scanFunc(pcie, base_function, depth);

		/* Check if this is multi function device and scan them */
		bool multi = ecamRead8((uintptr_t)pcie, bus, dev, 0, PCI_HEADER_TYPE) & PCI_HT_MULTI_FUNC;
		if (multi) {
			safe_printf("pcie: multiple func device %u\n", dev);
			for (uint8_t fn = 1; fn < 8; fn++) {
				uint16_t multi_function_vid = ecamRead16((uintptr_t)pcie, bus, dev, fn, PCI_VENDOR_ID);
				/* This function does not exist */
				if (0xffff == multi_function_vid) {
					continue;
				}

				pci_dev_t *additional_function = (pci_dev_t *)calloc(1, sizeof(pci_dev_t));
				if (NULL == additional_function) {
					safe_printf("pcie: Failed to allocate memory for PCI dev during enumeration, level :%d\n", depth);
					return;
				}

				additional_function->bus_no = bus;
				additional_function->dev_no = dev;
				additional_function->func_no = fn;
				last->next = additional_function;
				last = additional_function;

				pci_scanFunc(pcie, additional_function, depth);
			}
		}
	}
}

void pci_scanFunc(uint32_t *pcie, pci_dev_t *device, int depth)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	/* Read informations about device */
	uint16_t vid = ecamRead16((uintptr_t)pcie, bus, dev, fun, PCI_VENDOR_ID);
	uint16_t pid = ecamRead16((uintptr_t)pcie, bus, dev, fun, PCI_DEVICE_ID);
	uint32_t class24 = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_CLASSCODE);
	uint8_t classBase = class24 >> 24;
	uint8_t classSub = (class24 >> 16) & 0xff;
	uint8_t progIF = (class24 >> 8) & 0xff;
	uint8_t hdr = ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_HEADER_TYPE) & 0x7f;

	safe_printf("pcie: %02x:%02x.%u ven %04x dev %04x class %02x%02x%02x hdr 0x%02x\n",
			bus, dev, fun,
			vid, pid,
			classBase, classSub, progIF,
			hdr);

	ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_COMMAND, 0);

	/* Enable MEM-space and Bus Master if still disabled */
	uint16_t cmd = ecamRead16((uintptr_t)pcie, bus, dev, fun, PCI_COMMAND);
	if (!(cmd & (PCI_CMD_MEM | PCI_CMD_MASTER))) {
		safe_printf("pcie: enable memory space and bus master\n");
		ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_COMMAND, cmd | 0x4 | 0x2 | 0x1 | 0x40 | 0x100);
	}

	/* If this is a PCI-PCI bridge program buses and recurse */
	if (hdr == 0x01) {
		pci_configureBridge(pcie, device, depth);
		/* Make sure to enable reporting of timeout on TLP while configuring bridge */
		pci_enableTimeoutTLP(pcie, device);
		safe_printf("DRIVER: CONFIGURE BRIDGE\n");
		pci_setMaxPayloadSize(pcie, device);
	}
	else {
		/* If this is a PCI endpoint, configure device and return */
		safe_printf("DRIVER: CONFIGURE ENDPOINT\n");
		pci_configureEndpoint(pcie, device);
		/* Print some info about this device */
		pci_printBARs(pcie, device);
	}
	// print_capabilities(pcie, device);
}


void pci_configureBridge(uint32_t *pcie, pci_dev_t *device, int depth)
{
	static uint8_t next_bus = 1;

	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	/* read once */
	uint8_t sec = ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_SECONDARY_BUS);

	if (sec == 0) {
		sec = next_bus++;

		uint32_t cur = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_PRIMARY_BUS) & 0xff000000;
		uint32_t val = cur |
				((uint32_t)0xFF << 16) | /* subordinate = ff */
				((uint32_t)sec << 8) |   /* secondary   = sec */
				bus;                     /* primary     = bus */

		safe_printf("pcie: write pri, sec, sub 0x%x \n", val);
		ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_PRIMARY_BUS, val);
		uint32_t val_read = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_PRIMARY_BUS);
		safe_printf("pcie: validate pri, sec, sub 0x%x \n", val_read);
	}

	uint8_t sub = ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_SUBORDINATE_BUS);
	safe_printf("pcie: bridge bus primary %u secondary %u subordinate %u\n",
			bus, sec, sub);

	pci_enumerateRoot(pcie, device, sec, depth + 1);

	/* final subordinate = highest bus encountered */
	uint32_t cur = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_PRIMARY_BUS) & 0xff00ffff;
	uint32_t val = cur | ((uint32_t)(next_bus - 1) << 16);
	ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_PRIMARY_BUS, val);

	/* Map physical addresses that bridge has to concerned with */
	pci_mapBARsPhysAddToBridge(pcie, device);
	pci_configureCapBridge(pcie, device);

	safe_printf("pcie: Setting bridge bus to PCIe address translation\n");

	/* Configure translations */
	writeReg(pcie, 0x208, (uint32_t)(PCI_BAR0_ADD >> 32));
	writeReg(pcie, 0x20C, (uint32_t)(PCI_BAR0_ADD & ~0u));

	writeReg(pcie, 0x210, (uint32_t)(PCI_BAR0_ADD >> 32));
	writeReg(pcie, 0x214, (uint32_t)(PCI_BAR0_ADD & ~0u));

	writeReg(pcie, 0x218, (uint32_t)(PCI_BAR0_ADD >> 32));
	writeReg(pcie, 0x21C, (uint32_t)(PCI_BAR0_ADD & ~0u));

	writeReg(pcie, 0x220, (uint32_t)(PCI_BAR0_ADD >> 32));
	writeReg(pcie, 0x224, (uint32_t)(PCI_BAR0_ADD & ~0u));

	writeReg(pcie, 0x228, (uint32_t)(PCI_BAR0_ADD >> 32));
	writeReg(pcie, 0x22C, (uint32_t)(PCI_BAR0_ADD & ~0u));

	writeReg(pcie, 0x230, (uint32_t)(PCI_BAR0_ADD >> 32));
	writeReg(pcie, 0x234, (uint32_t)(PCI_BAR0_ADD & ~0u));

	/* Write address range for MSI */
	writeReg(pcie, 0x14C, (uint32_t)(PCI_MSI_BA >> 32));
	writeReg(pcie, 0x150, (uint32_t)(PCI_MSI_BA & ~0x0u));

	safe_printf("pcie: Managed to set bridge bus to PCIe address translation\n");

	pci_bridgeEnable(pcie, device);
}


void pci_mapBARsPhysAddToBridge(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;
	/* Map physical address range for I/O BARs*/
	/* Currently mapping for I/O BARs is not supported */
	/* I/0 BARs are largely deprecated */

	/* Map physical address range for non-prefetchble 32-bit BARs */
	ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BRIDGE_MEM_BASE,
			(PCI_32NP_BAR_BA >> 16) | (PCI_32NP_BAR_LIMIT & (0xFFFF << 16)));

	/* Map physical address range for */
	ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BRIDGE_MEM_PREF_BASE,
			((uint32_t)(PCI_64P_BAR_BA & ~0u) >> 16) | (uint32_t)(PCI_64P_BAR_LIMIT & ~0u));
	ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BRIDGE_MEM_PREF_UPPER, (uint32_t)(PCI_64P_BAR_BA >> 32));
	ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BRIDGE_MEM_PREF_LIM, (uint32_t)(PCI_64P_BAR_LIMIT >> 32));
}

void pci_configureCapBridge(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	/* Check if there is capabilities list */
	uint16_t status = ecamRead16((uintptr_t)pcie, bus, dev, fun, PCI_STATUS);
	if (!(status & (1 << 4))) {
		return;
	}

	/* Read capabilities list pointer */
	uint8_t ptr = ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_CAP_PTR);
	while (ptr >= 0x40) {
		uint8_t cap_id = ecamRead8((uintptr_t)pcie, bus, dev, fun, ptr);
		uint8_t next = ecamRead8((uintptr_t)pcie, bus, dev, fun, ptr + 1);

		/* If this is PCIe capability header enable timeout on invalid TLPs and enable reporting of such TLPs */
		if (0x10 == cap_id) {
			uint32_t reg = ecamRead32((uintptr_t)pcie, bus, dev, fun, ptr + 0x28);
			safe_printf("pcie: %d:%d.%d Current device control 2 is %x\n", bus, dev, fun, reg);
			return;
		}

		if (next == 0) {
			break;
		}
		ptr = next;
	}
}

void pci_bridgeEnable(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	safe_printf("pcie: Trying to enable bridge\n");
	/* Enable I/O, memory and bus master */
	uint32_t ret = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_BRIDGE_CMD);
	ret |= 0x7;
	ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BRIDGE_CMD, ret);
	safe_printf("pcie: Root bridge\n");
}

void pci_configureEndpoint(uint32_t *pcie, pci_dev_t *device)
{
	pci_configureEndpointBARs(pcie, device);
	pci_setMaxPayloadSize(pcie, device);
	pci_disableCompletionTimeout(pcie, device);
	pci_enableEndpoint(pcie, device);
}

void pci_configureEndpointBARs(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	const int bar_count = (ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_HEADER_TYPE) & 0x7f) == 0 ? 6 : 2;

	for (int i = 0; i < bar_count; i++) {
		uint32_t bar_low = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4);
		if (0 == bar_low) {
			continue;
		}

		bool io = 0x1 == (bar_low & 0x1);
		bool is_64_bit = (bar_low & 0x4) == 0x4;
		bool prefetchble = (bar_low & (1 << 3)) != 0;
		uint64_t size = 0;
		uintptr_t bar_phys_add = (uintptr_t)-1;

		if (io) {
			/* Read out I/O BAR size */
			ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4, ~0u);
			size = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4) | (~0ul << 32);

			/* Parse size */
			size &= ~(0xF);
			size = (~size) + 1;

			bar_phys_add = pci_getNextIORange(size);
			if ((uintptr_t)-1 == bar_phys_add) {
				safe_printf("pcie: %d:%d.%d - Failed to allocate physical memory range for I/O BAR%d\n",
						bus, dev, fun, i);
				continue;
			}

			/* Write allocated physical address range to BAR register */
			ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4,
					(uint32_t)bar_phys_add);
		}
		else if (!is_64_bit && !prefetchble) {
			ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4, ~0u);
			size = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4) | (~0ul << 32);

			/* Parse size */
			size &= ~(0xF);
			size = (~size) + 1;

			/* Allocate physical memory*/
			bar_phys_add = pci_getNext32RangeNonPref(size);
			if ((uintptr_t)-1 == bar_phys_add) {
				safe_printf("pcie: %d:%d.%d - Failed to allocate physical memory range for non-prefetchble 32-bit BAR%d\n",
						bus, dev, fun, i);
				continue;
			}

			/* Write allocated physical address range to BAR register */
			ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4,
					(uint32_t)bar_phys_add);
		}
		else if (!is_64_bit && prefetchble) {
			ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4, ~0u);
			size = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4) | (~0ul << 32);

			/* Parse size */
			size &= ~(0xF);
			size = (~size) + 1;

			/* Allocate physical memory*/
			bar_phys_add = pci_getNext32Range(size);
			if ((uintptr_t)-1 == bar_phys_add) {
				safe_printf("pcie: %d:%d.%d - Failed to allocate physical memory range for prefetchble 32-bit BAR%d\n",
						bus, dev, fun, i);
				continue;
			}

			/* Write allocated physical address range to BAR register */
			ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4,
					(uint32_t)bar_phys_add);
		}
		else {
			/* This has to be 64-bit prefetchble BAR */
			// uint32_t bar_high = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + (i + 1) * 4);

			ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4, ~0u);
			ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + (i + 1) * 4, ~0u);

			size = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4) |
					(uint64_t)ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + (i + 1) * 4) << 32;

			/* Parse size */
			size &= ~0xFul;
			size = (~size) + 1;

			bar_phys_add = pci_getNext64Range(size);
			if ((uintptr_t)-1 == bar_phys_add) {
				safe_printf("pcie: %d:%d.%d - Failed to allocate physical memory range for 64-bit BAR%d\n",
						bus, dev, fun, i);
				i++;
				continue;
			}

			uint32_t low = (uint32_t)(bar_phys_add & ~0u);
			uint32_t high = (uint32_t)(bar_phys_add >> 32);

			ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4, low);
			ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + (i + 1) * 4, high);
			i++;
		}

		int j = is_64_bit ? i - 1 : i;
		device->bar[j] = (uintptr_t)mmap(NULL, size, PROT_WRITE | PROT_READ,
				MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)bar_phys_add);
		if ((uintptr_t)MAP_FAILED == device->bar[j]) {
			safe_printf("pcie: %d:%d.%d - Failed to map BAR%d: wanted %016lx:%lx\n",
					bus, dev, fun, j, bar_phys_add, size);
		}
		else {
			device->size[j] = size;
		}
	}
}

void pci_enableEndpoint(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	uint16_t cmd = ecamRead16((uintptr_t)pcie, bus, dev, fun, PCI_COMMAND);
	ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_COMMAND, cmd | 0x4 | 0x2 | 0x1 | 0x40 | 0x100);
}


void pci_enableRootComplex(uint32_t *pcie)
{
	/* Enable root complex - this should be done only after enumeration is complete */
	writeReg(pcie, 0x148, 1);
}

void pci_dumpBAR(pci_dev_t *device, int bar_number)
{
	for (int i = 0; i < device->size[bar_number] / 4; i++) {
		uint32_t data = *((volatile uint32_t *)device->bar[bar_number] + i);
		safe_printf("%lx : %0x\n", (uint64_t)va2pa((uint32_t *)device->bar[bar_number] + i), data);
	}
}

int pci_irqHandler(unsigned int no, void *data)
{
	/* What exactly should this handler do? */
	return 0;
}

/*
Note: All devices - root complex included - on the same PCIe
fabric have to work on same MPS. This value has to be minimum
over all devices. Normally we should find the minimum and set
it for all devices. For now however, we will set this to 128
bytes for both read request and write requests.
*/
void pci_setMaxPayloadSize(uint32_t *pcie, pci_dev_t *device)
{
	/* 0x10 is PCIe capability ID*/
	uint16_t pcie_capability_offset = pci_findCapability(pcie, device, 0x10u);

	/* Offset of device control register of PCIe capability is 0x8 */
	uint32_t reg = ecamRead32(
			(uintptr_t)pcie,
			device->bus_no,
			device->dev_no,
			device->func_no,
			pcie_capability_offset + 0x8u);

	safe_printf("DRIVER: PCIe device control register: 0x%x\n", reg);

	/* Set unsuported request reporting */
	reg |= (1 << 3);
	/* Set MPS to 128bytes */
	reg &= ~(0b111 << 5);
	/* Read can be 4096bytes mulit-TLP response */
	reg |= (0b111 << 12);

	ecamWrite32(
			(uintptr_t)pcie,
			device->bus_no,
			device->dev_no,
			device->func_no,
			pcie_capability_offset + 0x8u,
			reg);

	uint32_t test = ecamRead32(
			(uintptr_t)pcie,
			device->bus_no,
			device->dev_no,
			device->func_no,
			pcie_capability_offset + 0x8u);

	safe_printf("DRIVER: PCIe device control register: 0x%x (should be 0x%x)\n", test, reg);
}

/* This is not really safe and should be used only for debugging */
void pci_disableCompletionTimeout(uint32_t *pcie, pci_dev_t *device)
{
	/* 0x10 is PCIe capability ID*/
	uint16_t pcie_capability_offset = pci_findCapability(pcie, device, 0x10u);

	uint32_t dev_ctrl2_reg = ecamRead32(
			(uintptr_t)pcie,
			device->bus_no,
			device->dev_no,
			device->func_no,
			pcie_capability_offset + 0x28u);

	safe_printf("DRIVER: PCIe device control register 2: 0x%x\n", dev_ctrl2_reg);
	dev_ctrl2_reg |= (1 << 4);

	ecamWrite32(
			(uintptr_t)pcie,
			device->bus_no,
			device->dev_no,
			device->func_no,
			pcie_capability_offset + 0x28u,
			dev_ctrl2_reg);

	uint32_t test = ecamRead32(
			(uintptr_t)pcie,
			device->bus_no,
			device->dev_no,
			device->func_no,
			pcie_capability_offset + 0x28u);

	safe_printf("DRIVER: PCIe device control register 2: 0x%x (should be 0x%x)\n", test, dev_ctrl2_reg);
}

void pci_enableTimeoutTLP(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	/* Check if there is capabilities list */
	uint16_t status = ecamRead16((uintptr_t)pcie, bus, dev, fun, PCI_STATUS);
	if (!(status & (1 << 4))) {
		return;
	}

	/* Read capabilities list pointer */
	uint8_t ptr = ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_CAP_PTR);
	while (ptr >= 0x40) {
		uint8_t cap_id = ecamRead8((uintptr_t)pcie, bus, dev, fun, ptr);
		uint8_t next = ecamRead8((uintptr_t)pcie, bus, dev, fun, ptr + 1);

		/* If this is AER capability structure - enable reporting */
		if (cap_id == 0x01) {
			safe_printf("pcie: Current value of AER UEMR is 0x%x\n", ecamRead32((uintptr_t)pcie, bus, dev, fun, (ptr + 0x08)));
			ecamWrite32((uintptr_t)pcie, bus, dev, fun, ptr + 0x08, ~(1u << 14));
			safe_printf("pcie: Updated value of AER UEMR is 0x%x\n", ecamRead32((uintptr_t)pcie, bus, dev, fun, (ptr + 0x08)));
		}

		if (next == 0) {
			break;
		}
		ptr = next;
	}
}

/* Return offset from on ECAM where capability with specified ID resides */
uint16_t pci_findCapability(uint32_t *pcie, pci_dev_t *device, uint32_t id)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	/* Check if there is capabilities list */
	uint16_t status = ecamRead16((uintptr_t)pcie, bus, dev, fun, PCI_STATUS);
	if (!(status & (1 << 4))) {
		return 0;
	}

	/* Read capabilities list pointer */
	uint8_t ptr = ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_CAP_PTR);
	while (ptr >= 0x40) {
		uint8_t cap_id = ecamRead8((uintptr_t)pcie, bus, dev, fun, ptr);
		uint8_t next = ecamRead8((uintptr_t)pcie, bus, dev, fun, ptr + 1);

		if (cap_id == id) {
			return ptr;
		}

		if (next == 0) {
			break;
		}
		ptr = next;
	}

	return 0;
}

int pci_enableMsiInterrupt(pci_context_t *context, pci_dev_t *device)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	uint16_t capability = pci_findCapability(context->pcie, device, PCI_CAP_MSI_ID);
	if (capability == 0) {
		return -1;
	}

	/* Configure MSI capability structure */
	/* Make sure to disable INTx */
	uint16_t cmd = ecamRead16((uintptr_t)context->pcie, bus, dev, fun, PCI_COMMAND);
	cmd |= 1 << 10; /* Disable assertion of INTx line */
	ecamWrite32((uintptr_t)context->pcie, bus, dev, fun, PCI_COMMAND, cmd);

	uint32_t control_register = ecamRead32((uintptr_t)context->pcie, bus, dev, fun, capability);
	/* Enable generation of MSI message */
	control_register |= 1 << 16;
	/* Set addres to which MSI message shall be targeted, must match one configured in RC */
	uint64_t msiAdd = pci_getNextMSIadd();
	uint32_t msiLowAdd = (uint32_t)(msiAdd & ~0x0u);
	uint32_t msiHighAdd = (uint32_t)(msiAdd >> 32);
	/* For now we can use simply incrementing ID */
	uint32_t msiPayload = pci_getNextMsiPayload();

	/* Since we need to look up which device has given MSI payload we maintain RB tree */
	device->msi_payload = msiPayload;
	lib_rbInsert(&context->msi_rb_tree, &device->msi_payload_rbnode);

	ecamWrite32((uintptr_t)context->pcie, bus, dev, fun, capability, control_register);
	ecamWrite32((uintptr_t)context->pcie, bus, dev, fun, capability + 0x4, msiLowAdd);
	ecamWrite32((uintptr_t)context->pcie, bus, dev, fun, capability + 0x8, msiHighAdd);
	ecamWrite32((uintptr_t)context->pcie, bus, dev, fun, capability + 0xC, msiPayload);

	return 0;
}

static int pci_msiRbTreeComp(rbnode_t *node1, rbnode_t *node2)
{
	pci_dev_t *dev1 = lib_treeof(pci_dev_t, msi_payload_rbnode, node1);
	pci_dev_t *dev2 = lib_treeof(pci_dev_t, msi_payload_rbnode, node2);
	return dev1->msi_payload > dev2->msi_payload;
}

int pci_initContext(pci_context_t *context)
{
	if (NULL == context) {
		return -EINVAL;
	}

	// lib_rbInit(&context->msi_rb_tree, pci_msiRbTreeComp, NULL);

	context->fpga_gpio = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, PCI_FPGA_GPIO_ADD);

	if (NULL == context->fpga_gpio) {
		return -ENOMEM;
	}

	context->pcie = mmap(NULL, 0x10000000, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, PCI_ROOT_COMPLEX_BASE_ADD);

	if (NULL == context->pcie) {
		munmap(context->fpga_gpio, 0x1000);
		return -ENOMEM;
	}

	/* Make sure to set correct AXI state */
	pci_resetAXIandRC(context);
	pci_prepareAXI();

	phy_link_status phy_link_status = pci_checkLinkStatus(context->pcie);
	if (phy_link_status.link_up) {
		safe_printf("pcie: phy LINK UP, link rate: %u, link width x%u, ltssm state: %s\n",
				phy_link_status.link_rate,
				phy_link_status.link_width,
				pci_ltssm_state2str(phy_link_status.ltssm_state));
	}
	else {
		safe_printf("pcie: PHY LINK DOWN, phy status reg: 0x%x\n", phy_link_status.raw_register_val);
		return -EAGAIN;
	}

	/* Disable interrupt */
	writeReg(context->pcie, 0x13c, 0x0);
	/* Clear pending interrupts */
	writeRegMsk(context->pcie, 0x138, 0x0ff30fe9, 0x0ff30fe9);
	/* MSI decode mode - route to root complex */
	writeReg(context->pcie, 0x178, 0xffffffff);
	writeReg(context->pcie, 0x17c, 0xffffffff);
	/* TODO: Possibly the same thing should eb be done for leagcy INTX interrupts */

	/* Make sure that root complex is turned off before enumeration */
	writeReg(context->pcie, 0x148, 0);

	context->device_root.bus_no = 0xff;
	context->device_root.dev_no = 0xff;
	context->device_root.func_no = 0xff;

	/* Enumerate devices and save in PCI context */
	pci_enumerateRoot((uint32_t *)context->pcie, &context->device_root, 0, 0);

	safe_printf("pcie: Managed to enumerate root complex\n");

	/* Create interrupt handling context */
	struct {
		handle_t mutex;
		handle_t cond;
		pci_context_t *context;
	} *irq_handler_thread_args = malloc(sizeof(*irq_handler_thread_args));

	if (NULL == irq_handler_thread_args) {
		munmap(context->fpga_gpio, 0x1000);
		munmap(context->pcie, 0x10000000);
		return -ENOMEM;
	}

	irq_handler_thread_args->context = context;

	if (condCreate(&(irq_handler_thread_args->cond)) < 0) {
		munmap(context->fpga_gpio, 0x1000);
		munmap(context->pcie, 0x10000000);
		free(irq_handler_thread_args);
		return -ENOMEM;
	}

	if (mutexCreate(&(irq_handler_thread_args->mutex)) < 0) {
		resourceDestroy(irq_handler_thread_args->cond);
		munmap(context->fpga_gpio, 0x1000);
		munmap(context->pcie, 0x10000000);
		free(irq_handler_thread_args);
		return -ENOMEM;
	}

	void *irq_handler_stack = malloc(PCI_IRQ_HANDELR_THREAD_STACK_SIZE);
	if (NULL == irq_handler_stack) {
		resourceDestroy(irq_handler_thread_args->mutex);
		resourceDestroy(irq_handler_thread_args->cond);
		munmap(context->fpga_gpio, 0x1000);
		munmap(context->pcie, 0x10000000);
		free(irq_handler_thread_args);
		return -ENOMEM;
	}

	safe_printf("pcie: Allocated resources for irq handling thread\n");

	beginthread(pci_irqHandlerThread, PCI_IRQ_HANDLER_THREAD_PRIO,
			irq_handler_stack, PCI_IRQ_HANDELR_THREAD_STACK_SIZE,
			irq_handler_thread_args);

	safe_printf("pcie: Started irq handling thread\n");

	interrupt(PCI_ROOT_COMPLEX_IRQ_NO, pci_irqCallback, context,
			irq_handler_thread_args->cond, NULL);

	safe_printf("pcie: Registered irq handler\n");

	/* Re-enable interrupts */
	/* Since for now only MSI interrupts are implemented we will ignore all
	other interrupt sources. */
	writeReg(context->pcie, 0x13c, 1u << 17);
	writeReg(context->pcie, 0x134, 0);

	pci_enableRootComplex(context->pcie);
	safe_printf("pcie: Root complex enabled\n");

	return 0;
}

int pci_irqCallback(unsigned int irq_no, void *arg)
{

	pci_context_t *context = arg;
	/* Disable irq pin assertion */
	writeReg(context->pcie, 0x13c, 0u);
	context->rc_status_reg = readReg(context->pcie, 0x138);

	uint32_t rc_status = readReg(context->pcie, 0x148);

	/* Clear error FIFO overflow if happend */
	if ((rc_status & (1 << 17)) != 0) {
		writeReg(context->pcie, 0x148, rc_status & (1 << 17));
	}

	while ((rc_status & (1 << 16))) {
		/* Clear error reporting FIFO */
		uint32_t error_msg = readReg(context->pcie, 0x154);
		int i = 0;
		while ((error_msg & (1 << 18)) != 0) {
			i++;
			if (i > 100)
				printf(".");
			/* !!! READS NON DESTRUCTIVE !!! */
			writeReg(context->pcie, 0x154, error_msg);
			error_msg = readReg(context->pcie, 0x154);
		}

		/* Clear error FIFO overflow */
		writeReg(context->pcie, 0x148, (1 << 17));

		rc_status = readReg(context->pcie, 0x148);
	}

	/* Clear EPs interrupt FIFO overflow if happend */
	if ((rc_status & (1 << 19)) != 0) {
		writeReg(context->pcie, 0x148, rc_status & (1 << 19));
	}

	int i = 0;
	/* Handle interrupts incoming from endpoints */
	/* Clear EP irq FIFO */
	while ((readReg(context->pcie, 0x148) & (1u << 18)) != 0) {
		uint32_t irq_status_register = readReg(context->pcie, 0x158);
		i++;
		/* No IRQ */
		// if ((irq_status_register & (1 << 31)) != 0)
		// 	break;
		if (i > 100) {
			printf("Crash");
		}

		pci_irq_info_t *irq_info = &context->irq_queue[context->irq_queue_wr_ptr];
		context->irq_queue_wr_ptr = (context->irq_queue_wr_ptr + 1) % PCI_CONTEXT_IRQ_BUF_SIZE;
		irq_info->requester_id = irq_status_register & ((1 << 16) - 1);
		/* Check if MSI or INTX and enqueue for handler thread */
		if ((irq_status_register & (1 << 30)) != 0) {
			/* MSI */
			irq_info->type = PCI_MSI_IRQ;
			irq_info->msi_address = (irq_status_register >> 16) & ((1 << 10) - 1);
			irq_info->msi_payload = readReg(context->pcie, 0x15c);
		}
		else {
			/* INTX*/
			irq_info->type = PCI_INTX_IRQ;
			if ((irq_status_register & (1 << 27)) == 0) {
				irq_info->intx_line = irq_status_register & (1 << 28) ? INTB : INTA;
			}
			else {
				irq_info->intx_line = irq_status_register & (1 << 28) ? INTD : INTC;
			}
		}
		/* Since reads are non-destructive we need to write to this
		register to get next irq from FIFO */
		writeReg(context->pcie, 0x158, 1u << 31);
	}

	/* Clear all interrupts */
	/* TODO: handle other interrupt sources not only EPs */
	writeReg(context->pcie, 0x138, ~0u);

	/* Enable irq pin assertion */
	/* TODO: handle level triggered INTX interrupt */
	writeReg(context->pcie, 0x13c, ~0u);
	return 1;
}

static pci_dev_t *pci_getDevWithMsi(pci_dev_t *root, uint32_t msi_payload)
{
	if (NULL == root) {
		return NULL;
	}
	if (root->msi_payload == msi_payload) {
		return root;
	}

	pci_dev_t *child = root->child;
	while (child != NULL) {
		pci_dev_t *ret = pci_getDevWithMsi(child, msi_payload);
		if (ret != NULL) {
			return ret;
		}
		child = child->next;
	}

	return pci_getDevWithMsi(root->next, msi_payload);
}

void pci_irqHandlerThread(void *data)
{
	struct {
		handle_t mutex;
		handle_t cond;
		pci_context_t *context;
	} *arg = data;

	pci_context_t *context = arg->context;
	pci_dev_t device_root = arg->context->device_root;
	uint64_t counter = 0;

	while (1) {
		mutexLock(arg->mutex);
		condWait(arg->cond, arg->mutex, 0);

		safe_printf("pci: Received IRQ - %lu : %d/%d - RC status - 0x%x\n", counter++,
				context->irq_queue_rd_ptr, context->irq_queue_wr_ptr, context->rc_status_reg);

		if (0x400 == context->rc_status_reg) {
			/* We have a non-fatal inspect Hailo? - we have known MSI paylaod*/
			pci_dev_t *hailo_device = pci_getDevWithMsi(&context->device_root, 0xfafa);


			uint16_t aer_capability_offset = pci_findCapability(context->pcie, hailo_device, 0x01);
			uint16_t pcie_capability_offest = pci_findCapability(context->pcie, hailo_device, 0x10);

			if (aer_capability_offset != 0) {

				uint32_t uncorrectable_status_register = ecamRead32(
						(uintptr_t)context->pcie,
						hailo_device->bus_no,
						hailo_device->dev_no,
						hailo_device->func_no,
						aer_capability_offset + 0x4);

				safe_printf("DRIVER: Inspect AER register: 0x%x\n", uncorrectable_status_register);

				ecamWrite32(
						(uintptr_t)context->pcie,
						hailo_device->bus_no,
						hailo_device->dev_no,
						hailo_device->func_no,
						aer_capability_offset + 0x4,
						uncorrectable_status_register);
			}
			else {
				safe_printf("There is no AER capability\n");
			}

			if (pcie_capability_offest != 0) {
				uint32_t device_status_and_control_register = ecamRead32(
						(uintptr_t)context->pcie,
						hailo_device->bus_no,
						hailo_device->dev_no,
						hailo_device->func_no,
						aer_capability_offset + 0x08);

				uint16_t device_status_register = (uint16_t)(device_status_and_control_register >> 16);
				safe_printf("DRIVER: Inspect PCIe status register: 0x%x\n", device_status_register);
			}
			else {
				safe_printf("DRIVER: There is no PCIE capability\n");
			}
		}

		while (context->irq_queue_rd_ptr != context->irq_queue_wr_ptr) {
			pci_irq_info_t *irq_info = &(context->irq_queue[context->irq_queue_rd_ptr]);
			context->irq_queue_rd_ptr = (context->irq_queue_rd_ptr + 1) % PCI_CONTEXT_IRQ_BUF_SIZE;
			safe_printf("pci: IRQ FIFO read - %d\n", context->irq_queue_rd_ptr);

			if (irq_info->type == PCI_MSI_IRQ) {
				/* Handle MSI irq */
				/* TODO: From msi_rbtree find device that is registered with given msi_payload, for now search normally */
				pci_dev_t *device = pci_getDevWithMsi(&device_root, irq_info->msi_payload);

				safe_printf("pci: MSI IRQ - payload %x\n", irq_info->msi_payload);
				safe_printf("pci: MSI IRQ - found device %p\n", device);

				if (NULL != device) {
					if (device->private_handler != NULL) {
						device->private_handler(device->irq, device->handler_data);
					}
				}
			}
			else {
				safe_printf("pci: INTx IRQ\n");
				/* Handle INTx irq*/
				/* TODO: Handle INTx irq */
				/* In this case we have no other way but to allow each handler check if their device triggered irq */
			}
		}

		mutexUnlock(arg->mutex);
	}
}

void pci_prepareAXI()
{
	/* Fix AXI configuration */
	/* Fix slave data width */
	volatile uint32_t *axi_slave1_width_reg = (volatile uint32_t *)mmap(NULL, 4, PROT_WRITE | PROT_READ,
			MAP_PHYSMEM | MAP_ANONYMOUS, -1, AXI_HPM1_FPD_REG);
	if (NULL == axi_slave1_width_reg) {
		safe_printf("pcie: mmap failed for this AXI width register\n");
		return;
	}

	uint32_t configuration = (*axi_slave1_width_reg & ~((1 << 10) | (1 << 11))) | (AXI_WIDTH_CFG_128B << 10);

	*axi_slave1_width_reg = configuration;
	wmb();

#define AXI_AFIFM0_RDCTRL 0xFD360000
#define AXI_AFIFM0_WRCTRL 0xFD360014

	/* Manually set HPD HPC0 width */
	volatile uint32_t *axi_afifm0_rdctrl = (volatile uint32_t *)mmap(NULL, 1024, PROT_WRITE | PROT_READ,
			MAP_PHYSMEM | MAP_ANONYMOUS, -1, AXI_AFIFM0_RDCTRL);

	if (NULL == axi_afifm0_rdctrl) {
		safe_printf("pcie: mmap failed on AXI AFIFM RDCTRL register");
		return;
	}

	*axi_afifm0_rdctrl = 1;
	*((char *)axi_afifm0_rdctrl + 0x14) = 1;
	wmb();

	munmap((void *)axi_slave1_width_reg, 4);
	munmap((void *)axi_afifm0_rdctrl, 1024);
}

static void deassertAxiInterconnectsReset(void)
{
	safe_printf("Deasserting AXI interconnect\n");
	platformctl_t ctl3 = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_s_axi_hpc_0_fpd,
		.devreset.state = 0,
	};
	int ret = platformctl(&ctl3);
	if (ret != 0) {
		safe_printf("pcie: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hpc_1_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		safe_printf("pcie: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hp_0_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		safe_printf("pcie: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hp_1_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		safe_printf("pcie: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hpc_2_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		safe_printf("pcie: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hpc_3_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		safe_printf("pcie: fail to deassert reset\n");
	}
}

void pci_resetAXIandRC(pci_context_t *context)
{
	/* Deassert reset on AXI Interconnect between PS and PL */
	deassertAxiInterconnectsReset();
	usleep(3 * 1000);

	/* Assert PCI Express pin for some time */
	// writeReg(gpioBase, 0x08, 0x0);
	// usleep(100 * 1000);

	// /* Deassert PCI Express reset pin */
	// writeReg(gpioBase, 0x08, 0x1);
	// /* Let PCI Express node initialise */
	// usleep(100 * 1000);

	// /* Deassert reset on AXI PCI Express bridge IP Core */
	// writeReg(gpioBase, 0x08, 0x3);
	// /* Let bridge turn link up */
	// usleep(100 * 1000);

	/*
	Currently GPIO pins are not connected to PERST in the endpoint
	and may not be connected in the future anyays. To reset the bus
	use bridge control register in RC bridge type1 header.
	*/
	uint32_t reg = ecamRead32((uintptr_t)context->pcie, 0, 0, 0, 0x3c);
	reg |= (1u << (16 + 6));
	ecamWrite32((uintptr_t)context->pcie, 0, 0, 0, 0x3c, reg);

	reg = ecamRead32((uintptr_t)context->pcie, 0, 0, 0, 0x3e);
	safe_printf("pcie: Reseting the bus: %d\n", reg & (1 << 22) >> 22);

	/* Wait 2ms according to spec */
	usleep(2 * 1000);

	reg &= ~(1u << (16 + 6));
	ecamWrite32((uintptr_t)context->pcie, 0, 0, 0, 0x3e, reg);

	reg = ecamRead32((uintptr_t)context->pcie, 0, 0, 0, 0x3e);
	safe_printf("pcie: Current secondary bus reset value %d\n", reg & (1 << 22) >> 22);

	/* Wait for the link */
	usleep(50 * 1000);
}

void pci_readBridgeInfoReg(uint32_t *pcie)
{
	uint32_t bridge_info = readReg(pcie, 0x130);

	bool gen2_capable = bridge_info & 0x1;
	bool root_port_present = bridge_info & 0x2;
	bool gen3_capable = bridge_info & 0x4;

	char *gen_cap = NULL;
	if (gen3_capable) {
		gen_cap = "PCI Express 3";
	}
	else if (gen2_capable) {
		gen_cap = "PCI Express 2";
	}
	else {
		gen_cap = "Unknown";
	}

	char *root_port_present_text = NULL;
	if (root_port_present) {
		root_port_present_text = "present";
	}
	else {
		root_port_present_text = "not present";
	}

	safe_printf("pcie: bridge generation: %s, root port: %s\n",
			gen_cap, root_port_present_text);
}

void pci_printTree(pci_dev_t *root, int depth)
{
	if (NULL == root) {
		return;
	}

	for (int i = 0; i < depth; i++) {
		safe_printf("%*s", depth, "\t");
	}
	safe_printf("-%d:%d.%d\n", root->bus_no, root->dev_no, root->func_no);

	pci_dev_t *child = root->child;
	while (child != NULL) {
		pci_printTree(child, depth + 1);
		child = child->next;
	}

	pci_printTree(root->next, depth);
}

pci_dev_t *pci_getDevFromTree(pci_dev_t *root, uint32_t vid, uint32_t pid)
{
	if (NULL == root) {
		return NULL;
	}
	if (root->vid == vid && root->pid == pid) {
		return root;
	}

	pci_dev_t *child = root->child;
	while (child != NULL) {
		pci_dev_t *ret = pci_getDevFromTree(child, vid, pid);
		if (ret != NULL) {
			return ret;
		}
		child = child->next;
	}

	return pci_getDevFromTree(root->next, vid, pid);
}

void pci_printBARs(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	const int bar_count = (ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_HEADER_TYPE) & 0x7f) == 0 ? 6 : 2;

	for (int i = 0; i < bar_count; i++) {
		uint32_t bar_low = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4);
		if (0 == bar_low) {
			continue;
		}

		/* Check if this is I/O BAR */
		if (0x1 == (bar_low & 0x1)) {
			safe_printf("pcie: BAR%d I/O 0x%08x\n", i, bar_low & ~0x3u);
		}
		else {
			bool is_64_bit = (bar_low & 0x4) == 0x4;
			bool prefetchble = (bar_low & (1 << 3)) != 0;
			uint64_t addr = bar_low & ~0xfu;

			if (is_64_bit) {
				uint32_t bar_high = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + (i + 1) * 4);
				addr |= ((uint64_t)bar_high) << 32;
				i++;
			}

			safe_printf("pcie: BAR%d MEM 0x%016llx (%s), (%s)\n",
					is_64_bit ? i - 1 : i, (unsigned long long)addr,
					is_64_bit ? "64-bit" : "32-bit",
					prefetchble ? "prefetchble" : "non-prefetchble");
		}
	}
}

void print_capabilities(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus_no;
	uint8_t dev = device->dev_no;
	uint8_t fun = device->func_no;

	/* Check if there is capabilities list */
	uint16_t status = ecamRead16((uintptr_t)pcie, bus, dev, fun, PCI_STATUS);
	if (!(status & (1 << 4))) {
		return;
	}

	/* Read capabilities list pointer */
	uint8_t ptr = ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_CAP_PTR);
	while (ptr >= 0x40) {
		uint8_t cap_id = ecamRead8((uintptr_t)pcie, bus, dev, fun, ptr);
		uint8_t next = ecamRead8((uintptr_t)pcie, bus, dev, fun, ptr + 1);

		safe_printf("pcie: CAP id 0x%02x address 0x%02x\n", cap_id, ptr);

		if (next == 0) {
			break;
		}
		ptr = next;
	}
}


/* This section manages physical addresses for BAR allocation*/
uintptr_t pci_getNextIORange(size_t bar_length)
{
	/* Checking for alignment */
	static uint64_t ptr = PCI_IO_BAR_BA;
	if ((ptr & (uint64_t)(bar_length - 1)) != 0) {
		ptr += (uint64_t)(bar_length);
		ptr &= ~((bar_length << 1) - 1);
	}
	if (ptr + bar_length < PCI_IO_BAR_LIMIT) {
		uintptr_t ret = (uintptr_t)ptr;
		ptr = ptr + (uint64_t)bar_length;
		return ret;
	}
	return (uintptr_t)-1;
}

uintptr_t pci_getNext32RangeNonPref(size_t bar_length)
{
	/* Checking for alignment */
	static uint64_t ptr = PCI_32NP_BAR_BA;
	if ((ptr & (uint64_t)(bar_length - 1)) != 0) {
		ptr += (uint64_t)(bar_length);
		ptr &= ~((bar_length << 1) - 1);
	}
	if (ptr + bar_length < PCI_32NP_BAR_LIMIT) {
		uintptr_t ret = (uintptr_t)ptr;
		ptr = ptr + (uint64_t)bar_length;
		return ret;
	}
	return (uintptr_t)-1;
}

uintptr_t pci_getNext32Range(size_t bar_length)
{
	/* Checking for alignment */
	static uint64_t ptr = PCI_32P_BAR_BA;
	if ((ptr & (uint64_t)(bar_length - 1)) != 0) {
		ptr += (uint64_t)(bar_length);
		ptr &= ~((bar_length << 1) - 1);
	}
	if (ptr + bar_length < PCI_32P_BAR_LIMIT) {
		uintptr_t ret = (uintptr_t)ptr;
		ptr = ptr + (uint64_t)bar_length;
		return ret;
	}
	return (uintptr_t)-1;
}

uintptr_t pci_getNext64Range(size_t bar_length)
{
	/* Checking for alignment */
	static uint64_t ptr = PCI_64P_BAR_BA;
	if ((ptr & (uint64_t)(bar_length - 1)) != 0) {
		ptr += (uint64_t)(bar_length);
		ptr &= ~((bar_length << 1) - 1);
	}
	if (ptr + bar_length < PCI_64P_BAR_LIMIT) {
		uintptr_t ret = (uintptr_t)ptr;
		ptr = ptr + (uint64_t)bar_length;
		return ret;
	}
	return (uintptr_t)-1;
}

unsigned long pci_getNextMSIadd(void)
{
	/* This is purely incrementing allocation - MSI requires 4KiB alligned address */
	// static uint64_t ptr = PCI_MSI_BA;
	// if (ptr < PCI_MSI_LIMIT) {
	// 	unsigned long ret = ptr;
	// 	ptr += (1 << 12);
	// 	return ret;
	// }
	// return (unsigned long)-1;
	return PCI_MSI_BA;
}

uint32_t pci_getNextMsiPayload(void)
{
	static uint32_t id = 0;
	return id++;
}
