/*
 * Phoenix-RTOS
 *
 * ZynqMP PCI Express driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mikolaj Matalowski
 *
 * %LICENSE%
 */

#include "pci_core.h"

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
			printf("pcie: Failed to allocate memory for PCI dev during enumeration, level :%d\n", depth);
			return;
		}

		if (NULL == root->child) {
			root->child = base_function;
		}

		base_function->vid = vendor_id;
		base_function->pid = product_id;
		base_function->bus = bus;
		base_function->dev = dev;
		base_function->func = 0;
		last = base_function;

		/* Scan first function of device */
		pci_scanFunc(pcie, base_function, depth);

		/* Check if this is multi function device and scan them */
		bool multi = ecamRead8((uintptr_t)pcie, bus, dev, 0, PCI_HEADER_TYPE) & PCI_HT_MULTI_FUNC;
		if (multi) {
			printf("pcie: multiple func device %u\n", dev);
			for (uint8_t fn = 1; fn < 8; fn++) {
				uint16_t multi_function_vid = ecamRead16((uintptr_t)pcie, bus, dev, fn, PCI_VENDOR_ID);
				/* This function does not exist */
				if (0xffff == multi_function_vid) {
					continue;
				}

				pci_dev_t *additional_function = (pci_dev_t *)calloc(1, sizeof(pci_dev_t));
				if (NULL == additional_function) {
					printf("pcie: Failed to allocate memory for PCI dev during enumeration, level :%d\n", depth);
					return;
				}

				additional_function->bus = bus;
				additional_function->dev = dev;
				additional_function->func = fn;
				last->next = additional_function;
				last = additional_function;

				pci_scanFunc(pcie, additional_function, depth);
			}
		}
	}
}

void pci_scanFunc(uint32_t *pcie, pci_dev_t *device, int depth)
{
	uint8_t bus = device->bus;
	uint8_t dev = device->dev;
	uint8_t fun = device->func;

	/* Read informations about device */
	uint16_t vid = ecamRead16((uintptr_t)pcie, bus, dev, fun, PCI_VENDOR_ID);
	uint16_t pid = ecamRead16((uintptr_t)pcie, bus, dev, fun, PCI_DEVICE_ID);
	uint32_t class24 = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_CLASSCODE);
	uint8_t classBase = class24 >> 24;
	uint8_t classSub = (class24 >> 16) & 0xff;
	uint8_t progIF = (class24 >> 8) & 0xff;
	uint8_t hdr = ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_HEADER_TYPE) & 0x7f;

	printf("pcie: %02x:%02x.%u ven %04x dev %04x class %02x%02x%02x hdr 0x%02x\n",
			bus, dev, fun,
			vid, pid,
			classBase, classSub, progIF,
			hdr);

	ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_COMMAND, 0);

	/* Enable MEM-space and Bus Master if still disabled */
	uint16_t cmd = ecamRead16((uintptr_t)pcie, bus, dev, fun, PCI_COMMAND);
	if (!(cmd & (PCI_CMD_MEM | PCI_CMD_MASTER))) {
		printf("pcie: enable memory space and bus master\n");
		ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_COMMAND, cmd | 0x4 | 0x2 | 0x1 | 0x40 | 0x100);
	}

	/* If this is a PCI-PCI bridge program buses and recurse */
	if (hdr == 0x01) {
		pci_configureBridge(pcie, device, depth);
	}
	else {
		/* If this is a PCI endpoint, configure device and return */
		pci_configureEndpoint(pcie, device);
		/* Print some info about this device */
		pci_printBARs(pcie, device);
	}
	// print_capabilities(pcie, device);
}


void pci_configureBridge(uint32_t *pcie, pci_dev_t *device, int depth)
{
	static uint8_t next_bus = 1;

	uint8_t bus = device->bus;
	uint8_t dev = device->dev;
	uint8_t fun = device->func;

	/* read once */
	uint8_t sec = ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_SECONDARY_BUS);

	if (sec == 0) {
		sec = next_bus++;

		uint32_t cur = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_PRIMARY_BUS) & 0xff000000;
		uint32_t val = cur |
				((uint32_t)0xFF << 16) | /* subordinate = ff */
				((uint32_t)sec << 8) |   /* secondary   = sec */
				bus;                     /* primary     = bus */

		printf("pcie: write pri, sec, sub 0x%x \n", val);
		ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_PRIMARY_BUS, val);
		uint32_t val_read = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_PRIMARY_BUS);
		printf("pcie: validate pri, sec, sub 0x%x \n", val_read);
	}

	uint8_t sub = ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_SUBORDINATE_BUS);
	printf("pcie: bridge bus primary %u secondary %u subordinate %u\n",
			bus, sec, sub);

	pci_enumerateRoot(pcie, device, sec, depth + 1);

	/* final subordinate = highest bus encountered */
	uint32_t cur = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_PRIMARY_BUS) & 0xff00ffff;
	uint32_t val = cur | ((uint32_t)(next_bus - 1) << 16);
	ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_PRIMARY_BUS, val);

	/* Map physical addresses that bridge has to concerned with */
	pci_mapBARsPhysAddToBridge(pcie, device);
	pci_configureCapBridge(pcie, device);

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

	pci_bridgeEnable(pcie, device);
}


void pci_mapBARsPhysAddToBridge(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus;
	uint8_t dev = device->dev;
	uint8_t fun = device->func;
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
	uint8_t bus = device->bus;
	uint8_t dev = device->dev;
	uint8_t fun = device->func;

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
			printf("pcie: %d:%d.%d Current device control 2 is %x\n", bus, dev, fun, reg);
		}

		if (next == 0) {
			break;
		}
		ptr = next;
	}
}

void pci_bridgeEnable(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus;
	uint8_t dev = device->dev;
	uint8_t fun = device->func;

	/* Enable I/O, memory and bus master */
	uint32_t ret = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_BRIDGE_CMD);
	ret |= 0x7;
	ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_BRIDGE_CMD, ret);
}

void pci_configureEndpoint(uint32_t *pcie, pci_dev_t *device)
{
	pci_configureEndpointBARs(pcie, device);
	pci_enableEndpoint(pcie, device);
}

void pci_configureEndpointBARs(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus;
	uint8_t dev = device->dev;
	uint8_t fun = device->func;

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
				printf("pcie: %d:%d.%d - Failed to allocate physical memory range for I/O BAR%d\n",
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
				printf("pcie: %d:%d.%d - Failed to allocate physical memory range for non-prefetchble 32-bit BAR%d\n",
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
				printf("pcie: %d:%d.%d - Failed to allocate physical memory range for prefetchble 32-bit BAR%d\n",
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
				printf("pcie: %d:%d.%d - Failed to allocate physical memory range for 64-bit BAR%d\n",
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
			printf("pcie: %d:%d.%d - Failed to map BAR%d: wanted %016lx:%lx\n",
					bus, dev, fun, j, bar_phys_add, size);
		}
		else {
			device->size[j] = size;
		}
	}
}

void pci_enableEndpoint(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus;
	uint8_t dev = device->dev;
	uint8_t fun = device->func;

	uint16_t cmd = ecamRead16((uintptr_t)pcie, bus, dev, fun, PCI_COMMAND);
	ecamWrite32((uintptr_t)pcie, bus, dev, fun, PCI_COMMAND, cmd | 0x4 | 0x2 | 0x1 | 0x40 | 0x100);
}


void pci_enableRootComplex(uint32_t *pcie)
{
	/* Enable root complex - this should be done only after enumeration is complete */
	writeReg(pcie, 0x148, 1);
}

/* This section implements debug printing functions */

void pci_printTree(pci_dev_t *root, int depth)
{
	if (NULL == root) {
		return;
	}

	for (int i = 0; i < depth; i++) {
		printf("%0*s", depth, "\t");
	}
	printf("-%d:%d.%d\n", root->bus, root->dev, root->func);

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
	uint8_t bus = device->bus;
	uint8_t dev = device->dev;
	uint8_t fun = device->func;

	const int bar_count = (ecamRead8((uintptr_t)pcie, bus, dev, fun, PCI_HEADER_TYPE) & 0x7f) == 0 ? 6 : 2;

	for (int i = 0; i < bar_count; i++) {
		uint32_t bar_low = ecamRead32((uintptr_t)pcie, bus, dev, fun, PCI_BAR0 + i * 4);
		if (0 == bar_low) {
			continue;
		}

		/* Check if this is I/O BAR */
		if (0x1 == (bar_low & 0x1)) {
			printf("pcie: BAR%d I/O 0x%08x\n", i, bar_low & ~0x3u);
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

			printf("pcie: BAR%d MEM 0x%016llx (%s), (%s)\n",
					is_64_bit ? i - 1 : i, (unsigned long long)addr,
					is_64_bit ? "64-bit" : "32-bit",
					prefetchble ? "prefetchble" : "non-prefetchble");
		}
	}
}

void print_capabilities(uint32_t *pcie, pci_dev_t *device)
{
	uint8_t bus = device->bus;
	uint8_t dev = device->dev;
	uint8_t fun = device->func;

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

		printf("pcie: CAP id 0x%02x address 0x%02x\n", cap_id, ptr);

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
