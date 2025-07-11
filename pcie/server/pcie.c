/*
 * Phoenix-RTOS
 *
 * PCI Express driver server
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <endian.h>
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <board_config.h>

#include <pcie.h>


#ifdef PCI_EXPRESS_INIT_TEBF0808_PHY
#include <tebf0808-pcie-refclk.h>
#include <tebf0808-ps-gtr-phy.h>
#endif


#ifdef PCI_EXPRESS_XILINX_NWL
#include <pcie-xilinx-nwl.h>
#endif


#ifdef PCI_EXPRESS_XILINX_AXI
#include <pcie-xilinx-axi.h>
#endif


/* ECAM commands */
#define PCI_CMD_IO_ENABLE         0x01
#define PCI_CMD_MEM_ENABLE        0x02
#define PCI_CMD_MASTER_ENABLE     0x04
#define PCI_CMD_PARITY_ERR_ENABLE 0x40
#define PCI_CMD_SERR_ERR_ENABLE   0x100


/* ECAM header offsets */
#define PCI_VENDOR_ID       0x00
#define PCI_DEVICE_ID       0x02
#define PCI_COMMAND         0x04
#define PCI_STATUS          0x06
#define PCI_CLASSCODE       0x08
#define PCI_HEADER_TYPE     0x0e
#define PCI_BAR0            0x10
#define PCI_PRIMARY_BUS     0x18
#define PCI_SECONDARY_BUS   0x19
#define PCI_SUBORDINATE_BUS 0x1a
#define PCI_CAP_PTR         0x34


#define PCI_HT_MULTI_FUNC 0x80

#define PCI_BAR_MEM_MASK (~0x0fUL)
#define PCI_BAR_IO_MASK  (~0x03UL)

/* ECAM addressing */
#define ECAM_BUS_SHIFT  20
#define ECAM_DEV_SHIFT  15
#define ECAM_FUNC_SHIFT 12


static void pcie_scanBus(void *ecam, uint8_t bus);


static inline volatile uint32_t *ecamRegPtr(void *ecam, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t reg)
{
	uintptr_t cfg_space_offset = ((uintptr_t)bus << ECAM_BUS_SHIFT) |
			((uintptr_t)dev << ECAM_DEV_SHIFT) |
			((uintptr_t)fn << ECAM_FUNC_SHIFT) |
			(reg & 0xfff);

	return (volatile uint32_t *)((uintptr_t)ecam + cfg_space_offset);
}


static inline uint32_t ecamRead32(void *ecam, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	return *ecamRegPtr(ecam, bus, dev, fn, off & ~0x3);
}


static inline void ecamWrite32(void *ecam, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint32_t val)
{
	*ecamRegPtr(ecam, bus, dev, fn, off & ~0x3) = val;
}


static inline uint16_t ecamRead16(void *ecam, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	uint32_t value_u32 = ecamRead32(ecam, bus, dev, fn, off);
	if (off & 2) {
		return value_u32 >> 16;
	}
	else {
		return value_u32 & 0xffff;
	}
}


static inline uint8_t ecamRead8(void *ecam, uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	uint32_t value_u32 = ecamRead32(ecam, bus, dev, fn, off);
	return (value_u32 >> ((off & 3) * 8)) & 0xff;
}


static void print_bars(void *ecam, uint8_t bus, uint8_t dev, uint8_t fn, uint8_t hdr)
{
	/* Choose number of bars depending on config type */
	const int bar_count = (hdr == 0x00) ? 6 : 2;

	/* Read all bars */
	for (int i = 0; i < bar_count; ++i) {

		uint32_t bar_low = ecamRead32(ecam, bus, dev, fn, PCI_BAR0 + i * 4);
		if (bar_low == 0) {
			continue;
		}

		if ((bar_low & 0x1) == 0x1) {
			printf("pcie: BAR%d I/O 0x%08x\n", i, bar_low & ~0x3);
		}
		else {
			bool is_64_bit = (bar_low & 0x4) == 0x4;
			uint64_t addr = bar_low & ~0xf;

			if (is_64_bit) {
				uint32_t bar_high = ecamRead32(ecam, bus, dev, fn, PCI_BAR0 + (i + 1) * 4);
				addr |= ((uint64_t)bar_high) << 32;
				i++;
			}
			printf("pcie: BAR%d MEM 0x%016llx (%s)\n",
					i, (unsigned long long)addr, is_64_bit ? "64-bit" : "32-bit");
		}
	}
}


static void print_capabilities(void *ecam, uint8_t bus, uint8_t dev, uint8_t fn)
{
	/* Check if there is capabilities list */
	uint16_t status = ecamRead16(ecam, bus, dev, fn, PCI_STATUS);
	if (!(status & (1 << 4))) {
		return;
	}

	/* Read capabilities list pointer */
	uint8_t ptr = ecamRead8(ecam, bus, dev, fn, PCI_CAP_PTR);
	while (ptr >= 0x40) {
		uint8_t cap_id = ecamRead8(ecam, bus, dev, fn, ptr);
		uint8_t next = ecamRead8(ecam, bus, dev, fn, ptr + 1);
		printf("pcie: CAP id 0x%02x address 0x%02x\n", cap_id, ptr);
		if (next == 0) {
			break;
		}
		ptr = next;
	}
}


static void scanFunc(void *ecam, uint8_t bus, uint8_t *next_bus, uint8_t dev, uint8_t fun)
{
	/* Read information about device */
	uint16_t vendor = ecamRead16(ecam, bus, dev, fun, PCI_VENDOR_ID);
	uint16_t device = ecamRead16(ecam, bus, dev, fun, PCI_DEVICE_ID);
	uint32_t class24 = ecamRead32(ecam, bus, dev, fun, PCI_CLASSCODE);
	uint8_t classBase = class24 >> 24;
	uint8_t classSub = (class24 >> 16) & 0xff;
	uint8_t progIF = (class24 >> 8) & 0xff;
	uint8_t hdr = ecamRead8(ecam, bus, dev, fun, PCI_HEADER_TYPE) & 0x7f;

	printf("pcie: %02x:%02x.%u ven %04x dev %04x class %02x%02x%02x hdr 0x%02x\n",
			bus, dev, fun,
			vendor, device,
			classBase, classSub, progIF,
			hdr);

	/* Enable MEM-space and Bus Master if still disabled */
	uint16_t cmd = ecamRead16(ecam, bus, dev, fun, PCI_COMMAND);
	if (!(cmd & (PCI_CMD_MEM_ENABLE | PCI_CMD_MASTER_ENABLE))) {
		printf("pcie: enable memory space and bus master\n");
		ecamWrite32(ecam, bus, dev, fun, PCI_COMMAND, cmd | PCI_CMD_MEM_ENABLE | PCI_CMD_MASTER_ENABLE);
	}

	/* Print some info about this device */
	print_bars(ecam, bus, dev, fun, hdr);
	print_capabilities(ecam, bus, dev, fun);

	/* If this is a PCI-PCI bridge program buses and recurse */
	if (hdr == 0x01) {
		uint8_t sec = ecamRead8(ecam, bus, dev, fun, PCI_SECONDARY_BUS);
		if (sec == 0) {
			/* Bridge not yet configured, assign fresh numbers */
			sec = (*next_bus)++;
			uint32_t cur = ecamRead32(ecam, bus, dev, fun, PCI_PRIMARY_BUS) & 0xff000000;
			uint32_t val = cur |
					((uint32_t)0xFF << 16) |
					((uint32_t)sec << 8) |
					bus;
			ecamWrite32(ecam, bus, dev, fun, PCI_PRIMARY_BUS, val);
		}

		uint8_t sub = ecamRead8(ecam, bus, dev, fun, PCI_SUBORDINATE_BUS);

		printf("pcie: bridge bus primary %u secondary %u subordinate %u\n",
				bus, sec, sub);

		pcie_scanBus(ecam, sec);

		/* After recursion write the real highest bus number reached */
		uint32_t cur = ecamRead32(ecam, bus, dev, fun, PCI_PRIMARY_BUS) & 0xff00ffff;
		uint32_t val = cur | ((uint32_t)((*next_bus) - 1) << 16);
		ecamWrite32(ecam, bus, dev, fun, PCI_PRIMARY_BUS, val);
	}
}


static void pcie_scanBus(void *ecam, uint8_t bus)
{
	uint8_t next_bus = 1;

	/* Iterate over all devices connected to the certain bus */
	for (uint8_t dev = 0; dev < (bus ? 32 : 1); ++dev) {
		/**
		 * In case there is no device under certain identifier the bridge
		 * returns all "ones" on read
		 */
		uint16_t vendor_id = ecamRead16(ecam, bus, dev, 0, PCI_VENDOR_ID);
		if (vendor_id == 0xffff) {
			continue;
		}

		/* Scan first function of device */
		scanFunc(ecam, bus, &next_bus, dev, 0);

		/* Check if this is multi function device and scan them */
		bool multi = ecamRead8(ecam, bus, dev, 0, PCI_HEADER_TYPE) & PCI_HT_MULTI_FUNC;
		if (multi) {
			printf("pcie: multiple func device %u\n", dev);
			for (uint8_t fn = 1; fn < 8; fn++) {
				vendor_id = ecamRead16(ecam, bus, dev, fn, PCI_VENDOR_ID);
				if (vendor_id == 0xffff) {
					continue;
				}
				scanFunc(ecam, bus, &next_bus, dev, fn);
			}
		}
	}
}


int main(int argc, char **argv)
{
	int ret = 0;

#ifdef PCI_EXPRESS_INIT_TEBF0808_PHY
	ret = tebf0808_pcieRefClkInit();
	if (ret != 0) {
		return ret;
	}

	ret = tebf0808_pciePsGtrPhyInit();
	if (ret != 0) {
		return ret;
	}
#endif

#ifdef PCI_EXPRESS_XILINX_NWL
	ret = pcie_xilinx_nwl_init();
	if (ret != 0) {
		return ret;
	}
#endif

#ifdef PCI_EXPRESS_XILINX_AXI
	ret = pcie_xilinx_axi_init();
	if (ret != 0) {
		return ret;
	}
#endif

	uint32_t *ecam = mmap(NULL, ECAM_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			ECAM_ADDRESS);
	if (MAP_FAILED == ecam) {
		fprintf(stderr, "pcie: fail to map ECAM memory\n");
		return -1;
	}

	pcie_scanBus((void *)ecam, 0);

	munmap((void *)ecam, ECAM_SIZE);

	return ret;
}
