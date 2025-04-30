/*
 * Phoenix-RTOS
 *
 * ZynqMP PCI Express (PS controller) driver
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

#include <phoenix/arch/aarch64/zynqmp/zynqmp.h>

#include <board_config.h>

/* Register set "AXIPCIE_MAIN" */
#define AXIPCIE_MAIN_SIZE    0x1000
#define AXIPCIE_MAIN_ADDRESS 0xfd0e0000

/* Register set "PCIE_ATTRIB"  */
#define PCIE_ATTRIB_SIZE    0x1000
#define PCIE_ATTRIB_ADDRESS 0xfd480000

/* Memory range used for ECAM space "PCIe HIGH" */
#define ECAM_SIZE    0x10000000
#define ECAM_ADDRESS 0x8000000000

/* Registers inside "AXIPCIE_MAIN" registers set */
#define E_BREG_CAPABILITIES 0x00000200
#define E_BREG_CONTROL      0x00000208
#define E_BREG_BASE_LO      0x00000210
#define E_BREG_BASE_HI      0x00000214
#define E_ECAM_CONTROL      0x00000228
#define E_ECAM_BASE_LO      0x00000230
#define E_ECAM_BASE_HI      0x00000234

/* Registers inside "PCIE_ATTRIB" registers set */
#define PCIE_STATUS 0x00000238

#define UPPER_32_BITS(n) ((uint32_t)(((n) >> 16) >> 16))
#define LOWER_32_BITS(n) ((uint32_t)((n) & 0xffffffff))

/* ECAM addressing */
#define ECAM_BUS_SHIFT  20
#define ECAM_DEV_SHIFT  15
#define ECAM_FUNC_SHIFT 12

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

/* ECAM commands */
#define PCI_CMD_IO_ENABLE         0x01
#define PCI_CMD_MEM_ENABLE        0x02
#define PCI_CMD_MASTER_ENABLE     0x04
#define PCI_CMD_PARITY_ERR_ENABLE 0x40
#define PCI_CMD_SERR_ERR_ENABLE   0x100

static void pcie_scanBus(uint8_t bus);

static struct {
	uint32_t *axipcie_main; /* Register set "AXIPCIE_MAIN" */
	uint32_t *pcie_attrib;  /* Register set "PCIE_ATTRIB"  */
	uint32_t *ecam;         /* Memory range used for ECAM space "PCIe HIGH" */
} common;

static inline volatile uint32_t *ecamRegPtr(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t reg)
{
	uintptr_t cfg_space_offset = ((uintptr_t)bus << ECAM_BUS_SHIFT) |
			((uintptr_t)dev << ECAM_DEV_SHIFT) |
			((uintptr_t)fn << ECAM_FUNC_SHIFT) |
			(reg & 0xfff);

	return (volatile uint32_t *)((uintptr_t)common.ecam + cfg_space_offset);
}

static inline uint32_t ecamRead32(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	return *ecamRegPtr(bus, dev, fn, off & ~0x3);
}

static inline void ecamWrite32(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint32_t val)
{
	*ecamRegPtr(bus, dev, fn, off & ~0x3) = val;
}

static inline uint16_t ecamRead16(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	uint32_t value_u32 = ecamRead32(bus, dev, fn, off);
	if (off & 2) {
		return value_u32 >> 16;
	}
	else {
		return value_u32 & 0xffff;
	}
}

static inline uint8_t ecamRead8(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	uint32_t value_u32 = ecamRead32(bus, dev, fn, off);
	return (value_u32 >> ((off & 3) * 8)) & 0xff;
}

static inline void ecamWrite8(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint8_t val)
{
	volatile uint8_t *p = (volatile uint8_t *)ecamRegPtr(bus, dev, fn, off);
	*p = val;
}

static void print_bars(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t hdr)
{
	/* Choose number of bars depending on config type */
	const int bar_count = (hdr == 0x00) ? 6 : 2;

	/* Read all bars */
	for (int i = 0; i < bar_count; ++i) {

		uint32_t bar_low = ecamRead32(bus, dev, fn, PCI_BAR0 + i * 4);
		if (bar_low == 0) {
			continue;
		}

		if ((bar_low & 0x1) == 0x1) {
			printf("pcie-nwl: BAR%d I/O 0x%08x\n", i, bar_low & ~0x3);
		}
		else {
			bool is_64_bit = (bar_low & 0x4) == 0x4;
			uint64_t addr = bar_low & ~0xf;

			if (is_64_bit) {
				uint32_t bar_high = ecamRead32(bus, dev, fn, PCI_BAR0 + (i + 1) * 4);
				addr |= ((uint64_t)bar_high) << 32;
				i++;
			}
			printf("pcie-nwl: BAR%d MEM 0x%016llx (%s)\n",
					i, (unsigned long long)addr, is_64_bit ? "64-bit" : "32-bit");
		}
	}
}

static void print_capabilities(uint8_t bus, uint8_t dev, uint8_t fn)
{
	/* Check if there is capabilities list */
	uint16_t status = ecamRead16(bus, dev, fn, PCI_STATUS);
	if (!(status & (1 << 4))) {
		return;
	}

	/* Read capabilities list pointer */
	uint8_t ptr = ecamRead8(bus, dev, fn, PCI_CAP_PTR);
	while (ptr >= 0x40) {
		uint8_t cap_id = ecamRead8(bus, dev, fn, ptr);
		uint8_t next = ecamRead8(bus, dev, fn, ptr + 1);
		printf("pcie-nwl: CAP id 0x%02x address 0x%02x\n", cap_id, ptr);
		if (next == 0) {
			break;
		}
		ptr = next;
	}
}

static void scanFunc(uint8_t bus, uint8_t dev, uint8_t fun)
{
	static uint8_t next_bus = 1;

	/* Read informations about device */
	uint16_t vendor = ecamRead16(bus, dev, fun, PCI_VENDOR_ID);
	uint16_t device = ecamRead16(bus, dev, fun, PCI_DEVICE_ID);
	uint32_t class24 = ecamRead32(bus, dev, fun, PCI_CLASSCODE);
	uint8_t classBase = class24 >> 24;
	uint8_t classSub = (class24 >> 16) & 0xff;
	uint8_t progIF = (class24 >> 8) & 0xff;
	uint8_t hdr = ecamRead8(bus, dev, fun, PCI_HEADER_TYPE) & 0x7f;

	printf("pcie-nwl: %02x:%02x.%u ven %04x dev %04x class %02x%02x%02x hdr 0x%02x\n",
			bus, dev, fun,
			vendor, device,
			classBase, classSub, progIF,
			hdr);

	/* Enable MEM-space and Bus Master if still disabled */
	uint16_t cmd = ecamRead16(bus, dev, fun, PCI_COMMAND);
	if (!(cmd & (PCI_CMD_MEM_ENABLE | PCI_CMD_MASTER_ENABLE))) {
		printf("pcie-nwl: enable memory space and bus master\n");
		ecamWrite32(bus, dev, fun, PCI_COMMAND, cmd | PCI_CMD_MEM_ENABLE | PCI_CMD_MASTER_ENABLE);
	}

	/* Print some info about this device */
	print_bars(bus, dev, fun, hdr);
	print_capabilities(bus, dev, fun);

	/* If this is a PCI-PCI bridge program buses and recurse */
	if (hdr == 0x01) {
		uint8_t sec = ecamRead8(bus, dev, fun, PCI_SECONDARY_BUS);
		if (sec == 0) {
			/* Bridge not yet configured, assign fresh numbers */
			sec = next_bus++;
			ecamWrite8(bus, dev, fun, PCI_PRIMARY_BUS, bus);
			ecamWrite8(bus, dev, fun, PCI_SECONDARY_BUS, sec);
			ecamWrite8(bus, dev, fun, PCI_SUBORDINATE_BUS, 0xFF);
		}

		uint8_t sub = ecamRead8(bus, dev, fun, PCI_SUBORDINATE_BUS);

		printf("pcie-nwl: bridge bus primary %u secondary %u subordinate %u\n",
				bus, sec, sub);

		pcie_scanBus(sec);

		/* After recursion write the real highest bus number reached */
		ecamWrite8(bus, dev, fun, PCI_SUBORDINATE_BUS, next_bus - 1);
	}
}

static void pcie_scanBus(uint8_t bus)
{
	/* Iterate over all devices connected to the certain bus */
	for (uint8_t dev = 0; dev < 32; ++dev) {
		for (uint8_t fn = 0; fn < 8; fn++) {
			/**
			 * In case there is no device under certain identifier the bridge
			 * returns all "ones" on read
			 */
			uint16_t vendor_id = ecamRead16(bus, dev, fn, PCI_VENDOR_ID);
			if (vendor_id == 0xffff) {
				continue;
			}

			/* Scan first function of device */
			scanFunc(bus, dev, 0);
		}
	}
}

static uint32_t readReg(uint32_t *base, uint32_t offset)
{
	return *((volatile uint32_t *)((char *)base + offset));
}

static void writeReg(uint32_t *base, uint32_t offset, uint32_t value)
{
	*((volatile uint32_t *)((char *)base + offset)) = value;
}

static void writeRegMsk(uint32_t *base, uint32_t offset, uint32_t clr, uint32_t set)
{
	uint32_t value = readReg(base, offset);
	value &= ~clr;
	value |= set;
	writeReg(base, offset, value);
}

static int pcie_initPcie(void)
{
	/* Map registers memory */
	common.axipcie_main = mmap(NULL, AXIPCIE_MAIN_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			AXIPCIE_MAIN_ADDRESS);
	if (NULL == common.axipcie_main) {
		printf("pcie-nwl: fail to map AXI PCIE MAIN registers memory\n");
		return -1;
	}
	common.pcie_attrib = mmap(NULL, PCIE_ATTRIB_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			PCIE_ATTRIB_ADDRESS);
	if (NULL == common.pcie_attrib) {
		printf("pcie-nwl: fail to map PCIE ATTRIB registers memory\n");
		return -1;
	}
	common.ecam = mmap(NULL, ECAM_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			ECAM_ADDRESS);
	if (NULL == common.ecam) {
		printf("pcie-nwl: fail to map PCIE CFG registers memory\n");
		return -1;
	}

	/* Map the bridge register aperture */
	writeReg(common.axipcie_main, E_BREG_BASE_LO, LOWER_32_BITS(AXIPCIE_MAIN_ADDRESS));
	writeReg(common.axipcie_main, E_BREG_BASE_HI, UPPER_32_BITS(AXIPCIE_MAIN_ADDRESS));

	/* Enable BREG */
	writeReg(common.axipcie_main, E_BREG_CONTROL, 0x1);

	/* Check PHY link */
	bool phy_link_up = false;
	static const int LINK_WAIT_MAX_RETRIES = 10;
	for (int retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
		uint32_t link_status = readReg(common.pcie_attrib, PCIE_STATUS);
		if (link_status & 0x2) {
			phy_link_up = true;
			break;
		}
		usleep(100000);
	}
	if (phy_link_up) {
		printf("pcie-nwl: phy link up\n");
	}
	else {
		printf("pcie-nwl: fail, phy link down\n");
		return -1;
	}

	/* Enable ECAM */
	writeRegMsk(common.axipcie_main, E_ECAM_CONTROL, 0x1, 0x1);
	/* Set size of translation window */
	writeRegMsk(common.axipcie_main, E_ECAM_CONTROL, (0x1f << 16), (16 << 16));
	/* Configure address for ECAM space */
	writeReg(common.axipcie_main, E_ECAM_BASE_LO, LOWER_32_BITS(ECAM_ADDRESS));
	writeReg(common.axipcie_main, E_ECAM_BASE_HI, UPPER_32_BITS(ECAM_ADDRESS));

	/* Check PCI Express link */
	bool pcie_link_up = false;
	for (int retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
		uint32_t pcie_link_status = readReg(common.pcie_attrib, PCIE_STATUS);
		if (pcie_link_status & 0x1) {
			pcie_link_up = true;
			break;
		}
		usleep(100000);
	}
	if (pcie_link_up) {
		printf("pcie-nwl: pcie link up\n");
	}
	else {
		printf("pcie-nwl: fail pcie link down\n");
		return -1;
	}

	/* Read local config space */
	uint32_t config = readReg(common.ecam, 0x4);

	/* Enable needed functionality */
	config |= (PCI_CMD_IO_ENABLE | PCI_CMD_MEM_ENABLE | PCI_CMD_MASTER_ENABLE |
			PCI_CMD_PARITY_ERR_ENABLE | PCI_CMD_SERR_ERR_ENABLE);
	writeReg(common.ecam, 0x4, config);

	/* Unmap memory */
	munmap((void *)common.axipcie_main, AXIPCIE_MAIN_SIZE);
	munmap((void *)common.pcie_attrib, PCIE_ATTRIB_SIZE);

	return 0;
}

int main(int argc, char **argv)
{
	int ret = 0;

	/* Wait for PS GTR PHY initialisation */
	oid_t oid;
	while (lookup("/dev/pcie-phy", NULL, &oid) < 0) {
		usleep(10000);
	}

	/* Configure PCI peripheral */
	printf("pcie-nwl: configure PCI Express peripheral...\n");
	ret = pcie_initPcie();
	if (ret != 0) {
		printf("pcie-nwl: fail to configure PCI Express peripheral\n");
		return ret;
	}

	/* Enumerate devices */
	printf("pcie-nwl: enumeration... \n");
	pcie_scanBus(0);

	munmap((void *)common.ecam, ECAM_SIZE);

	return 0;
}
