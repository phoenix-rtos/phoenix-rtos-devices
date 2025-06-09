/*
 * Phoenix-RTOS
 *
 * ZynqMP PCI Express driver
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

#define BREG_SIZE    0x1000
#define BREG_ADDRESS 0xfd0e0000

#define PCIREG_SIZE    0x1000
#define PCIREG_ADDRESS 0xfd480000

#define CFG_SIZE    0x10000000
#define CFG_ADDRESS 0x8000000000

#define GIC_PCIE_INTX_IRQ 148

#define BRCFG_PCIE_RX0           0x00000000
#define BRCFG_PCIE_RX1           0x00000004
#define BRCFG_INTERRUPT          0x00000010
#define BRCFG_PCIE_RX_MSG_FILTER 0x00000020

#define E_BREG_CAPABILITIES 0x00000200
#define E_BREG_CONTROL      0x00000208
#define E_BREG_BASE_LO      0x00000210
#define E_BREG_BASE_HI      0x00000214
#define E_ECAM_CAPABILITIES 0x00000220
#define E_ECAM_CONTROL      0x00000228
#define E_ECAM_BASE_LO      0x00000230
#define E_ECAM_BASE_HI      0x00000234

#define I_MSII_CAPABILITIES 0x00000300
#define I_MSII_CONTROL      0x00000308
#define I_MSII_BASE_LO      0x00000310
#define I_MSII_BASE_HI      0x00000314
#define I_ISUB_CONTROL      0x000003E8

#define PS_LINKUP_OFFSET 0x00000238

#define upper_32_bits(n) ((uint32_t)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((uint32_t)((n) & 0xffffffff))

#define ECAM_BUS_SHIFT  20
#define ECAM_DEV_SHIFT  15
#define ECAM_FUNC_SHIFT 12

#define PCI_VENDOR_ID     0x00
#define PCI_DEVICE_ID     0x02
#define PCI_COMMAND       0x04
#define PCI_CMD_IO        0x01
#define PCI_CMD_MEM       0x02
#define PCI_CMD_MASTER    0x04
#define PCI_STATUS        0x06
#define PCI_CLASSCODE     0x08
#define PCI_HEADER_TYPE   0x0e
#define PCI_HT_MULTI_FUNC 0x80
#define PCI_BIST          0x0f

#define PCI_BAR0    0x10
#define PCI_CAP_PTR 0x34

#define PCI_PRIMARY_BUS     0x18
#define PCI_SECONDARY_BUS   0x19
#define PCI_SUBORDINATE_BUS 0x1a

static void pcie_scanBus(uint8_t bus, int depth);

static struct {
	uint32_t *breg;
	uint32_t *pcireg;
	uint32_t *cfg;
	handle_t cond;
	handle_t inth;
	handle_t lock;
} pcie;

static inline volatile uint32_t *ecamRegPtr(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t reg)
{
	uintptr_t cfg_space_offset = ((uintptr_t)bus << ECAM_BUS_SHIFT) |
			((uintptr_t)dev << ECAM_DEV_SHIFT) |
			((uintptr_t)fn << ECAM_FUNC_SHIFT) |
			(reg & 0xfffu);

	return (volatile uint32_t *)((uintptr_t)pcie.cfg + cfg_space_offset);
}

static inline uint32_t ecamRead32(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	return *ecamRegPtr(bus, dev, fn, off & ~0x3u);
}

static inline void ecamWrite32(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint32_t val)
{
	*ecamRegPtr(bus, dev, fn, off & ~0x3u) = val;
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

static void print_bars(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t hdr, int depth)
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
			printf("pcie: BAR%d I/O 0x%08x\n", i, bar_low & ~0x3u);
		}
		else {
			bool is_64_bit = (bar_low & 0x4) == 0x4;
			uint64_t addr = bar_low & ~0xfu;

			if (is_64_bit) {
				uint32_t bar_high = ecamRead32(bus, dev, fn, PCI_BAR0 + (i + 1) * 4);
				addr |= ((uint64_t)bar_high) << 32;
				i++;
			}
			printf("pcie: BAR%d MEM 0x%016llx (%s)\n",
					i, (unsigned long long)addr, is_64_bit ? "64-bit" : "32-bit");
		}
	}
}

static void print_capabilities(uint8_t bus, uint8_t dev, uint8_t fn, int depth)
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

		printf("pcie: CAP id 0x%02x address 0x%02x\n", cap_id, ptr);

		if (next == 0) {
			break;
		}
		ptr = next;
	}
}

static void scanFunc(uint8_t bus, uint8_t dev, uint8_t fun, int depth)
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

	printf("pcie: %02x:%02x.%u ven %04x dev %04x class %02x%02x%02x hdr 0x%02x\n",
			bus, dev, fun,
			vendor, device,
			classBase, classSub, progIF,
			hdr);

	/* Special log for Ethernet controller (Base Class 0x02) */
	if (classBase == 0x02) {
		printf("pcie: ethernet controller detected\n");
	}

	/* Enable MEM-space and Bus Master if still disabled */
	uint16_t cmd = ecamRead16(bus, dev, fun, PCI_COMMAND);
	if (!(cmd & (PCI_CMD_MEM | PCI_CMD_MASTER))) {
		printf("pcie: enable memory space and bus master\n");
		ecamWrite32(bus, dev, fun, PCI_COMMAND, cmd | PCI_CMD_MEM | PCI_CMD_MASTER);
	}

	/* Print some info about this device */
	print_bars(bus, dev, fun, hdr, depth + 1);
	print_capabilities(bus, dev, fun, depth + 1);

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

		printf("pcie: bridge bus primary %u secondary %u subordinate %u\n",
				bus, sec, sub);

		pcie_scanBus(sec, depth + 1);

		/* After recursion write the real highest bus number reached */
		ecamWrite8(bus, dev, fun, PCI_SUBORDINATE_BUS, next_bus - 1);
	}
}

static void pcie_scanBus(uint8_t bus, int depth)
{
	/* Iterate over all devices connected to the certain bus */
	for (uint8_t dev = 0; dev < 32; ++dev) {
		/**
		 * In case there is no device under certain identifier the bridge
		 * returns all "ones" on read
		 */
		uint16_t vendor_id = ecamRead16(bus, dev, 0, PCI_VENDOR_ID);
		if (vendor_id == 0xffff) {
			continue;
		}

		/* Scan first function of device */
		scanFunc(bus, dev, 0, depth);

		/* Check if this is multi function device and scan them */
		bool multi = ecamRead8(bus, dev, 0, PCI_HEADER_TYPE) & PCI_HT_MULTI_FUNC;
		if (multi) {
			printf("pcie: multiple func device %u\n", dev);
			for (uint8_t fn = 1; fn < 8; fn++) {
				scanFunc(bus, dev, fn, depth);
			}
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

static int pcie_pciIntxIrqHandler(unsigned int id, void *arg)
{
	return 0;
}

static int pcie_initPcie(void)
{
	/* Map registers memory */
	pcie.breg = mmap(NULL, BREG_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			BREG_ADDRESS);
	if (NULL == pcie.breg) {
		printf("pcie: fail to map AXI PCIE MAIN registers memory\n");
		return -1;
	}
	pcie.pcireg = mmap(NULL, PCIREG_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			PCIREG_ADDRESS);
	if (NULL == pcie.pcireg) {
		printf("pcie: fail to map PCIE ATTRIB registers memory\n");
		return -1;
	}
	pcie.cfg = mmap(NULL, CFG_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			CFG_ADDRESS);
	if (NULL == pcie.pcireg) {
		printf("pcie: fail to map PCIE CFG registers memory\n");
		return -1;
	}

	/* Initialize synchronization primitives */
	if (condCreate(&pcie.cond) != 0) {
		printf("pcie: failed to create conditional variable\n");
		return -1;
	}
	if (mutexCreate(&pcie.lock) != 0) {
		printf("pcie: failed to create mutex\n");
		return -1;
	}

	/* Register PCI INTx interrupt handler */
	if (interrupt(GIC_PCIE_INTX_IRQ, pcie_pciIntxIrqHandler, &pcie, pcie.cond, &pcie.inth) != 0) {
		printf("pcie: failed to register interrupt handler\n");
		return -1;
	}

	/* Check if BREG is present */
	uint32_t breg_value = readReg(pcie.breg, E_BREG_CAPABILITIES);
	if (!(breg_value & 0x1)) {
		printf("pcie: fail lack of Egress Bridge Register Translation\n");
		return -1;
	}

	/* Map the bridge register aperture */
	writeReg(pcie.breg, E_BREG_BASE_LO, BREG_ADDRESS);
	writeReg(pcie.breg, E_BREG_BASE_HI, 0x0);

	/* Enable BREG */
	writeReg(pcie.breg, E_BREG_CONTROL, 0x1);

	/* Disable DMA */
	writeRegMsk(pcie.breg, BRCFG_PCIE_RX0, 0x7, 0x7);

	/* Enable Ingress subtractive decode translation */
	writeReg(pcie.breg, I_ISUB_CONTROL, 0x1);

	/* Enable msg filtering details */
	writeReg(pcie.breg, BRCFG_PCIE_RX_MSG_FILTER, ((1 << 1) | (1 << 2) | (1 << 3)));

	/* This routes the PCIe DMA traffic to go through CCI path (?) */
	writeRegMsk(pcie.breg, BRCFG_PCIE_RX1, 0xff, 0xff);

	/* Check if the phy link is up or not */
	bool phy_link_up = false;
	static const int LINK_WAIT_MAX_RETRIES = 10;
	for (int retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
		uint32_t link_status = readReg(pcie.pcireg, PS_LINKUP_OFFSET);
		if (link_status & 0x2) {
			phy_link_up = true;
			break;
		}
		usleep(100000);
	}

	if (phy_link_up) {
		printf("pcie: phy link up\n");
	}
	else {
		printf("pcie: fail, phy link down\n");
		return -1;
	}

	uint32_t ecam_value = readReg(pcie.breg, E_ECAM_CAPABILITIES);
	if (ecam_value & 0x1) {
		printf("pcie: ecam present\n");
	}
	else {
		printf("pcie: fail ecam is not present\n");
		return -1;
	}

	/* Enable ECAM */
	writeRegMsk(pcie.breg, E_ECAM_CONTROL, 0x1, 0x1);
	/* Set size of translation window */
	writeRegMsk(pcie.breg, E_ECAM_CONTROL, (0x1f << 16), (16 << 16));  // 256 MB

	writeReg(pcie.breg, E_ECAM_BASE_LO, lower_32_bits(CFG_ADDRESS));
	writeReg(pcie.breg, E_ECAM_BASE_HI, upper_32_bits(CFG_ADDRESS));

	/* Check link status */
	bool pcie_link_up = false;
	for (int retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
		uint32_t pcie_link_status = readReg(pcie.pcireg, PS_LINKUP_OFFSET);
		if (pcie_link_status & 0x1) {
			pcie_link_up = true;
			break;
		}
		usleep(100000);
	}

	if (pcie_link_up) {
		printf("pcie: pcie link up\n");
	}
	else {
		printf("pcie: fail pcie link down\n");
		return -1;
	}

	/* Read local config space */
	uint32_t config = readReg(pcie.cfg, 0x4);

	config |= (0x4 | 0x2 |
			0x1 | 0x40 |
			0x100);
	writeReg(pcie.cfg, 0x4, config);

	/* Unmap memory */
	munmap((void *)pcie.breg, BREG_SIZE);
	munmap((void *)pcie.pcireg, PCIREG_SIZE);

	return 0;
}

int main(int argc, char **argv)
{
	int ret = 0;

	/* Wait for PS GTR PHY initialisation */
	oid_t oid;
	while (lookup("/dev/fsbl", NULL, &oid) < 0) {
		usleep(10000);
	}

	/* Configure PCI peripheral */
	printf("pcie: configure PCI Express peripheral...\n");
	ret = pcie_initPcie();
	if (ret != 0) {
		printf("pcie: fail to configure PCI Express peripheral\n");
		return ret;
	}

	printf("pcie: enumeration... \n");
	pcie_scanBus(0, 0);

	return 0;
}
