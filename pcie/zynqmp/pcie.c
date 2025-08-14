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
#include <stdatomic.h>
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

#define upper_32_bits(n) ((uint32_t)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((uint32_t)((n) & 0xffffffff))

#define ECAM_BUS_SHIFT  20
#define ECAM_DEV_SHIFT  15
#define ECAM_FUNC_SHIFT 12

// #define PCI_BAR0_ADD 0xB0000000
// #define PCI_BAR1_ADD 0xB0010000
// #define PCI_BAR2_ADD 0xB0020000
// #define PCI_BAR3_ADD 0xB0030000
// #define PCI_BAR4_ADD 0xB0040000
// #define PCI_BAR5_ADD 0xB0050000
/* Keep in mind this is permuted for some reason SMH... */
/* Apparently we need both physical 32-bit and 64-bit addresses to properly map ranges */
/* Now we will have only prefetchble 64-bit addresses and no others */
#define PCI_BAR0_ADD 0x520000000
#define PCI_BAR1_ADD 0x530000000
#define PCI_BAR2_ADD 0x540000000
#define PCI_BAR3_ADD 0x550000000
#define PCI_BAR4_ADD 0x560000000
#define PCI_BAR5_ADD 0x570000000

uintptr_t bar_add[] = {
	PCI_BAR0_ADD,
	PCI_BAR1_ADD,
	PCI_BAR2_ADD,
	PCI_BAR3_ADD,
	PCI_BAR4_ADD,
	PCI_BAR5_ADD
};

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

#define BAR0_PHYS_ADD 0xB0000000

/* Adding read/write memory barriers */
#define rmb() atomic_thread_fence(memory_order_acquire)
#define wmb() atomic_thread_fence(memory_order_release)

static void
pcie_scanBus(uint8_t bus, int depth);

static struct {
	uint32_t *fpga_gpio;
	uint32_t *pcie;
} common;

typedef struct {
	/* Use to create linked list of detected devices */
	struct pci_dev_t *next;
	/* Geographical address of PCIe endpoint */
	uint8_t bus;
	uint8_t device;
	uint8_t func;

	/* Bar info */
	uintptr_t bar0;
	size_t bar0_len;

	uintptr_t bar1_phys_add;
	size_t bar1_len;

	uintptr_t bar2_phys_add;
	size_t bar2_len;

	uintptr_t bar3_phys_add;
	size_t bar3_len;

	uintptr_t bar4_phys_add;
	size_t bar4_len;

	uintptr_t bar5_phys_add;
	size_t bar5_len;
} pci_dev_t;

pci_dev_t device;

typedef enum {
	/* Detect substates */
	ltssm_detect_quiet = 0x00,
	ltssm_detect_active = 0x01,
	/* Polling substates */
	ltssm_polling_active = 0x02,
	ltssm_polling_compliance = 0x03,
	ltssm_polling_configuration = 0x04,
	/* Configuration substates */
	ltssm_cfg_linkwidth_start = 0x05,
	ltssm_cfg_linkwidth_accept = 0x06,
	ltssm_cfg_lanenum_accept = 0x07,
	ltssm_cfg_lanenum_wait = 0x08,
	ltssm_cfg_complete = 0x09,
	ltssm_cfg_idle = 0x0a,
	/* Recovery substates */
	ltssm_recovery_rcvrlock = 0x0b,
	ltssm_recovery_speed = 0x0c,
	ltssm_recovery_rcvrcfg = 0x0d,
	ltssm_recovery_idle = 0x0e,
	/* L0 state */
	ltssm_l0 = 0x10,
	/* 0x11-0x16: reserved */
	ltssm_reserved_11 = 0x11,
	ltssm_reserved_12 = 0x12,
	ltssm_reserved_13 = 0x13,
	ltssm_reserved_14 = 0x14,
	ltssm_reserved_15 = 0x15,
	ltssm_reserved_16 = 0x16,
	/* L1 substates */
	ltssm_l1_entry = 0x17,
	ltssm_l1_idle = 0x18,
	/* 0x19-0x1a: reserved */
	ltssm_reserved_19 = 0x19,
	ltssm_reserved_1a = 0x1a,
	/* Disabled state */
	ltssm_disabled = 0x20,
	/* Loopback substates */
	ltssm_loopback_entry_master = 0x21,
	ltssm_loopback_active_master = 0x22,
	ltssm_loopback_exit_master = 0x23,
	ltssm_loopback_entry_slave = 0x24,
	ltssm_loopback_active_slave = 0x25,
	ltssm_loopback_exit_slave = 0x26,
	/* Hot reset state */
	ltssm_hot_reset = 0x27,
	/* Recovery equalization phase substates */
	ltssm_recovery_eq_phase0 = 0x28,
	ltssm_recovery_eq_phase1 = 0x29,
	ltssm_recovery_eq_phase2 = 0x2a,
	ltssm_recovery_eq_phase3 = 0x2b
} ltssm_state_t;

typedef struct {
	uint32_t link_rate;
	uint32_t link_width;
	ltssm_state_t ltssm_state;
	bool link_up;
	uint32_t raw_register_val;
} phy_link_status;

static const char *ltssm_state2str(ltssm_state_t state)
{
	static const char *const names[] = {
		[ltssm_detect_quiet] = "Detect.Quiet",
		[ltssm_detect_active] = "Detect.Active",
		[ltssm_polling_active] = "Polling.Active",
		[ltssm_polling_compliance] = "Polling.Compliance",
		[ltssm_polling_configuration] = "Polling.Configuration",
		[ltssm_cfg_linkwidth_start] = "Configuration.Linkwidth.Start",
		[ltssm_cfg_linkwidth_accept] = "Configuration.Linkwidth.Accept",
		[ltssm_cfg_lanenum_accept] = "Configuration.Lanenum.Accept",
		[ltssm_cfg_lanenum_wait] = "Configuration.Lanenum.Wait",
		[ltssm_cfg_complete] = "Configuration.Complete",
		[ltssm_cfg_idle] = "Configuration.Idle",
		[ltssm_recovery_rcvrlock] = "Recovery.RcvrLock",
		[ltssm_recovery_speed] = "Recovery.Speed",
		[ltssm_recovery_rcvrcfg] = "Recovery.RcvrCfg",
		[ltssm_recovery_idle] = "Recovery.Idle",
		[ltssm_l0] = "L0",
		[ltssm_reserved_11] = "Reserved (0x11)",
		[ltssm_reserved_12] = "Reserved (0x12)",
		[ltssm_reserved_13] = "Reserved (0x13)",
		[ltssm_reserved_14] = "Reserved (0x14)",
		[ltssm_reserved_15] = "Reserved (0x15)",
		[ltssm_reserved_16] = "Reserved (0x16)",
		[ltssm_l1_entry] = "L1.Entry",
		[ltssm_l1_idle] = "L1.Idle",
		[ltssm_reserved_19] = "Reserved (0x19)",
		[ltssm_reserved_1a] = "Reserved (0x1a)",
		[ltssm_disabled] = "Disabled",
		[ltssm_loopback_entry_master] = "Loopback_Entry_Master",
		[ltssm_loopback_active_master] = "Loopback_Active_Master",
		[ltssm_loopback_exit_master] = "Loopback_Exit_Master",
		[ltssm_loopback_entry_slave] = "Loopback_Entry_Slave",
		[ltssm_loopback_active_slave] = "Loopback_Active_Slave",
		[ltssm_loopback_exit_slave] = "Loopback_Exit_Slave",
		[ltssm_hot_reset] = "Hot_Reset",
		[ltssm_recovery_eq_phase0] = "Recovery_Equalization_Phase0",
		[ltssm_recovery_eq_phase1] = "Recovery_Equalization_Phase1",
		[ltssm_recovery_eq_phase2] = "Recovery_Equalization_Phase2",
		[ltssm_recovery_eq_phase3] = "Recovery_Equalization_Phase3",
	};

	if (state < (ltssm_state_t)(sizeof(names) / sizeof(names[0])) &&
			names[state] != NULL)
		return names[state];

	return "Reserved/Unknown";
}

/* Helper read/write functions that ensure memory ordering */
uint32_t ioread32(uintptr_t ptr)
{
	rmb();
	return *(uint32_t *)ptr;
}

uint16_t ioread16(uintptr_t ptr)
{
	rmb();
	return *(uint16_t *)ptr;
}

uint8_t ioread8(uintptr_t ptr)
{
	rmb();
	return *(uint8_t *)ptr;
}

void iowrite32(uint32_t val, uintptr_t ptr)
{
	*(uint32_t *)ptr = val;
	wmb();
}

void iowrite16(uint16_t val, uintptr_t ptr)
{
	*(uint16_t *)ptr = val;
	wmb();
}

void iowrite8(uint8_t val, uintptr_t ptr)
{
	*(uint8_t *)ptr = val;
	wmb();
}

static void writeReg(uint32_t *base, uint32_t offset, uint32_t value)
{
	*((volatile uint32_t *)((char *)base + offset)) = value;
}

static volatile uint32_t *ecamRegPtr(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t reg)
{
	uintptr_t cfg_space_offset = ((uintptr_t)bus << ECAM_BUS_SHIFT) |
			((uintptr_t)dev << ECAM_DEV_SHIFT) |
			((uintptr_t)fn << ECAM_FUNC_SHIFT) |
			(reg & 0xfffu);

	return (volatile uint32_t *)((uintptr_t)common.pcie + cfg_space_offset);
}

// static inline uintptr_t ecamRegPtr(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t reg)
// {
// 	return (uintptr_t)common.pcie + (((uintptr_t)bus << ECAM_BUS_SHIFT) |
// 			((uintptr_t)dev << ECAM_DEV_SHIFT) |
// 			((uintptr_t)fn << ECAM_FUNC_SHIFT) |
// 			(reg & 0xfffu));
// }

static uint32_t ecamRead32(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
{
	return *ecamRegPtr(bus, dev, fn, off & ~0x3u);
	// return ioread32(ecamRegPtr(bus, dev, fn, off));
}

static void ecamWrite32(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off, uint32_t val)
{
	*ecamRegPtr(bus, dev, fn, off & ~0x3u) = val;
	// iowrite32(val, ecamRegPtr(bus, dev, fn, off));
}

static uint16_t ecamRead16(uint8_t bus, uint8_t dev, uint8_t fn, uint16_t off)
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

/* Should probably split print bars into
printing and actually mapping those to
calling process. There is similar mechanism
on Linux where driver has to acquire BARs */

static void print_bars(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t hdr, int depth)
{
	printf("pcie: Reading out BARs\n");
	/* Choose number of bars depending on config type */
	const int bar_count = (hdr == 0x00) ? 6 : 2;

	int detected_bars = 0;

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
			bool prefetchble = (bar_low & (1 << 3)) != 0;
			uint64_t addr = bar_low & ~0xfu;

			if (is_64_bit) {
				uint32_t bar_high = ecamRead32(bus, dev, fn, PCI_BAR0 + (i + 1) * 4);
				addr |= ((uint64_t)bar_high) << 32;
				i++;
			}

			/* We will now read out size of this BAR */
			if (is_64_bit) {
				ecamWrite32(bus, dev, fn, PCI_BAR0 + (i - 1) * 4, ~((uint32_t)0));
				ecamWrite32(bus, dev, fn, PCI_BAR0 + i * 4, ~((uint32_t)0));

				uint64_t bar_size = ecamRead32(bus, dev, fn, PCI_BAR0 + (i - 1) * 4);
				bar_size |= (uint64_t)ecamRead32(bus, dev, fn, PCI_BAR0 + i * 4) << 32;

				/* Ignore information bits */
				bar_size &= ~(0b1111ul);
				/* Negate result */
				bar_size = ~bar_size;
				/* Add one */
				bar_size++;

				device.bar0_len = bar_size;

				printf("pcie: BAR%d size: %016llx, %s\n",
						i - 1, (unsigned long long)bar_size,
						prefetchble ? "(prefetchble)" : "(non-prefetchble)");

				/* Inserting BAR address */
				uintptr_t local_bar_add = bar_add[detected_bars++];
				uint32_t low_add = (uint32_t)(local_bar_add & ~((uint32_t)0));
				uint32_t high_add = (uint32_t)(local_bar_add >> 32);

				ecamWrite32(bus, dev, fn, PCI_BAR0 + (i - 1) * 4, low_add);
				ecamWrite32(bus, dev, fn, PCI_BAR0 + i * 4, high_add);

				addr = (uint64_t)ecamRead32(bus, dev, fn, PCI_BAR0 + i * 4) << 32 |
						ecamRead32(bus, dev, fn, PCI_BAR0 + (i - 1) * 4);
			}
			else {
				ecamWrite32(bus, dev, fn, PCI_BAR0 + i * 4, ~((uint32_t)0));
				uint32_t bar_size = ecamRead32(bus, dev, fn, PCI_BAR0 + i * 4);

				device.bar0_len = bar_size;

				printf("pcie: BAR%d size: %08x\n", i, (unsigned int)bar_size);

				/* Inserting BAR address */
				uint32_t local_bar_add = (uint32_t)bar_add[detected_bars++];

				ecamWrite32(bus, dev, fn, PCI_BAR0 + i * 4, local_bar_add);

				addr = ecamRead32(bus, dev, fn, PCI_BAR0 + i * 4);
			}

			if (i == 5)
				continue;

			writeReg(common.pcie, 0x148, 1);

			addr = addr & ~(0xful);

			printf("pcie: BAR%d MEM 0x%016llx (%s)\n",
					is_64_bit ? i - 1 : i, (unsigned long long)addr,
					is_64_bit ? "64-bit" : "32-bit");
			printf("Testing access to BAR%d\n", is_64_bit ? i - 1 : i);

			void *device_bar_add = mmap(NULL, device.bar0_len, PROT_WRITE | PROT_READ,
					MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)addr);
			if (device_bar_add != MAP_FAILED) {
				uintptr_t phys_add = (uintptr_t)va2pa(device_bar_add);
				printf("Reading from virt-add: 0x%lx, phys-add: %08lx\n", (uintptr_t)device_bar_add, (uintptr_t)phys_add);
				printf("First 4 bytes of BAR%d: 0x%08x\n", is_64_bit ? i - 1 : i,
						*(uint32_t *)device_bar_add + 16);
			}
			else {
				printf("Failed to map BAR%d\n", is_64_bit ? i - 1 : i);
			}

			writeReg(common.pcie, 0x148, 0);
			munmap(device_bar_add, device.bar0_len);
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

#define PCIE_BRIDGE_COMMAND_OFF            0x4
#define PCIE_BRIDGE_MEMBASE_OFF            020
#define PCIE_BRIDGE_PREF_MEM_BASE_LOW_OFF  0x24
#define PCIE_BRIDGE_PREF_MEM_BASE_HIGH_OFF 0x28
#define PCIE_BRIDGE_PREF_MEM_LIM_OFF       0x2c

static void pcie_configureBridge(uint8_t bus, uint8_t dev, uint8_t fun)
{
	printf("pcie: Detected device is a bridge...\n");
	printf("pcie: For subordinate devices to work this has to be configured\n");

	printf("pcie: Setting command register\n");
	uint32_t temp = ecamRead32(bus, dev, fun, PCIE_BRIDGE_COMMAND_OFF);
	temp |= 0x7; /* Enable I/O space, memory space and bus master */
	ecamWrite32(bus, dev, fun, PCIE_BRIDGE_COMMAND_OFF, temp);

	printf("pcie: Setting memory base and memory limit\n");
	/* We will set only prefetchble base and limit upper (we have only 64-bit addresses now in bitstream) */
	/* Check note next to definition of BAR addresses */
	ecamWrite32(bus, dev, fun, PCIE_BRIDGE_PREF_MEM_BASE_LOW_OFF, 0x70002000);
	ecamWrite32(bus, dev, fun, PCIE_BRIDGE_PREF_MEM_BASE_HIGH_OFF, (uint32_t)(PCI_BAR0_ADD >> 32));
	ecamWrite32(bus, dev, fun, PCIE_BRIDGE_PREF_MEM_LIM_OFF, (uint32_t)(PCI_BAR5_ADD >> 32));

	printf("pcie: Configured mem-pref-base:%08x\n", ecamRead32(bus, dev, fun, PCIE_BRIDGE_PREF_MEM_BASE_LOW_OFF));
	printf("pcie: Configured mem-pref-base-high:%016lx, mem-pref-lim:%016lx\n",
			(uint64_t)ecamRead32(bus, dev, fun, PCIE_BRIDGE_PREF_MEM_BASE_HIGH_OFF) << 32,
			(uint64_t)ecamRead32(bus, dev, fun, PCIE_BRIDGE_PREF_MEM_LIM_OFF) << 32);
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

	ecamWrite32(bus, dev, fun, PCI_COMMAND, 0);

	/* Enable MEM-space and Bus Master if still disabled */
	uint16_t cmd = ecamRead16(bus, dev, fun, PCI_COMMAND);
	if (!(cmd & (PCI_CMD_MEM | PCI_CMD_MASTER))) {
		printf("pcie: enable memory space and bus master\n");
		ecamWrite32(bus, dev, fun, PCI_COMMAND, cmd | 0x4 | 0x2 | 0x1 | 0x40 | 0x100);
	}

	/* Print some info about this device */
	print_bars(bus, dev, fun, hdr, depth + 1);
	print_capabilities(bus, dev, fun, depth + 1);

	/* If this is a PCI-PCI bridge program buses and recurse */
	if (hdr == 0x01) {
		/* read once */
		uint8_t sec = ecamRead8(bus, dev, fun, PCI_SECONDARY_BUS);

		/* Configure bridge */
		pcie_configureBridge(bus, dev, fun);

		if (sec == 0) {
			sec = next_bus++;

			uint32_t cur = ecamRead32(bus, dev, fun, PCI_PRIMARY_BUS) & 0xff000000;
			uint32_t val = cur |
					((uint32_t)0xFF << 16) | /* subordinate = ff */
					((uint32_t)sec << 8) |   /* secondary   = sec */
					bus;                     /* primary     = bus */

			printf("pcie: write pri, sec, sub 0x%x \n", val);
			ecamWrite32(bus, dev, fun, PCI_PRIMARY_BUS, val);
			uint32_t val_read = ecamRead32(bus, dev, fun, PCI_PRIMARY_BUS);
			printf("pcie: validate pri, sec, sub 0x%x \n", val_read);
		}

		uint8_t sub = ecamRead8(bus, dev, fun, PCI_SUBORDINATE_BUS);
		printf("pcie: bridge bus primary %u secondary %u subordinate %u\n",
				bus, sec, sub);

		pcie_scanBus(sec, depth + 1);

		/* final subordinate = highest bus encountered */
		uint32_t cur = ecamRead32(bus, dev, fun, PCI_PRIMARY_BUS) & 0xff00ffff;
		uint32_t val = cur | ((uint32_t)(next_bus - 1) << 16);
		ecamWrite32(bus, dev, fun, PCI_PRIMARY_BUS, val);
	}
}

static void pcie_scanBus(uint8_t bus, int depth)
{
	/* Iterate over all devices connected to the certain bus */
	for (uint8_t dev = 0; dev < (bus ? 32 : 1); ++dev) {
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


static void writeRegMsk(uint32_t *base, uint32_t offset, uint32_t clr, uint32_t set)
{
	uint32_t value = readReg(base, offset);
	value &= ~clr;
	value |= set;
	writeReg(base, offset, value);
}

static void deassertAxiInterconnectsReset(void)
{
	printf("Deasserting AXI interconnect\n");
	platformctl_t ctl3 = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_s_axi_hpc_0_fpd,
		.devreset.state = 0,
	};
	int ret = platformctl(&ctl3);
	if (ret != 0) {
		printf("pcie: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hpc_1_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		printf("pcie: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hp_0_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		printf("pcie: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hp_1_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		printf("pcie: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hpc_2_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		printf("pcie: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hpc_3_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		printf("pcie: fail to deassert reset\n");
	}
}

static void readBridgeInfoReg(void)
{
	uint32_t bridge_info = readReg(common.pcie, 0x130);

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

	printf("pcie: bridge generation: %s, root port: %s\n",
			gen_cap, root_port_present_text);
}

static phy_link_status checkLinkStatus(void)
{
	phy_link_status ret = { 0 };

	uint32_t phy_link_control_reg = readReg(common.pcie, 0x144);
	ret.raw_register_val = phy_link_control_reg;
	if (phy_link_control_reg & (1 << 12)) {
		ret.link_rate = 3;
	}
	else if (phy_link_control_reg & (1 << 10)) {
		ret.link_rate = 2;
	}
	else {
		ret.link_rate = 1;
	}

	ret.link_width = 1 << (((phy_link_control_reg >> 1) & 0x3) | ((phy_link_control_reg >> 11) & 0x4));

	ret.ltssm_state = (ltssm_state_t)((phy_link_control_reg >> 3) & 0x3f);

	ret.link_up = (bool)((phy_link_control_reg >> 11) & 0x1);

	return ret;
}

#define AXI_HPM1_FPD_REG 0x00FD615000

#define AXI_WIDTH_CFG_32B  0x0
#define AXI_WIDTH_CFG_64B  0x1
#define AXI_WIDTH_CFG_128B 0x2

int main(int argc, char **argv)
{
	printf("Entering PCIe test app\n");
	usleep(10000);

	/* Map memory */
	common.fpga_gpio = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0xa0020000);
	if (NULL == common.fpga_gpio) {
		printf("pcie: fail to map FPGA GPIO memory\n");
		return -1;
	}
	common.pcie = mmap(NULL, 0x10000000, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0x500000000);
	if (NULL == common.pcie) {
		printf("pcie: fail to map AXI PCIE bridge memory\n");
		return -1;
	}

	volatile uint32_t *axi_slave1_width_reg = (volatile uint32_t *)mmap(NULL, 4, PROT_WRITE | PROT_READ,
			MAP_PHYSMEM | MAP_ANONYMOUS, -1, AXI_HPM1_FPD_REG);

	if (NULL == axi_slave1_width_reg) {
		printf("pcie: mmap failed for this AXI width register\n");
		return -1;
	}

	uint32_t configuration = (*axi_slave1_width_reg & ~((1 << 10) | (1 << 11))) | (AXI_WIDTH_CFG_128B << 10);
	*axi_slave1_width_reg = configuration;

	printf("Configuring AXI slave 1 FPD width: %x\n", *axi_slave1_width_reg);

	printf("Managed to map PCIe bridge addresses\n");

	/* Deassert reset on AXI Interconnect between PS and PL */
	deassertAxiInterconnectsReset();
	usleep(3 * 1000);

	/* Deassert PCI Express reset pin */
	writeReg(common.fpga_gpio, 0x08, 0x1);
	/* Let PCI Express node initialise */
	usleep(100 * 1000);

	/* Deassert reset on AXI PCI Express bridge IP Core */
	writeReg(common.fpga_gpio, 0x08, 0x3);
	/* Let bridge turn link up */
	usleep(100 * 1000);

	/* Check Bridge whoami info */
	readBridgeInfoReg();

	/* Check PHY link status */
	phy_link_status phy_link_status = checkLinkStatus();
	if (phy_link_status.link_up) {
		printf("pcie: phy LINK UP, link rate: %u, link width x%u, ltssm state: %s\n",
				phy_link_status.link_rate,
				phy_link_status.link_width,
				ltssm_state2str(phy_link_status.ltssm_state));
	}
	else {
		printf("pcie: PHY LINK DOWN, phy status reg: 0x%x\n", phy_link_status.raw_register_val);
		return -1;
	}

	printf("Disable interrupts\n");
	/* Disable interrupt */
	writeReg(common.pcie, 0x13c, 0x0);

	printf("Clear MSIx and legacy interrupts\n");
	/* Clear pending interrupts */
	writeRegMsk(common.pcie, 0x138, 0x0ff30fe9, 0x0ff30fe9);

	/* MSI decode mode */
	writeReg(common.pcie, 0x178, 0xffffffff);
	writeReg(common.pcie, 0x17c, 0xffffffff);

	/* Make sure that root complex is turned off */
	writeReg(common.pcie, 0x148, 0);

	printf("Scanning bus0...\n");
	pcie_scanBus(0, 0);

	munmap((void *)common.pcie, 0x10000000);
	munmap((void *)common.fpga_gpio, 0x1000);

	return 0;
}
