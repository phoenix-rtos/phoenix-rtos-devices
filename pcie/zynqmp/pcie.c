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

#include "pci_utils.h"
#include "pci_core.h"
#include "pci_hailo.c"

#define upper_32_bits(n) ((uint32_t)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((uint32_t)((n) & 0xffffffff))

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


#define PCIE_BRIDGE_COMMAND_OFF            0x4
#define PCIE_BRIDGE_MEMBASE_OFF            020
#define PCIE_BRIDGE_PREF_MEM_BASE_LOW_OFF  0x24
#define PCIE_BRIDGE_PREF_MEM_BASE_HIGH_OFF 0x28
#define PCIE_BRIDGE_PREF_MEM_LIM_OFF       0x2c

#define PCI_BAR0    0x10
#define PCI_CAP_PTR 0x34

#define BAR0_PHYS_ADD 0xB0000000

/* Adding read/write memory barriers */
#define rmb() atomic_thread_fence(memory_order_acquire)
#define wmb() atomic_thread_fence(memory_order_release)

static struct {
	uint32_t *fpga_gpio;
	uint32_t *pcie;
} common;


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

void test_NVME(pci_dev_t *root)
{
	pci_dev_t *nvme_dev = pci_getDevFromTree(root, 0x1e4b, 0x1202);
	printf("Found device %p\n", nvme_dev);

	uint32_t bar0_test = *((volatile uint32_t *)nvme_dev->bar[0]);
	printf("Reading from BAR0: %x\n", bar0_test);

	hailo_dumpBAR(nvme_dev, 0);
}

void test_HAILO(pci_dev_t *root)
{
	pci_dev_t *hailo_dev = pci_getDevFromTree(root, 0x1e60, 0x2864);
	printf("Found device %p\n", hailo_dev);

	uint32_t bar0_test = *((volatile uint32_t *)hailo_dev->bar[0]);
	printf("Reading from BAR0: %x\n", bar0_test);
}

/* There is wrong AXI width configuration in plo */
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
	if (MAP_FAILED == common.pcie) {
		printf("pcie: fail to map AXI PCIE bridge memory\n");
		return -1;
	}

	/* Fix AXI configuration */
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

	/* Assert PCI Express pin for some time */
	writeReg(common.fpga_gpio, 0x08, 0x0);
	usleep(100 * 1000);

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

	pci_dev_t root = { 0 };
	root.bus = 0xff;
	root.dev = 0xff;
	root.func = 0xff;
	pci_enumerateRoot(common.pcie, &root, 0, 0);
	pci_printTree(&root, 0);

	/* Enable root complex */
	pci_enableRootComplex(common.pcie);
	usleep(10 * 1000);

	test_NVME(&root);

	munmap((void *)(uintptr_t)common.pcie, 0x10000000);
	munmap((void *)common.fpga_gpio, 0x1000);

	return 0;
}
