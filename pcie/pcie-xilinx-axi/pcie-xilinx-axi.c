/*
 * Phoenix-RTOS
 *
 * PCI Express Xilinx AXI bridge driver
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
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <posix/utils.h>

#include <phoenix/arch/aarch64/zynqmp/zynqmp.h>

#include <board_config.h>

#include <pcie.h>


#define FPGA_CTRL_SIZE    0x1000
#define FPGA_CTRL_ADDRESS 0xa0020000


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
			names[state] != NULL) {
		return names[state];
	}

	return "Reserved/Unknown";
}


static void deassertAxiInterconnectsReset(void)
{
	platformctl_t ctl3 = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_s_axi_hpc_0_fpd,
		.devreset.state = 0,
	};
	int ret = platformctl(&ctl3);
	if (ret != 0) {
		fprintf(stderr, "pcie-xilinx-axi: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hpc_1_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		fprintf(stderr, "pcie-xilinx-axi: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hp_0_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		fprintf(stderr, "pcie-xilinx-axi: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hp_1_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		fprintf(stderr, "pcie-xilinx-axi: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hpc_2_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		fprintf(stderr, "pcie-xilinx-axi: fail to deassert reset\n");
	}

	ctl3.devreset.dev = pctl_devreset_fpd_s_axi_hpc_3_fpd;
	ret = platformctl(&ctl3);
	if (ret != 0) {
		fprintf(stderr, "pcie-xilinx-axi: fail to deassert reset\n");
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

	printf("pcie-xilinx-axi: bridge generation: %s, root port: %s\n",
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


int pcie_xilinx_axi_init(void)
{
	/* Map memory */
	common.fpga_gpio = mmap(NULL, FPGA_CTRL_SIZE, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, FPGA_CTRL_ADDRESS);
	if (MAP_FAILED == common.fpga_gpio) {
		fprintf(stderr, "pcie-xilinx-axi: fail to map FPGA GPIO memory\n");
		return -1;
	}
	common.pcie = mmap(NULL, ECAM_SIZE, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, ECAM_ADDRESS);
	if (MAP_FAILED == common.pcie) {
		munmap((void *)common.fpga_gpio, FPGA_CTRL_SIZE);
		fprintf(stderr, "pcie-xilinx-axi: fail to map AXI PCIE bridge memory\n");
		return -1;
	}

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
		printf("pcie-xilinx-axi: phy LINK UP, link rate: %u, link width x%u, ltssm state: %s\n",
				phy_link_status.link_rate,
				phy_link_status.link_width,
				ltssm_state2str(phy_link_status.ltssm_state));
	}
	else {
		munmap((void *)common.pcie, ECAM_SIZE);
		munmap((void *)common.fpga_gpio, FPGA_CTRL_SIZE);
		fprintf(stderr, "pcie-xilinx-axi: PHY LINK DOWN, phy status reg: 0x%x\n", phy_link_status.raw_register_val);
		return -1;
	}

	/* Disable interrupt */
	writeReg(common.pcie, 0x13c, 0x0);

	/* Clear pending interrupts */
	writeRegMsk(common.pcie, 0x138, 0x0ff30fe9, 0x0ff30fe9);

	/* MSI decode mode */
	writeReg(common.pcie, 0x178, 0xffffffff);
	writeReg(common.pcie, 0x17c, 0xffffffff);

	munmap((void *)common.pcie, ECAM_SIZE);
	munmap((void *)common.fpga_gpio, FPGA_CTRL_SIZE);

	return 0;
}
