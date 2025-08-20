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

#include "pci_link.h"

const char *pci_ltssm_state2str(ltssm_state_t state)
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

phy_link_status pci_checkLinkStatus(uint32_t *pcie)
{
	phy_link_status ret = { 0 };

	uint32_t phy_link_control_reg = readReg(pcie, 0x144);
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