#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "pci_utils.h"

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

const char *pci_ltssm_state2str(ltssm_state_t state);
phy_link_status pci_checkLinkStatus(uint32_t *pcie);