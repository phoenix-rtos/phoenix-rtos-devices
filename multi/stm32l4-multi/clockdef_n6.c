/*
 * Phoenix-RTOS
 *
 * stm32l4-multi
 *
 * Clock tree definition and parser
 *
 * Copyright 2025 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>

#include "clockdef_n6.h"
#include "stm32n6_regs.h"

#define CLOCK_LITERAL(x) \
	{ \
		.type = clockdef_type_literal, .literal = (x) \
	}

static const clockdef_literal_t msi_base_vals[] = { 4 * 1000 * 1000, 16 * 1000 * 1000 };
static const clockdef_clkID_t sysa_mux_vals[] = { clkid_hsi, clkid_msi, clkid_hse, clkid_ic1 };
static const clockdef_clkID_t sysb_mux_vals[] = { clkid_hsi, clkid_msi, clkid_hse, clkid_ic2 };
static const clockdef_clkID_t sysc_mux_vals[] = { clkid_hsi, clkid_msi, clkid_hse, clkid_ic6 };
static const clockdef_clkID_t sysd_mux_vals[] = { clkid_hsi, clkid_msi, clkid_hse, clkid_ic11 };
static const clockdef_clkID_t per_mux_vals[] = { clkid_hsi, clkid_msi, clkid_hse, clkid_ic19, clkid_ic5, clkid_ic10, clkid_ic15, clkid_ic20 };

static const clockdef_obj_t clockObjects[] = {
	[clkid_lsi] = { .base = CLOCK_LITERAL(32000), .nom = CLOCK_LITERAL(1), .denom = CLOCK_LITERAL(1) },
	[clkid_lse] = { .base = CLOCK_LITERAL(32768), .nom = CLOCK_LITERAL(1), .denom = CLOCK_LITERAL(1) },
	[clkid_msi] = {
		.base = {
			.type = clockdef_type_register_lookup,
			.reg_lookup = {
				.r = {
					.offset = rcc_msicfgr,
					.shift = 9,
					.mask = 0x1,
					.val_offset = 0,
				},
				.vals = msi_base_vals,
			},
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_hsi] = { .base = CLOCK_LITERAL(64 * 1000 * 1000), .nom = CLOCK_LITERAL(1), .denom = CLOCK_LITERAL(1) },
	[clkid_hsi_div] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_hsi,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = {
				.offset = rcc_hsicfgr,
				.shift = 7,
				.mask = 0x3,
				.val_offset = 0,
			},
		},
	},
	[clkid_hse] = { .base = CLOCK_LITERAL(48 * 1000 * 1000), .nom = CLOCK_LITERAL(1), .denom = CLOCK_LITERAL(1) },
	[clkid_hse_div2_osc] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_hse,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = {
				.offset = rcc_hsecfgr,
				.shift = 6,
				.mask = 0x1,
				.val_offset = 0,
			},
		},
	},
	[clkid_hse_rtc] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_hse,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register,
			.reg = {
				.offset = rcc_ccipr7,
				.shift = 12,
				.mask = 0x3f,
				.val_offset = 1,
			},
		},
	},
	[clkid_sysa] = {
		.base = {
			.type = clockdef_type_register_mux,
			.reg_mux = {
				.r = {
					.offset = rcc_cfgr1,
					.shift = 20,
					.mask = 0x3,
					.val_offset = 0,
				},
				.vals = sysa_mux_vals,
			},
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_sysb] = {
		.base = {
			.type = clockdef_type_register_mux,
			.reg_mux = {
				.r = {
					.offset = rcc_cfgr1,
					.shift = 28,
					.mask = 0x3,
					.val_offset = 0,
				},
				.vals = sysb_mux_vals,
			},
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_sysc] = {
		.base = {
			.type = clockdef_type_register_mux,
			.reg_mux = {
				.r = {
					.offset = rcc_cfgr1,
					.shift = 28,
					.mask = 0x3,
					.val_offset = 0,
				},
				.vals = sysc_mux_vals,
			},
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_sysd] = {
		.base = {
			.type = clockdef_type_register_mux,
			.reg_mux = {
				.r = {
					.offset = rcc_cfgr1,
					.shift = 28,
					.mask = 0x3,
					.val_offset = 0,
				},
				.vals = sysd_mux_vals,
			},
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
	[clkid_timg] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_sysb,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = {
				.offset = rcc_cfgr2,
				.shift = 24,
				.mask = 0x3,
				.val_offset = 0,
			},
		},
	},
	[clkid_hclk] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_sysb,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = {
				.offset = rcc_cfgr2,
				.shift = 20,
				.mask = 0x7,
				.val_offset = 0,
			},
		},
	},
	[clkid_pclk1] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_sysb,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = {
				.offset = rcc_cfgr2,
				.shift = 0,
				.mask = 0x7,
				.val_offset = 0,
			},
		},
	},
	[clkid_pclk2] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_sysb,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = {
				.offset = rcc_cfgr2,
				.shift = 4,
				.mask = 0x7,
				.val_offset = 0,
			},
		},
	},
	[clkid_pclk4] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_sysb,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = {
				.offset = rcc_cfgr2,
				.shift = 12,
				.mask = 0x7,
				.val_offset = 0,
			},
		},
	},
	[clkid_pclk5] = {
		.base = {
			.type = clockdef_type_clkID,
			.clkID = clkid_sysb,
		},
		.nom = CLOCK_LITERAL(1),
		.denom = {
			.type = clockdef_type_register_log,
			.reg_log = {
				.offset = rcc_cfgr2,
				.shift = 16,
				.mask = 0x7,
				.val_offset = 0,
			},
		},
	},
	[clkid_per] = {
		.base = {
			.type = clockdef_type_register_mux,
			.reg_mux = {
				.r = {
					.offset = rcc_ccipr7,
					.shift = 0,
					.mask = 0x7,
					.val_offset = 0,
				},
				.vals = per_mux_vals,
			},
		},
		.nom = CLOCK_LITERAL(1),
		.denom = CLOCK_LITERAL(1),
	},
};


static const size_t clockObjects_size = sizeof(clockObjects) / sizeof(clockObjects[0]);


static uint32_t clockdef_getReg(const clockdef_register_t *reg)
{
	uint32_t v = clockdef_getRegHW(reg->offset);
	v = (v >> reg->shift) & reg->mask;
	return v + reg->val_offset;
}

static int clockdef_getNumber(const clockdef_number_t *n, uint32_t *num_out)
{
	switch (n->type) {
		case clockdef_type_literal:
			*num_out = n->literal;
			return EOK;
		case clockdef_type_register:
			*num_out = clockdef_getReg(&n->reg);
			return EOK;
		case clockdef_type_register_log:
			*num_out = (1 << clockdef_getReg(&n->reg));
			return EOK;
		case clockdef_type_register_lookup:
			*num_out = n->reg_lookup.vals[clockdef_getReg(&n->reg_lookup.r)];
			return EOK;
		default:
			return -EINVAL;
	}
}


static int clockdef_getBase(const clockdef_base_t *n, uint32_t *num_out, clockdef_clkID_t *prev)
{
	switch (n->type) {
		case clockdef_type_literal:
			*num_out = n->literal;
			*prev = clkid_none;
			return EOK;
		case clockdef_type_clkID:
			*num_out = 0;
			*prev = n->clkID;
			return EOK;
		case clockdef_type_register_lookup:
			*num_out = n->reg_lookup.vals[clockdef_getReg(&n->reg_lookup.r)];
			*prev = clkid_none;
			return EOK;
		case clockdef_type_register_mux:
			*num_out = 0;
			*prev = n->reg_mux.vals[clockdef_getReg(&n->reg_mux.r)];
			return EOK;
		default:
			return -EINVAL;
	}
}


int clockdef_getClock(clockdef_clkID_t id, uint64_t *output)
{
	if (id == clkid_none) {
		return -ENOENT;
	}

	int ret;
	uint32_t base;
	uint64_t total_nom = 1;
	uint64_t total_denom = 1;
	clockdef_clkID_t prev;
	do {
		if (id >= clockObjects_size) {
			ret = clockdef_getClockHW(id, &base, &total_nom, &total_denom, &prev);
			if (ret < 0) {
				return ret;
			}
		}
		else {
			uint32_t nom, denom;
			const clockdef_obj_t *tf = &clockObjects[id];
			ret = clockdef_getNumber(&tf->nom, &nom);
			if (ret < 0) {
				return ret;
			}

			ret = clockdef_getNumber(&tf->denom, &denom);
			if (ret < 0) {
				return ret;
			}

			ret = clockdef_getBase(&tf->base, &base, &prev);
			if (ret < 0) {
				return ret;
			}

			total_nom *= nom;
			total_denom *= denom;
		}

		id = prev;
	} while (prev != clkid_none);

	if (total_denom == 0) {
		return -EINVAL;
	}

	*output = (base * total_nom) / total_denom;
	return EOK;
}
