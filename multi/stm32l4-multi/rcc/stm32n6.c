/*
 * Phoenix-RTOS
 *
 * STM32N6 reset and clock controller driver
 *
 * Copyright 2017, 2018, 2020, 2025 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Aleksander Kaminski, Jacek Maksymowicz, Mateusz Karcz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>

#include "../common.h"
#include "../rcc.h"
#include "../stm32n6_regs.h"


static const uint8_t rcc_clksels[pctl_ipclks_count][RCC_MAX_CLOCK_CHOICES] = {
	[pctl_ipclk_adf1sel] = { clkid_hclk, clkid_per, clkid_ic7, clkid_ic8, clkid_msi, clkid_hsi_div, clkid_i2s_in, clkid_timg },
	[pctl_ipclk_adc12sel] = { clkid_hclk, clkid_per, clkid_ic7, clkid_ic8, clkid_msi, clkid_hsi_div, clkid_i2s_in, clkid_timg },
	[pctl_ipclk_dcmippsel] = { clkid_pclk5, clkid_per, clkid_ic17, clkid_hsi_div },
	[pctl_ipclk_eth1ptpsel] = { clkid_hclk, clkid_per, clkid_ic13, clkid_hse },
	[pctl_ipclk_eth1clksel] = { clkid_hclk, clkid_per, clkid_ic12, clkid_hse },
	[pctl_ipclk_eth1sel] = {},
	[pctl_ipclk_eth1refclksel] = {},
	[pctl_ipclk_fdcansel] = { clkid_pclk1, clkid_per, clkid_ic19, clkid_hse },
	[pctl_ipclk_fmcsel] = { clkid_hclk, clkid_per, clkid_ic3, clkid_ic4 },
	[pctl_ipclk_i2c1sel] = { clkid_pclk1, clkid_per, clkid_ic10, clkid_ic15, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_i2c2sel] = { clkid_pclk1, clkid_per, clkid_ic10, clkid_ic15, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_i2c3sel] = { clkid_pclk1, clkid_per, clkid_ic10, clkid_ic15, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_i2c4sel] = { clkid_pclk1, clkid_per, clkid_ic10, clkid_ic15, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_i3c1sel] = { clkid_pclk1, clkid_per, clkid_ic10, clkid_ic15, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_i3c2sel] = { clkid_pclk1, clkid_per, clkid_ic10, clkid_ic15, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_ltdcsel] = { clkid_pclk5, clkid_per, clkid_ic16, clkid_hsi_div },
	[pctl_ipclk_mco1sel] = { clkid_hsi_div, clkid_lse, clkid_msi, clkid_lsi, clkid_hse, clkid_ic5, clkid_ic10, clkid_sysa },
	[pctl_ipclk_mco2sel] = { clkid_hsi_div, clkid_lse, clkid_msi, clkid_lsi, clkid_hse, clkid_ic15, clkid_ic20, clkid_sysb },
	[pctl_ipclk_mdf1sel] = { clkid_hclk, clkid_per, clkid_ic7, clkid_ic8, clkid_msi, clkid_hsi_div, clkid_i2s_in, clkid_timg },
	[pctl_ipclk_xspi1sel] = { clkid_hclk, clkid_per, clkid_ic3, clkid_ic4 },
	[pctl_ipclk_xspi2sel] = { clkid_hclk, clkid_per, clkid_ic3, clkid_ic4 },
	[pctl_ipclk_xspi3sel] = { clkid_hclk, clkid_per, clkid_ic3, clkid_ic4 },
	[pctl_ipclk_otgphy1sel] = { clkid_hse, clkid_per, clkid_ic15, clkid_hse_div2_osc },
	[pctl_ipclk_otgphy1ckrefsel] = {},
	[pctl_ipclk_otgphy2sel] = { clkid_hse, clkid_per, clkid_ic15, clkid_hse_div2_osc },
	[pctl_ipclk_otgphy2ckrefsel] = {},
	[pctl_ipclk_persel] = { clkid_hsi, clkid_msi, clkid_hse, clkid_ic19, clkid_ic5, clkid_ic10, clkid_ic15, clkid_ic20 },
	[pctl_ipclk_pssisel] = { clkid_hclk, clkid_per, clkid_ic20, clkid_hsi_div },
	[pctl_ipclk_rtcsel] = { 0, clkid_lse, clkid_lsi, clkid_hse_rtc },
	[pctl_ipclk_sai1sel] = { clkid_pclk2, clkid_per, clkid_ic7, clkid_ic8, clkid_msi, clkid_hsi_div, clkid_i2s_in, clkid_spdif_symb },
	[pctl_ipclk_sai2sel] = { clkid_pclk2, clkid_per, clkid_ic7, clkid_ic8, clkid_msi, clkid_hsi_div, clkid_i2s_in, clkid_spdif_symb },
	[pctl_ipclk_sdmmc1sel] = { clkid_hclk, clkid_per, clkid_ic4, clkid_ic5 },
	[pctl_ipclk_sdmmc2sel] = { clkid_hclk, clkid_per, clkid_ic4, clkid_ic5 },
	[pctl_ipclk_spdifrx1sel] = { clkid_pclk1, clkid_per, clkid_ic7, clkid_ic8, clkid_msi, clkid_hsi_div, clkid_i2s_in },
	[pctl_ipclk_spi1sel] = { clkid_pclk2, clkid_per, clkid_ic8, clkid_ic9, clkid_msi, clkid_hsi_div, clkid_i2s_in },
	[pctl_ipclk_spi2sel] = { clkid_pclk1, clkid_per, clkid_ic8, clkid_ic9, clkid_msi, clkid_hsi_div, clkid_i2s_in },
	[pctl_ipclk_spi3sel] = { clkid_pclk1, clkid_per, clkid_ic8, clkid_ic9, clkid_msi, clkid_hsi_div, clkid_i2s_in },
	[pctl_ipclk_spi4sel] = { clkid_pclk2, clkid_per, clkid_ic9, clkid_ic14, clkid_msi, clkid_hsi_div, clkid_hse },
	[pctl_ipclk_spi5sel] = { clkid_pclk2, clkid_per, clkid_ic9, clkid_ic14, clkid_msi, clkid_hsi_div, clkid_hse },
	[pctl_ipclk_spi6sel] = { clkid_pclk4, clkid_per, clkid_ic8, clkid_ic9, clkid_msi, clkid_hsi_div, clkid_i2s_in },
	[pctl_ipclk_lptim1sel] = { clkid_pclk1, clkid_per, clkid_ic15, clkid_lse, clkid_lsi, clkid_timg },
	[pctl_ipclk_lptim2sel] = { clkid_pclk4, clkid_per, clkid_ic15, clkid_lse, clkid_lsi, clkid_timg },
	[pctl_ipclk_lptim3sel] = { clkid_pclk4, clkid_per, clkid_ic15, clkid_lse, clkid_lsi, clkid_timg },
	[pctl_ipclk_lptim4sel] = { clkid_pclk4, clkid_per, clkid_ic15, clkid_lse, clkid_lsi, clkid_timg },
	[pctl_ipclk_lptim5sel] = { clkid_pclk4, clkid_per, clkid_ic15, clkid_lse, clkid_lsi, clkid_timg },
	[pctl_ipclk_usart1sel] = { clkid_pclk2, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_usart2sel] = { clkid_pclk1, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_usart3sel] = { clkid_pclk1, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_uart4sel] = { clkid_pclk1, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_uart5sel] = { clkid_pclk1, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_usart6sel] = { clkid_pclk2, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_uart7sel] = { clkid_pclk1, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_uart8sel] = { clkid_pclk1, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_uart9sel] = { clkid_pclk2, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_usart10sel] = { clkid_pclk2, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
	[pctl_ipclk_lpuart1sel] = { clkid_pclk4, clkid_per, clkid_ic9, clkid_ic14, clkid_lse, clkid_msi, clkid_hsi_div },
};


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


const clockdef_obj_t *clockdef_getObject(clockdef_clkID_t id)
{
	return clockObjects + id;
}


const size_t clockdef_getSize(void)
{
	return sizeof(clockObjects) / sizeof(clockObjects[0]);
}


const enum clock_ids *rcc_getClkselOptions(enum ipclks ipclk)
{
	return rcc_clksels[ipclk];
}


int clockdef_getClockHW(
		clockdef_clkID_t id,
		uint32_t *base_out,
		uint64_t *nom,
		uint64_t *denom,
		clockdef_clkID_t *prev_out)
{
	(void)base_out;
	if (id >= clkid_pll1 && id <= clkid_pll4) {
		uint32_t v = *(rcc_common.base + rcc_pll1cfgr1 + 4 * (id - clkid_pll1));
		uint32_t src = (v >> 28) & 0x3;
		uint32_t bypass = (v >> 27) & 0x1;
		if (bypass == 0) {
			uint32_t div = (v >> 20) & 0x3f;
			uint32_t mul = (v >> 8) & 0xfff;
			v = *(rcc_common.base + rcc_pll1cfgr3 + 4 * (id - clkid_pll1));
			div *= (v >> 27) & 0x7;
			div *= (v >> 24) & 0x7;
			if ((v & 0xc) == 0xc) {
				/* Fractional divider active */
				uint32_t frac = *(rcc_common.base + rcc_pll1cfgr2 + 4 * (id - clkid_pll1));
				frac &= (1 << 24) - 1;
				*nom *= (((uint64_t)mul) << 24) + frac;
				*denom *= ((uint64_t)div) << 24;
			}
			else {
				*nom *= mul;
				*denom *= div;
			}
		}

		static const enum clock_ids src_to_id[4] = { clkid_hsi, clkid_msi, clkid_hse, clkid_i2s_in };
		*prev_out = src_to_id[src];
		return EOK;
	}

	if (id >= clkid_ic1 && id <= clkid_ic20) {
		uint32_t v = *(rcc_common.base + rcc_ic1cfgr + (id - clkid_ic1));
		uint32_t pll = (v >> 28) & 0x3;
		uint32_t divider = ((v >> 16) & 0xff) + 1;
		*denom *= divider;
		*prev_out = clkid_pll1 + pll;
		return EOK;
	}

	return -ENOENT;
}


void pwr_lockFromIRQ(uint32_t previous)
{
	/* Disable writing to backup domain if it was disabled previously */
	uint32_t tmp = *(rcc_common.pwr + pwr_dbpcr) & ~1u;
	*(rcc_common.pwr + pwr_dbpcr) = tmp | (previous & 1);
}


uint32_t pwr_unlockFromIRQ(void)
{
	uint32_t previous = *(rcc_common.pwr + pwr_dbpcr);
	/* Enable writing to backup domain */
	*(rcc_common.pwr + pwr_dbpcr) = previous | 1;
	return previous & 1;
}
