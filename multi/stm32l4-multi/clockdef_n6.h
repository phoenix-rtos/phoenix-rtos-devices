/*
 * Phoenix-RTOS
 *
 * stm32l4-multi
 *
 * Clock tree definition and parser header file
 *
 * Copyright 2025 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _CLOCKDEF_N6_H_
#define _CLOCKDEF_N6_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum clock_ids {
	clkid_none = 0,
	clkid_lsi,
	clkid_lse,
	clkid_msi,
	clkid_hsi,
	clkid_hsi_div,
	clkid_hse,
	clkid_hse_div2_osc,
	clkid_hse_rtc,
	clkid_sysa,
	clkid_sysb,
	clkid_sysc,
	clkid_sysd,
	clkid_timg,
	clkid_hclk,
	clkid_pclk1,
	clkid_pclk2,
	clkid_pclk4,
	clkid_pclk5,
	clkid_per,
	clkid_i2s_in,
	clkid_jtag_tck,
	clkid_spdif_symb,
	clkid_pll1,
	clkid_pll2,
	clkid_pll3,
	clkid_pll4,
	clkid_ic1,
	clkid_ic2,
	clkid_ic3,
	clkid_ic4,
	clkid_ic5,
	clkid_ic6,
	clkid_ic7,
	clkid_ic8,
	clkid_ic9,
	clkid_ic10,
	clkid_ic11,
	clkid_ic12,
	clkid_ic13,
	clkid_ic14,
	clkid_ic15,
	clkid_ic16,
	clkid_ic17,
	clkid_ic18,
	clkid_ic19,
	clkid_ic20,
	clkids_count,
};

enum clockdef_type {
	clockdef_type_literal,
	clockdef_type_clkID,
	clockdef_type_register,
	clockdef_type_register_log,
	clockdef_type_register_lookup,
	clockdef_type_register_mux,
};

typedef uint32_t clockdef_literal_t;

typedef enum clock_ids clockdef_clkID_t;

typedef struct {
	uint32_t offset;    /* Offset of register within the peripheral */
	uint32_t mask;      /* Value mask */
	uint8_t shift;      /* Value bit position */
	int16_t val_offset; /* Offset to add to value after masking */
} clockdef_register_t;

typedef clockdef_register_t clockdef_register_log_t;

typedef struct {
	clockdef_register_t r;
	const clockdef_literal_t *vals;
} clockdef_register_lookup_t;

typedef struct {
	clockdef_register_t r;
	const clockdef_clkID_t *vals;
} clockdef_register_mux_t;

typedef struct {
	enum clockdef_type type;
	union {
		clockdef_literal_t literal;
		clockdef_clkID_t clkID;
		clockdef_register_lookup_t reg_lookup;
		clockdef_register_mux_t reg_mux;
	};
} clockdef_base_t;

typedef struct {
	enum clockdef_type type;
	union {
		clockdef_literal_t literal;
		clockdef_register_t reg;
		clockdef_register_log_t reg_log;
		clockdef_register_lookup_t reg_lookup;
	};
} clockdef_number_t;

typedef struct {
	clockdef_base_t base;
	clockdef_number_t nom;
	clockdef_number_t denom;
} clockdef_obj_t;


extern uint32_t clockdef_getRegHW(uint32_t offset);


extern int clockdef_getClockHW(
		clockdef_clkID_t id,
		uint32_t *base_out,
		uint64_t *nom,
		uint64_t *denom,
		clockdef_clkID_t *prev);


extern int clockdef_getClock(clockdef_clkID_t id, uint64_t *output);

#endif /* _CLOCKDEF_N6_H_ */
