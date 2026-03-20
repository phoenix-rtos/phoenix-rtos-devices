/*
 * Phoenix-RTOS
 *
 * stm32l4-multi
 *
 * Clock tree definition and parser header file
 *
 * Copyright 2025 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Jacek Maksymowicz, Mateusz Karcz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _CLOCKDEF_H_
#define _CLOCKDEF_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(__CPU_STM32N6)
#include "stm32n6.h"
#else
#error "Unsupported platform!"
#endif

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


extern const clockdef_obj_t *clockdef_getObject(clockdef_clkID_t id);


extern const size_t clockdef_getSize(void);


#endif /* _CLOCKDEF_H_ */
