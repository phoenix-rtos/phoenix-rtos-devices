/*
 * Phoenix-RTOS
 *
 * stm32l4-multi
 *
 * Clock tree definition and parser
 *
 * Copyright 2025 Phoenix Systems
 * Copyright 2026 Apator Metrix
 * Author: Jacek Maksymowicz, Mateusz Karcz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <sys/platform.h>

#include "../rcc.h"


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
	size_t size = clockdef_getSize();

	do {
		if (id >= size) {
			ret = clockdef_getClockHW(id, &base, &total_nom, &total_denom, &prev);
			if (ret < 0) {
				return ret;
			}
		}
		else {
			uint32_t nom, denom;
			const clockdef_obj_t *tf = clockdef_getObject(id);
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


uint32_t clockdef_getRegHW(uint32_t offset)
{
	return *(rcc_common.base + offset);
}


int rcc_setClksel(enum ipclks ipclk, enum clock_ids clkID)
{
	if ((ipclk < 0) || (ipclk > pctl_ipclks_count) || (clkID == clkid_none)) {
		return -EINVAL;
	}

	size_t setting;
	const enum clock_ids *options = rcc_getClkselOptions(ipclk);
	for (setting = 0; setting < RCC_MAX_CLOCK_CHOICES; setting++) {
		enum clock_ids settingID = options[setting];
		if (settingID == clkID) {
			break;
		}
	}

	if (setting >= RCC_MAX_CLOCK_CHOICES) {
		return -EINVAL;
	}

	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_ipclk,
		.ipclk = {
			.ipclk = ipclk,
			.setting = setting,
		}
	};

	return platformctl(&pctl);
}
