/*
 * Phoenix-RTOS
 *
 * STM32 Memory Cipher Engine (MCE) driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "mce.h"
#include <errno.h>
#include "../common.h"
#include "../stm32n6_base.h"

#define MCE_SR_ENCDIS (1UL << 4)

#define MCE_REGCR_BREN       (1UL << 0)
#define MCE_REGCR_CTXID_OFF  9UL
#define MCE_REGCR_CTXID_MASK (3UL << 9)
#define MCE_REGCR_ENC_OFF    14UL
#define MCE_REGCR_ENC_MASK   (3UL << 14)

#define MCE_MODE_STREAM 1UL
#define MCE_MODE_NBLOCK 2UL
#define MCE_MODE_FBLOCK 3UL
#define MCE_MODE_MASK   3UL

#define MCE_REGCR(reg_id) (mce_regcr1 + 4 * (reg_id))
#define MCE_SADDR(reg_id) (mce_saddr1 + 4 * (reg_id))
#define MCE_EADDR(reg_id) (mce_eaddr1 + 4 * (reg_id))

#define MCE1_REG_START 0x90000000
#define MCE1_REG_END   0x9fffffff
#define MCE2_REG_START 0x70000000
#define MCE2_REG_END   0x7fffffff
#define MCE3_REG_START 0x80000000
#define MCE3_REG_END   0x8fffffff
#define MCE4_REG_START 0x60000000
#define MCE4_REG_END   0x6fffffff

#define MCE_REG_BOUNDARY (4 * 1024)


typedef enum {
	mce1,
	mce2,
	mce3,
	mce4,
	mce_count,
} mce_t;

enum {
	mce_cr,
	mce_sr,
	mce_iasr,
	mce_iacr,
	mce_iaier,
	mce_iaddr = 0x9,
	mce_regcr1 = 0x10,
	mce_saddr1,
	mce_eaddr1,
	mce_regcr2 = 0x14,
	mce_saddr2,
	mce_eaddr2,
	mce_regcr3 = 0x18,
	mce_saddr3,
	mce_eaddr3,
	mce_regcr4 = 0x1c,
	mce_saddr4,
	mce_eaddr4,
	mce_mkeyr1 = 0x80,
	mce_mkeyr2,
	mce_mkeyr3,
	mce_mkeyr4,
	mce_mkeyr5,
	mce_mkeyr6,
	mce_mkeyr7,
	mce_mkeyr8,
	mce_fmkeyr1 = 0x88,
	mce_fmkeyr2,
	mce_fmkeyr3,
	mce_fmkeyr4,
	mce_fmkeyr5,
	mce_fmkeyr6,
	mce_fmkeyr7,
	mce_fmkeyr8,
	mce_cc1cfgr = 0x90,
	mce_cc1nr0,
	mce_cc1nr1,
	mce_cc1keyr0,
	mce_cc1keyr1,
	mce_cc1keyr2,
	mce_cc1keyr3,
	mce_cc2cfgr,
	mce_cc2nr0,
	mce_cc2nr1,
	mce_cc2keyr0,
	mce_cc2keyr1,
	mce_cc2keyr2,
	mce_cc2keyr3,
};


static const struct mce_setup {
	volatile uint32_t *base;
	struct {
		uint32_t base;
		uint32_t last;
	} region;
} mce_setup[mce_count] = {
	[mce1] = { .base = MCE1_BASE, .region = { MCE1_REG_START, MCE1_REG_END } },
	[mce2] = { .base = MCE2_BASE, .region = { MCE2_REG_START, MCE2_REG_END } },
	[mce3] = { .base = MCE3_BASE, .region = { MCE3_REG_START, MCE3_REG_END } },
	[mce4] = { .base = MCE4_BASE, .region = { MCE4_REG_START, MCE4_REG_END } },
};


static int mce_getMceForDevice(int dev)
{
	switch (dev) {
		case pctl_mce1: return mce1;
		case pctl_mce2: return mce2;
		case pctl_mce3: return mce3;
		case pctl_mce4: return mce4;
		default: return mce_count;
	}
}


static int mce_getDeviceState(int dev)
{
	int ret;
	platformctl_t pctl;

	pctl.action = pctl_get;
	pctl.type = pctl_devclk;
	pctl.devclk.dev = dev;
	ret = platformctl(&pctl);
	if (ret < 0) {
		return ret;
	}

	return pctl.devclk.state;
}


int mce_getRegionSetup(int mceDev, mce_reg_t region, mce_regionInfo_t *info)
{
	mce_t per = mce_getMceForDevice(mceDev);
	if ((per >= mce_count) || (region < 0) || (region >= mce_regcount)) {
		return -EINVAL;
	}

	int ret = mce_getDeviceState(mceDev);
	if (ret < 0) {
		return ret;
	}

	if (ret == 0) {
		return -ENODEV;
	}

	uint32_t sr = mce_setup[per].base[mce_sr];
	uint32_t regcr = mce_setup[per].base[MCE_REGCR(region)];
	info->enabled =
			((sr & MCE_SR_ENCDIS) == 0) &&
			((regcr & MCE_REGCR_BREN) != 0) &&
			(((regcr >> MCE_REGCR_ENC_OFF) & MCE_MODE_MASK) != 0);

	if (info->enabled) {
		info->start = mce_setup[per].base[MCE_SADDR(region)];
		info->end = mce_setup[per].base[MCE_EADDR(region)] + 1;
		switch ((regcr >> MCE_REGCR_ENC_OFF) & MCE_MODE_MASK) {
			case MCE_MODE_STREAM: info->granularity = 0; break;
			case MCE_MODE_NBLOCK: info->granularity = 16; break;
			case MCE_MODE_FBLOCK: info->granularity = 16; break;
			default: info->granularity = 0; break;
		}
	}
	else {
		info->start = 0;
		info->end = 0;
		info->granularity = 0;
	}

	return EOK;
}


int mce_setRegionEnabled(int mceDev, mce_reg_t region, bool enabled)
{
	mce_t per = mce_getMceForDevice(mceDev);
	if ((per >= mce_count) || (region < 0) || (region >= mce_regcount)) {
		return -EINVAL;
	}

	int ret = mce_getDeviceState(mceDev);
	if (ret < 0) {
		return ret;
	}

	if (ret == 0) {
		return -ENODEV;
	}

	if (enabled) {
		mce_setup[per].base[MCE_REGCR(region)] |= MCE_REGCR_BREN;
	}
	else {
		mce_setup[per].base[MCE_REGCR(region)] &= ~MCE_REGCR_BREN;
	}

	return EOK;
}
