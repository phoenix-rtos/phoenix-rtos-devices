/*
 * Phoenix-RTOS
 *
 * MCX N94x common
 *
 * Copyright 2017, 2018, 2024, 2025 Phoenix Systems
 * Author: Aleksander Kaminski, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/platform.h>
#include <phoenix/arch/armv8m/mcx/n94x/mcxn94x.h>

#include "common.h"


int common_setClock(int dev, int sel, int div, int enable)
{
	int res;
	platformctl_t pctl;

	pctl.action = pctl_get;
	pctl.type = pctl_devclk;
	pctl.devClk.dev = dev;

	res = platformctl(&pctl);

	if (res != 0) {
		return res;
	}

	pctl.action = pctl_set;

	if (sel >= 0) {
		pctl.devClk.sel = (unsigned int)sel;
	}

	if (div >= 0) {
		pctl.devClk.div = (unsigned int)div;
	}

	if (enable >= 0) {
		pctl.devClk.enable = enable;
	}

	return platformctl(&pctl);
}
