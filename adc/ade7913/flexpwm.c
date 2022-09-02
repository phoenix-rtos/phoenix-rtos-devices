/*
 * Phoenix-RTOS
 *
 * i.MX RT1170 FlexPWM input capture
 *
 * Copyright 2021-2022 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include "flexpwm.h"


static volatile struct flexpwm_regs_s *flexpwm_regs;


void flexpwm_init(void *base)
{
	flexpwm_regs = base;
}


int flexpwm_input_capture(unsigned int no, unsigned int cap0_edge, unsigned int cap1_edge)
{
	if (no > 3 || cap0_edge > 3 || cap1_edge > 3) {
		return -EINVAL;
	}

	/* use PWM_X as input capture */
	flexpwm_regs->outen &= ~(1 << no);

	/* enable free running input capture without edge counting */
	flexpwm_regs->sm[no].capctrlx = (cap1_edge << 4) | (cap0_edge << 2) | 1;

	/* enable DMA on PWM_X */
	flexpwm_regs->sm[no].dmaen = (1 << 6) | ((cap1_edge != 0) << 1) | (cap0_edge != 0);

	/* disable fault pin condition */
	flexpwm_regs->sm[no].dismap0 = 0;

	return EOK;
}
