/*
 * Phoenix-RTOS
 *
 * i.MX RT1170 FlexPWM input capture
 *
 * Copyright 2021-2023 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include "flexpwm.h"


struct flexpwm_submodule_regs_s {
	uint16_t cnt;
	uint16_t init;
	uint16_t ctrl2;
	uint16_t ctrl;
	uint16_t reserved0;
	uint16_t val0;
	uint16_t fracval1;
	uint16_t val1;
	uint16_t fracval2;
	uint16_t val2;
	uint16_t fracval3;
	uint16_t val3;
	uint16_t fracval4;
	uint16_t val4;
	uint16_t fracval5;
	uint16_t val5;
	uint16_t frctrl;
	uint16_t octrl;
	uint16_t sts;
	uint16_t inten;
	uint16_t dmaen;
	uint16_t tctrl;
	uint16_t dismap0;
	uint16_t reserved1;
	uint16_t dtcnt0;
	uint16_t dtcnt1;
	uint16_t captctrla;
	uint16_t captcompa;
	uint16_t captctrlb;
	uint16_t captcompb;
	uint16_t captctrlx;
	uint16_t captcompx;
	uint16_t cval0;
	uint16_t cval0cyc;
	uint16_t cval1;
	uint16_t cval1cyc;
	uint16_t cval2;
	uint16_t cval2cyc;
	uint16_t cval3;
	uint16_t cval3cyc;
	uint16_t cval4;
	uint16_t cval4cyc;
	uint16_t cval5;
	uint16_t cval5cyc;
	uint16_t reserved2[4];
};


struct flexpwm_regs_s {
	struct flexpwm_submodule_regs_s sm[4];
	uint16_t outen;
	uint16_t mask;
	uint16_t swcout;
	uint16_t dtsrcsel;
	uint16_t mctrl;
	uint16_t mctrl2;
	uint16_t fctrl0;
	uint16_t fsts0;
	uint16_t ffilt0;
	uint16_t ftst0;
	uint16_t fctrl2;
};


static volatile struct flexpwm_regs_s *flexpwm_regs;


int flexpwm_init(unsigned int dev)
{
	switch (dev) {
		case FLEXPWM_PWM1: flexpwm_regs = ((void *)0x4018C000); break;
		case FLEXPWM_PWM2: flexpwm_regs = ((void *)0x40190000); break;
		case FLEXPWM_PWM3: flexpwm_regs = ((void *)0x40194000); break;
		case FLEXPWM_PWM4: flexpwm_regs = ((void *)0x40198000); break;
		default: flexpwm_regs = NULL; return -EINVAL;
	}

	return 0;
}


int flexpwm_input_capture(unsigned int sm, unsigned int pwm, unsigned int cap0_edge, unsigned int cap1_edge)
{
	volatile uint16_t *captctrl;

	if ((flexpwm_regs == NULL) || (sm > 3) || (cap0_edge > FLEXPWM_CAP_EDGE_ANY) || (cap1_edge > FLEXPWM_CAP_EDGE_ANY)) {
		return -EINVAL;
	}

	switch (pwm) {
		case FLEXPWM_PWMX: captctrl = &flexpwm_regs->sm[sm].captctrlx; break;
		case FLEXPWM_PWMB: captctrl = &flexpwm_regs->sm[sm].captctrlb; break;
		case FLEXPWM_PWMA: captctrl = &flexpwm_regs->sm[sm].captctrla; break;
		default: return -EINVAL;
	}

	/* use PWM as input capture */
	flexpwm_regs->outen &= ~(1 << (sm + 4 * pwm));

	/* enable free running input capture without edge counting */
	*captctrl = (cap1_edge << 4) | (cap0_edge << 2) | 1;

	/* enable DMA on selected PWM */
	flexpwm_regs->sm[sm].dmaen = (1 << 6) |
		((cap1_edge != FLEXPWM_CAP_DISABLED) << (1 + 2 * pwm)) |
		((cap0_edge != FLEXPWM_CAP_DISABLED) << (0 + 2 * pwm));

	/* disable fault pin condition */
	flexpwm_regs->sm[sm].dismap0 = 0;

	return 0;
}
