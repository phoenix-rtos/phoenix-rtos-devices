/*
 * Phoenix-RTOS
 *
 * i.MX FlexPWM
 *
 * Copyright 2021-2022 Phoenix Systems
 * Author: Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef FLEXPWM_H
#define FLEXPWM_H

#include <stdint.h>


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
	uint16_t capctrla;
	uint16_t captcompa;
	uint16_t captctrlb;
	uint16_t captcompb;
	uint16_t capctrlx;
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


enum cap_edge_e {
	cap_disabled = 0,
	cap_edge_falling,
	cap_edge_rising,
	cap_edge_any
};


/* Initialize base address */
void flexpwm_init(void *base);


/* Configure input capture mode */
int flexpwm_input_capture(unsigned int no, unsigned int cap0_edge, unsigned int cap1_edge);


#endif /* end of FLEXPWM_H */
