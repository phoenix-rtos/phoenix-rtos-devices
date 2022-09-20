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
