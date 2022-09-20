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


#define FLEXPWM_CAP_DISABLED     0
#define FLEXPWM_CAP_EDGE_FALLING 1
#define FLEXPWM_CAP_EDGE_RISING  2
#define FLEXPWM_CAP_EDGE_ANY     3


/* Initialize base address */
void flexpwm_init(void *base);


/* Configure input capture mode */
int flexpwm_input_capture(unsigned int no, unsigned int cap0_edge, unsigned int cap1_edge);


#endif /* end of FLEXPWM_H */
