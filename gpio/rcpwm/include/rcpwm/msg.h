/*
 * Phoenix-RTOS
 *
 * Xilinx Zynq7000 / ZynqMP PWM driver binary interface
 *
 * Copyright 2023 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef RCPWM_PRIV_H
#define RCPWM_PRIV_H

#include <stdint.h>
#include <sys/msg.h>


typedef struct {
	enum { rcpwm_msgSet,
		rcpwm_msgGet } type;
	uint32_t compval[8];
	uint8_t mask;
} rcpwm_imsg_t;


typedef struct {
	int err;
	uint32_t compval[8];
} rcpwm_omsg_t;


#endif
