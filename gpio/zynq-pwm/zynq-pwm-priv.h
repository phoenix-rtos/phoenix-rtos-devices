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

#ifndef ZYNQ_PWM_PRIV_H
#define ZYNQ_PWM_PRIV_H

#include <stdint.h>
#include <sys/msg.h>


typedef struct {
	enum { zynqpwm_msgSet,
		zynqpwm_msgGet } type;
	uint32_t compval[8];
	uint8_t mask;
} zynqpwm_imsg_t;


typedef struct {
	int err;
	uint32_t compval[8];
} zynqpwm_omsg_t;


#endif
