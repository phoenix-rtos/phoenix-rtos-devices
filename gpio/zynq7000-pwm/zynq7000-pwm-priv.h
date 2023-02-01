/*
 * Phoenix-RTOS
 *
 * Xilinx Zynq 7000 PWM driver binary interface
 *
 * Copyright 2023 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef ZYNQ7000_PWM_PRIV_H
#define ZYNQ7000_PWM_PRIV_H

#include <stdint.h>
#include <sys/msg.h>


typedef struct {
	enum { zynq7000pwm_msgSet, zynq7000pwm_msgGet } type;
	uint32_t compval[8];
	uint8_t mask;
} zynq7000pwm_imsg_t;


typedef struct {
	int err;
	uint32_t compval[8];
} zynq7000pwm_omsg_t;


#endif
