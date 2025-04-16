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

#ifndef ZYNQ_PWM_MSG_H
#define ZYNQ_PWM_MSG_H

#include <stdint.h>
#include <sys/msg.h>


#define ZYNQ_PWM_CHANNELS 8


int zynqpwm_set(oid_t *oid, uint32_t compval[ZYNQ_PWM_CHANNELS], uint8_t mask);


int zynqpwm_get(oid_t *oid, uint32_t compval[ZYNQ_PWM_CHANNELS]);


#endif
