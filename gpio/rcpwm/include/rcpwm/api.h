/*
 * Phoenix-RTOS
 *
 * Remote control PWM driver binary interface
 *
 * Copyright 2023 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef RCPWM_MSG_H
#define RCPWM_MSG_H

#include <stdint.h>
#include <sys/msg.h>


#define RCPWM_CHANNELS 8


int rcpwm_set(oid_t *oid, uint32_t compval[RCPWM_CHANNELS], uint8_t mask);


int rcpwm_get(oid_t *oid, uint32_t compval[RCPWM_CHANNELS]);


#endif
