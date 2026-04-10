/*
 * Phoenix-RTOS
 *
 * GRLIB GPIO message interface
 *
 * Copyright 2026 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef GRGPIO_MSG_H_
#define GRGPIO_MSG_H_

#include <stdint.h>

#include <sys/msg.h>
#include <sys/types.h>


/* Returns GPIO port value */
int gpiomsg_readPort(const oid_t *oid, uint32_t *val);


/* Sets GPIO port value with mask */
int gpiomsg_writePort(const oid_t *oid, uint32_t val, uint32_t mask);


/* Returns GPIO port direction (0 - input, 1 - output) */
int gpiomsg_readDir(const oid_t *oid, uint32_t *val);


/* Sets GPIO port direction with mask (0 - input, 1 - output) */
int gpiomsg_writeDir(const oid_t *oid, uint32_t val, uint32_t mask);


/* Returns GPIO pin state (0 - low, 1 - high) */
int gpiomsg_readPin(const oid_t *oid, uint32_t pin, uint32_t *val);


/* Sets GPIO pin state (0 - low, 1 - high) */
int gpiomsg_writePin(const oid_t *oid, uint32_t pin, uint32_t val);


/* Returns GPIO pin direction (0 - input, 1 - output) */
int gpiomsg_readPinDir(const oid_t *oid, uint32_t pin, uint32_t *val);


/* Sets GPIO pin direction (0 - input, 1 - output) */
int gpiomsg_writePinDir(const oid_t *oid, uint32_t pin, uint32_t val);


int gpiomsg_open(uint32_t port, oid_t *oid);


#endif
