/*
 * Phoenix-RTOS
 *
 * MCX N94x M33 Mailbox core driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Daniel Sawka
 *
 * %LICENSE%
 */

#ifndef MCXN94X_CPU_H_
#define MCXN94X_CPU_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>


ssize_t cpu_fifoWrite(const unsigned char *buf, size_t len, bool blocking, uint8_t ctl);


int cpu_waitForEvent(uint32_t timeoutUs);

#endif
