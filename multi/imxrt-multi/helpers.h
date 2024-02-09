/*
 * Phoenix-RTOS
 *
 * i.MX RT helper macros
 *
 * Copyright 2024 Phoenix Systems
 * Author: Daniel Sawka
 *
 * %LICENSE%
 */

#ifndef _HELPERS_H_
#define _HELPERS_H_

#define CONCAT2(x, y)    x##y
#define CONCAT3(x, y, z) x##y##z

#define PIN2MUX(x) CONCAT2(pctl_mux_gpio_, x)
#define PIN2PAD(x) CONCAT2(pctl_pad_gpio_, x)

#endif
