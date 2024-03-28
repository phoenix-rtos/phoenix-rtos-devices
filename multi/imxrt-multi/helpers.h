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

/* For the following macros, anything that expands to a numerical expression (or to an empty value) is fine.
String literals don't work.

/* True if X is defined, is not empty, and has a non-zero value. */
#define ISTRUTHY(X) ((X + 0) != 0)

/* True if X is empty (has no value). The result in #if is valid only if defined(X) is true */
#define ISEMPTY(X) ((0 - X - 1) == 1 && (X + 0) != -2)
/* True if X is either 0 or 1. The result in #if is valid only if defined(X) is true */
#define ISBOOLEAN(X) (!ISEMPTY(X) && ((X + 0) == 0 || (X + 0) == 1))

#endif
