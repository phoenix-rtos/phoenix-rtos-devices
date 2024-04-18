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

/*
These macros should only be used in `#if` and `#elif` directives, because undefined identifiers expand to 0 there.
Otherwise there will be "use of undefined identifier" errors (an exception: identifier is first checked for
existence with e.g. `#ifdef`).

Anything that expands to a numerical expression (or to an empty value) is fine. String literals don't work.
*/

/* True if X expands to a non-zero value. Also works with undefined and empty identifiers. */
#define ISTRUTHY(X) ((X + 0) != 0)

/*
These macros produce a logically correct result only if X is defined. You may use `#ifdef` or `defined()`
for this purpose. Unfortunately, `defined()` cannot be conveniently put inside a macro as this is undefined
behavior (see -Wexpansion-to-defined for details), so you have to use it directly on the spot, for example:
`#if defined(PERIPH1) && !ISEMPTY(PERIPH1)
// Use PERIPH1
#endif`
*/

/* True if X is empty (has no value). The result in #if is valid only if defined(X) is true */
#define ISEMPTY(X) ((0 - X - 1) == 1 && (X + 0) != -2)
/* True if X is either 0 or 1. The result in #if is valid only if defined(X) is true */
#define ISBOOLEAN(X) (!ISEMPTY(X) && ((X + 0) == 0 || (X + 0) == 1))

#endif
