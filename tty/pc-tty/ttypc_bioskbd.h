/*
 * Phoenix-RTOS
 *
 * 101-key US BIOS keyboard
 *
 * Copyright 2020 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _TTYPC_BIOSKBD_H_
#define _TTYPC_BIOSKBD_H_

#include "ttypc.h"


/* Updates keyboard LEDs */
extern int _ttypc_bioskbd_updateled(ttypc_t *ttypc);


/* Destroys keyboard */
extern void ttypc_bioskbd_destroy(ttypc_t *ttypc);


/* Initializes BIOS keyboard */
extern int ttypc_bioskbd_init(ttypc_t *ttypc);


#endif
