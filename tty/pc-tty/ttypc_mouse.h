/*
 * Phoenix-RTOS
 *
 * PS/2 3-button mouse
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "ttypc_vt.h"

int ttypc_mouse_init(ttypc_t *ttypc);

void ttypc_mouse_handle_event(ttypc_t *ttypc);

void ttypc_mouse_destroy(ttypc_t *ttypc);
