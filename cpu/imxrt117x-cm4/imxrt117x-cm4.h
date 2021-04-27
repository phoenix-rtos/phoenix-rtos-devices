/*
 * Phoenix-RTOS
 *
 * i.MX RT117x M4 cpu core driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * %LICENSE%
 */

#ifndef IMXRT117X_HM4_H
#define IMXRT117X_HM4_H

enum { m4_loadFile = 0, m4_loadBuff, m4_runCore };

typedef struct {
	int type;
} imxrt117xM4DevCtli_t;

typedef struct {
	int err;
} imxrt117xM4DevCtlo_t;

#endif
