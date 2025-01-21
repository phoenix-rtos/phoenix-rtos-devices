/*
 * Phoenix-RTOS
 *
 * GRLIB SPIMCTRL driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _SPIMCTRL_H_
#define _SPIMCTRL_H_

#include <stdint.h>
#include <sys/types.h>


struct spimctrl {
	volatile uint32_t *base;

	uint8_t ear;
};


struct xferOp {
	/* clang-format off */
	enum { xfer_opRead = 0, xfer_opWrite } type;
	/* clang-format on */
	const uint8_t *cmd;
	size_t cmdLen;
	union {
		const uint8_t *txData;
		uint8_t *rxData;
	};
	size_t dataLen;
};


/* Execute a transfer through spimctrl */
int spimctrl_xfer(const struct spimctrl *spimctrl, struct xferOp *op);


/* Reset spimctrl core */
void spimctrl_reset(const struct spimctrl *spimctrl);


/* Initialize spimctrl instance */
int spimctrl_init(struct spimctrl *spimctrl, addr_t mctrlBase);


void spimctrl_destroy(struct spimctrl *spimctrl);


#endif
