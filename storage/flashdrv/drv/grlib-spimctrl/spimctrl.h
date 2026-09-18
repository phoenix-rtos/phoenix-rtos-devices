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

#define FLASH0_AHB_ADDR 0xC0000000

#ifndef SPIMCTRL0_BASE
#define SPIMCTRL0_BASE ((void *)0xFFF00000)
#endif


struct spimctrl {
	volatile uint32_t *base;

	uint8_t ear; /* extended address register (3-byte mode) */
	uint8_t extendedAddress; /* 4 byte mode  */
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


/* Destroy spimctrl instance */
void spimctrl_destroy(struct spimctrl *spimctrl);


/* Enter default SPI mode (1-1-1) */
void spimctrl_oneSPI(struct spimctrl *spimctrl);


/* Enter dual output SPI mode (1-2-2) */
void spimctrl_doutSPI(struct spimctrl *spimctrl);


/* Enter dual SPI mode (2-2-2) */
void spimctrl_dSPI(struct spimctrl *spimctrl);


/* Enter quad output SPI mode (1-4-4) */
void spimctrl_qoutSPI(struct spimctrl *spimctrl);


/* Enter quad SPI mode (4-4-4) */
void spimctrl_qSPI(struct spimctrl *spimctrl);


/* Set dummy byte */
void spimctrl_setDummyByte(volatile uint32_t *spimctrlBase);


#endif
