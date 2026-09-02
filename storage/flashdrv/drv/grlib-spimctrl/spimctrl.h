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


typedef enum {
	MEXTENDED_SPI,
	MDUAL_OUTPUT,
	MDSPI,
	MQUAD_OUTPUT,
	MQSPI
}SPIMode_t;


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


/* Select SPI mode */
int spimctrl_spiMode(const struct spimctrl *spimctrl, SPIMode_t spi_mode);


/* Enable alternate scaler */
void spimctrl_enableAlternateScaler(volatile uint32_t *spimctrlBase);


/* Disable alternate scaler */
void spimctrl_disableAlternateScaler(volatile uint32_t *spimctrlBase);


/* Set dummy byte */
void spimctrl_setDummyByte(volatile uint32_t *spimctrlBase);


/* Set dummy cyclec*/
int spimctrl_setDummyCycles(volatile uint32_t *spimctrlBase, uint8_t numCycles);


#endif
