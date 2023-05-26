/*
 * Phoenix-RTOS
 *
 * SPI Memory Controller driver
 *
 * Copyright 2023 Phoenix Systems
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
#include <phoenix/arch/gr716.h>


#define SPIMCTRL_NUM 2

#define FLASH0_AHB_ADDR 0x02000000
#define FLASH1_AHB_ADDR 0x04000000

#define SPIMCTRL0_BASE ((void *)0xFFF00100)
#define SPIMCTRL1_BASE ((void *)0xFFF00200)


/* clang-format off */
enum { spimctrl_instance0 = 0, spimctrl_instance1 };
/* clang-format on */

typedef struct _spimctrl_t {
	volatile uint32_t *base;
	addr_t ahbStartAddr;
	uint8_t instance;
	uint8_t ear;
} spimctrl_t;


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


static inline addr_t spimctrl_ahbAddr(int instance)
{
	switch (instance) {
		case spimctrl_instance0: return FLASH0_AHB_ADDR;
		case spimctrl_instance1: return FLASH1_AHB_ADDR;
		default: return 0;
	}
}


static inline void *spimctrl_getBase(int instance)
{
	switch (instance) {
		case spimctrl_instance0: return SPIMCTRL0_BASE;
		case spimctrl_instance1: return SPIMCTRL1_BASE;
		default: return NULL;
	}
}


/* Execute a transfer through spimctrl */
extern int spimctrl_xfer(spimctrl_t *spimctrl, struct xferOp *op);


/* Reset spimctrl core */
extern void spimctrl_reset(spimctrl_t *spimctrl);


/* Initialize spimctrl instance */
extern int spimctrl_init(spimctrl_t *spimctrl, int instance);


#endif
