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

#include <errno.h>
#include <sys/mman.h>
#include <sys/platform.h>

#include <board_config.h>

#include "spimctrl.h"

/* Control register */

#define USR_CTRL (1 << 0)
#define CHIP_SEL (1 << 3)
#define CORE_RST (1 << 4)

/* Status register */

#define OPER_DONE   (1 << 0)
#define CORE_BUSY   (1 << 1)
#define INITIALIZED (1 << 2)


enum {
	flash_cfg,   /* Flash configuration : 0x00 */
	flash_ctrl,  /* Flash control       : 0x04 */
	flash_stat,  /* Flash status        : 0x08 */
	flash_rx,    /* Flash receive       : 0x0C */
	flash_tx,    /* Flash transmit      : 0x10 */
	flash_econf, /* EDAC configuration  : 0x14 */
	flash_estat  /* EDAC status         : 0x18 */
};


static void spimctrl_userCtrl(volatile uint32_t *spimctrlBase)
{
	*(spimctrlBase + flash_ctrl) = USR_CTRL;
	*(spimctrlBase + flash_ctrl) &= ~CHIP_SEL;
}


static int spimctrl_busy(const struct spimctrl *spimctrl)
{
	return (*(spimctrl->base + flash_stat) & CORE_BUSY) >> 1;
}


static int spimctrl_ready(const struct spimctrl *spimctrl)
{
	uint32_t val = (*(spimctrl->base + flash_stat) & (INITIALIZED | OPER_DONE));

	return (val == INITIALIZED) ? 1 : 0;
}


static void spimctrl_tx(volatile uint32_t *spimctrlBase, uint8_t cmd)
{
	*(spimctrlBase + flash_tx) = cmd;
	while ((*(spimctrlBase + flash_stat) & OPER_DONE) == 0) { }
	*(spimctrlBase + flash_stat) |= OPER_DONE;
}


static uint8_t spimctrl_rx(volatile uint32_t *spimctrlBase)
{
	return *(spimctrlBase + flash_rx) & 0xff;
}


static void spimctrl_read(const struct spimctrl *spimctrl, struct xferOp *op)
{
	spimctrl_userCtrl(spimctrl->base);

	/* send command */
	for (size_t i = 0; i < op->cmdLen; i++) {
		spimctrl_tx(spimctrl->base, op->cmd[i]);
	}

	/* read data */
	for (size_t i = 0; i < op->dataLen; i++) {
		spimctrl_tx(spimctrl->base, 0x00u);
		op->rxData[i] = spimctrl_rx(spimctrl->base);
	}

	*(spimctrl->base + flash_ctrl) &= ~USR_CTRL;
}


static void spimctrl_write(const struct spimctrl *spimctrl, struct xferOp *op)
{
	spimctrl_userCtrl(spimctrl->base);

	/* Send command */
	for (size_t i = 0; i < op->cmdLen; i++) {
		spimctrl_tx(spimctrl->base, op->cmd[i]);
	}

	/* Send data */
	for (size_t i = 0; i < op->dataLen; i++) {
		spimctrl_tx(spimctrl->base, op->txData[i]);
	}

	*(spimctrl->base + flash_ctrl) &= ~USR_CTRL;
}


int spimctrl_xfer(const struct spimctrl *spimctrl, struct xferOp *op)
{
	if ((spimctrl_busy(spimctrl) == 1) || spimctrl_ready(spimctrl) == 0) {
		return -EBUSY;
	}

	switch (op->type) {
		case xfer_opRead:
			spimctrl_read(spimctrl, op);
			break;
		case xfer_opWrite:
			spimctrl_write(spimctrl, op);
			break;
		default:
			return -EINVAL;
	}
	return 0;
}


void spimctrl_reset(const struct spimctrl *spimctrl)
{
	*(spimctrl->base + flash_ctrl) = CORE_RST;
}


int spimctrl_init(struct spimctrl *spimctrl, addr_t mctrlBase)
{
	spimctrl->base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, mctrlBase);
	if (spimctrl->base == MAP_FAILED) {
		return -ENOMEM;
	}

	/* Reset core */
	spimctrl_reset(spimctrl);

	return 0;
}


void spimctrl_destroy(struct spimctrl *spimctrl)
{
	(void)munmap((void *)spimctrl->base, _PAGE_SIZE);
}
