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
#include <sys/time.h>
#include <unistd.h>
#include <board_config.h>

#include "spimctrl.h"
#include "../commands/flash_cmds.h"
#include "../sfdp-flash/MT25Q_cmds.h"

#include <stdio.h> /* rmv later */


/* Configuration register */

#define DCYCLES (0xFUL << 8)
#define READ_CMD_MASK (0xFFu)
#define WRITE_CMD_MASK   (0xFFu << 24)
#define DSPI (1 << 12)
#define QSPI (1 << 13)
#define EXTENDED_ADDRESS (1 << 14)
#define DBYTE (1 << 15)
#define DOUT (1 << 16)
#define QOUT (1 << 17)
#define DIN (1 << 18)
#define QIN (1 << 19)
#define XIP (1 << 20)

/* Control register */

#define USR_CTRL (1 << 0)
#define EAS (1 << 2)
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
	uint32_t ctrl = *(spimctrlBase + flash_ctrl);
    ctrl |= USR_CTRL;
    ctrl &= ~CHIP_SEL;
    *(spimctrlBase + flash_ctrl) = ctrl;
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


static void spimctrl_addressMode(struct spimctrl *spimctrl)
{
    uint32_t cfg = *(spimctrl->base + flash_cfg);

    #if USE_4BYTE_MODE
		uint8_t readcmd = FLASH_CMD_4B_READ;
        spimctrl->extendedAddress = 1;
		cfg &= ~READ_CMD_MASK;
		cfg |= readcmd;
        cfg |= EXTENDED_ADDRESS;
    #else
		uint8_t readcmd = FLASH_CMD_READ;
		cfg &= ~READ_CMD_MASK;
		cfg |= readcmd;
        spimctrl->extendedAddress = 0;
        cfg &= ~EXTENDED_ADDRESS;
    #endif /* USE_4BYTE_MODE */

    *(spimctrl->base + flash_cfg) = cfg;
}


static uint32_t spimctrl_updateReadWriteCmds(struct spimctrl *spimctrl, uint32_t cfg, uint8_t readCmd, uint8_t writeCmd)
{
	uint8_t readcmd = readCmd;
	cfg &= ~READ_CMD_MASK;
	cfg |= readcmd;

	uint8_t writecmd = writeCmd;
	cfg &= ~WRITE_CMD_MASK;
	cfg |= ((uint32_t)writecmd << 24);

    return cfg;
}


// int spimctrl_setDummyCycles(volatile uint32_t *spimctrlBase, uint8_t numCycles)
// {
// 	int res = 0;
// 	if (numCycles < 0xFu) {
// 		uint32_t cfg = *(spimctrlBase + flash_cfg);
// 		cfg &= ~(DCYCLES | DBYTE); 
// 		cfg |= ((numCycles & 0xFUL) << 8); 
// 		*(spimctrlBase + flash_cfg) = cfg;  
// 	}
// 	else {
// 		res = -EINVAL;
// 	}

// 	return res;
// }
static uint32_t spimctrl_setDummyCycles(struct spimctrl *spimctrl, uint32_t cfg, uint8_t numCycles)
{
	if (numCycles < 0xFu) {
		cfg &= ~(DCYCLES | DBYTE); 
		cfg |= ((numCycles & 0xFUL) << 8);
	}
	else {
		cfg &= ~(DCYCLES | DBYTE); 
		cfg |= ((0 & 0xFUL) << 8);
	}

	return cfg;
}


void spimctrl_oneSPI(struct spimctrl *spimctrl)
{
    uint32_t cfg = *(spimctrl->base + flash_cfg);
    cfg &= ~(DSPI | DOUT | QOUT | QSPI | DIN | QIN);

	uint8_t readCmd;
	uint8_t writeCmd;

	if (!(spimctrl->extendedAddress)) {
		readCmd = FLASH_CMD_READ;
		writeCmd = FLASH_CMD_PP;
	}
	else {
		readCmd = FLASH_CMD_4B_READ;
		writeCmd = FLASH_CMD_4B_PP;
	}

	cfg = spimctrl_updateReadWriteCmds(spimctrl, cfg, readCmd, writeCmd);
	cfg = spimctrl_setDummyCycles(spimctrl, cfg, 0);
	*(spimctrl->base + flash_cfg) = cfg;
}


void spimctrl_doutSPI(struct spimctrl *spimctrl)
{
    uint32_t cfg = *(spimctrl->base + flash_cfg);
    cfg &= ~(DSPI | DOUT | QOUT | QSPI | DIN | QIN);
	cfg |= DOUT;

	uint8_t readCmd;
	uint8_t writeCmd;

	if (!(spimctrl->extendedAddress)) {
		readCmd = FLASH_CMD_DOUTPUT_FASTREAD;
		writeCmd = FLASH_CMD_DIN_FP;
	}
	else {
		readCmd = FLASH_CMD_4B_DOUTPUT_FASTREAD;
		writeCmd = FLASH_CMD_QIN_FP;
	}

	cfg = spimctrl_updateReadWriteCmds(spimctrl, cfg, readCmd, writeCmd);
	cfg = spimctrl_setDummyCycles(spimctrl, cfg, 8);
    *(spimctrl->base + flash_cfg) = cfg;
}


void spimctrl_dSPI(struct spimctrl *spimctrl)
{
    uint32_t cfg = *(spimctrl->base + flash_cfg);
    cfg &= ~(DSPI | DOUT | QOUT | QSPI | DIN | QIN);
	cfg |= DSPI;

	uint8_t readCmd;
	uint8_t writeCmd;

	if (!(spimctrl->extendedAddress)) {
		readCmd = FLASH_CMD_DIO_FASTREAD;
		writeCmd = FLASH_CMD_DIN_FP;
	}
	else {
		readCmd = FLASH_CMD_4B_DIO_FASTREAD;
		writeCmd = FLASH_CMD_4B_PP;
	}

	cfg = spimctrl_updateReadWriteCmds(spimctrl, cfg, readCmd, writeCmd);
	cfg = spimctrl_setDummyCycles(spimctrl, cfg, 8);
    *(spimctrl->base + flash_cfg) = cfg;
}


void spimctrl_qoutSPI(struct spimctrl *spimctrl)
{
    uint32_t cfg = *(spimctrl->base + flash_cfg);
    cfg &= ~(DSPI | DOUT | QOUT | QSPI | DIN | QIN);
	cfg |= QOUT;

	uint8_t readCmd;
	uint8_t writeCmd;

	if (!(spimctrl->extendedAddress)) {
		readCmd = FLASH_CMD_QOUTPUT_FASTREAD;
		writeCmd = FLASH_CMD_QIN_FP;
	}
	else {
		readCmd = FLASH_CMD_4B_QOUTPUT_FASTREAD;
		writeCmd = FLASH_CMD_4B_QIN_FP;
	}

	cfg = spimctrl_updateReadWriteCmds(spimctrl, cfg, readCmd, writeCmd);
	cfg = spimctrl_setDummyCycles(spimctrl, cfg, 8);
    *(spimctrl->base + flash_cfg) = cfg;	
}


void spimctrl_qSPIx(struct spimctrl *spimctrl)
{
    uint32_t cfg = *(spimctrl->base + flash_cfg);
    cfg &= ~(DSPI | DOUT | QOUT | QSPI | DIN | QIN);
	cfg |= QSPI;

	// uint8_t readcmd = 0xECu;
	// cfg &= ~READ_CMD_MASK;
	// cfg |= readcmd;

	cfg = spimctrl_setDummyCycles(spimctrl, cfg, 10);
    *(spimctrl->base + flash_cfg) = cfg;
}


void spimctrl_qSPI(struct spimctrl *spimctrl)
{
    uint32_t cfg = *(spimctrl->base + flash_cfg);
    cfg &= ~(DSPI | DOUT | QOUT | QSPI | DIN | QIN);
	cfg |= QSPI;

	uint8_t readCmd;
	uint8_t writeCmd;

	if (!(spimctrl->extendedAddress)) {
		readCmd = FLASH_CMD_QIO_FASTREAD;
		writeCmd = FLASH_CMD_EXTENDED_QIN_FP;
	}
	else {
		readCmd = FLASH_CMD_4B_QIO_FASTREAD;
		writeCmd = FLASH_CMD_4B_EXTENDED_QIN_FP;
	}

	cfg = spimctrl_updateReadWriteCmds(spimctrl, cfg, readCmd, writeCmd);
	cfg = spimctrl_setDummyCycles(spimctrl, cfg, 10);
    *(spimctrl->base + flash_cfg) = cfg;
}


static void spimctrl_alternateScaler(volatile uint32_t *spimctrlBase)
{
	#if ENABLE_ALTERNATE_SCALER
		*(spimctrlBase + flash_ctrl) |= EAS;
	#else
		*(spimctrlBase + flash_ctrl) &= ~EAS;
	#endif /* ENABLE_ALTERNATE_SCALER */
}


void spimctrl_setDummyByte(volatile uint32_t *spimctrlBase)
{
	*(spimctrlBase + flash_cfg) |= DBYTE;
}


static void spimctrl_tx(volatile uint32_t *spimctrlBase, uint8_t cmd)
{
	*(spimctrlBase + flash_tx) = cmd;
	while ((*(spimctrlBase + flash_stat) & OPER_DONE) == 0) { }
	*(spimctrlBase + flash_stat) |= OPER_DONE;
}


// static uint8_t spimctrl_rx(volatile uint32_t *spimctrlBase)
// {
//     return *(spimctrlBase + flash_rx) & 0xff;
// }


static uint8_t spimctrl_rx(volatile uint32_t *spimctrlBase)
{
    while ((*(spimctrlBase + flash_stat) & CORE_BUSY) != 0) { }
    uint32_t val = *(spimctrlBase + flash_rx) & 0xff;
    *(spimctrlBase + flash_stat) |= OPER_DONE;
    return val;
}


static void spimctrl_read(const struct spimctrl *spimctrl, struct xferOp *op)
{
	uint32_t cfg = *(spimctrl->base + flash_cfg);
    spimctrl_userCtrl(spimctrl->base);

    /* send command */
    for (size_t i = 0; i < op->cmdLen; i++) {
        spimctrl_tx(spimctrl->base, op->cmd[i]);
    }

    /* read data */
    for (size_t i = 0; i < op->dataLen; i++) {
		while ((*(spimctrl->base + flash_stat) & CORE_BUSY) != 0) { }
		if(!(cfg & (QSPI | DSPI | QOUT | DOUT)))
        	spimctrl_tx(spimctrl->base, 0x00u);
		else
			*(spimctrl->base + flash_rx) = 0x00u;
        
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

	/* Set address mode */
	spimctrl_addressMode(spimctrl);

	/* Set alternate scaler policy */
	spimctrl_alternateScaler(spimctrl->base);

	return 0;
}


void spimctrl_destroy(struct spimctrl *spimctrl)
{
	(void)munmap((void *)spimctrl->base, _PAGE_SIZE);
}
