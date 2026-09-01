/*
 * Phoenix-RTOS
 *
 * TDA4VM MCU SPI driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Rafal Mikielis
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _MCSPI_TDA4VM_REG_
#define _MCSPI_TDA4VM_REG_


#define MCU_MCSPI0_CFG (uint32_t)0x40300000UL
#define MCU_MCSPI1_CFG (uint32_t)0x40310000UL
#define MCU_MCSPI2_CFG (uint32_t)0x40320000UL

#define MCU_MCSPI0_INTR 20U
#define MCU_MCSPI1_INTR 21U
#define MCU_MCSPI2_INTR 22U

/* SYSCONFIG bits*/
#define MCSPI_SYSCONFIG_AUTOIDLE  0U
#define MCSPI_SYSCONFIG_SOFTRESET 1U
#define MCSPI_SYSCONFIG_ENWAKEUP  2U
#define MCSPI_SYSCONFIG_SIDLE     3U
#define MCSPI_SYSCONFIG_CLOCKACT  8U

/* SYSSTATUS bits */
#define MCSPI_SYSSTATUS_RESETDONE 0U

/* IRQENABLE bits */
#define MCSPI_IRQENABLE_TX_EMPTY     0U
#define MCSPI_IRQENABLE_TX_UNDERFLOW 1U
#define MCSPI_IRQENABLE_RX_FULL      2U
#define MCSPI_IRQENABLE_RX_OVERFLOW  3U
#define MCSPI_IRQENABLE_EOW          17U

/* XFERLEVEL bits */
#define MCSPI_XFERLEVEL_AEL  0U
#define MCSPI_XFERLEVEL_AFL  8U
#define MCSPI_XFERLEVEL_WCNT 16U

/* CHCTRL bits */
#define MCSPI_CHCTRL_EN 0U

/* CHSTAT bits */
#define MCSPI_CHSTAT_RXS   0U
#define MCSPI_CHSTAT_TXS   1U
#define MCSPI_CHSTAT_EOT   2U
#define MCSPI_CHSTAT_TXFFE 3U
#define MCSPI_CHSTAT_TXFFF 4U
#define MCSPI_CHSTAT_RXFFE 5U
#define MCSPI_CHSTAT_RXFFF 6U

/* MODULCTRL bits */
#define MCSPI_MODCTRL_SINGLE  0U
#define MCSPI_MODCTRL_PIN34   1U
#define MCSPI_MODCTRL_MS      2U
#define MCSPI_MODCTRL_SYSTEST 3U
#define MCSPI_MODCTRL_INITDLY 4U
#define MCSPI_MODCTRL_MOA     7U
#define MCSPI_MODCTRL_FDAA    8U

/* CHCONF bits */
#define MCSPI_CHCONF_PHA      0U
#define MCSPI_CHCONF_POL      1U
#define MCSPI_CHCONF_CLKD     2U
#define MCSPI_CHCONF_EPOL     6U
#define MCSPI_CHCONF_WL       7U
#define MCSPI_CHCONF_TRM      12U
#define MCSPI_CHCONF_DMAW     14U
#define MCSPI_CHCONF_DMAR     15U
#define MCSPI_CHCONF_DPE0     16U
#define MCSPI_CHCONF_DPE1     17U
#define MCSPI_CHCONF_IS       18U
#define MCSPI_CHCONF_TURBO    19U
#define MCSPI_CHCONF_FORCE    20U
#define MCSPI_CHCONF_SPIENSLV 21U
#define MCSPI_CHCONF_SBE      23U
#define MCSPI_CHCONF_SBOL     24U
#define MCSPI_CHCONF_TCS0     25U
#define MCSPI_CHCONF_FFEW     27U
#define MCSPI_CHCONF_FFER     28U
#define MCSPI_CHCONF_CLKG     29U


struct mcspi_hwctx {
	uint32_t baseAddr;
	uint8_t irqNum;
};

struct mcspi_xfer {
	struct mcspi_xferReq *req;
	uint16_t xferCntRx, xferCntTx;
	uint8_t wordShift;
	uint8_t cmplt;
	uint8_t rxTrig, txTrig; /* FIFO xferlevels */
	uint8_t fifoSize;       /* FIFO size */
};

struct mcspi_dev {
	volatile uint32_t *base;
	uint8_t initialized;
	uint8_t type;
	uint32_t choffs; /* offset for channel-specific registers computed at init */
	handle_t irqMutex, devMutex;
	handle_t irqCond, devCond;
	handle_t irqHandle;
	struct mcspi_modulctrl mode; /* save for now - debugging purposes */

	int threadNum;
	void *stack;
	struct mcspi_hwctx *hwctx;

	struct mcspi_xfer xfer; /* current transaction descriptor */
	uint8_t xferBusy;       /* transfer ongoing */
};


/* MCSPI registers */
enum {
	hl_rev = 0,
	hl_hwinfo,
	hl_sysconfig = 4,

	revision = 64,
	sysconfig = 68,
	sysstatus,
	irqstatus,
	irqenable,
	wakeupenable,
	syst,
	modulctrl,

	chconf,
	chstat,
	chctrl,
	tx,
	rx,

	xferlevel = 95,
	daftx,
	dafrx = 104,
};

#endif /* _MCSPI_TDA4VM_REG_ */
