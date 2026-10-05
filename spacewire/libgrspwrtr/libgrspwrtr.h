/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Andrzej Tlomak, Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef LIBGRSPWRTR_H_
#define LIBGRSPWRTR_H_


#include <stdint.h>


/* Router port numbering:
 *   0                        - configuration port
 *   1 .. nSpw                - SpaceWire ports
 *   nSpw+1 .. nSpw+nAmba     - AMBA (DMA) ports
 *   nSpw+nAmba+1 .. +nFifo   - FIFO ports
 * Addresses 1..31 are physical (path) addresses, 32..255 are logical addresses.
 */

#define SPWRTR_LS_ERR_RESET  0
#define SPWRTR_LS_ERR_WAIT   1
#define SPWRTR_LS_READY      2
#define SPWRTR_LS_STARTED    3
#define SPWRTR_LS_CONNECTING 4
#define SPWRTR_LS_RUN        5

/* Routing table address control */
#define SPWRTR_RTACTRL_SR (1U << 3) /* spill-if-not-ready */
#define SPWRTR_RTACTRL_EN (1U << 2) /* address enable */
#define SPWRTR_RTACTRL_PR (1U << 1) /* high priority */
#define SPWRTR_RTACTRL_HD (1U << 0) /* header deletion */

/* Port status error bits */
#define SPWRTR_PSTS_ERR_PL (1U << 29) /* packet length truncation */
#define SPWRTR_PSTS_ERR_RS (1U << 27) /* RMAP/P&P spill */
#define SPWRTR_PSTS_ERR_SR (1U << 26) /* spill-if-not-ready */
#define SPWRTR_PSTS_ERR_TS (1U << 18) /* timeout spill */
#define SPWRTR_PSTS_ERR_ME (1U << 17) /* port buffer memory error */
#define SPWRTR_PSTS_ERR_IA (1U << 4)  /* invalid address - no route for incoming packet */
#define SPWRTR_PSTS_ERR_CE (1U << 3)  /* credit error */
#define SPWRTR_PSTS_ERR_ER (1U << 2)  /* escape error */
#define SPWRTR_PSTS_ERR_DE (1U << 1)  /* disconnect error */
#define SPWRTR_PSTS_ERR_PE (1U << 0)  /* parity error */

#define SPWRTR_PSTS_ERR_MASK (SPWRTR_PSTS_ERR_PL | SPWRTR_PSTS_ERR_RS | SPWRTR_PSTS_ERR_SR | SPWRTR_PSTS_ERR_TS | SPWRTR_PSTS_ERR_ME | \
		SPWRTR_PSTS_ERR_IA | SPWRTR_PSTS_ERR_CE | SPWRTR_PSTS_ERR_ER | SPWRTR_PSTS_ERR_DE | SPWRTR_PSTS_ERR_PE)


typedef struct {
	volatile struct spwrtr_regs *regs;
	uint8_t nSpwPorts;
	uint8_t nAmbaPorts;
	uint8_t nFifoPorts;
	uint8_t nPorts; /* Total including configuration port 0 */
} spwrtr_dev_t;


/* Router port number of the n-th AMBA port (0-based), negative errno if it does not exist */
int spwrtr_ambaPort(const spwrtr_dev_t *dev, unsigned int n);


/* Set port map of a physical (1..nPorts-1) or logical (32..255) address. enPorts is a bitmask of router ports. */
int spwrtr_setPortMapping(spwrtr_dev_t *dev, uint8_t addr, uint32_t enPorts);


int spwrtr_getPortMapping(spwrtr_dev_t *dev, uint8_t addr, uint32_t *enPorts);


/* Set routing table address control (SPWRTR_RTACTRL_*) */
int spwrtr_setAddrCtrl(spwrtr_dev_t *dev, uint8_t addr, uint32_t ctrl);


int spwrtr_getAddrCtrl(spwrtr_dev_t *dev, uint8_t addr, uint32_t *ctrl);


/* Program a complete route: port map + address control (EN is forced on), verified by read-back */
int spwrtr_setRoute(spwrtr_dev_t *dev, uint8_t addr, uint32_t enPorts, uint32_t ctrl);


/* Disable a route (clears EN and the port map) */
int spwrtr_clearRoute(spwrtr_dev_t *dev, uint8_t addr);


int spwrtr_resetDevice(spwrtr_dev_t *dev);


int spwrtr_setClockDiv(spwrtr_dev_t *dev, uint8_t port, uint32_t clkdiv);


int spwrtr_getClockDiv(spwrtr_dev_t *dev, uint8_t port, uint32_t *clkdiv);


/* Configure SpW port run-state link frequency (Hz) and start the link */
int spwrtr_portStart(spwrtr_dev_t *rtrDev, uint8_t port, uint32_t spwFreq);


int spwrtr_getLinkState(spwrtr_dev_t *dev, uint8_t port, uint8_t *linkState);


/* Poll until SpW port reaches Run state. Returns 0, -ETIMEDOUT or other negative errno. */
int spwrtr_waitLinkRun(spwrtr_dev_t *dev, uint8_t port, uint32_t timeoutUs);


/* Raw port status register (any port except 0) */
int spwrtr_getPortStatus(spwrtr_dev_t *dev, uint8_t port, uint32_t *sts);


/* Read sticky error bits (SPWRTR_PSTS_ERR_*) of a port and clear them. errs may be NULL. */
int spwrtr_clearPortErrors(spwrtr_dev_t *dev, uint8_t port, uint32_t *errs);


/* Print decoded port status (link state, errors, busy flags) */
void spwrtr_logPortStatus(spwrtr_dev_t *dev, uint8_t port);


/* Print decoded route (port map and address control) */
void spwrtr_logRoute(spwrtr_dev_t *dev, uint8_t addr);


const char *spwrtr_linkStateName(uint8_t linkState);


int spwrtr_init(spwrtr_dev_t *rtrDev, unsigned int n);


#endif
