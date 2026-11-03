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

#ifndef _MCSPI_TDA4VM_
#define _MCSPI_TDA4VM_

#include <sys/types.h>

#include <stdint.h>

/* user macros */
#define MCSPI_DEVICE_SLAVE  1U
#define MCSPI_DEVICE_MASTER 2U

#define MCU_MCSPI0 0U
#define MCU_MCSPI1 1U
#define MCU_MCSPI2 2U

#define MCU_MCSPI_CH0 0U
#define MCU_MCSPI_CH1 1U
#define MCU_MCSPI_CH2 2U
#define MCU_MCSPI_CH3 3U

#define MCSPI_XFER_TXRX 0U
#define MCSPI_XFER_RX   1U
#define MCSPI_XFER_TX   2U


/* MCSPI_MODULCTRL */
struct mcspi_modulctrl {
	uint16_t single : 1;      /* 0 - Multi channel, 1 - Single channel MCSP*/
	uint16_t pin34 : 1;       /* 0 - CS used, 1 - CS not used */
	uint16_t ms : 1;          /* 0 - Master, 1 - Slave */
	uint16_t system_test : 1; /* 0 - functional mode, 1 - system test mode */
	uint16_t initdly : 3;     /* delay for first transfer */
	uint16_t moa : 1;         /* multiple word configuration */
};


/* MCSPI_CHCONF */
struct mcspi_chconf {
	uint32_t pha : 1;   /* SPICLK phase: 0 - data latched at odd edges, 1 - data latched at even edges */
	uint32_t pol : 1;   /* SPICLK polarity: 0 - low at inactive, 1 - high at inactive */
	uint32_t clkd : 4;  /* SPICLK divider: divide SPICLKREF to get required SPICLK. Values are power of 2 (max 2^16) */
	uint32_t epol : 1;  /* SPIEN polarity: 0 - held high during active, 1 - held low during active */
	uint32_t wl : 5;    /* world length in bits: write n-1 value in hex - set at transaction init */
	uint32_t trm : 2;   /* transmission mode: 0 - transmit-and-receive, 1 - receive-only, 2 - transmit only*/
	uint32_t dmaw : 1;  /* DMA write request: 0 - dis, 1 - en */
	uint32_t dmar : 1;  /* DMA read request: 0 - dis, 1 - en */
	uint32_t dpe0 : 1;  /* Transmission enable for SPIDAT[0]: 0 - enable, 1 - disable */
	uint32_t dpe1 : 1;  /* Transmission enable for SPIDAT[1]: 0 - enable, 1 - disable */
	uint32_t is : 1;    /* Input select: 0 - SPIDAT[0] selected for reception, 1 - SPIDAT[1] selected for reception */
	uint32_t turbo : 1; /* turbo mode: 0 - dis, 1 - en */
	uint32_t force : 1; /* SPIEN active during multi-word transmission: 0 - dis, 1 - en */
	uint32_t spienslv : 2;
	uint32_t sbe : 1;  /* Start-bit enable */
	uint32_t sbol : 1; /* Start-bit polarity */
	uint32_t tcs0 : 2; /* Chip-select delay: n.of interface clocks between CS active - first MCSPI edge (0.5; 1.5; 2.5; 3.5 cycles accord.)*/
	uint32_t ffew : 1; /* FIFO enabled for TX */
	uint32_t ffer : 1; /* FIFO enabled for RX */
	uint32_t clkg : 1;
};


struct mcspi_xferReq {
	uint8_t devnum;
	uint8_t channel;
	uint8_t wordSize;
	uint16_t wordCount;
	void *txBuff, *rxBuff;
	uint8_t xferType; /* MCSPI_XFER_TXRX, MCSPI_XFER_RX, MCSPI_XFER_TX*/
};


int mcspi_init(uint8_t devnum, uint8_t chnum, uint8_t devtype, struct mcspi_modulctrl *modctrl, struct mcspi_chconf *chconf);


int mcspi_deinit(uint8_t devnum);


int mcspi_xferStart(struct mcspi_xferReq *xferReq);


#endif
