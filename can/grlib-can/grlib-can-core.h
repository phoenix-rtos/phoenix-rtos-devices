/*
 * Phoenix-RTOS
 *
 * GRCANFD driver
 *
 * GRLIB CANFD driver file
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mikolaj Matalowski
 *
 * %LICENSE%
 */

#ifndef GRLIB_CAN_CORE_H
#define GRLIB_CAN_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

#include "grlib-can-shared.h"

extern uint64_t curr_rx_irq_handler;
extern uint64_t prev_rx_irq_handler;
extern uint32_t sleep_counter;

#define VERBOSE 1
#undef VERBOSE

/* Memory used by DMA has to be aligned to 1kbyte segments (here used 4kbyte) */
#define PAGE_ALIGN(addr) (((addr_t)(addr) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1))

#define GRLIB_CAN_DEF_CIRCBUFSZ 16
#define GRLIB_MAX_CAN_DEVICES   8

/* IRQ definitions */
#define GRLIB_CAN_TXLOSS_IRQ   (1u << 16) /* Loss of arbitration during transmission */
#define GRLIB_CAN_RXMISS_IRQ   (1u << 15) /* Message filtered away */
#define GRLIB_CAN_TXERRCNT_IRQ (1u << 14) /* TX error counter incremented */
#define GRLIB_CAN_RXERRCNT_IRQ (1u << 13) /* RX error counter incremented */
#define GRLIB_CAN_TXSYNC_IRQ   (1u << 12) /* Sync message send by the TX line */
#define GRLIB_CAN_RXSYNC_IRQ   (1u << 11) /* Sync message send on RX line */
#define GRLIB_CAN_TX_IRQ       (1u << 10) /* Successful transmission of the message */
#define GRLIB_CAN_RX_IRQ       (1u << 9)  /* Successful reception of the message */
#define GRLIB_CAN_TXEMPTY_IRQ  (1u << 8)  /* Successful transmission of all messages */
#define GRLIB_CAN_RXFULL_IRQ   (1u << 7)  /* All stored frames read */
#define GRLIB_CAN_TXPTR_IRQ    (1u << 6)  /* TX buffer read pointer equal to txIrq pointer*/
#define GRLIB_CAN_RXPTR_IRQ    (1u << 5)  /* RX buffer write pointer equal to rxIrq pointer*/
#define GRLIB_CAN_BMRDERR_IRQ  (1u << 4)  /* Error during AMBA read */
#define GRLIB_CAN_BMWRERR_IRQ  (1u << 3)  /* Error during AMBA write */
#define GRLIB_CAN_OR_IRQ       (1u << 2)  /* Over-run during reception */
#define GRLIB_CAN_BUSSOFF_IRQ  (1u << 1)  /* Bus-off condition */
#define GRLIB_CAN_PASS_IRQ     1u         /* Error-passive condition */

/* Structure that follows GRCANFD register structure */
typedef struct {
	volatile uint32_t confReg;      /* Configuration register, only last byte not reserved */
	volatile uint32_t statReg;      /* Status register */
	volatile uint32_t ctrlReg;      /* Select between CAN/CANFD and reset */
	volatile const uint32_t capReg; /* Capability register */

	volatile const uint32_t rsvrd0[2];

	volatile uint32_t syncMask; /* Sync mask filter register */
	volatile uint32_t syncCode; /* Sync code filter register */

	volatile const uint32_t rsvrd1[8];

	/* Timing configuration*/
	volatile uint32_t nomTimConf;      /* Nominal timing configuration */
	volatile uint32_t dataTimConf;     /* Data timing configuration */
	volatile uint32_t transDelCompReg; /* Transmission delay compensation */

	volatile const uint32_t rsvrd2[13];

	/* CANopen configuration */
	volatile uint32_t copReg;             /* CANopen heartbeat configuration register */
	volatile uint32_t ohtTimReg;          /* CANopen heartbeat timeout register */
	volatile const uint32_t ohtCntReg;    /* CANopen heartbeat counter register */
	volatile const uint32_t ohtStatusReg; /* CANopen heartbeat status register */

	volatile const uint32_t rsvrd3[28];

	/* IRQ configuration */
	volatile uint32_t penIsrMsk; /* Pending interrupt masked status register */
	volatile uint32_t penIrqMsk; /* Pending interrupt masked register */
	volatile uint32_t penIsr;    /* Pending interrupt status register */
	volatile uint32_t penIrq;    /* Pending interrupt register */
	volatile uint32_t irqMskSet; /* Interrupt mask register */
	volatile uint32_t irqClrReg; /* Pending interrupt clear register */

	volatile const uint32_t rsrvd4[58];

	/* TX channel configuration */
	volatile uint32_t txCtrlReg; /* TX channel control register */
	volatile uint32_t txBufAdd;  /* TX channel circular buffer base address */
	volatile uint32_t txBufSz;   /* TX channel circular buffer size */
	volatile uint32_t txWrtPtr;  /* TX channel circular buffer write pointer */
	volatile uint32_t txRdPtr;   /* TX channel circular buffer read pointer*/
	volatile uint32_t txIrqReg;  /* TX channel interrupt configuration register */

	volatile const uint32_t rsrvd5[58];

	/* RX channel configuration */
	volatile uint32_t rxCtrlReg; /* RX channel control register */
	volatile uint32_t rxBufAdd;  /* RX channel circular buffer base address */
	volatile uint32_t rxBufSz;   /* RX channel circular buffer size */
	volatile uint32_t rxWrtPtr;  /* RX channel circular buffer write pointer */
	volatile uint32_t rxRdPtr;   /* RX channel circular buffer read pointer */
	volatile uint32_t rxIrqReg;  /* RX channel interrupt configuration register */
	volatile uint32_t rxAccMask; /* RX channel acceptance mask */
	volatile uint32_t rxAccCode; /* RX channel acceptance code */
} grlibCan_hwDev_t;

typedef enum {
	deviceIdle = 0,
	deviceReady = 1,
	deviceError = -1
} deviceStatus;

typedef enum {
	txOngoing = 0,
	txDone = 1
} txStatus;

typedef enum {
	txBufReady = 0,
	txBufFull = 1
} txBufStatus;

typedef enum {
	rxBufReady = 0,
	rxBufEmpty = 1
} rxBufStatus;

typedef struct {
	int canId;
	msg_rid_t ownerPid;
	unsigned int irqNum;
	grlibCan_hwDev_t *device;

	deviceStatus deviceStatus;
	txStatus txStatus;

	handle_t ctrlLock; /* Lock for configuration and control of the device */
	handle_t txLock;
	handle_t rxLock;
	handle_t cond; /* Conditional passed to the interrupt */

	uint32_t rxIrqCounter;
	uint32_t txIrqCounter;

	/* RX buffer */
	grlibCan_msg_t *rxBufAdd;
	rxBufStatus rxBufStatus;
	uint32_t rxBufSz;

	/* TX buffer */
	grlibCan_msg_t *txBufAdd;
	txBufStatus txBufStatus;
	uint32_t txBufSz;
} grlibCan_dev_t;

typedef struct {
	uint32_t port;
	grlibCan_dev_t *devices;
	int num;
} grlibCan_driver_t;

/* Core driver functions */


/* Create devices */
int grlibCan_initDevices(grlibCan_dev_t *dev, int num);


/* Reset codec */
int grlibCan_resetDevice(grlibCan_dev_t *dev);


/* Allocate resources */
int grlibCan_allocateResources(grlibCan_dev_t *dev, uintptr_t base, int id);


/* Allocate TX/RX buffers */
int grlibCan_allocateBuffers(grlibCan_dev_t *dev);


/* Register devices on the system */
int grlibCan_registerDevices(oid_t *port, grlibCan_dev_t *devices, uint32_t length);


/* Cleanup resources */
void grlibCan_cleanupResources(grlibCan_dev_t *dev);


/* Copy current device config */
void grlibCan_copyConfig(grlibCan_dev_t *device, grlibCan_config_t *config);


/* Apply default config */
int grlibCan_applyDefConf(grlibCan_dev_t *dev);


/* Apply user provided config  */
int grlibCan_applyConfig(grlibCan_dev_t *dev, grlibCan_config_t *config);


/* Driver interrupt handler */
int grlibCan_irqHandler(unsigned int irqNum, void *arg);
/* Transmit buffer of frames in blocking mode */
int grlibCan_transmitSync(grlibCan_dev_t *dev, const grlibCan_msg_t *buffer,
		const uint32_t length);


/* Transmit buffer of frames in non-blocking mode */
int grlibCan_transmitAsync(grlibCan_dev_t *dev, const grlibCan_msg_t *buffer,
		const uint32_t length);


/* Receive at most length frames in blocking mode */
int grlibCan_recvSync(grlibCan_dev_t *dev, grlibCan_msg_t *buffer,
		const uint32_t length, uint32_t *pending);


/* Receive at most length frame in non-blocking mode */
int grlibCan_recvAsync(grlibCan_dev_t *dev, grlibCan_msg_t *buffer,
		const uint32_t length);


/* Calculate register configuration for given baud rate */
uint32_t grlibCan_setBdRateData(double dataBdRate, uint32_t prevSetting);


/* Calculate register configuration for given baud rate */
uint32_t grlibCan_setBdRateNom(double nomBdRate, uint32_t prevSetting);


/* Remove device files */
int grlibCan_unregisterDevices(oid_t *port, grlibCan_dev_t *devices, uint32_t length);


/* Pop one CAN frame from circular buffer */
int grlibCan_popFrame(grlibCan_dev_t *dev, grlibCan_msg_t *msg);


/* Push one CAN frame to circular buffer */
int grlibCan_pushFrame(grlibCan_dev_t *dev, const grlibCan_msg_t *msg);


/* Use platformctl syscall to query system for GRCANFD controllers*/
int grlibCan_queryForDevices(grlibCan_dev_t *dev);


#endif
