/*
 * Phoenix-RTOS
 *
 * GRCANFD driver
 *
 * GRLIB CANFD driver header file
 *
 * Copyright 2025 Phoenix Systems
 * Author: Mikolaj Matalowski
 *
 * %LICENSE%
 */

#ifndef GRLIB_CAN_H
#define GRLIB_CAN_H

/* temporary */
#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

#define VERBOSE 1

/* Memory used by DMA has to be alligned to 1kbyte segments (here used 4kbyte) */
#define PAGE_ALIGN(addr) (((addr_t)(addr) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1))

/* General definitions */
#define GRLIB_CAN_NUM 2

#define GRLIB_CAN0_BASE_ADD 0xff400000
#define GRLIB_CAN1_BASE_ADD 0xff401000

#define GRLIB_CAN0_IRQ 8
#define GRLIB_CAN1_IRQ 9

#define GRLIB_CAN_DEF_CIRCBUFSZ 16

/* Register masks definition */
/* General register masks*/
#define GRLIB_CAN_confReg_MASK  0xFF
#define GRLIB_CAN_ctrlReg_MASK  0x3
#define GRLIB_CAN_syncMask_MASK 0x1FFFFFFF
#define GRLIB_CAN_syncCode_MASK 0x1FFFFFFF

/* Timing masks*/
#define GRLIB_CAN_timConf_MASK   (0xFFFFFFFF >> 8)
#define GRLIB_CAN_txDelComp_MASK (0xFFFFFFFF >> 26)

/* TX registers masks*/
#define GRLIB_CAN_txCtrLReg_MASK (0xFFFFFFFF >> 28)
#define GRLIB_CAN_txAddReg_MASK  (0xFFFFFFFF << 10)
#define GRLIB_CAN_txSzReg_MASK   (0xFFFFFFFF >> 11) && ~(0xFFFFFFFF << 6)
#define GRLIB_CAN_txWrPtr_MASK   (0xFFFFFFFF >> 12) && ~(0xFFFFFFFF >> 28)
#define GRLIB_CAN_txRdPtr_MASK   (0xFFFFFFFF >> 12) && ~(0xFFFFFFFF >> 28)
#define GRLIB_CAN_txIrqSet_MASK  (0xFFFFFFFF >> 12) && ~(0xFFFFFFFF >> 28)

/* RX registers masks*/
#define GRLIB_CAN_rxCtrLReg_MASK (0xFFFFFFFF >> 28)
#define GRLIB_CAN_rxAddReg_MASK  (0xFFFFFFFF << 10)
#define GRLIB_CAN_rxSzReg_MASK   (0xFFFFFFFF >> 11) && ~(0xFFFFFFFF >> 26)
#define GRLIB_CAN_rxWrPtr_MASK   (0xFFFFFFFF >> 12) && ~(0xFFFFFFFF >> 28)
#define GRLIB_CAN_rxRdPtr_MASK   (0xFFFFFFFF >> 12) && ~(0xFFFFFFFF >> 28)
#define GRLIB_CAN_rxIrqSet_MASK  (0xFFFFFFFF >> 12) && ~(0xFFFFFFFF >> 28)
#define GRLIB_CAN_rxAccMask_MASK 0x1FFFFFFF

#define GRLIB_CAN_irqReg_MASK (0xFFFFFFFF >> 11)

/* IRQ definitions */
#define GRLIB_CAN_txLoss_irq   (1u << 16) /* Loss of arbitration during transmission */
#define GRLIB_CAN_rxMiss_irq   (1u << 15) /* Message filtered away */
#define GRLIB_CAN_txErrCnt_IRQ (1u << 14) /* TX error counter incremented */
#define GRLIB_CAN_rxErrCnt_IRQ (1u << 13) /* RX error counter incremented */
#define GRLIB_CAN_txSync_IRQ   (1u << 12) /* Sync message send by the TX line */
#define GRLIB_CAN_rxSync_IRQ   (1u << 11) /* Sync message send on RX line */
#define GRLIB_CAN_tx_IRQ       (1u << 10) /* Successful transmission of the message */
#define GRLIB_CAN_rx_IRQ       (1u << 9)  /* Successful reception of the message */
#define GRLIB_CAN_txEmpty_IRQ  (1u << 8)  /* Successful transmission of all messages */
#define GRLIB_CAN_rxFull_IRQ   (1u << 7)  /* All stored frames read */
#define GRLIB_CAN_txPtr_IRQ    (1u << 6)  /* TX buffer read pointer equal to txIrq pointer*/
#define GRLIB_CANrxPtr_IRQ     (1u << 5)  /* RX buffer write pointer equal to rxIrq pointer*/
#define GRLIB_CAN_bmRdErr_IRQ  (1u << 4)  /* Error during AMBA read */
#define GRLIB_CAN_bmWrErr      (1u << 3)  /* Error during AMBA write */
#define GRLIB_CAN_or_IRQ       (1u << 2)  /* Over-run during reception */
#define GRLIB_CAN_busOff_IRQ   (1u << 1)  /* Bus-off condition */
#define GRLIB_CAN_pass_IRQ     1u         /* Error-passive condition */

/* This structure is either a CAN frame or pure payload */
typedef struct
{
	union {
		struct {
			uint32_t head; /* Head contains CAN packet mode and IDs */
			uint32_t stat;

			uint8_t payload[8]; /* Payload */
		} frame;

		uint8_t payload[16];
	};
} grlibCan_msg_t;

typedef struct
{
	/* Base configuration */
	uint32_t conf;
	uint32_t syncMask;
	uint32_t syncCode;

	uint32_t nomBdRate;    /* Nominal baud-rate in kbps */
	uint32_t dataBdRate;   /* Data transfer baud-rate in kbps */
	uint32_t transDelComp; /* Tranmission delat compensation */

	/* CANopen currentlly omited */

	/* TX configuration */
	uint32_t txCtrlReg;

	/* RX configuration */
	uint32_t rxCtrlReg;
	uint32_t rxAccMask;
	uint32_t rxAccCode;
} grlibCan_config_t;

/* Private structures used by the driver*/

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
	union {
		struct {
			volatile const uint8_t rsrvd; /* Reserved */
			volatile uint8_t scaler;      /* Scaler */
			volatile uint16_t timConf;    /* Timing confguration */
		};
		uint32_t reg;
	} nomTimConf; /* Nominal timing configuration */

	union {
		struct {
			volatile const uint8_t rsrvd; /* Reserved */
			volatile uint8_t scaler;      /* Scaler */
			volatile uint16_t timConf;    /* Timing confguration */
		};
		uint32_t reg;
	} dataTimConf; /* Data timing configuration */

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

#define RX_RST       0
#define RX_EMPTY     1
#define RX_RDY       2
#define RX_ERR       3
#define RX_NOT_EMPTY 4

#define TX_RST       0
#define TX_FULL      1
#define TX_RDY       2
#define TX_ERR       3
#define TX_NOT_EMPTY 4

#define GRLIB_CAN_TRANSACTION_NULL    0
#define GRLIB_CAN_TRANSACTION_ONGOING 1
#define GRLIB_CAN_TRANSATION_FINISHED 2
#define GRLIB_CAN_TRANSATION_FAILED   3

/* Structure used to handle devices within the driver */
typedef struct {
	int canId;
	grlibCan_hwDev_t *device;

	handle_t ctrlLock; /* Lock for configuration and control of the device */
	handle_t txLock;
	handle_t rxLock;
	handle_t cond; /* Conditional passed to the interrupt */


	volatile uint32_t recv;
	volatile uint32_t sent;
	volatile uint32_t txErr;
	volatile uint32_t rxMiss;
	volatile uint32_t rxErrCntr;
	volatile uint32_t overRun;
	volatile uint32_t counter;

	/* RX buffer */
	grlibCan_msg_t *rxBufAdd;
	size_t rxBufSz;
	volatile uint32_t rxBufStatus;
	volatile uint32_t rxBusStatus;

	/* TX buffer */
	grlibCan_msg_t *txBufAdd;
	size_t txBufSz;
	volatile uint32_t txBufStatus;
	volatile uint32_t txBusStatus;
	volatile uint32_t txTransactionStatus;
} grlibCan_dev_t;

union {
	struct {
		volatile const uint8_t rsrvd; /* Reserved */
		volatile uint8_t scaler;      /* Scaler */
		volatile uint16_t timConf;    /* Timing confguration */
	} conf;                           /* Configuration for bd-rate of 115200 */
	uint32_t reg;
} defTimConfNom = { .reg = (15 << 16) | (40 << 10) | (14 << 5) | 14 };

union {
	struct {
		volatile const uint8_t rsrvd; /* Reserved */
		volatile uint8_t scaler;      /* Scaler */
		volatile uint16_t timConf;    /* Timing confguration */
	} conf;                           /* Configuration for bd-rate of 115200 */
	uint32_t reg;
} defTimConfData = { .reg = (15 << 16) | (40 << 10) | (8 << 5) | 8 };

/* Driver status struct */
typedef struct {
	uint32_t port;
	grlibCan_dev_t *devices;
} grlibCan_driver_t;

/* Shared functions*/

/* Private functions used by driver*/
/* Main routine of the driver*/
static void grlibCan_messageThread(void *arg);
/* Create devices */
static int grlibCan_initDevices(grlibCan_dev_t *dev, int num, bool loopback);
/* */
static int grlibCan_allocateResources(grlibCan_dev_t *dev, uint32_t base, int id);
/* */
static int grlibCan_allocateBuffers(grlibCan_dev_t *dev, uint32_t bufLen);
/* */
static int grlibCan_registerDevices(oid_t *port, grlibCan_dev_t *devices, uint32_t length);
/* */
static void grlibCan_cleanupResources(grlibCan_dev_t *dev);
/* */
static int grlibCan_applyDefConf(grlibCan_dev_t *dev);
/* Apply user config after initialization of the driver */
static int grlibCan_applyConfig(grlibCan_dev_t *dev, grlibCan_config_t *config);
/* Normal interrupt handler */
static int grlibCan_irqHandler(unsigned int irqNum, void *arg);
/*  */
static int grlibCan_transmitSync(grlibCan_dev_t *dev, const grlibCan_msg_t *buffer,
		const uint32_t length);

static int grlibCan_transmitAsync(grlibCan_dev_t *dev, const grlibCan_msg_t *buffer,
		const uint32_t length);

static int grlibCan_recvSync(grlibCan_dev_t *dev, grlibCan_msg_t *buffer,
		const uint32_t length);

static int grlibCan_recvAsync(grlibCan_dev_t *dev, grlibCan_msg_t *buffer,
		const uint32_t length);

/* Fragment and place date into CAN message */
// static int grlibCan_prepareData(const uint8_t *buffer, const uint32_t len,
// 		grlibCan_msg_t **packets, uint32_t *pLen);

static inline int grlibCan_popFrame(grlibCan_dev_t *dev, grlibCan_msg_t *msg);
static inline int grlibCan_pushFrame(grlibCan_dev_t *dev, const grlibCan_msg_t *msg);
#endif
