/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Driver for Intel EtherExpress 100 interface
 *
 * This driver is for the Intel EtherExpress Pro100 (Speedo3) design. It should work with all i82557/558/559 boards.
 * Driver has been ported to Phoenix-RTOS from IMMOS DPMI32 UDP/IP stack. Originally written 1996-1999 by Donald Becker,
 * 1998-2000 Andrey V. Savochkin <saw@saw.sw.com.sg>
 *
 * Copyright 2012 Phoenix Systems
 * Copyright 2006 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Pawel Kolodziej
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DEV_EEPRO100_H_
#define _DEV_EEPRO100_H_

#include <net/if.h>
#include <dev/if.h>
#include "if.h"


#define MII_BMSR      1
#define BMSR_LSTATUS  4

#define TX_RING_SIZE  64
#define RX_RING_SIZE  64

#define TX_MULTICAST_SIZE   2
#define TX_MULTICAST_RESERV (TX_MULTICAST_SIZE * 2)


/* Actual number of TX packets queued, must be <= TX_RING_SIZE-TX_MULTICAST_RESERV. */
#define TX_QUEUE_LIMIT  (TX_RING_SIZE - TX_MULTICAST_RESERV)

/* Hysteresis marking queue as no longer full */
#define TX_QUEUE_UNFULL (TX_QUEUE_LIMIT - 4)

/* Time in jiffies before concluding the transmitter is hung */
#define TX_TIMEOUT		(2*HZ)

/* Size of an pre-allocated Rx buffer: <Ethernet MTU> + slack*/
#define PKT_BUF_SZ		1536


/* Offsets to the various registers. All accesses need not be longword aligned. */
enum speedo_offsets {
	SCBStatus = 0, SCBCmd = 2,     /* Rx/Command Unit command and status. */
	SCBPointer = 4,                /* General purpose pointer. */
	SCBPort = 8,                   /* Misc. commands and operands.  */
	SCBflash = 12, SCBeeprom = 14, /* EEPROM and flash memory control. */
	SCBCtrlMDI = 16,               /* MDI interface control. */
	SCBEarlyRx = 20,               /* Early receive byte count. */
};


/* Commands that can be put in a command list entry */
enum commands {
	CmdNOp = 0, CmdIASetup = 0x10000, CmdConfigure = 0x20000,
	CmdMulticastList = 0x30000, CmdTx = 0x40000, CmdTDR = 0x50000,
	CmdDump = 0x60000, CmdDiagnose = 0x70000,
	CmdSuspend = 0x40000000,
	CmdIntr = 0x20000000,
	CmdTxFlex = 0x00080000,
};


/*
 * Clear CmdSuspend (1<<30) avoiding interference with the card access to the
 * status bits.  Previous driver versions used separate 16 bit fields for
 * commands and statuses.
 */
//#define clear_suspend(cmd)  ((u16 *)&(cmd)->cmd_status)[1] &= ~0x4000

#define clear_suspend(cmd)  ((cmd)->status &= ~0x4000)


enum SCBCmdBits {
	SCBMaskCmdDone = 0x8000, SCBMaskRxDone = 0x4000, SCBMaskCmdIdle = 0x2000,
	SCBMaskRxSuspend = 0x1000, SCBMaskEarlyRx = 0x0800, SCBMaskFlowCtl = 0x0400,
	SCBTriggerIntr = 0x0200, SCBMaskAll = 0x0100,

	/* The rest are Rx and Tx commands. */
	CUStart = 0x0010, CUResume = 0x0020, CUStatsAddr = 0x0040,
	CUShowStats = 0x0050, CUCmdBase = 0x0060, CUDumpStats = 0x0070,

	RxStart = 0x0001, RxResume = 0x0002, RxAbort = 0x0004, RxAddrLoad = 0x0006,
	RxResumeNoResources=0x0007,
};


enum SCBPort_cmds {
	PortReset = 0, PortSelfTest = 1, PortPartialReset = 2, PortDump = 3,
};


/*
 * The Speedo3 Rx and Tx frame/buffer descriptors
 */

#pragma pack(push)
#pragma pack(1)

struct descriptor {
	u16 cmd;
	u16 status;
	u32 link;
  u8 params[1];
};

#pragma pack(pop)

struct RxFD {
	s32 status;
	u32 link;
	u32 rx_buf_addr;
	u32 count;
};


/* Selected elements of the Tx/RxFD.status word. */
enum RxFD_bits {
	RxComplete = 0x8000, RxOK = 0x2000,
	RxErrCRC = 0x0800, RxErrAlign = 0x0400, RxErrTooBig = 0x0200,
	RxErrSymbol = 0x0010, RxEth2Type = 0x0020, RxNoMatch = 0x0004,
	RxNoIAMatch = 0x0002, TxUnderrun = 0x1000,  StatusComplete = 0x8000,
};


#define TX_DESCR_BUF_OFFSET 16
#define CONFIG_DATA_SIZE 22


struct TxFD {
	s32 status;
	u32 link;
	u32 tx_desc_addr;	/* Always points to the tx_buf_addr element */
	s32 count;			/* # of TBD (=1), Tx start thresh., etc. */
	u32 tx_buf_addr0;	/* void *, frame to be transmitted */
	s32 tx_buf_size0;	/* Length of Tx frame */
	u32 tx_buf_addr1;	/* void *, frame to be transmitted */
	s32 tx_buf_size1;	/* Length of Tx frame */

	/*
	 * the structure must have space for at least CONFIG_DATA_SIZE starting
	 * from tx_desc_addr field
	 */
};


/* Multicast filter setting block */
struct speedo_mc_block {
	struct speedo_mc_block *next;
	unsigned int tx;
	addr_t frame_dma;
	unsigned int len;
	struct descriptor frame; /*__attribute__ ((__aligned__(16))); */
};


/* Elements of the dump_statistics block. This block must be lword aligned */
struct speedo_stats {
	u32 tx_good_frames;
	u32 tx_coll16_errs;
	u32 tx_late_colls;
	u32 tx_underruns;
	u32 tx_lost_carrier;
	u32 tx_deferred;
	u32 tx_one_colls;
	u32 tx_multi_colls;
	u32 tx_total_colls;
	u32 rx_good_frames;
	u32 rx_crc_errs;
	u32 rx_align_errs;
	u32 rx_resource_errs;
	u32 rx_overrun_errs;
	u32 rx_colls_errs;
	u32 rx_runt_errs;
	u32 done_marker;
};


enum Rx_ring_state_bits {
	RrNoMem = 1, RrPostponed = 2, RrNoResources = 4, RrOOMReported = 8,
};


#define TX_RING_ELEM_DMA(sp, n) ((sp)->tx_ring_dma + (n)*sizeof(struct TxFD))


struct speedo_private {
	struct TxFD *tx_ring;
	struct RxFD *rx_ringp[RX_RING_SIZE];

	/* The addresses of a Tx/Rx-in-place packets/buffers. */
	struct pbuf *tx_skbuff[TX_RING_SIZE];
	struct pbuf *rx_skbuff[RX_RING_SIZE];

	/* Mapped addresses of the rings */
	addr_t tx_ring_dma;
	addr_t rx_ring_dma[RX_RING_SIZE];
	struct descriptor *last_cmd;          /* Last command sent */
	unsigned int cur_tx, dirty_tx;        /* The ring entries to be free()ed */
	spinlock_t lock;                      /* Group with Tx control cache line */
	mutex_t mutex;                        /* Group with Tx control cache line */
	u32 tx_threshold;                     /* The value for txdesc.count */
	struct RxFD *last_rxf;                /* Last filled RX buffer */
	addr_t last_rxf_dma;
	unsigned int cur_rx, dirty_rx;        /* The next free ring entry */
	long last_rx_time;                    /* Last Rx, in jiffies, to handle Rx hang */
	netif_stat_t stats;
	struct speedo_stats *lstats;
	addr_t lstats_dma;
	int chip_id;
	pci_device_t *pcidev;
	struct speedo_mc_block *mc_setup_head;
	struct speedo_mc_block *mc_setup_tail;
	void * base;
	int irq;
	int stopped;
	long in_interrupt;
	unsigned char acpi_pwr;
	char rx_mode;                           /* Current PROMISC/ALLMULTI setting */
	unsigned int tx_full:1;                 /* The Tx queue is full */
	unsigned int full_duplex:1;             /* Full-duplex operation requested */
 	unsigned int flow_ctrl:1;               /* Use 802.3x flow control */
	unsigned int rx_bug:1;                  /* Work around receiver hang errata */
	unsigned char default_port:8;           /* Last dev->if_port value */
	unsigned char rx_ring_state;            /* RX ring status flags */
	unsigned short phy[2];                  /* PHY media interfaces available */
	unsigned short advertising;             /* Current PHY advertised caps */
	unsigned short partner;                 /* Link partner caps */
};


/* The parameters for a CmdConfigure operation */
static const char i82557_config_cmd[CONFIG_DATA_SIZE] = {
	22, 0x08, 0, 0,  0, 0, 0x32, 0x03, 1, /* 1=Use MII  0=Use AUI */
	0, 0x2E, 0, 0x60, 0,
	0xf2, 0x48, 0, 0x40, 0xf2, 0x80, 		/* 0x40=Force full-duplex */
	0x3f, 0x05,
};


static const char i82558_config_cmd[CONFIG_DATA_SIZE] = {
	22, 0x08, 0, 1, 0, 0, 0x22, 0x03, 1, /* 1=Use MII  0=Use AUI */
	0, 0x2E, 0,  0x60, 0x08, 0x88,
	0x68, 0, 0x40, 0xf2, 0x84, /* Disable FC */
	0x31, 0x05,
};


enum phy_chips { NonSuchPhy=0, I82553AB, I82553C, I82503, DP83840, S80C240, S80C24, I82555, DP83840A=10, };



#endif
