/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * 8139too.c: RealTek RTL-8139 Fast Ethernet driver
 *
 * This driver has been ported to Phoenix-RTOS from IMMOS DPMI32 UDP/IP stack.
 * IMMOS DPMI32 UDP/IP stack driver based on Linux kernel driver.
 *
 * Copyright 2012 Phoenix Systems
 * Copyright 2003, 2006 Pawel Pisaczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _DEV_8139too_H_
#define _DEV_8139too_H_

#define DRV_NAME    "8139too"
#define DRV_VERSION "0.9.25"
#include <net/mii.h>
#include <net/net.h>
#include <dev/pci.h>
#include "if.h"

/* media options */

#define MAX_UNITS 8
static int media[MAX_UNITS] = {-1, -1, -1, -1, -1, -1, -1, -1};
static int full_duplex[MAX_UNITS] = {-1, -1, -1, -1, -1, -1, -1, -1};

/* size of the in-memory receive ring */
#define RX_BUF_LEN_IDX   2
#define RX_BUF_LEN       (8192 << RX_BUF_LEN_IDX)
#define RX_BUF_PAD       16
#define RX_BUF_WRAP_PAD  2048
#define RX_BUF_TOT_LEN   (RX_BUF_LEN + RX_BUF_PAD + RX_BUF_WRAP_PAD)


/* size of the Tx bounce buffers - must be at least (dev->mtu+14+4) */
#define NUM_TX_DESC         4
#define MAX_ETH_FRAME_SIZE  1536
#define TX_BUF_SIZE         MAX_ETH_FRAME_SIZE
#define TX_BUF_TOT_LEN      (TX_BUF_SIZE * NUM_TX_DESC)

/* Registers */
enum RTL8139_registers {
	MAC0 = 0,               /* Ethernet hardware address */
	MAR0 = 8,               /* Multicast filter */
	TxStatus0 = 0x10,       /* Transmit status (Four 32bit registers) */
	TxAddr0 = 0x20,         /* Tx descriptors (also four 32bit) */
	RxBuf = 0x30,
	ChipCmd = 0x37,
	RxBufPtr = 0x38,
	RxBufAddr = 0x3A,
	IntrMask = 0x3C,
	IntrStatus = 0x3E,
	TxConfig = 0x40,
	ChipVersion = 0x43,
	RxConfig = 0x44,
	Timer = 0x48,           /* A general-purpose counter */
	RxMissed = 0x4C,        /* 24 bits valid, write clears */
	Cfg9346 = 0x50,
	Config0 = 0x51,
	Config1 = 0x52,
	FlashReg = 0x54,
	MediaStatus = 0x58,
	Config3 = 0x59,
	Config4 = 0x5A,         /* absent on RTL-8139A */
	HltClk = 0x5B,
	MultiIntr = 0x5C,
	TxSummary = 0x60,
	BasicModeCtrl = 0x62,
	BasicModeStatus = 0x64,
	NWayAdvert = 0x66,
	NWayLPAR = 0x68,
	NWayExpansion = 0x6A,

	/* Undocumented registers, but required for proper operation */
	FIFOTMS = 0x70,         /* FIFO Control and test */
	CSCR = 0x74,            /* Chip Status and Configuration Register */
	PARA78 = 0x78,
	PARA7c = 0x7c,          /* Magic transceiver parameter register */
	Config5 = 0xD8,         /* absent on RTL-8139A */
};


enum ClearBitMasks {
	MultiIntrClear = 0xF000,
	ChipCmdClear = 0xE2,
	Config1Clear = (1<<7)|(1<<6)|(1<<3)|(1<<2)|(1<<1),
};


enum ChipCmdBits {
	CmdReset = 0x10,
	CmdRxEnb = 0x08,
	CmdTxEnb = 0x04,
	RxBufEmpty = 0x01,
};


/* Interrupt register bits, using my own meaningful names */
enum IntrStatusBits {
	PCIErr = 0x8000,
	PCSTimeout = 0x4000,
	RxFIFOOver = 0x40,
	RxUnderrun = 0x20,
	RxOverflow = 0x10,
	TxErr = 0x08,
	TxOK = 0x04,
	RxErr = 0x02,
	RxOK = 0x01,
	RxAckBits = RxFIFOOver | RxOverflow | RxOK,
};


enum TxStatusBits {
	TxHostOwns = 0x2000,
	TxUnderrun = 0x4000,
	TxStatOK = 0x8000,
	TxOutOfWindow = 0x20000000,
	TxAborted = 0x40000000,
	TxCarrierLost = 0x80000000,
};


enum RxStatusBits {
	RxMulticast = 0x8000,
	RxPhysical = 0x4000,
	RxBroadcast = 0x2000,
	RxBadSymbol = 0x0020,
	RxRunt = 0x0010,
	RxTooLong = 0x0008,
	RxCRCErr = 0x0004,
	RxBadAlign = 0x0002,
	RxStatusOK = 0x0001,
};


/* Bits in RxConfig */
enum rx_mode_bits {
	AcceptErr = 0x20,
	AcceptRunt = 0x10,
	AcceptBroadcast = 0x08,
	AcceptMulticast = 0x04,
	AcceptMyPhys = 0x02,
	AcceptAllPhys = 0x01,
};


/* Bits in TxConfig */
enum tx_config_bits {
	TxIFG1 = (1 << 25),                 /* Interframe Gap Time */
	TxIFG0 = (1 << 24),                 /* Enabling these bits violates IEEE 802.3 */
	TxLoopBack = (1 << 18) | (1 << 17), /* enable loopback test mode */
	TxCRC = (1 << 16),                  /* DISABLE appending CRC to end of Tx packets */
	TxClearAbt = (1 << 0),              /* Clear abort (WO) */
	TxDMAShift = 8,                     /* DMA burst value (0-7) is shifted this many bits */
	TxRetryShift = 4,                   /* TXRR value (0-15) is shifted this many bits */
	TxVersionMask = 0x7C800000,         /* mask out version bits 30-26, 23 */
};


/* Bits in Config1 */
enum Config1Bits {
	Cfg1_PM_Enable = 0x01,
	Cfg1_VPD_Enable = 0x02,
	Cfg1_PIO = 0x04,
	Cfg1_MMIO = 0x08,
	LWAKE = 0x10,            /* not on 8139, 8139A */
	Cfg1_Driver_Load = 0x20,
	Cfg1_LED0 = 0x40,
	Cfg1_LED1 = 0x80,
	SLEEP = (1 << 1),        /* only on 8139, 8139A */
	PWRDN = (1 << 0),        /* only on 8139, 8139A */
};


/* Bits in Config3 */
enum Config3Bits {
	Cfg3_FBtBEn    = (1 << 0), /* 1 = Fast Back to Back */
	Cfg3_FuncRegEn = (1 << 1), /* 1 = enable CardBus Function registers */
	Cfg3_CLKRUN_En = (1 << 2), /* 1 = enable CLKRUN */
	Cfg3_CardB_En  = (1 << 3), /* 1 = enable CardBus registers */
	Cfg3_LinkUp    = (1 << 4), /* 1 = wake up on link up */
	Cfg3_Magic     = (1 << 5), /* 1 = wake up on Magic Packet (tm) */
	Cfg3_PARM_En   = (1 << 6), /* 0 = software can set twister parameters */
	Cfg3_GNTSel    = (1 << 7), /* 1 = delay 1 clock from PCI GNT signal */
};


/* Bits in Config4 */
enum Config4Bits {
	LWPTN = (1 << 2),	/* not on 8139, 8139A */
};


/* Bits in Config5 */
enum Config5Bits {
	Cfg5_PME_STS     = (1 << 0), /* 1 = PCI reset resets PME_Status */
	Cfg5_LANWake     = (1 << 1), /* 1 = enable LANWake signal */
	Cfg5_LDPS        = (1 << 2), /* 0 = save power when link is down */
	Cfg5_FIFOAddrPtr = (1 << 3), /* Realtek internal SRAM testing */
	Cfg5_UWF         = (1 << 4), /* 1 = accept unicast wakeup frame */
	Cfg5_MWF         = (1 << 5), /* 1 = accept multicast wakeup frame */
	Cfg5_BWF         = (1 << 6), /* 1 = accept broadcast wakeup frame */
};


enum RxConfigBits {
	RxCfgFIFOShift = 13,                      /* rx fifo threshold */
	RxCfgFIFONone = (7 << RxCfgFIFOShift),
	RxCfgDMAShift = 8,                        /* Max DMA burst */
	RxCfgDMAUnlimited = (7 << RxCfgDMAShift),
	RxCfgRcv8K = 0,                           /* rx ring buffer length */
	RxCfgRcv16K = (1 << 11),
	RxCfgRcv32K = (1 << 12),
	RxCfgRcv64K = (1 << 11) | (1 << 12),
	RxNoWrap = (1 << 7),                      /* Disable packet wrap at end of Rx buffer */
};


/* Twister tuning parameters from RealTek - completely undocumented, but required to tune bad links */
enum CSCRBits {
	CSCR_LinkOKBit = 0x0400,
	CSCR_LinkChangeBit = 0x0800,
	CSCR_LinkStatusBits = 0x0f000,
	CSCR_LinkDownOffCmd = 0x003c0,
	CSCR_LinkDownCmd = 0x0f3c0,
};


enum Cfg9346Bits {
	Cfg9346_Lock = 0x00,
	Cfg9346_Unlock = 0xC0,
};


#define PARA78_default	0x78fa8388
#define PARA7c_default  0xcb38de43
#define PARA7c_xxx      0xcb38de43


static const unsigned long param[4][4] = {
	{0xcb39de43, 0xcb39ce43, 0xfb38de03, 0xcb38de43},
	{0xcb39de43, 0xcb39ce43, 0xcb39ce83, 0xcb39ce83},
	{0xcb39de43, 0xcb39ce43, 0xcb39ce83, 0xcb39ce83},
	{0xbb39de43, 0xbb39ce43, 0xbb39ce83, 0xbb39ce83}
};


/* Threshold is bytes transferred to chip before transmission starts */
#define TX_FIFO_THRESH   256       /* In bytes, rounded down to 32 byte units */

/* The following settings are log_2(bytes)-4:  0 == 16 bytes .. 6==1024, 7==end of packet */
#define RX_FIFO_THRESH  7        /* Rx buffer level before first PCI xfer  */
#define RX_DMA_BURST    7        /* Maximum PCI burst, '6' is 1024 */
#define TX_DMA_BURST    6        /* Maximum PCI burst, '6' is 1024 */
#define TX_RETRY        8        /* 0-15.  retries = 16 + (TX_RETRY * 16) */
#define TX_TIMEOUT     (6 * HZ)  /* Time in jiffies before concluding the transmitter is hung. */


enum {
	HAS_MII_XCVR = 0x010000,
	HAS_CHIP_XCVR = 0x020000,
	HAS_LNK_CHNG = 0x040000,
};


#define RTL_MIN_IO_SIZE  0x80
#define RTL8139B_IO_SIZE 256

#define RTL8129_CAPS	HAS_MII_XCVR
#define RTL8139_CAPS	HAS_CHIP_XCVR|HAS_LNK_CHNG


typedef enum {
	RTL8139 = 0,
	RTL8139_CB,
	SMC1211TX,
	DELTA8139,
	ADDTRON8139,
	DFE538TX,
	DFE690TXD,
	FE2000VX,
	ALLIED8139,
	RTL8129,
} board_t;

#if 0
static struct {
	const char *name;
	u32        hw_flags;
} board_info[] = {
	{ "RealTek RTL8139 Fast Ethernet",             RTL8139_CAPS },
	{ "RealTek RTL8139B PCI/CardBus",              RTL8139_CAPS },
	{ "SMC1211TX EZCard 10/100 (RealTek RTL8139)", RTL8139_CAPS },
	{ "Delta Electronics 8139 10/100BaseTX",       RTL8139_CAPS },
	{ "Addtron Technolgy 8139 10/100BaseTX",       RTL8139_CAPS },
	{ "D-Link DFE-538TX (RealTek RTL8139)",        RTL8139_CAPS },
	{ "D-Link DFE-690TXD (RealTek RTL8139)",       RTL8139_CAPS },
	{ "AboCom FE2000VX (RealTek RTL8139)",         RTL8139_CAPS },
	{ "Allied Telesyn 8139 CardBus",               RTL8139_CAPS },
	{ "RealTek RTL8129",                           RTL8129_CAPS },
};
#endif

static pci_id_t rtl8139_pci_tbl[] = {
	{0x10ec, 0x8139, PCI_ANY, PCI_ANY },
	{0x10ec, 0x8138, PCI_ANY, PCI_ANY },
	{0x1113, 0x1211, PCI_ANY, PCI_ANY },
	{0x1500, 0x1360, PCI_ANY, PCI_ANY },
	{0x4033, 0x1360, PCI_ANY, PCI_ANY },
	{0x1186, 0x1300, PCI_ANY, PCI_ANY },
	{0x1186, 0x1340, PCI_ANY, PCI_ANY },
	{0x13d1, 0xab06, PCI_ANY, PCI_ANY },
	{0x1259, 0xa117, PCI_ANY, PCI_ANY },

	{PCI_ANY, 0x8139, 0x10ec, 0x8139 },
	{PCI_ANY, 0x8139, 0x1186, 0x1300 },
	{PCI_ANY, 0x8139, 0x13d1, 0xab06 },
	{0,}
};


/*
 * Chip types
 */


typedef enum {
	CH_8139 = 0,
	CH_8139_K,
	CH_8139A,
	CH_8139B,
	CH_8130,
	CH_8139C,
} chip_t;

enum chip_flags {
	HasHltClk = (1 << 0),
	HasLWake = (1 << 1),
};

/* directly indexed by chip_t, above */
const static struct {
	const char *name;
	u8 version;        /* from RTL8139C docs */
	u32 RxConfigMask;  /* should clear the bits supported by this chip */
	u32 flags;
} rtl_chip_info[] = {
	{ "RTL-8139",       0x40, 0xf0fe0040, HasHltClk },
	{ "RTL-8139 rev K", 0x60, 0xf0fe0040, HasHltClk },
	{ "RTL-8139A",      0x70, 0xf0fe0040, HasHltClk },
	{ "RTL-8139B",      0x78, 0xf0fc0040, HasLWake  },
	{ "RTL-8130",       0x7C, 0xf0fe0040, HasLWake  },
	{ "RTL-8139C",      0x74, 0xf0fc0040, HasLWake  },
	{ NULL, }
};


/*
 * Statistics
 */


struct rtl_extra_stats {
	unsigned long early_rx;
	unsigned long tx_buf_mapped;
	unsigned long tx_timeouts;
	unsigned long rx_lost_in_ring;
};


/*
 * MII definitions
 */


static char mii_2_8139_map[8] = {
	BasicModeCtrl,
	BasicModeStatus,
	0,
	0,
	NWayAdvert,
	NWayLPAR,
	NWayExpansion,
	0
};


/*
 * EEPROM macros
 */


/*  EEPROM_Ctrl bits */
#define EE_SHIFT_CLK  0x04             /* EEPROM shift clock */
#define EE_CS         0x08             /* EEPROM chip select */
#define EE_DATA_WRITE 0x02             /* EEPROM chip data in */
#define EE_WRITE_0		0x00
#define EE_WRITE_1		0x02
#define EE_DATA_READ  0x01             /* EEPROM chip data out */
#define EE_ENB        (0x80 | EE_CS)

/*
 * Delay between EEPROM clock transitions.
 * No extra delay is needed with 33Mhz PCI, but 66Mhz may change this.
 */
#define eeprom_delay()	readl(ee_addr)

/* The EEPROM commands include the alway-set leading bit */
#define EE_WRITE_CMD	(5)
#define EE_READ_CMD		(6)
#define EE_ERASE_CMD	(7)

struct rtl8139_private {
	void *mmio_addr;
	int drv_flags;
	pci_device_t *pci_dev;
	netif_stat_t stats;
	unsigned char *rx_ring;
	unsigned int cur_rx;                /* Index into the Rx buffer of next Rx pkt. */
	unsigned int tx_flag;
	unsigned long cur_tx;
	unsigned long dirty_tx;
	unsigned char *tx_buf[NUM_TX_DESC];	/* Tx bounce buffers */
	unsigned char *tx_bufs;             /* Tx bounce buffer region */
	addr_t rx_ring_dma;
	addr_t tx_bufs_dma;
	signed char phys[4];                /* MII device addresses */
	char twistie, twist_row, twist_col;	/* Twister tune state */
	unsigned int default_port:4;        /* Last dev->if_port value */
	unsigned int medialock:1;           /* Don't sense media type */
	spinlock_t lock;
	spinlock_t xmit_lock;
	chip_t chipset;
	u32 rx_config;
	struct rtl_extra_stats xstats;
	int time_to_die;
	struct mii_if_info mii;
	int irq;
	int stopped;
};


#define RTL_R8(reg)         hal_inb((ioaddr) + (reg))
#define RTL_R16(reg)        hal_inw((ioaddr) + (reg))
#define RTL_R32(reg)        ((u32)hal_inl((ioaddr) + (reg)))
#define RTL_W8(reg, val8)   hal_outb((ioaddr) + (reg), val8)
#define RTL_W16(reg, val16) hal_outw((ioaddr) + (reg), val16)
#define RTL_W32(reg, val32) hal_outl((ioaddr) + (reg), val32)
#define RTL_W8_F            RTL_W8
#define RTL_W16_F           RTL_W16
#define RTL_W32_F           RTL_W32

#define readb(addr)         hal_inb(addr)
#define readw(addr)         hal_inw(addr)
#define readl(addr)         hal_inl(addr)
#define writeb(val,addr)    hal_outb(addr, val)
#define writew(val,addr)    hal_outw(addr, val)
#define writel(val,addr)    hal_outl(addr, val)


#define ETH_ZLEN    60



#endif
