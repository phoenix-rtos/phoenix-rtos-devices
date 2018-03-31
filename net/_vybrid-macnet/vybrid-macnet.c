/**
 * Vybrid-MACNET driver
 *
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @file
 *
 * @copyright 2014 Phoenix Systems
 *
 * @author Horacio Mijail Anton Quiles <horacio.anton@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

 /*MAC-NET seems to be mostly compatible with the Fast Ethernet Controller (FEC) found in
  * other Freescale/Motorola uC (Kinetis, ColdFire, i.MX, ...)*/

/* Things to do to improve ethernet performance, from easiest to hardest:
 * --use hardware acceleration (checksum calculations, padding for alignment): easy (change regs RACC, TACC in Vybrid,
 * 		and find a way to source LWIP opts from a file in the driver directory or bsp)
 * --enable uncached memory/disable explicit cache management for buffers: immediate if the cache is working correctly
 * --use zero copy: medium (easy changes, but official support is not clear, so needs careful examination of support /
 * 		stability / what happens inside of LWIP. Also, zero copy might make explicit cache management the best option,
 * 		to keep the single copy cached )
 * --apply lessons learned from DPDK (minimize standard processing that is not only unwanted but counterproductive:
 * 		minimize copies + minimize syscalls/context switches + exploit caches by allowing batching; allow userland
 * 		to bypass network stack; allow polling mode): harder, and implies purposely-coded programs, and so a new API for them
 */

#include <hal/if.h>
#include <main/if.h>
#include <vm/if.h>
#include <dev/if.h>
#include <net/if.h>
#include <stdbool.h>

#include <hal/MVF50GS10MK50.h>
#include <hal/cortex-a.h>
#include "if.h"

#ifdef CONFIG_NET

#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/ethip6.h"
#include "netif/etharp.h"
#include "lwip/tcpip.h"

#ifdef DEBUG_BUILD
	#define DEBUG_BOOL 1
	#define debug_printf(...)	main_printf(ATTR_FAILURE, __VA_ARGS__)
	#define DEBUG_FUNC			__attribute__((unused))
	#pragma message  "*** DEBUG BUILD ***"
#else
	#define DEBUG_BOOL 0
	/* safe NOP macro*/
	#define debug_printf(...)	do {} while (0)
	#define DEBUG_FUNC			__attribute__((unused)) __attribute__((warning("A debug function is still being used!")))
#endif


/* XXX CACHE PROBLEMS
 * THE ROOT PROBLEM: BDs are structs in memory which are read and written by both us and MACNET, so every access implies a cache invalidate or flush.
 * THE BEST BUT NONWORKING SOLUTION: access the BDs through a non-caching mapping.
 * WHY BEST? because we avoid useless cache pressure, and avoid having to care about cache management.
 * WHY NONWORKING? because the BDs get corrupted after a little time
 * HOW TO RECOGNIZE THAT IT IS NOT WORKING? Frame tx fails after a couple of frames (usually <5). What happens is that the tx BDs get corrupted, and MACNET stops working when it touches a bad BD.
 * The IRQ stops triggering, no frames are received, and the tx frames ring gets full because it is not emptied by MACNET.
 * So after a while there will be error messages about lack of free TxBDs.
 *
 * THE WORSE BUT WORKING SOLUTION: manage explicitly the cache before/after every access to the BDs.
 * WHY WORSE: because this causes constant, useless cache pressure, which probably doesn't affect the MACNET driver but does affect everyone else.
 *
 * STRANGE HAPPENINGS:
 * 		- the noncaching mapping works initially, since the BDs are seen as correctly initialized for some time. But they get corrupted after a few accesses.
 * 		- even if using explicit cache mgmt does work, if we *add* the noncaching mapping, the corruption appears again
 *		- if we use the noncaching mapping for *both* BDs and buff pointers (which are alocated contiguous to the BDs), everything works OK.
 *			(but we don't know WHY or WHAT is happening, so it could be accidental, so we don't want to rely on that)
 *		- "non-working" binaries sometimes work! To make sure that a working binary really is good, try after a power cycle.
 *		- if we run a working binary in the Vybrid, from that moment on until a *full power cycle* (not only reset!) a normally nonworking binary WILL work.
 *			So, to recognize a nonworking binary, first power off and on again the board. Also, take into account that sometimes a non-working binary WILL work by itself,
 *			but will fail after a power cycle. Which probably means that "bad" binaries fail to set something which "good" binaries do set; and sometimes that setting is
 *			randomly well set anyway, so even a "bad" binary can work randomly.
 */

#define USE_EXPLICIT_CACHE_MGMT_FOR_BDS

//#define USE_NONCACHED_BDS					/* avoid cache pressure from BDs, which should not be cached anyway*/
//#define USE_NONCACHED_BDS_AND_BUFP		/* use noncached buffer pointers (useless, but workaround for cache problems with BDs)*/

/*if the main debug flag is off, make sure that all the subflags are off too*/
#ifdef DEBUG_BUILD
	/*particular aspects to debug*/

	/* show lifecycle of rx frames*/
//	#define DEBUG_FRAMES_RX		true
	#define DEBUG_FRAMES_RX		false

	/* show lifecycle of tx frames*/
//	#define DEBUG_FRAMES_TX		true
	#define DEBUG_FRAMES_TX		false

	/* warn when "holes" appear in the bd rings*/
//	#define DEBUG_BUBBLES		true
	#define DEBUG_BUBBLES		false

	/* check for corruption of BDs*/
//	#define DEBUG_CACHE_PARANOIC	true
	#define DEBUG_CACHE_PARANOIC	false

	/* show when the ISR is called*/
//	#define DEBUG_ISR	true
	#define DEBUG_ISR	false

	/*keep repeating the network hardware tests forever*/
//	#define DEBUG_TEST_FOREVER	true
	#define DEBUG_TEST_FOREVER	false

	/*show the evolution of the hardware tests*/
//	#define DEBUG_TEST_STEPS	true
	#define DEBUG_TEST_STEPS	false

	/*show the evolution of the link config thread */
//	#define DEBUG_LINKTHREAD	true
	#define DEBUG_LINKTHREAD	false

	/*show when multiple received frames are processed at once by the rx thread */
//	#define DEBUG_RXROW true
	#define DEBUG_RXROW false

#else
	#define DEBUG_FRAMES_RX			false
	#define DEBUG_FRAMES_TX			false
	#define DEBUG_BUBBLES			false
	#define DEBUG_CACHE_PARANOIC	false
	#define DEBUG_ISR				false
	#define DEBUG_TEST_FOREVER		false
	#define DEBUG_LINKTHREAD		false
	#define DEBUG_TEST_STEPS		false
	#define DEBUG_RXROW false

#endif



/** (Legacy) Buffer Descriptors (BD); MAC-NET assumes this structure*/
struct macnet_bd {
	uint16_t data_length;		/**< frame length in bytes */
	uint16_t status;			/**< BD's status*/
	uint32_t data_pointer_phys;	/**< frame's buffer's address, physical */
#ifdef ENHANCED_BUFFERS
	/*can't use anonymous structure to syntethize the enhanced BD from the legacy BD, because that is a GNU extension */
	/** Enhanced Descriptor's fields */
	uint16_t udma_status;
	uint16_t config;
	uint32_t reserved1;
	uint16_t bdu;
	uint32_t timestamp;
	uint32_t reserved2[2];
#endif
};// __attribute__((__aligned__(SIZE_CACHE_LINE)));

#define MDIO_NAME_LEN 32
#define PHY_MAX_ADDR 32

/** Private data for MACNET management
 *
 * The BDs contain physical address pointers to the buffers, because that is what MACNET needs.
 * For our access, the virtual addresses of the buffers are also kept in the xbuffs_virt arrays.
 *
 * The bd_base physical addresses are not stored because they are only needed once, when init'ing
 * the MACNET regs.
 * */
struct macnet_priv {
	spinlock_t lock;
	event_t rx_evt;

	ENET_Type	*regs;			/**< base address for the MACNET instance registers */
	addr_t		ENET_addr;		/**< physical MACNET addr, for identification */
	u64 MAC_addr;

	struct macnet_bd *	rbd_base;	/**< First Rx BD */
	u8 * * rbuffs_virt;			/**< array of virtual addresses for the Rx buffers*/
	struct macnet_bd * tbd_base;	/**< First Tx BD */
	u8 * * tbuffs_virt;			/**< array of virtual addresses for the Tx buffers*/
	struct pbuf	* tbd_pbuf;		/**< array of pointers to pbufs; relates each Tx BD with a Tx pbuf */
	u8 rbd_index;				/**< next Rx BD to use */
	u8 tbd_index;			/**< XXX next Tx BD in which to put an outgoing frame; handled by macnet_send*/
	u8 PHY_addr;
	u16 PHY_physid2;
#if DEBUG_BOOL == 1
	struct macnet_bd *	rbd_base_backup;
	struct macnet_bd *	tbd_base_backup;
#endif
};

enum PHYSID2_known_values {
	PHYSID2_KSZ8041NL 	=	0x1513,
	PHYSID2_DP83849C	=	0x5CA2,
	PHYSID2_KSZ8081RNA	=	0x1560,
};


/** Number of BDs for RX. As many buffers of size MACNET_MAX_PKT_SIZE+FEC_MAX_PKT_SIZE_MARGIN will be allocated forever. */
#define MACNET_RBD_NUM		5
/** Number of BDs for TX. The actual buffers will be allocated and free'd as needed. */
#define MACNET_TBD_NUM		5

/** Ethernet rx frame size limit in memory
 * If a rx'ed frame was larger than this, MAC-NET would spill it off into multiple BDs. We rather try to keep one BD = one frame.
 */
#define MTU					1500									/* standard, RFC894 */
#define ETH_FRAME_OVERHEAD_VLAN_TAGS	22
#define MACNET_MAX_PKT_SIZE	MTU + ETH_FRAME_OVERHEAD_VLAN_TAGS		/* Received frames bigger than this setting will cause a babbling error and frame length violation, but are serviced*/
#define MACNET_MAX_PKT_SIZE_TRUNC	MACNET_MAX_PKT_SIZE					/*length at which packets get truncated and must be discarded*/

/* Receive BD status bits */
#define MACNET_RBD_EMPTY	0x8000	/* Receive BD status: Buffer is empty */
#define MACNET_RBD_WRAP	0x2000	/* Receive BD status: Last BD in ring */
/* Receive BD status: Buffer is last in frame (useless here!) */
#define MACNET_RBD_LAST	0x0800
#define MACNET_RBD_MISS	0x0100	/* Receive BD status: Miss bit for prom mode */
/* Receive BD status: The received frame is broadcast frame */
#define MACNET_RBD_BC	0x0080
/* Receive BD status: The received frame is multicast frame */
#define MACNET_RBD_MC	0x0040
#define MACNET_RBD_LG	0x0020	/* Receive BD status: Frame length violation */
#define MACNET_RBD_NO	0x0010	/* Receive BD status: Nonoctet align frame */
#define MACNET_RBD_CR	0x0004	/* Receive BD status: CRC error */
#define MACNET_RBD_OV	0x0002	/* Receive BD status: Receive FIFO overrun */
#define MACNET_RBD_TR	0x0001	/* Receive BD status: Frame is truncated */
#define MACNET_RBD_ERR	(MACNET_RBD_LG | MACNET_RBD_NO | MACNET_RBD_CR | \
			MACNET_RBD_OV | MACNET_RBD_TR)

/* Transmit BD status bits */
#define MACNET_TBD_READY	0x8000	/* Tansmit BD status: Buffer is ready */
#define MACNET_TBD_WRAP	0x2000	/* Tansmit BD status: Mark as last BD in ring */
#define MACNET_TBD_LAST	0x0800	/* Tansmit BD status: Buffer is last in frame */
#define MACNET_TBD_TC	0x0400	/* Tansmit BD status: *ADD* the CRC */
#define MACNET_TBD_ABC	0x0200	/* Tansmit BD status: Append bad CRC */
#define MACNET_TBD_TO1 0x4000	/* Transmit BD status: TO1 flag		*/

/* MII-related definitios */
#define MACNET_MII_DATA_ST		0x40000000	/* Start of frame delimiter */
#define MACNET_MII_DATA_OP_RD	0x20000000	/* Perform a read operation */
#define MACNET_MII_DATA_OP_WR	0x10000000	/* Perform a write operation */
#define MACNET_MII_DATA_PA_MSK	0x0f800000	/* PHY Address field mask */
#define MACNET_MII_DATA_RA_MSK	0x007c0000	/* PHY Register field mask */
#define MACNET_MII_DATA_TA		0x00020000	/* Turnaround */
#define MACNET_MII_DATA_DATAMSK	0x0000ffff	/* PHY data field */

#define MACNET_MII_DATA_RA_SHIFT	18	/* MII Register address bits */
#define MACNET_MII_DATA_PA_SHIFT	23	/* MII PHY address bits */




/* Generic MII registers. */
#define MII_BMCR		0x00	/* Basic mode control register */
#define MII_BMSR		0x01	/* Basic mode status register  */
#define MII_PHYSID1		0x02	/* PHYS ID 1                   */
#define MII_PHYSID2		0x03	/* PHYS ID 2                   */
#define MII_ADVERTISE		0x04	/* Advertisement control reg   */
#define MII_LPA			0x05	/* Link partner ability reg    */
#define MII_ANER		0x06	/* Expansion register          */
#define MII_CTRL1000		0x09	/* 1000BASE-T control          */
#define MII_STAT1000		0x0a	/* 1000BASE-T status           */
#define	MII_MMD_CTRL		0x0d	/* MMD Access Control Register */
#define	MII_MMD_DATA		0x0e	/* MMD Access Data Register */
#define MII_ESTATUS		0x0f	/* Extended Status             */

/*vendor-specific (>0x0F)*/
/* PHY model is detected at runtime, so can't use some ifdef */
/* MII registers for DP83849C */
#define MII_PHYSTS		0x10
#define MII_PCSR		0x16
#define MII_RBR			0x17
#define MII_PHYCR		0x19
#define MII_CDCTRL1		0x1b
#define MII_10BTSCR		0x1a



/* Basic mode control register, 0x0 */
#define BMCR_RESV		0x003f	/* Unused...                   */
//#define BMCR_SPEED1000		0x0040	/* MSB of Speed (1000)         */
#define BMCR_CTST		0x0080	/* Collision test              */
#define BMCR_FULLDPLX		0x0100	/* Full duplex                 */
#define BMCR_ANRESTART		0x0200	/* Auto negotiation restart    */
#define BMCR_ISOLATE		0x0400	/* Isolate data paths from MII */
#define BMCR_PDOWN		0x0800	/* Enable low power state      */
#define BMCR_ANENABLE		0x1000	/* Enable auto negotiation     */
#define BMCR_SPEED100		0x2000	/* Select 100Mbps              */
#define BMCR_LOOPBACK		0x4000	/* TXD loopback bits           */
#define BMCR_RESET		0x8000	/* Reset to default state      */

/* Basic mode status register, 0x1 */
#define BMSR_ERCAP		0x0001	/* Ext-reg capability          */
#define BMSR_JCD		0x0002	/* Jabber detected             */
#define BMSR_LSTATUS		0x0004	/* Link status                 */
#define BMSR_ANEGCAPABLE	0x0008	/* Able to do auto-negotiation */
#define BMSR_RFAULT		0x0010	/* Remote fault detected       */
#define BMSR_ANEGCOMPLETE	0x0020	/* Auto-negotiation complete   */
#define BMSR_RESV		0x00c0	/* Unused...                   */
#define BMSR_ESTATEN		0x0100	/* Extended Status in R15      */
#define BMSR_100HALF2		0x0200	/* Can do 100BASE-T2 HDX       */
#define BMSR_100FULL2		0x0400	/* Can do 100BASE-T2 FDX       */
#define BMSR_10HALF		0x0800	/* Can do 10mbps, half-duplex  */
#define BMSR_10FULL		0x1000	/* Can do 10mbps, full-duplex  */
#define BMSR_100HALF		0x2000	/* Can do 100mbps, half-duplex */
#define BMSR_100FULL		0x4000	/* Can do 100mbps, full-duplex */
#define BMSR_100BASE4		0x8000	/* Can do 100mbps, 4k packets  */

/* Advertisement control register. */
#define ADVERTISE_SLCT		0x001f	/* Selector bits               */
#define ADVERTISE_CSMA		0x0001	/* Only selector supported     */
#define ADVERTISE_10HALF	0x0020	/* Try for 10mbps half-duplex  */
#define ADVERTISE_1000XFULL	0x0020	/* Try for 1000BASE-X full-duplex */
#define ADVERTISE_10FULL	0x0040	/* Try for 10mbps full-duplex  */
#define ADVERTISE_1000XHALF	0x0040	/* Try for 1000BASE-X half-duplex */
#define ADVERTISE_100HALF	0x0080	/* Try for 100mbps half-duplex */
#define ADVERTISE_1000XPAUSE	0x0080	/* Try for 1000BASE-X pause    */
#define ADVERTISE_100FULL	0x0100	/* Try for 100mbps full-duplex */
#define ADVERTISE_1000XPSE_ASYM	0x0100	/* Try for 1000BASE-X asym pause */
#define ADVERTISE_100BASE4	0x0200	/* Try for 100mbps 4k packets  */
#define ADVERTISE_PAUSE_CAP	0x0400	/* Try for pause               */
#define ADVERTISE_PAUSE_ASYM	0x0800	/* Try for asymetric pause     */
#define ADVERTISE_RESV		0x1000	/* Unused...                   */
#define ADVERTISE_RFAULT	0x2000	/* Say we can detect faults    */
#define ADVERTISE_LPACK		0x4000	/* Ack link partners response  */
#define ADVERTISE_NPAGE		0x8000	/* Next page bit               */

#define ADVERTISE_FULL		(ADVERTISE_100FULL | ADVERTISE_10FULL | \
				  ADVERTISE_CSMA)
#define ADVERTISE_ALL		(ADVERTISE_10HALF | ADVERTISE_10FULL | \
				  ADVERTISE_100HALF | ADVERTISE_100FULL)

/* Link partner ability register. */
#define LPA_SLCT		0x001f	/* Same as advertise selector  */
#define LPA_10HALF		0x0020	/* Can do 10mbps half-duplex   */
#define LPA_1000XFULL		0x0020	/* Can do 1000BASE-X full-duplex */
#define LPA_10FULL		0x0040	/* Can do 10mbps full-duplex   */
#define LPA_1000XHALF		0x0040	/* Can do 1000BASE-X half-duplex */
#define LPA_100HALF		0x0080	/* Can do 100mbps half-duplex  */
#define LPA_1000XPAUSE		0x0080	/* Can do 1000BASE-X pause     */
#define LPA_100FULL		0x0100	/* Can do 100mbps full-duplex  */
#define LPA_1000XPAUSE_ASYM	0x0100	/* Can do 1000BASE-X pause asym*/
#define LPA_100BASE4		0x0200	/* Can do 100mbps 4k packets   */
#define LPA_PAUSE_CAP		0x0400	/* Can pause                   */
#define LPA_PAUSE_ASYM		0x0800	/* Can pause asymetrically     */
#define LPA_RESV		0x1000	/* Unused...                   */
#define LPA_RFAULT		0x2000	/* Link partner faulted        */
#define LPA_LPACK		0x4000	/* Link partner acked us       */
#define LPA_NPAGE		0x8000	/* Next page bit               */

#define LPA_DUPLEX		(LPA_10FULL | LPA_100FULL)
#define LPA_100			(LPA_100FULL | LPA_100HALF | LPA_100BASE4)

/* Expansion register for auto-negotiation. */
#define EXPANSION_NWAY		0x0001	/* Can do N-way auto-nego      */
#define EXPANSION_LCWP		0x0002	/* Got new RX page code word   */
#define EXPANSION_ENABLENPAGE	0x0004	/* This enables npage words    */
#define EXPANSION_NPCAPABLE	0x0008	/* Link partner supports npage */
#define EXPANSION_MFAULTS	0x0010	/* Multiple faults detected    */
#define EXPANSION_RESV		0xffe0	/* Unused...                   */

#define ESTATUS_1000_TFULL	0x2000	/* Can do 1000BT Full          */
#define ESTATUS_1000_THALF	0x1000	/* Can do 1000BT Half          */

/* N-way test register. */
#define NWAYTEST_RESV1		0x00ff	/* Unused...                   */
#define NWAYTEST_LOOPBACK	0x0100	/* Enable loopback for N-way   */
#define NWAYTEST_RESV2		0xfe00	/* Unused...                   */

/* Flow control flags */
#define FLOW_CTRL_TX		0x01
#define FLOW_CTRL_RX		0x02

/* MMD Access Control register fields */
#define MII_MMD_CTRL_DEVAD_MASK	0x1f	/* Mask MMD DEVAD*/
#define MII_MMD_CTRL_ADDR	0x0000	/* Address */
#define MII_MMD_CTRL_NOINCR	0x4000	/* no post increment */
#define MII_MMD_CTRL_INCR_RDWT	0x8000	/* post increment on reads & writes */
#define MII_MMD_CTRL_INCR_ON_WT	0xC000	/* post increment on writes only */

/* PHYCR flags*/
#define PHYCR_BIST_START	(1<<8)	/* 1 = start*/
#define PHYCR_BIST_STATUS	(1<<9)	/* 1 = test OK*/
#define PHYCR_PSR15			(1<<10)
#define PHYCR_BIST_FE		(1<<11)	/* force BIST error, self-clearing*/
#define PHYCR_AUTOMDIX		(1ul<<15) /* autodetect crossed cables*/

#define PHYSTS_LB_STATUS	(1<<3)	/*loopback status*/
#define PHYSTS_AUTONEG_COMPLETE	(1<<4)	/* autonegotiation process finished*/

#define ANER_LP_AN_ABLE		1		/*is the Link Parter able to AutoNegotiate?*/
#define ANER_PDF			(1<<4)	/*Parallel Detection Fault*/

#define RBR_PMD_LOOP		(1<<8)	/* "PMD loopback" - RMII specific?*/

#define PCSR_FORCE100OK		(1<<5)	/* force 100mbps link up */

#define TenBTSCR_FORCE10OK	(1<<6)	/* force 10mbps link up*/

/* MII_LPA */
#define PHY_ANLPAR_PSB_802_3	0x0001
#define PHY_ANLPAR_PSB_802_9	0x0002


/* phy seed setup */
#define AUTO			99
#define _1000BASET		1000
#define _100BASET		100
#define _10BASET		10
#define HALF			22
#define FULL			44


#define TP_ETHTYPE_LENGTH "\x00\x54"
#define TP_LENGTH	12+2+70
#define TP_ETHTYPE "\x05\xDD" /*Eth frame type 0x05DD is undefined*/
#define TP_10DIG "0123456789"
#define TEST_PACKET "ddddddssssss" TP_ETHTYPE_LENGTH \
	TP_10DIG TP_10DIG TP_10DIG TP_10DIG TP_10DIG TP_10DIG TP_10DIG


/** Wait for the MII interrupt
 *
 * @param macnet
 * @return	EIO if there was a timeout
 */
static int macnet_waitMII(struct macnet_priv *macnet)
{
	/* wait for the MII interrupt */
	u16 i=0;
	while (!(macnet->regs->EIR & ENET_EIR_MII_MASK)) {
		/*busywait because this should return quickly, and MDI is only used during setup*/
		if(++i==0){
			return -EIO;	/*timeout*/
		}
	}

	/* clear MII interrupt bit */
	macnet->regs->EIR = ENET_EIR_MII_MASK;
	return EOK;

}

/** Read a register from PHY
 *
 * @param fec		contains the PHY address
 * @param regAddr	PHY register to read
 * @param val		value read from the register
 * @return			EIO if there was a problem
 */
static int macnet_mdioRead(struct macnet_priv *macnet, u8 regAddr, u16 * val)
{
	int err;

	macnet->regs->EIR = ENET_EIR_MII_MASK; /*ensure that the "MII-completed" bit is cleared*/
	macnet->regs->MMFR = ENET_MMFR_ST(1) | ENET_MMFR_OP(2) | ENET_MMFR_TA(2) | ENET_MMFR_RA(regAddr) | ENET_MMFR_PA(macnet->PHY_addr);

	err = macnet_waitMII(macnet);

	/* it's now safe to read the PHY's register */
	*val = macnet->regs->MMFR;
	return err;
}

static int macnet_mdioWrite(struct macnet_priv *macnet, u8 regAddr, u16 data)
{
	int err;

	macnet->regs->EIR = ENET_EIR_MII_MASK;  /*ensure that the MII-completed bit is cleared*/
	macnet->regs->MMFR = ENET_MMFR_ST(1) | ENET_MMFR_OP(1) | ENET_MMFR_TA(2) | ENET_MMFR_RA(regAddr) | ENET_MMFR_PA(macnet->PHY_addr) | ENET_MMFR_DATA(data);

	err = macnet_waitMII(macnet);

	return err;
}

/** Returns first multiple of y which is >=x */
#define roundup(x, y)		({	typeof (x) x_ = (x);		\
								typeof (y) y_ = (y);		\
								/*returns*/ (((x_ + (y_ - 1)) / y_) * y_); })

static void * macnet_kmallocAligned(unsigned int size)
/* NOTE: according to the Manual, Tx BDs, Tx buffers and Rx BDs require 8-byte alignment,
 * and the Rx buffers require 16-byte.
 * MQX uses 16-byte alignment for everything.
 * uboot used 64 bytes, saying it is "current upper bound and default for ARM's L1 cache"*/
{
	const u8 alignment_bytes = 16;
	unsigned int size_aligned;
	size_aligned=size+alignment_bytes-1;	/*the offset needed is yet unknown but might be as big as <alignment>-1, so <size> has to be extended as much*/
	void * ptr=vm_kmalloc(size_aligned);
	void * ptr_aligned=(void *)roundup((addr_t)ptr, alignment_bytes);
	/*up to <alignment_bytes>-1 are wasted*/
	return ptr_aligned;
}

/** print out all the BDs and their status
 * Note that we depend on the WRAP flag to detect the end of the ring, as MACNET does.
 * So a corrupted flag will cause different numbers of BDs than we allocated.
 */
static void macnet_bdReport(struct macnet_priv * macnet)
{
	u8 i;
	struct macnet_bd * bd;
	bool finished = false;
	for(i = 0; finished == false ; i++){
		bd = &macnet->rbd_base[i];
		#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
			hal_cpuInvalCache(bd, sizeof(struct macnet_bd));
		#endif
		finished = bd->status & MACNET_RBD_WRAP;
		debug_printf("rbd %d status %08X\n", i, bd->status);
	}
	finished = false;
	for(i = 0; finished == false ; i++){
		bd = &macnet->tbd_base[i];
		#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
			hal_cpuInvalCache(bd, sizeof(struct macnet_bd));
		#endif
		finished = bd->status & MACNET_TBD_WRAP;
		debug_printf("tbd %d status %08X\n", i, bd->status);
	}
}


/** Return whether a rx BD does contain a frame or not*/
static bool macnet_RbdIsFilled(struct macnet_bd *rbd)
{
	#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
		hal_cpuInvalCache(rbd, sizeof(struct macnet_bd));
	#endif
	return ((rbd->status & MACNET_RBD_EMPTY) == 0);
}


/** count how many RxBDs are empty*/
static uint8_t macnet_rbdFreeCount(struct macnet_priv * macnet) DEBUG_FUNC;
static uint8_t macnet_rbdFreeCount(struct macnet_priv * macnet)
{
	u8 i;
	u8 count = 0;
	struct macnet_bd * bd;
	bool finished = false;
	for(i = 0; finished == false ; i++){
		bd = &macnet->rbd_base[i];
		#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
			hal_cpuInvalCache(bd, sizeof(struct macnet_bd));
		#endif
		finished = bd->status & MACNET_RBD_WRAP;
		if(!macnet_RbdIsFilled(bd)){
			count++;
		}
	}
	debug_printf("%d rbd, %d free\n", i, count);
	return count;
}


static bool macnet_TbdIsNotBusy(struct macnet_bd *tbd){
	#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
		hal_cpuInvalCache(tbd, sizeof(struct macnet_bd));
	#endif
	bool free = (tbd->status & MACNET_TBD_READY) == 0;
	return (free);
}


/** count how many TBDs are free for tx*/
static uint8_t macnet_tbdFreeCount(struct macnet_priv * macnet) DEBUG_FUNC;
static uint8_t macnet_tbdFreeCount(struct macnet_priv * macnet)
{
	u8 i;
	u8 count = 0;
	struct macnet_bd * bd;
	bool finished = false;
	for(i = 0; finished == false ; i++){
		bd = &macnet->tbd_base[i];
		#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
			hal_cpuInvalCache(bd, sizeof(struct macnet_bd));
		#endif
		finished = bd->status & MACNET_TBD_WRAP;
		if(macnet_TbdIsNotBusy(bd)){
			count++;
		}
	}
	debug_printf("%d tbd, %d free\n", i, count);
	return count;
}

/** count how many BDs are*/
static uint8_t macnet_bdCount(struct macnet_bd * bdbase) DEBUG_FUNC;
static uint8_t macnet_bdCount(struct macnet_bd * bdbase)
{
	u8 i;
	bool finished = false;
	struct macnet_bd * bd;
	for(i = 0; finished == false ; i++){
		bd = &bdbase[i];
		#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
			hal_cpuInvalCache(bd, sizeof(struct macnet_bd));
		#endif
		finished = bd->status & MACNET_TBD_WRAP; /* TBD_WRAP == RBD_WRAP */
		if(i==255)
			break;	/*255 is bad enough*/
	}
	return i;
}


static void macnet_checkBDs(struct macnet_priv *macnet)		DEBUG_FUNC;
static void macnet_checkBDs(struct macnet_priv *macnet)
{
	if(DEBUG_BOOL == 0)
		return;	/*avoid compiler warnings*/
	bool bad = false;

	u8 rbds = macnet_bdCount(macnet->rbd_base);
	if(rbds != MACNET_RBD_NUM){
		debug_printf("RBDS BAD rbds = %d\n", rbds);
		bad = true;
	}
	u8 tbds = macnet_bdCount(macnet->tbd_base);
	if(tbds != MACNET_TBD_NUM){
		debug_printf("TBDS BAD tbds = %d\n", tbds);
		bad = true;
	}
	if(bad)
		macnet_bdReport(macnet);
}


#ifndef USE_NONCACHED_BDS
	static void * macnet_getNoncachedMapping(void * vaddr, u32 size) __attribute__((unused));
	/*this way, the compiler keeps checking the function correctness even if unused*/
#endif

/* Get a new vaddr to access the given vaddr through a noncaching mapping*/
static void * macnet_getNoncachedMapping(void * vaddr, u32 size)
{
	int error;
	addr_t paddr, paddr_uncached, page_paddr;
	page_t * page;
	void * page_vaddr, * vaddr_uncached;

	/* just in case: flush the zone now to avoid the cache getting flushed later
	 * and overwriting non-cached writes*/
	hal_cpuFlushCache(vaddr, size);

	/*get the phys address*/
	error = vm_kmapResolve(vaddr, &paddr);
	assert(error == EOK);
	/* get the page containing that phys address*/
	page = vm_addr2page(paddr);
	/* get the phys addr at the beginning of the page */
	page_paddr = page->addr;
	/* get a new, noncached mapping for the page's phys addr*/
	/* the size has to be adjusted, since the address changed*/
	size += SIZE_PAGE;	/*max size that could be needed*/
	error = vm_iomap(page_paddr, size, PGHD_DEV_RW, &page_vaddr);
	assert(error == EOK);
	/* get the new vaddr from the vaddr of the page address + the last bits of the original vaddr*/
	vaddr_uncached = (void *)(((addr_t)page_vaddr) | (((addr_t)vaddr) & ~PAGE_ADDR_MASK));

	/* sanity check */
	error = vm_kmapResolve(vaddr_uncached, &paddr_uncached);
	assert(error == EOK);
	assert(paddr == paddr_uncached);

	return vaddr_uncached;
}


/** Clean up the given RxBD so it is ready for a new Rx
 *
 * @param pRbd buffer descriptor to be marked as free
 */
static void macnet_rbdClean(struct macnet_bd *pRbd)
{
	#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
		hal_cpuInvalCache(pRbd, sizeof(struct macnet_bd));
	#endif
	/*flag the bd as Empty, keeping the Wrap flag if it was already set*/
	u16 flags = (pRbd->status & MACNET_RBD_WRAP) | MACNET_RBD_EMPTY;
	pRbd->status=flags;
	pRbd->data_length=0;
	#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
		hal_cpuFlushCache(pRbd, sizeof(struct macnet_bd));
	#endif
}


/** Clean up the flags in the given TxBD so it is ready for a new Tx
 *
 * @param pTbd buffer descriptor to be marked as free
 */
static void macnet_tbdClean(struct macnet_bd *pTbd)
{
#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
	hal_cpuInvalCache(pTbd, sizeof(struct macnet_bd));
#endif
	/*clean the bd flags, but keeping the Wrap flag if it was already set*/
	u16 flags = (pTbd->status & MACNET_TBD_WRAP);
	pTbd->status=flags;
#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
	hal_cpuFlushCache(pTbd, sizeof(struct macnet_bd));
#endif
}


static void macnet_cleanBDs(struct macnet_priv * macnet)
{
	u8 i=0;
	for(i=0; i<MACNET_RBD_NUM; i++){
		macnet_rbdClean(&macnet->rbd_base[i]);
	}
	assert((macnet->rbd_base[i-1].status & MACNET_RBD_WRAP) != 0);

	for(i=0; i<MACNET_TBD_NUM; i++){
		macnet_tbdClean(&macnet->tbd_base[i]);
	}
	assert((macnet->tbd_base[i-1].status & MACNET_TBD_WRAP) != 0);

}


static int macnet_allocAndInit(struct macnet_priv *macnet)
{
	u16 size_tbds, size_tbs_v, size_rbds, size_rbs_v, size_rb, size_tb, size;
	int i = 0;
	uint8_t *data;
	struct macnet_bd *tbd_base, *rbd_base;
	int error;
	enum checkpoints {AC_START = 0, AC_TBD, AC_RBD, AC_RB, AC_TB} alloc_checkpoint = AC_START;

	do{
		/* Allocate Tx BDs */
		size_tbds = MACNET_TBD_NUM * sizeof(struct macnet_bd);
		size_tbs_v = (MACNET_TBD_NUM * sizeof(void *)); /*pointers to link each bd's to the virt. address of its buffer*/
		size = size_tbds + size_tbs_v;
		tbd_base = macnet_kmallocAligned(size); /*allocate everything at once for better alignment / minimum padding */
		if (tbd_base == NULL){
			main_printf(ATTR_ERROR,"FEC: Could not allocate tx descs");
			break;
		}
		alloc_checkpoint = AC_TBD;
		macnet->tbd_base = tbd_base;
		macnet->tbuffs_virt = (void *)&tbd_base[MACNET_TBD_NUM]; /*the array of buffer virtual addresses starts at the end of the array of bd*/

		#ifdef USE_NONCACHED_BDS
			macnet->tbd_base = macnet_getNoncachedMapping(tbd_base, size);
			#ifdef USE_NONCACHED_BDS_AND_BUFP
				macnet->tbuffs_virt = (void *)&(macnet->tbd_base[MACNET_TBD_NUM]); /*the array of buffer virtual addresses starts at the end of the array of bd*/
				hal_cpuFlushCache(macnet->tbuffs_virt, size_tbs_v);
			#endif
		#endif
		#if DEBUG_BOOL == 1
			macnet->tbd_base_backup = macnet->tbd_base;	/*for paranoic checking*/
		#endif

		/* Allocate Rx BDs */
		size_rbds = MACNET_RBD_NUM * sizeof(struct macnet_bd);
		size_rbs_v = MACNET_RBD_NUM * sizeof(void *); /*pointers to link each bd's to the virt. address of its buffer*/
		size = size_rbds + size_rbs_v;
		rbd_base = macnet_kmallocAligned(size); /*allocate everything at once for better alignment / minimum padding */
		if (rbd_base == NULL){
			main_printf(ATTR_ERROR,"FEC: Could not allocate rx descs");
			break;
		}
		alloc_checkpoint = AC_RBD;
		macnet->rbd_base = rbd_base;
		macnet->rbuffs_virt = (void *)&rbd_base[MACNET_RBD_NUM];

		#ifdef USE_NONCACHED_BDS
			macnet->rbd_base = macnet_getNoncachedMapping(rbd_base, size);
			#ifdef USE_NONCACHED_BDS_AND_BUFP
				macnet->rbuffs_virt = (void *)&(macnet->rbd_base[MACNET_RBD_NUM]);
				hal_cpuFlushCache(macnet->rbuffs_virt, size_rbs_v);
			#endif
		#endif
		#if DEBUG_BOOL == 1
			macnet->rbd_base_backup = macnet->rbd_base;	/*for paranoic checking*/
		#endif

		/* Allocate Rx buffers. */
		size_rb = MACNET_MAX_PKT_SIZE;
		for (i = 0; i < MACNET_RBD_NUM; i++) {
			data = macnet_kmallocAligned(size_rb);
			if (data==NULL) {
				main_printf(ATTR_ERROR,"%s: error allocating Rxbuf %d\n", __func__, i);
				break;
			}
			macnet->rbuffs_virt[i] = data;
			macnet->rbd_base[i].status = MACNET_RBD_EMPTY;
			macnet->rbd_base[i].data_length = 0;

			addr_t data_phys;
			error=vm_kmapResolve(data, &data_phys);
			assert(error == EOK);
			assert(data_phys != 0);
			macnet->rbd_base[i].data_pointer_phys = data_phys;
		}

		if(i<MACNET_RBD_NUM){
			break;
		}
		alloc_checkpoint = AC_RB;

		/* Mark the last RBD to close the ring. */
		macnet->rbd_base[MACNET_RBD_NUM - 1].status |= MACNET_RBD_WRAP;


		/* Allocate TX buffers. */
		size_tb = MACNET_MAX_PKT_SIZE;
		for (i = 0; i < MACNET_TBD_NUM; i++) {
			data = macnet_kmallocAligned(size_tb);
			if (data==NULL) {
				main_printf(ATTR_ERROR,"%s: error allocating Txbuf %d\n", __func__, i);
				break;
			}
			macnet->tbuffs_virt[i] = data;
			macnet->tbd_base[i].status = 0;
			macnet->tbd_base[i].data_length = 0;
			#ifdef ENHANCED_BUFFERS
				macnet->tbd_base[i].config = 0x6000u; /*generate event/error flags and tstamps*/
			#endif

			addr_t data_phys;
			int error=vm_kmapResolve(data, &data_phys);
			assert(error == EOK);
			assert(data_phys != 0);
			macnet->tbd_base[i].data_pointer_phys = data_phys;
		}

		if(i<MACNET_RBD_NUM){
			break;
		}
		alloc_checkpoint = AC_TB;

		/* Mark the last TBD to close the ring. */
		macnet->tbd_base[MACNET_TBD_NUM - 1].status |= MACNET_TBD_WRAP;

		#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
			hal_cpuFlushCache(macnet->tbd_base, size_tbds+size_tbs_v);
			hal_cpuFlushCache(macnet->rbd_base, size_rbds+size_rbs_v);
		#endif
		macnet->rbd_index = 0;
		macnet->tbd_index = 0;

		if(DEBUG_CACHE_PARANOIC)
			macnet_checkBDs(macnet);

		return EOK;
	} while(false);	/*jump-point in case of errors*/

	u8 j;
	switch(alloc_checkpoint){
		case AC_RB:	/*failed when trying to alloc tx buffer number i*/
			for(j=0;j<i;j++){
				vm_kfree(macnet->tbuffs_virt[j]);
			}
			i = MACNET_RBD_NUM; /*prepare for fall-through to rx buffers*/
			/* no break */

		case AC_RBD: /*failed when trying to alloc rx buffer number i*/
			for(j=0;j<i;j++){
				vm_kfree(macnet->rbuffs_virt[j]);
			}
			vm_kfree(rbd_base);
			/* no break */

		case AC_TBD: /*failed when rx BDs failed to be alloc'd*/
			vm_kfree(tbd_base);
			/* no break */

		case AC_START:	/*failed when tx BDs failed to be alloc'd*/
			/*nothing to free*/
			 return ENOMEM;

		default:
			assert(false);
	}

	main_printf(ATTR_ERROR, "MACNET: alloc unknown problem\n");
	return ENOMEM;
}


static void macnet_txStart(struct macnet_priv *macnet)
{
	macnet->regs->TDAR = ENET_TDAR_TDAR_MASK;
}


static void macnet_rxStart(struct macnet_priv *macnet)
{
	macnet->regs->RDAR = ENET_RDAR_RDAR_MASK;
}


/** Find the next BD in a given ring fulfilling a given condition
 *
 * If all goes well this searching should be unnecessary, since at each point in time it
 * should be enough to keep an index to the next bd in the ring. But in practice one can imagine
 * cases where "holes" will appear in the ring, so only looking at the next bd would not be enough
 * to find a free/used bd.
 * So this function starts searching in the expected position, but if that bd is not "good",
 * it loops around the whole ring searching for a "good" bd.
 *
 * @param bd_base		base pointer for the ring of bd's in which to search
 * @param idx[IN][OUT]	index in which the search starts and results (the search will wrap)
 * @param bd_num		num of bd's in the ring
 * @param is_good		function which gets a candidate bd and returns whether it is "good"
 * @param debug_str		string to help identify who is searching
 * @return 				whether an adequate BD was found
 */
static bool macnet_findBd(struct macnet_bd * bd_base, uint8_t *idx, uint8_t bd_num, bool (*is_good)(struct macnet_bd * bd), char* debug_str)
{
	uint8_t index = *idx;		/*we start searching here*/
	struct macnet_bd *bd;
	bool compliant;
	bool dbgaux = false;

	/*is the bd[idx] compliant?*/
	do {
		bd = &bd_base[index];
		compliant = is_good(bd);	/*will manage the cache*/
		if(!compliant){
			if(DEBUG_BUBBLES && (debug_str != NULL)){
				debug_printf("B%x(st%04X)",index, bd->status);
				dbgaux = true;
			}
			index = (index+1) % bd_num;
		}
		/*if it is not compliant and we still didn't check the next bd, then check that one*/
	} while (!compliant && (index != *idx));

	if(dbgaux)
		debug_printf("(%s)", debug_str);

	if(!compliant && (index == *idx)){
		/*we looped through all the BDs and found none compliant, so report failure*/
		if(DEBUG_BUBBLES){
			debug_printf("NOTfound!\n");
		}
		return false;
	} else {
		*idx = index;
		if(DEBUG_BUBBLES && dbgaux){
			debug_printf("found\n");
		}
		return true;
	}
}




/** Send one eth frame
 *
 * Copies the contents of the pbuf into one of our Tx bufs, sets the needed MACNET
 * bd flags and signals that the bd+buffer are ready.
 *
 */
static err_t macnet_send(struct netif *dev, struct pbuf *skb)
{
	void *addr;
	int ret = 0;
	struct macnet_bd *bd;

	/* Check for valid length of data */
	int length = skb->tot_len;
	if ((length > MACNET_MAX_PKT_SIZE) || (length <= 0)) {
		main_printf(ATTR_ERROR,"Payload (%d) too large\n", length);
		return -1;
	}

	struct macnet_priv *macnet = (struct macnet_priv *)dev->state;

	if(DEBUG_CACHE_PARANOIC)
		macnet_checkBDs(macnet);


	/* search for a free TxBD to put the frame in*/
	bool found = macnet_findBd(macnet->tbd_base, &macnet->tbd_index, MACNET_TBD_NUM, macnet_TbdIsNotBusy, "freeTBD");
	if(!found){
		main_printf(ATTR_FAILURE,"MACNET: can't send frame, all TxBDs are busy!\n");
		ret = -EINVAL;
		macnet_txStart(macnet);
	} else {
		/*copy/gather the data out of LWIP's pbuf chain into the found free tx buffer*/
		/*note that sometimes we receive chained pbufs from LWIP*/
		pbuf_copy_partial(skb, macnet->tbuffs_virt[macnet->tbd_index], length, 0);

		/* flush the buffer to RAM */
		addr = macnet->tbuffs_virt[macnet->tbd_index];
		hal_cpuFlushCache(addr, length);

		/* prepare the Tx BD*/
		bd = &(macnet->tbd_base[macnet->tbd_index]);
		macnet_tbdClean(bd);
		bd->data_length = length;
		/* prepare the status flags for sending */
		bd->status |= MACNET_TBD_LAST 		| 	/* we don't use BD chains, so the BD is always last*/
						MACNET_TBD_TC 		| 	/* MACNET will add a CRC*/
						MACNET_TBD_READY;

		#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
			hal_cpuFlushCache(bd, sizeof(struct macnet_bd));
		#endif

		/*signal to MACNET that a bd+buffer awaits*/
		macnet_txStart(macnet);

		/* the current buffer will now be processed; later, the ISR or txThread
		 * will either free it, or mark it so it will be freed by LWIP.
		 */

		if(DEBUG_FRAMES_TX)
			debug_printf("S%x-",macnet->tbd_index);
		/*move the index for the next tx*/
		macnet->tbd_index = (macnet->tbd_index + 1) % MACNET_TBD_NUM;

		LINK_STATS_INC(link.xmit);
	}

	return ret;
}


/** Provide a MAC address for the given ethernet interface
 *
 * If the chip fuses contain a MAC addr, return it. If not, return a fallback MAC addr
 * based on the one that was preset in the BSP config + chip ID.
 *
 **/
static u64 macnet_getMACAddr(struct macnet_priv * macnet)
{
	OCOTP_Type * OCOTP_virt;
	u64 MAC_addr;
	int res = vm_iomap(OCOTP_BASE, sizeof(OCOTP_Type), PGHD_DEV_RW, (void **)&OCOTP_virt);
	assert(res == EOK);

	/* try to get a MAC address from the fuses
	 * The Ref Manual does not explain how to build a MAC address from the data in the
	 * OCOTP MACx registers. This is how uboot does it.
	 * Note that the higher 16 bit of MAC0 and MAC2 will stay unused.
	 */
	if(macnet->ENET_addr == ENET0_BASE){
		MAC_addr = (((u64)OCOTP_virt->MAC0) <<32) | (u64)OCOTP_virt->MAC1;
	} else {
		MAC_addr = (((u64)OCOTP_virt->MAC2) <<32) | (u64)OCOTP_virt->MAC3;
	}

	if(MAC_addr == 0){ /*the fuses did NOT contain a MAC address*/
		main_printf(ATTR_INFO, "MACNET: OTP module contains no MAC address, generating one based on the chip ID.\n");
		/* generate a unique MAC from the unique chip ID*/
		/*u64 chip_id = (((u64)OCOTP_virt->CFG1) <<32) | (u64)OCOTP_virt->CFG0;*/

		/* use the chip ID to change the lower bytes of the fallback MAC addr*/
		union {	/* for (lawfully) casting between u32 and char[] */
			char buf[20];
			u32 arr[5];
		} un1, un2;
		un1.arr[0] = OCOTP_virt->CFG0;
		un1.arr[1] = OCOTP_virt->CFG1;
		sha1_hash(un2.buf, un1.buf, 8);
		MAC_addr = macnet->MAC_addr ^ un2.arr[0];
	}

	res = vm_iounmap((void *)OCOTP_virt, sizeof(OCOTP_Type));
	assert(res == EOK);

	return MAC_addr;
}


/** Reset the PHY and store its physid2 register
 *
 * @param macnet
 * @return	EIO if there was a problem
 */
static int macnet_phyReset(struct macnet_priv *macnet)
{
	int err;
	err = macnet_mdioWrite(macnet, MII_BMCR, BMCR_RESET);
	if(err != EOK)
		return err;
	proc_threadSleep(1000); /*DP83849C's manual asks for 4 us, KSZ8041NL says nothing*/
	err = macnet_mdioRead(macnet, MII_PHYSID2, &macnet->PHY_physid2); /*get PHY id data*/

	/* KSZ8081RNA starts with the PHY address 0 defined as PHY-level broadcast
	 * (looks like the 802.11 std defines that). That is only a problem if 2 PHYs
	 * are listening on that address.
	 */
	/* disable the PHY address 0 as broadcast in KSZ8081RNA */
/*	if(macnet->PHY_physid2 == PHYSID2_KSZ8081RNA){
		u16 v;
		int err2;
		err2 = macnet_mdioRead(macnet, 0x16, &v);
		debug_printf("disabling broadcast: reg 0x16 was %04X, is ", v);
		err2 = macnet_mdioWrite(macnet, 0x16, 0x0202);
		assert(err2 == EOK);
		proc_threadSleep(1000);
		err2 = macnet_mdioRead(macnet, 0x16, &v);
		assert(err2 == EOK);
		debug_printf("%04X\n", v);
	}
*/
	return err;
}

/** Try to reset and read the PHYSID2 of every possible address in the MDIO bus */
static void macnet_phyScan(struct macnet_priv *macnet)	DEBUG_FUNC;
static void macnet_phyScan(struct macnet_priv *macnet)
{
	u8 PHY_addr_orig = macnet->PHY_addr;
	u8 i;
	for(i=0;i<32;i++){
		int err;
		macnet->PHY_addr = i;
		err = macnet_phyReset(macnet);
		if(err){
			debug_printf("PHY addr %d returned error\n", i);
		} else {
			debug_printf("PHY addr %d: PHYSID2 = 0x%04X\n", i, macnet->PHY_physid2);
		}
	}
	macnet->PHY_addr = PHY_addr_orig;
}


bool macnet_reset(struct macnet_priv *macnet)
{
	u8 i;

	/* initialize the hardware */
	macnet->regs ->ECR = ENET_ECR_RESET_MASK; /*resets and disables MACNET*/
	i = 255;
	while ((macnet->regs ->ECR & (ENET_ECR_RESET_MASK | ENET_ECR_ETHEREN_MASK)) != 0) {	/* wait for deassertion*/
		hal_cpuReschedule();
		if(i-- == 0)
			return false;
	}

	macnet->tbd_index = 0;
	macnet->rbd_index = 0;
	macnet_cleanBDs(macnet);

	return true;
}


/** Print all the registers from the PHY addressed through the macnet_priv */
void phyDump(struct macnet_priv *macnet)
{
	u8 i;
	for(i=0;i<32;i++){
		u16 v;
		int err = macnet_mdioRead(macnet, i, &v);
		assert (err == EOK);
		debug_printf("reg %02X = %04X\n", i, v);
	}
}


/** Init the MACNET registers, starting from its reset */
bool macnet_initRegs(struct netif *netif)
{
	bool OK;
	struct macnet_priv *macnet = netif->state;

	OK = macnet_reset(macnet);
	assert(OK);

	macnet->regs->EIMR = 0; 				/*no interrupts*/
	macnet->regs->EIR = 0xFFFFFFFFul; 		/*clear the interrupt flags */
	macnet->regs->FTRL = MACNET_MAX_PKT_SIZE_TRUNC;	/*truncate frames bigger than this*/
	/*note from MQX's/macnet_init.c:
	Set Receive Frame size
    NOTE: Oddly, the Receive Control Register (RCR) affects the transmit side too.  The RCR is used to determine if the
          transmitter is babbling, which means, if the RX buffer size < Tx Buffer size, we can get babling transmitter
          errors if we set RCR to the maximum Receive frame length.  We really have no choice but to set RCR to one
          of ENET_FRAMESIZE or ENET_FRAMESIZE_VLAN. */
	macnet->regs->RCR = /*ENET_RCR_FCE_MASK 					|	no pause frames*/
						ENET_RCR_MII_MODE_MASK 				|
						ENET_RCR_RMII_MODE_MASK 			|
						ENET_RCR_CRCFWD_MASK				|	/*trim off the CRC on rx*/
						ENET_RCR_MAX_FL(MACNET_MAX_PKT_SIZE);
	macnet->regs->TCR = ENET_TCR_FDEN_MASK; /*full duplex*/

	/* set the MII speed; std says it should be max 2.5MHz, and has no effect on eth speed*/
	/* with a 264MHz system clock, the relationship is:
	 * MII_SPEED = 9 -> measured clock at MDC is 2.2MHz
	 * SPEED = 10 -> 2 MHz
	 * SPEED = 8 -> 2.44 MHz
	 * TWR-SER2 works with 8, TWR-SER only slower than 9 */
	macnet->regs->MSCR = ENET_MSCR_HOLDTIME(0) | ENET_MSCR_MII_SPEED(20);

	macnet->regs->ECR |= ENET_ECR_DBSWP_MASK; /* Mandatory MACNET HW endian swap */
#ifdef ENHANCED_BUFFERS
	fec->regs->ECR |= ENET_ECR_EN1588_MASK; /*enhanced buffers*/
#endif
	macnet->regs->TFWR |= ENET_TFWR_STRFWD_MASK; /* Enable ENET store&forward mode */

	macnet->regs->RAEM = 6; /*docs say this should be at least 6, and is set to 4 at reset*/
	macnet->regs->RSFL = 0; /*store&forward mode also for RX*/

	macnet->regs ->OPD = ENET_OPD_PAUSE_DUR(20); /*duration requested when tx'ing pause frames*/

	/*MIB counters reset*/
	macnet->regs->MIBC = ENET_MIBC_MIB_CLEAR_MASK;

	/*MIB enable*/
	macnet->regs->MIBC = 0;

	/*max size of rx buffer. MRBR has forced "alignment" size!*/
	macnet->regs->MRBR = roundup(MACNET_MAX_PKT_SIZE, 16);

	/* size and address of each buffer */
	addr_t tbd_base_phys, rbd_base_phys;
	int error;
	error = vm_kmapResolve(macnet->tbd_base, &tbd_base_phys);
	if(error)
		return false;
	error = vm_kmapResolve(macnet->rbd_base, &rbd_base_phys);
	if(error)
		return false;
	macnet->regs->TDSR = tbd_base_phys;
	macnet->regs->RDSR = rbd_base_phys;

	/* set the hardware addresses (multicast, group, physical) for MACNET*/
	macnet->regs ->IAUR = 0;
	macnet->regs ->IALR = 0;
	macnet->regs ->GAUR = 0;
	macnet->regs ->GALR = 0;
	macnet->regs ->PALR = (netif->hwaddr[0] << 24) + (netif->hwaddr[1] << 16) + (netif->hwaddr[2] << 8) + netif->hwaddr[3];
	macnet->regs ->PAUR = (netif->hwaddr[4] << 24) + (netif->hwaddr[5] << 16);

	/* at this point, MACNET only is missing the speed/duplex configuration, which might
	 * come from the PHY's autonegotiation.
	 * When those settings are known, set them and enable MACNET.
	 */
	return true;
}


static bool macnet_phyForceLinkUp(struct macnet_priv * macnet, u16 mbitrate)
{
	int err;
	switch (macnet->PHY_physid2) {
		case PHYSID2_DP83849C:
			if(mbitrate == 100){
				u16 PCSR_saved;
				err = macnet_mdioRead(macnet, MII_PCSR, &PCSR_saved);
				assert(err == EOK);
				err = macnet_mdioWrite(macnet, MII_PCSR, PCSR_saved | PCSR_FORCE100OK); /*force link-up at 100Mbps*/
				assert(err == EOK);
			} else {
				u16 TenBTSCR_saved;
				err = macnet_mdioRead(macnet, MII_10BTSCR, &TenBTSCR_saved);
				assert(err == EOK);
				err = macnet_mdioWrite(macnet, MII_10BTSCR, TenBTSCR_saved | TenBTSCR_FORCE10OK); /*force link-up at 10Mbps*/
				assert(err == EOK);
			}
			break;
		case PHYSID2_KSZ8041NL:
		case PHYSID2_KSZ8081RNA:
			{
				u16 PHYctrl2_saved;
				err = macnet_mdioRead(macnet, 0x1F, &PHYctrl2_saved); /*reg PHY Control 2*/
				assert(err == EOK);
				err = macnet_mdioWrite(macnet, 0x1F, PHYctrl2_saved | (1<<11));
				assert(err == EOK);
			}
			break;
		default:
			return false;

	}
	return true;
}


static bool macnet_phyAutoMDIXEnable(struct macnet_priv * macnet)
{
	switch (macnet->PHY_physid2) {
		case PHYSID2_DP83849C:
			{
				u16 PHYCR_saved;
				macnet_mdioRead(macnet, MII_PHYCR, &PHYCR_saved);
				macnet_mdioWrite(macnet, MII_PHYCR, PHYCR_saved | PHYCR_AUTOMDIX);
			}
			break;
		case PHYSID2_KSZ8041NL:
		case PHYSID2_KSZ8081RNA:
			{
				u16 PHYctrl2_saved;
				macnet_mdioRead(macnet, 0x1F, &PHYctrl2_saved); /*reg PHY Control 2*/
				macnet_mdioWrite(macnet, 0x1F, PHYctrl2_saved & ~(1<13));
			}
			break;
		default:
			main_printf(ATTR_INFO, "MACNET: unknown PHY, can't enable AutoMDI/X\n");
			return false;

	}
	return true;
}

static void macnet_stopGracefully(struct macnet_priv *macnet)
{
	if((macnet->regs->ECR & ENET_ECR_ETHEREN_MASK) != 0){
		macnet->regs->TCR |= ENET_TCR_GTS_MASK;
		macnet->regs->RCR |= ENET_RCR_GRS_MASK;
		while((macnet->regs->EIR & ENET_EIR_GRA_MASK) == 0){
			hal_cpuReschedule();
			debug_printf("wait graceful stop\n");
		}
		macnet->regs->ECR &= ~ENET_ECR_ETHEREN_MASK;

	}

	macnet->regs->ECR &= ~ENET_ECR_ETHEREN_MASK;
	/*clean up*/
	macnet->regs->TCR &= ~ENET_TCR_GTS_MASK;
	macnet->regs->RCR &= ~ENET_RCR_GRS_MASK;
	macnet->regs->EIR &= ~ENET_EIR_GRA_MASK;

}

static void macnet_enableIRQ(struct macnet_priv *macnet)
{
	/*activate interesting interrupts*/
	macnet->regs->EIMR = ENET_EIMR_RXF_MASK | ENET_EIMR_TXF_MASK | ENET_EIMR_LC_MASK;
}

static void macnet_enable(struct macnet_priv *macnet)
{
	macnet->regs ->ECR |= ENET_ECR_ETHEREN_MASK;
	macnet_rxStart(macnet);

	/*force small pause - unclear if this is needed*/
	proc_threadSleep(1000);
}


//XXX should be published in the header file?
/** Send the given buffer as an eth frame
 *
 * Manages all the setup and cleanup.
 *
 * @param netdev
 * @param buf
 * @param len	length of the buffer
 * @return		true if the frame was sent correctly
 */
bool macnet_send_buf(struct netif *netdev, u8 buf[], u16 len)
{
	struct macnet_priv *macnet = netdev->state;
	u8 cnt;
	bool ret;
	err_t f;
	u8 tbd_index_orig = macnet->tbd_index;

	struct pbuf *pb = pbuf_alloc(PBUF_RAW, len, PBUF_RAM);
	pbuf_take(pb, buf, len);
	f=macnet_send(netdev, pb);
	pbuf_free(pb);
	if(f != ERR_OK){
		//main_printf(ATTR_ERROR, "MACNET: buffer tx failed\n");
		ret = false;
		return ret;
	}

	cnt = 255;
	while(	((macnet->regs->EIR & ENET_EIR_TXF_MASK) == 0) && 	/*wait until frame is txed */
			(cnt > 0) ){ 									/*or timeout*/
		hal_cpuReschedule();
		cnt--;
	}

	macnet->regs->EIR = ENET_EIR_TXF_MASK; 						/*clear tx-done flag*/
	assert(macnet->tbd_index != tbd_index_orig);

	if(cnt == 0){
		//main_printf(ATTR_ERROR, "MACNET: buffer tx timed out, EIR = 0x%x\n", macnet->regs->EIR);
		ret = false;
		return ret;
	}

	//main_printf(ATTR_INFO, "MACNET: buffer sent\n");
	return true;
}


/** Self-tests
 *
 * Runs PHY's self-test, and checks reliability of control and data lines between PHY and MAC.
 * Inits everything it needs in the given hardware, which will need a reset later.
 *
 * @return whether tests seem correct.
 **/
#define TEST_BITRATE	100
static bool macnet_tests(struct netif *netdev)
{
	struct macnet_priv *macnet = netdev->state;
	struct macnet_bd *bd = NULL;
	u16 r1;
	bool ret;
	u8 cnt;
	u16 test_mode;
	int err;
	u16 BMCR_saved, BMCR2;
	u16 check;
	size_t size = MACNET_MAX_PKT_SIZE;

	ret = macnet_reset(macnet);
	assert(ret);
	ret = macnet_initRegs(netdev);
	if(!ret)
		return false;

	/* this is a good moment for general debugging blind testing */
/*	macnet_phyScan(macnet);
	phyDump(macnet);

*/

	err = macnet_phyReset(macnet);

	if(err != EOK)
		return false;

	do{
		/* first, check if MDIO is working OK
		 * An unreliable MDIO is unusable, since there is no way to aguard against or fix any comm problems  */
		cnt = 0;
		/*ensure that the data stays the same if we re-read n times*/
		do {
			u16 aux;
			macnet_mdioRead(macnet, MII_PHYSID2, &aux);
			if(macnet->PHY_physid2 != aux){
				main_printf(ATTR_ERROR, "MACNET: PHY or MDIO problem (reps=%d)\n", cnt);
				ret = false;
				break;
			}
			cnt++;
		} while (cnt < 255);	/*repeat n times; if this simple test fails, assume MDIO is unusable*/

		if(!ret)
			break;

		/* MDIO looks sane, test further*/
		if(DEBUG_TEST_STEPS){
			u16 physid1;
			macnet_mdioRead(macnet, MII_PHYSID1, &physid1);
			debug_printf("MDIO tested OK, PHYSID1 = 0x%04X, PHYSID2 = 0x%04X\n", physid1, macnet->PHY_physid2);
		}

		/*enable internal loopback at PHY for next steps*/
		/* XXX move the setting and reading-back to its own func */
		macnet_mdioRead(macnet, MII_BMCR, &BMCR_saved);
		BMCR2 = BMCR_saved & ~(BMCR_ANENABLE);
		BMCR2 |= BMCR_LOOPBACK | BMCR_FULLDPLX;
		#if TEST_BITRATE == 100
			BMCR2 |= BMCR_SPEED100;
		#else
			BMCR2 &= ~BMCR_SPEED100;
		#endif
		macnet_mdioWrite(macnet, MII_BMCR, BMCR2);

		/* check that loopback is enabled */
		if(	(macnet->PHY_physid2 == PHYSID2_KSZ8041NL) ||
			(macnet->PHY_physid2 == PHYSID2_KSZ8081RNA))
			{
			macnet_mdioRead(macnet, MII_BMCR, &check);
			if(check != BMCR2){
				main_printf(ATTR_ERROR, "MACNET: failed to set PHY loopback\n");
				ret = false;
				break;
			}
		} else if(macnet->PHY_physid2 == PHYSID2_DP83849C){
			 /* For this PHY, it is not OK to read back the BMCR*/
			//XXX find another way?
		}

		debug_printf("MACNET: loopback enabled\n");

		proc_threadSleep(1000);	/*just in case*/

		if(!macnet_phyForceLinkUp(macnet, TEST_BITRATE)){
			main_printf(ATTR_INFO, "MACNET: unknown PHY, can not force link up for tests. Trying to continue...\n");
		}

		if(macnet->PHY_physid2 == PHYSID2_DP83849C){	/* this PHY has a self-test procedure: run it before continuing */
			/* force the wanted link mode up at PHY - needed by DP84849's BIST*/
			//macnet_phyForceLinkUp(macnet, TEST_BITRATE);
			/*execute Buil-In Self Test in PHY*/
			u16 PHYCR_saved;
			macnet_mdioRead(macnet, MII_PHYCR, &PHYCR_saved);
			test_mode = PHYCR_saved | PHYCR_BIST_START | PHYCR_PSR15; /*PHYCR: start BIST w/ 15 bit test seq*/
			macnet_mdioWrite(macnet, MII_PHYCR, test_mode);

			/* we'll inject a number of errors, and check if the error counter found exactly those*/
			cnt = 0;					/*counter for injected errors*/
			r1 = 0;
			proc_threadSleep(10000); 	/*meanwhile the BIST is running*/
			do{
				u16 aux;
				macnet_mdioRead(macnet, MII_PHYCR, &aux);
				if((aux & PHYCR_BIST_START) == 0){
					main_printf(ATTR_ERROR, "MACNET: PHY self-test aborted\n");
					ret = false;
					break;
				}
				macnet_mdioWrite(macnet, MII_PHYCR, test_mode | PHYCR_BIST_FE);	/*assert the self-clearing forced error*/
				cnt++;
				proc_threadSleep(10000); /*meanwhile the BIST is running*/
				macnet_mdioRead(macnet, MII_CDCTRL1, &r1);
				r1 = (r1 & 0xFF00)>>8; /*BIST error counter*/
			} while((cnt == r1) && (cnt < 5));

			macnet_mdioWrite(macnet, MII_PHYCR, PHYCR_saved);	/*stop BIST*/

			if(cnt != r1){	/*the error counter shows a different number of errors than we injected*/
				main_printf(ATTR_ERROR, "MACNET: PHY self-test failed: injected %d, got %d\n", cnt, r1);
				ret = false;
				break;
			}
		}

		/* now test data lines*/

		/* config MACNET for same link mode as PHY*/
		macnet->regs->TCR |= ENET_TCR_FDEN_MASK;		/* Set full duplex*/
	#if TEST_BITRATE == 100
		macnet->regs->RCR &= ~ENET_RCR_RMII_10T_MASK; 	/*set 100Mbps*/
	#else
		macnet->regs->RCR |= ENET_RCR_RMII_10T_MASK;	/*set 10Mbps*/
	#endif
		macnet->regs->RCR |= ENET_RCR_PROM_MASK;	/*promiscuous mode, so MAC addr does not matter*/

		macnet->regs->EIR = 0xFFFFFFFFul; 			/*clear the interrupt flags*/

		macnet_enable(macnet);

		/*send a test packet*/
		char pkt[]=TEST_PACKET;
		u16 pkt_len = TP_LENGTH;
		u16 rx_len;
		ret = macnet_send_buf(netdev, (u8 *)pkt, pkt_len);
		if(!ret){
			main_printf(ATTR_ERROR, "MACNET: failed to send test packet\n");
			break;
		}

		/*wait for rx*/
		cnt = 255;
		while(	((macnet->regs->EIR & ENET_EIR_RXF_MASK) == 0) &&	/*wait until a frame is rxed*/
				(cnt > 0)){										/*or timeout*/
			hal_cpuReschedule();
			cnt--;
		}

		if(cnt == 0){
			main_printf(ATTR_ERROR, "MACNET: test rx timed out, EIR = 0x%x\n", macnet->regs->EIR);
			ret = false;
			break;
		}

		bd = &macnet->rbd_base[macnet->rbd_index];
		#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
			hal_cpuInvalCache(bd, sizeof(struct macnet_bd));
		#endif
		rx_len = bd->data_length;
		hal_cpuInvalCache(macnet->rbuffs_virt[macnet->rbd_index], size);
		/*check that the rx frame has correct flags and identical contents to the tx frame*/
		if(((bd->status & (MACNET_RBD_NO | MACNET_RBD_CR | MACNET_RBD_OV | MACNET_RBD_TR | MACNET_RBD_LAST)) != MACNET_RBD_LAST) ||
				(rx_len != pkt_len)	||
				(memcmp(pkt, macnet->rbuffs_virt[macnet->rbd_index], pkt_len) != 0)){
			/*the echo frame was not received OK. hex-dump it */
			char * txbuf = vm_kmalloc(1000);		/*for the tx packet's hex dump*/
			char * rxbuf = vm_kmalloc(1000);		/*for the rx packet's hex dump*/
			main_hexDump(txbuf, 1000, pkt, pkt_len);
			main_hexDump(rxbuf, 1000, macnet->rbuffs_virt[macnet->rbd_index], rx_len);
			main_printf(ATTR_ERROR, "MACNET: bad loopback test: bd status = 0x%x, EIR = 0x%x\nSENT (%d bytes)\n%s\nRECEIVED (%d bytes)\n%s\n",
					bd->status, macnet->regs->EIR, pkt_len, txbuf, rx_len, rxbuf);
			vm_kfree(txbuf);
			vm_kfree(rxbuf);
			ret = false;
			break;
		}
		debug_printf("MACNET tests OK\n");
	} while (false); /* just as a jump point for the breaks*/

	/*cleanup buffers*/
	if(bd != NULL)
		macnet_rbdClean(bd);
	macnet->rbd_index = 0;
	macnet->tbd_index = 0;
	macnet_tbdClean(&macnet->tbd_base[macnet->tbd_index]);

	return ret;
}

/** Report whether the PHY found any link problem*/
static void macnet_linkSanityTest(struct macnet_priv *macnet)
{
	bool rf ;
	u16 aux;
	macnet_mdioRead(macnet, MII_BMSR, &aux);
	rf = aux & BMSR_RFAULT;
	if(rf)
		main_printf(ATTR_ERROR, "MACNET: PHY reports Remote Fault\n");
	macnet_mdioRead(macnet, MII_ANER, &aux);
	bool pdf = aux & ANER_PDF;
	if(pdf)
		main_printf(ATTR_ERROR, "MACNET: PHY reports PD Fault\n");
}

/** Thread to poll the status of the link and manage the network stack accordingly */
static int macnet_linkThread(void *v)
{
	u16 speed, duplex;
	u16 aux;
	struct netif *netif = v;
	struct macnet_priv *macnet = netif->state;
	u16 bmcr, bmsr;
	bool autonegotiation;
	bool link_failed;
	bool OK;
	u16 bmcr_saved;
	enum an_state {AN_OFF, AN_IN_PROCESS, AN_FINISHED, AN_LINKUP};
	static enum an_state an_state = AN_OFF;

	for(;;){
		switch(an_state){
			case AN_OFF:
				OK = macnet_reset(macnet);
				assert(OK);
				OK = macnet_initRegs(netif);
				if(!OK)
					return ERR_DEV_DETECT;
				macnet_cleanBDs(macnet);
				macnet_phyReset(macnet);

				/* prepare autonegotiation advertisement*/
				/* NOTE: forcing a state *through autonegotiation* must take into account the capability
				 * priorities defined by the 802.3u standard and the intersection of capabilities between
				 * partners.
				 * So, to force a mode, better just disable autoneg and set the mode bits manually.*/

				/* NOTE: unsuccessful autonegotiation can still detect the link speed thaks to Parallel Detect,
				 * but the duplex mode is not directly detectable, so it is supposed half-duplex.
				 * BUT IF THE LINK PARTNER IS CONFIGURED FOR FULL DUPLEX, THERE WILL BE CONNECTIVITY THOUGH WITH VERY BAD PERFORMANCE.
				 * This is a classical network configuration problem. */
				macnet_mdioWrite(macnet, MII_ADVERTISE,  LPA_100FULL 				|
															LPA_100HALF 			|
															LPA_10FULL 				|
															LPA_10HALF 				|
															PHY_ANLPAR_PSB_802_3);

				/*enable auto-MDI-X*/
				macnet_phyAutoMDIXEnable(macnet);

				/*start autonegotiation*/
				macnet_mdioRead(macnet, MII_BMCR, &bmcr_saved);
				macnet_mdioWrite(macnet, MII_BMCR, bmcr_saved | BMCR_ANENABLE | BMCR_ANRESTART);

				/*miiphy_wait_aneg*/
				an_state = AN_IN_PROCESS;
				if(DEBUG_LINKTHREAD)
					debug_printf("AN in process\n");
				break;

			case AN_IN_PROCESS:
				macnet_mdioRead(macnet, MII_BMSR, &bmsr);
				if(bmsr & BMSR_ANEGCOMPLETE){
					an_state = AN_FINISHED;
					if(DEBUG_LINKTHREAD)
						debug_printf("AN finished\n");
				}
				break;

			case AN_FINISHED:
				/* looks like the autonegotiation process is understood to finish after the link is "up",
				 * so we wait for the end of autonegotiation */

				/* was it a "real" autoneg or did it fall back to Parallel Detection? */
				macnet_mdioRead(macnet, MII_ANER, &aux);
				bool an_pd = (aux & ANER_LP_AN_ABLE) == 0;
				/* in PHY DP83849C, result of the PD sets anyway the bits at MII_LPA, so there is no need
				 * to distinguish PD or AN. But this seems to be implementation-dependent*/
				char *an_mode = an_pd?"PD":"AN";

				/*get speed from PHY if autonegotiated*/
				macnet_mdioRead(macnet, MII_BMCR, &bmcr);
				autonegotiation = bmcr & BMCR_ANENABLE;
				/* assumes that this flag is enough, though this is just a command, not the result */
				if (autonegotiation) {
					/* BMCR bits setting speed and duplex are meaningless in this case*/
					u16 anlpar;
					macnet_mdioRead(macnet, MII_LPA, &anlpar);
					/* NOTE: The proper thing to do now would be to get the most prioritary mode (as defined
					 * by the standard) in the intersection of our and their advertised capabilities, and
					 * configure the MAC accordingly.
					 * But if we assume that we advertised for the same or better capabilities than the
					 * Link Partner, then the Link Partner is the one limiting the link capabilities,
					 * so we can just configure what it asks for. This is how uboot works, we do the same.
					 */
					speed = (anlpar & LPA_100) ? _100BASET : _10BASET;
					if (anlpar & (LPA_10FULL | LPA_100FULL)) {
						duplex = FULL;
					} else {
						duplex = HALF;
					}
				} else {
					/*no autonegotiation, so get it from what was set*/
					speed = (bmcr & BMCR_SPEED100) ? _100BASET : _10BASET;
					duplex = (bmcr & BMCR_FULLDPLX);
				}

				if (speed != _100BASET) {
					macnet->regs->RCR |= ENET_RCR_RMII_10T_MASK;
				} else
					macnet->regs->RCR &= ~ENET_RCR_RMII_10T_MASK;

				/* Initialize the snmp variables and counters inside the struct netif */
				NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, (speed == 100BASET)?100000000:10000000);

				if(duplex == HALF){
					macnet->regs->TCR &= ~ENET_TCR_FDEN_MASK;
				} else
					macnet->regs->TCR |= ENET_TCR_FDEN_MASK;

				main_printf(ATTR_INFO, "net: link is up: speed=%d, duplex=%s (%s)\n", speed, duplex == FULL ? "full" : "half",
						autonegotiation?an_mode:"forced");

				macnet_linkSanityTest(macnet);

				macnet_enable(macnet);
				macnet_enableIRQ(macnet);

				netif_set_link_up(netif);
				an_state = AN_LINKUP;
				if(DEBUG_LINKTHREAD)
					debug_printf("AN up\n");
				break;
			case AN_LINKUP:
				macnet_mdioRead(macnet, MII_BMSR, &bmsr);
				link_failed = ((bmsr & BMSR_LSTATUS) == 0);
				if(link_failed){
					an_state = AN_OFF;
					if(DEBUG_LINKTHREAD)
						debug_printf("AN off\n");
					netif_set_link_down(netif);
					/*disable MACNET, needed for changing the link settings*/
					macnet_stopGracefully(macnet);
					main_printf(ATTR_INFO, "net: link is down\n");
				}
		} /*switch*/

		/*recheck after 1 sec*/
		proc_threadSleep(1000000);
	}

	return 0;
}


/* Check that the link is up before putting anything into the tx ring */
static err_t macnet_send_filtered(struct netif *dev, struct pbuf *skb)
{
	if(!netif_is_link_up(dev)){
		if(DEBUG_FRAMES_TX)
			debug_printf("F!");
		return ERR_RTE;
	} else
		return macnet_send(dev, skb);
}

/** Init the hardware enough to be ready to RX and TX, using/storing whatever state is needed
 * in netif->state, and preparing rest of netif */
static err_t macnet_initLwip(struct netif *netif)
{
	/**based on lwip/netif/ethernetif.c template, implements what is done in uboot's fec_mcx.c/fec_probe
	 * netif fields that should be init'ed: hwaddr_len, hwaddr[], mtu, flags
	 */
	u8 i;
	err_t ret = ERR_OK;

	/*if private data is needed: allocate space for appropriate blob, put pointer to it in netif->state*/
	LWIP_ASSERT("netif != NULL", (netif != NULL));

	struct macnet_priv *macnet = netif->state;

#if LWIP_NETIF_HOSTNAME
	/* Initialize interface hostname */
	netif->hostname = "vybrid_phoenix";
	#warning "allocateme!"
#endif /* LWIP_NETIF_HOSTNAME */

	strcpy(netif->name, "eth"); /* LWIP adds an index number */

	netif->output = etharp_output;
#if LWIP_IPV6
	netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
	netif->linkoutput = macnet_send_filtered;

	/* maximum transfer unit */
	netif->mtu = MTU;

	/* device capabilities */
	netif->flags = NETIF_FLAG_ETHARP | NETIF_FLAG_BROADCAST /*| NETIF_FLAG_LINK_UP*/;

	/* set MAC hardware address and its length for LWIP*/
	netif->hwaddr_len = ETHARP_HWADDR_LEN;
	for(i = 0;i<ETHARP_HWADDR_LEN;i++){
		netif->hwaddr[i] = macnet->MAC_addr >> ((ETHARP_HWADDR_LEN-1-i)*8);
	}

	while(DEBUG_TEST_FOREVER){
		macnet_tests(netif);
		proc_threadSleep(2000000);
	}

	/* run some hardware tests */
	if(!macnet_tests(netif)){
		main_printf(ATTR_ERROR, "\nMACNET: selftests failed\n");
		return ERR_DEV_DETECT;
	}

	/* Start thread to poll the link going up or down */
	proc_thread(NULL, macnet_linkThread, NULL, 0, netif, ttRegular);

	return ret;
}


int macnet_isr(unsigned irq, cpu_context_t *ctx, void *arg)
{
	struct macnet_priv *macnet = (struct macnet_priv *)arg;
	ENET_Type *ENET = macnet->regs;
	u32 EIR;

	proc_spinlockSet(&macnet->lock);	/*only needed for when we'll be multiprocessor*/

	if(DEBUG_ISR)
		debug_printf("I");

	EIR = ENET->EIR;
	if(EIR & ENET_EIR_LC_MASK){ /*Late Collision error: this should not happen in a well configured network*/
		/*this can be a symptom of full/half duplex mismatch in the network*/
		static u8 cnt;
		ENET->EIR = ENET_EIR_LC_MASK;	/*clear flag*/
		if(cnt==255){
			main_printf(ATTR_FAILURE, "MACNET: Late Collision errors: check network config\n");
			ENET->EIMR &= ~ENET_EIMR_LC_MASK;	/*disable the interrupts for this reason*/
		} else
			cnt++;
	}

	if (EIR & ENET_EIR_TXF_MASK) {
		/* Acknowledge TX-completed interrupt */
		ENET->EIR = ENET_EIR_TXF_MASK;
		if(DEBUG_FRAMES_TX)
			debug_printf("T");

		/* because of errata e6358, TDAR could get deasserted and so any waiting buffers
		 * would stay waiting. So, reassert it just in case.
		 * If there are no buffers waiting, MACNET will just deassert again:
		 * no IRQ generated, and no extra work for us.*/
		macnet_txStart(macnet);
	}

	if (EIR & ENET_EIR_RXF_MASK) {
		if(DEBUG_FRAMES_RX)
			debug_printf("R");
		/* the RxThread will drain all queued BDs, so disable the Rx interrupt*/
		macnet->regs->EIMR &= ~ENET_EIMR_RXF_MASK;
		/* Acknowledge the Rx interrupt */
		ENET->EIR = ENET_EIR_RXF_MASK;
		/* Indicate to the receive task that frames are available */
		proc_eventSignal(&macnet->rx_evt);
	}

	proc_spinlockClear(&macnet->lock, sopGetCycles);

	return IHRES_HANDLED;
}



static int macnet_rx(struct pbuf *p, struct netif *netif)
{
	struct eth_hdr *ethhdr;
	int status = EOK;

	/*the pbuf contents is an eth frame*/
	ethhdr = p->payload;
	switch (ntohs(ethhdr->type)) {
			/* IP or ARP packet? */
		case ETHTYPE_IP:
		case ETHTYPE_ARP:
			/* full packet send to tcpip_thread to process */
			status = netif->input(p, netif);
			if (status != ERR_OK){
				debug_printf("MACNET: IP input error %d\n", status);
			}
			break;

		default:
			status = -EINVAL;
			LINK_STATS_INC(link.proterr);
	}
	return status;
}


/**
 * Receive thread.
 *
 * When awaken by the ISR, searches through the array of rx BDs for a newly arrived frame.
 * If found, keeps draining contiguous received BDs until an empty one is found.
 *
 * Keeps track of where frames should land in the array to avoid searching if possible.
 *
 * @param netif the LWIP network interface to pass received frames to
 */
/* Only this thread ever touches the RxBD array and index (once they have been init'ed),
 * so no need to lock them.
 * The "Empty" flag semantics control concurrent access to a given BD by MACNET and us. */
static int macnet_rxThread(void *arg)
{
	struct netif *netdev = (struct netif *)arg;
	struct macnet_priv *macnet = (struct macnet_priv *)netdev->state;

	for (;;){
		if (proc_eventWait(&macnet->rx_evt, 0) != EOK){
			break;
		}

		if(DEBUG_CACHE_PARANOIC)
			macnet_checkBDs(macnet);

		if(DEBUG_FRAMES_RX)
			debug_printf("R");

		uint8_t index = macnet->rbd_index;
		struct macnet_bd *bd;
		unsigned int frames_in_a_row = 1;
		bool nextBdIsFilled;

		/* search where in the rx ring did MACNET drop the received frame */
		bool found = macnet_findBd(macnet->rbd_base, &index, MACNET_RBD_NUM,
						macnet_RbdIsFilled, "filledRBD");

		/*if we looped through all the BDs but found none filled, then do nothing more*/
		if(found){
			/* we found a filled BD */
			bd = &macnet->rbd_base[index];
			#ifdef USE_EXPLICIT_CACHE_MGMT_FOR_BDS
				/* supposedly findBd will have done this already*/
				hal_cpuInvalCache(bd, sizeof(struct macnet_bd));
			#endif
			do { 			/* we'll keep draininig the filled BDs until an empty one is found*/
				if(DEBUG_FRAMES_RX)
					debug_printf("%d", index);

				if((bd->status & MACNET_RBD_LAST) == 0){	/* "Last" might be cleared if specific errors happened*/
					debug_printf("MACNET: non-Last frame: status %08X, length %d, idx %d, EIR %08X - CONTINUING!\n",bd->status, bd->data_length, index, macnet->regs->EIR);
				}

				if((bd->status & (MACNET_RBD_LAST | MACNET_RBD_NO | MACNET_RBD_CR | MACNET_RBD_OV | MACNET_RBD_TR))== MACNET_RBD_LAST){ /*try to accept quickly any good frame*/
					if((bd->status & MACNET_RBD_LG) != 0){ /* Length violation warning - will go on anyway*/
						debug_printf("MACNET: Rx frame length violation, but frame accepted\n");
						/*LINK_STATS_INC(link.lenerr);*/
					}
					uint16_t len = bd->data_length;
					struct pbuf *p = pbuf_alloc(PBUF_RAW, len + ETH_PAD_SIZE, PBUF_RAM);

					if(p == NULL){
						main_printf(ATTR_ERROR, "MACNET: could not allocate pbuf for Rx, frame dropped\n");
						LINK_STATS_INC(link.memerr);
					} else {

						/*XXX MACNET has an option to use a 2-byte padding before the eth frame so the eth payload is aligned,
						 * for improved speed; and looks like LWIP's ETH_PAD_SIZE would automate taking care of that padding.
						 * But it should be enabled in net/lwip/phoenix/lwipopts.h, so we need to make that dependent
						 * on the makefile's TARGET, maybe*/
#if ETH_PAD_SIZE
						pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif
						/* copy received frame into the new pbuf */
						hal_cpuInvalCache(macnet->rbuffs_virt[index], len);
						pbuf_take(p, macnet->rbuffs_virt[index], len);
#if ETH_PAD_SIZE
						pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
						/* record that a packet was rxed*/
						LINK_STATS_INC(link.recv);
						/* pass the copied frame to the upper layer to be processed, or free its buffer if rejected */
						err_t err = macnet_rx(p, netdev);
						if (err != EOK){
							pbuf_free(p);
							/*LINK_STATS_INC(link.memerr); no, because this is rather a higher layer problem, not link-layer*/
							if(DEBUG_FRAMES_RX)
								debug_printf("J");
						} else {
							/* everything OK */
						}
					}
				} else if((bd->status & MACNET_RBD_TR) != 0){ /* TRuncated frames MUST be dropped, no matter the flags*/
					debug_printf("MACNET: Rx frame truncated, dropped\n");
					LINK_STATS_INC(link.lenerr);
				} else if((bd->status & MACNET_RBD_OV) != 0){ /* OVerrun FIFO causes other error flags to be invalid*/
					debug_printf("MACNET: Rx FIFO overrun, frame dropped\n");
					LINK_STATS_INC(link.err);
				} else if((bd->status & MACNET_RBD_CR) != 0){ /* CRC error */
					debug_printf("MACNET: Rx CRC error\n");
					LINK_STATS_INC(link.chkerr);
					if(lwip_stats.link.chkerr == 255){	/*only warn once, after a lot of errors*/
						main_printf(ATTR_ERROR, "MACNET: CRC errors on RX: check network config\n");
						/*because this might point to a Duplex mismatch*/
					}
				} else {
					if(DEBUG_BOOL){
						debug_printf("MACNET: Rx error: status = %08X, frame dropped\n", bd->status);
					}
					LINK_STATS_INC(link.err);
				}

				/* clear the rx BD */
				macnet_rbdClean(bd);
				/* tell MACNET that a new empty buffer awaits */
				macnet_rxStart(macnet);

				/*advance the index and the pointed-to RxBD*/
				index = (index+1) % MACNET_RBD_NUM;
				bd = &macnet->rbd_base[index];

				/*try to diagnose whether we are losing traffic*/
				if(DEBUG_BOOL){
					static uint16_t dropped_IEEE_old;
					uint16_t dropped_IEEE = macnet->regs->IEEE_R_DROP;
					if(dropped_IEEE != dropped_IEEE_old){
						debug_printf("MACNET dropped frame counter +%d\n", dropped_IEEE-dropped_IEEE_old);
						dropped_IEEE_old = dropped_IEEE;
					}
				}

				/*if the next BD also contains a frame, keep processing*/
				nextBdIsFilled = macnet_RbdIsFilled(bd);
				if(nextBdIsFilled){
					/*next BD is not empty, so we're going to loop*/
					frames_in_a_row++;
				} else {
					/* did we loop through the whole ring? */
					if(frames_in_a_row > MACNET_RBD_NUM){
						/*the ring became full before we could empty it, so maybe frames got dropped*/
						debug_printf("MACNET: rx buffers got full, packets might have been dropped\n");
					}

					if(DEBUG_RXROW){
						if(frames_in_a_row > 1)
							debug_printf("MACNET: rxed %d frames in a row\n", frames_in_a_row);
					}
				}


			} while (nextBdIsFilled) ;

			/* MACNET will next use the next Empty BD, trying first the BD next to the last it wrote; so we move along our index, tentatively.
			 * Note that this is only an educated guess, since MACNET will skip non-empty BDs .*/
			macnet->rbd_index = index;
		} else{
			if(DEBUG_FRAMES_RX)
				debug_printf("X");
		}
		/* re-enable interrupts. We just open the interrupt mask, but not reset the event bit; that way any
		 * interrupt arrived since the original one will be serviced now.
		 * However, that might cause a new run of the RxThread when in fact all the BDs have just been emptied. No harm done.
		 * The event bit is reset in the ISR.*/

		macnet->regs->EIMR = ENET_EIMR_RXF_MASK;
	}

	debug_printf("MACNET: Rx thread exited\n");
	return 0;
}


/** Tests existence of device, inits the OS's infrastructures for its management,
 * registers it with LWIP
 *
 * (Nothing to test in Vybrid's case, since MACNET is embedded)
 * */
static int macnet_initOne(u8 index, addr_t ENET, uint8_t phy_addr, u32 irq, u64 MAC_addr)
{
	struct netif *netdev, *netdev2;
	int ret;
	struct macnet_priv *macnet;
	u8 i;

	/*alloc netdev*/
	if ((netdev = macnet_kmallocAligned(sizeof(struct netif))) == NULL ) {
		main_printf(ATTR_ERROR, "net: not enough memory for macnet's netdev\n");
		return -ENOMEM;
	}

	/*if private data is needed: allocate space for appropriate blob.
	 * netif_add() will link it to the netif*/
	macnet = (struct macnet_priv *) macnet_kmallocAligned(sizeof(struct macnet_priv));
	if (macnet == NULL ) {
		main_printf(ATTR_ERROR, "net: not enough memory for macnet_priv\n");
		vm_kfree(netdev);
		ret = ERR_MEM;
		return ret;
	}
	memset(macnet, 0, sizeof(*macnet));

	ret = vm_iomap(ENET, sizeof(ENET_Type), PGHD_DEV_RW, (void **)&macnet->regs);
	assert(ret == EOK);
	if (ret != EOK)
		return ret;

	ret = macnet_allocAndInit(macnet);
	if (ret) {
		main_printf(ATTR_ERROR, "net: not enough memory for macnet's buffers\n");
		vm_kfree(macnet);
		return ret;
	}

	macnet->PHY_addr = phy_addr;
	macnet->ENET_addr = ENET;
	macnet->MAC_addr = MAC_addr;	/* as a hint for fallback address generation */
	macnet->MAC_addr = macnet_getMACAddr(macnet);	/* definitive */

	main_printf(ATTR_INFO, "net: #%d at 0x%x irq=%d phy=%d mac_addr=",
			index, ENET, irq, phy_addr);
	for(i = 0;i<ETHARP_HWADDR_LEN;i++){
		main_printf(ATTR_INFO, "%02x%s", (u8)(macnet->MAC_addr >> ((ETHARP_HWADDR_LEN-1-i)*8)), i<ETHARP_HWADDR_LEN-1?":":"");
	}
	main_printf(ATTR_INFO,"\n");

	proc_spinlockCreate(&macnet->lock, "MACNET multiproc");
	proc_eventCreate(&macnet->rx_evt);

	/* according to LWIP's docs, the netif is passed untouched to be init'ed by the init
	 * callback, which will also finish preparing any private data needed by the driver.
	 * netif_add will also link the private data to the netif. */
	netdev2 = netif_add(netdev, NULL, NULL, NULL, macnet, macnet_initLwip, tcpip_input); /*using the "tcpip thread" mode of lwip*/
	if(netdev2 == NULL){
		main_printf(ATTR_ERROR, "Failed to initialize LWIP netif %d\n", index);
		vm_iounmap(macnet->regs, sizeof(ENET_Type));
		vm_kfree(macnet);
		vm_kfree(netdev);
		return -1;
	}

	/* Start task to receive packets */
	proc_thread(NULL, macnet_rxThread, NULL, 0, netdev, ttRegular);

	/* Install driver ISR */
	hal_interruptsSetHandler(irq, macnet_isr, (void *)macnet);

	return 0;
}

#endif // #ifdef CONFIG_NET

/** Entry point for Phoenix
 * Theoretically searches for instances of the hardware and inits them one by one */
void macnet_init(void)
{
#ifdef CONFIG_NET
	u8 i;
	struct eth_config {
			addr_t base;
			u64 MAC_addr_hint;
			u16 IRQ;
			u8 PHY_addr;
	} eth_configs[] = ETH_CONFIGS;
	u8 macnet_num = sizeof(eth_configs)/sizeof(struct eth_config);

	for(i=0;i<macnet_num;i++){
		macnet_initOne(i, eth_configs[i].base, eth_configs[i].PHY_addr, eth_configs[i].IRQ, eth_configs[i].MAC_addr_hint);
	}
#endif // #ifdef CONFIG_NET
}
