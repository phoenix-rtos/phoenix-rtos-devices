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

#define DEBUG 1

#include <config.h>
#include <hal/if.h>
#include <main/if.h>
#include <vm/if.h>

#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/ethip6.h"
#include "netif/etharp.h"
#include "lwip/tcpip.h"

#include <dev/pci.h>
#include <dev/rtl8139/8139too.h>


static int read_eeprom(void *ioaddr, int location, int addr_len);
static void rtl8139_init_ring(struct netif *dev);
static void rtl8139_hw_start(struct netif *dev);
static int rtl8139_open(struct netif *dev);
//static int rtl8139_close(struct netif *dev);
static err_t rtl8139_start_xmit(struct netif *dev, struct pbuf *skb);
//static netdev_stat_t *rtl8139_getstats(netdev_t *dev);
static int rtl8139_interrupt(unsigned int irq,cpu_context_t *ctx, void *dev_instance);


static const u16 rtl8139_intr_mask = PCIErr | PCSTimeout | RxUnderrun | RxOverflow | RxFIFOOver | TxErr | TxOK | RxErr | RxOK;

static const unsigned int rtl8139_rx_config = RxCfgRcv32K | RxNoWrap | (RX_FIFO_THRESH << RxCfgFIFOShift) | (RX_DMA_BURST << RxCfgDMAShift);

static const unsigned int rtl8139_tx_config = (TX_DMA_BURST << TxDMAShift) | (TX_RETRY << TxRetryShift);


/*
 * MII routines
 */


static int mdio_read(struct netif *dev, int phy_id, int location)
{
	struct rtl8139_private *tp = dev->state;
	int retval = 0;

	if (phy_id > 31) {
		return location < 8 && mii_2_8139_map[location] ?
			readw ((void *)((char *)tp->mmio_addr + mii_2_8139_map[location])) : 0;
	}
	return (retval >> 1) & 0xffff;
}


static void mdio_write(struct netif *dev, int phy_id, int location, int value)
{
	struct rtl8139_private *tp = dev->state;

	if (phy_id > 31) {
		void *ioaddr = tp->mmio_addr;
		if (location == 0) {
			RTL_W8(Cfg9346, Cfg9346_Unlock);
			RTL_W16(BasicModeCtrl, value);
			RTL_W8(Cfg9346, Cfg9346_Lock);
		}
		else if (location < 8 && mii_2_8139_map[location])
			RTL_W16(mii_2_8139_map[location], value);
		return;
	}
}


/*
 * EEPROM routines
 */


static int read_eeprom (void *ioaddr, int location, int addr_len)
{
	int i;
	unsigned retval = 0;
	void *ee_addr;
	int read_cmd = location | (EE_READ_CMD << addr_len);

	ee_addr = (void *)((char *)ioaddr + Cfg9346);
	writeb (EE_ENB & ~EE_CS, ee_addr);
	writeb (EE_ENB, ee_addr);
	eeprom_delay();

	/* Shift the read command bits out */
	for (i = 4 + addr_len; i >= 0; i--) {
		int dataval = (read_cmd & (1 << i)) ? EE_DATA_WRITE : 0;
		writeb (EE_ENB | dataval, ee_addr);
		eeprom_delay();
		writeb (EE_ENB | dataval | EE_SHIFT_CLK, ee_addr);
		eeprom_delay();
	}
	writeb(EE_ENB, ee_addr);
	eeprom_delay();

	for (i = 16; i > 0; i--) {
		writeb(EE_ENB | EE_SHIFT_CLK, ee_addr);
		eeprom_delay ();
		retval = (retval << 1) | ((readb (ee_addr) & EE_DATA_READ) ? 1 : 0);
		writeb(EE_ENB, ee_addr);
		eeprom_delay();
	}

	/* Terminate the EEPROM access */
	writeb(~EE_CS, ee_addr);
	eeprom_delay();
	return retval;
}


/*
 * Initialization routines
 */


static void rtl8139_chip_reset(void *ioaddr)
{
	int i;

	/* Soft reset the chip */
	RTL_W8(ChipCmd, CmdReset);

	/* Check that the chip has finished the reset */
	for (i = 1000; i > 0; i--) {
		if ((RTL_R8(ChipCmd) & CmdReset) == 0)
			break;
		//XXX pkj udelay (10);
		{
			int k;
			for (k = 0; k < 10000000; k++);
		}
	}
}


static int rtl8139_init_board(pci_device_t *pdev, struct netif **dev_out)
{
	void *ioaddr=NULL;
	struct netif *dev;
	struct rtl8139_private *tp;
	u8 tmp8;
	int rc;
	unsigned int i;
	u32 tmp;

	*dev_out = NULL;

	/* Create new network device */
	if ((dev = vm_kmalloc( sizeof(struct netif) + sizeof(struct rtl8139_private) ) ) == NULL) {
		main_printf(ATTR_ERROR, "Can't register new network device!\n");
		return -ENOMEM;
	}
	dev->state = ((void*)dev) + sizeof(struct netif);
	tp = dev->state;
	tp->pci_dev = pdev;

	for (i = 0; i < 6; i++) {
		if (pdev->resources[i].base & 1) {
			ioaddr = (void *)(pdev->resources[i].base & ~1);
			break;
		}
	}

	tp->mmio_addr = ioaddr;

	/* Bring old chips out of low-power mode */
	RTL_W8(HltClk, 'R');

	/* check for missing/broken hardware */
	if (RTL_R32 (TxConfig) == 0xFFFFFFFF) {
		main_printf(ATTR_ERROR, "Chip not responding, ignoring board!\n");
		rc = -ENOMEM;
		goto err_out;
	}

	/* identify chip attached to board */
	tmp = RTL_R8 (ChipVersion);
	for (i = 0;; i++) {
		if (rtl_chip_info[i].name == NULL)
			break;
		if (tmp == rtl_chip_info[i].version) {
			tp->chipset = i;
			goto match;
		}
	}

match:
	if (tp->chipset >= CH_8139B) {
		u8 new_tmp8 = tmp8 = RTL_R8(Config1);
		if ((rtl_chip_info[tp->chipset].flags & HasLWake) && (tmp8 & LWAKE))
			new_tmp8 &= ~LWAKE;
		new_tmp8 |= Cfg1_PM_Enable;
		if (new_tmp8 != tmp8) {
			RTL_W8 (Cfg9346, Cfg9346_Unlock);
			RTL_W8 (Config1, tmp8);
			RTL_W8 (Cfg9346, Cfg9346_Lock);
		}
		if (rtl_chip_info[tp->chipset].flags & HasLWake) {
			tmp8 = RTL_R8 (Config4);
			if (tmp8 & LWPTN)
				RTL_W8 (Config4, tmp8 & ~LWPTN);
		}
	}
	else {
		tmp8 = RTL_R8 (Config1);
		tmp8 &= ~(SLEEP | PWRDN);
		RTL_W8 (Config1, tmp8);
	}

	rtl8139_chip_reset(ioaddr);
	*dev_out = dev;
	return 0;

err_out:
	return rc;
}


static err_t rtl8139_init_lwip(struct netif *dev)
{
	struct rtl8139_private *tp = dev->state;
	int addr_len, i;
	void *ioaddr = tp->mmio_addr;;

	addr_len = read_eeprom (ioaddr, 0, 8) == 0x8129 ? 8 : 6;
	for (i = 0; i < 3; i++)
		((u16 *)(dev->hwaddr))[i] = le16_to_cpu(read_eeprom (ioaddr, i + 7, addr_len));
	dev->hwaddr_len = addr_len;
	dev->linkoutput = rtl8139_start_xmit;
	dev->output = etharp_output;
	dev->mtu = MAX_ETH_FRAME_SIZE - 18;

	strcpy(dev->name,"eth");
	dev->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_UP | NETIF_FLAG_ETHERNET;
	rtl8139_open(dev);
	return ERR_OK;
}

static int rtl8139_init_one(pci_device_t *pdev)
{
	struct netif *dev = NULL;
	struct rtl8139_private *tp;
	int i, option;
	static int board_idx = -1;

	board_idx++;

	/* Test the chip revision */

	/* set as bus master */
	dev_setBusmaster(pdev, 1);

	i = rtl8139_init_board(pdev, &dev);
	if (i < 0)
		return i;

	tp = dev->state;

	tp->irq = pdev->irq;
	tp = dev->state;
	tp->stopped = 0;

	/*  tp->drv_flags = board_info[ent].hw_flags; */
	proc_spinlockCreate(&tp->lock, "tp.lock");
	proc_spinlockCreate(&tp->xmit_lock, "tp.xmit_lock");

	main_printf(ATTR_DEBUG, "Identified 8139 chip type '%s'\n",rtl_chip_info[tp->chipset].name);

	/*
	 * Find the connected MII xcvrs. Doing this in open() would allow detecting
	 * external xcvrs later, but takes too much time.
	 */
	tp->phys[0] = 32;
	tp->mii.phy_id = tp->phys[0];

	/* The lower four bits are the media type */
	option = (board_idx >= MAX_UNITS) ? 0 : media[board_idx];
	if (option > 0) {
		tp->mii.full_duplex = (option & 0x210) ? 1 : 0;
		tp->default_port = option & 0xFF;
		if (tp->default_port)
			tp->medialock = 1;
	}
	if (board_idx < MAX_UNITS  &&  full_duplex[board_idx] > 0)
		tp->mii.full_duplex = full_duplex[board_idx];
	if (tp->mii.full_duplex) {
		main_printf(ATTR_INFO, "Media type forced to Full Duplex.\n");

		/* Changing the MII-advertised media because might prevent re-connection */
		tp->mii.duplex_lock = 1;
	}

	if (tp->default_port) {
		main_printf(ATTR_INFO, "  Forcing %dMbps %s-duplex operation.\n",
				(option & 0x20 ? 100 : 10), (option & 0x10 ? "full" : "half"));
		mdio_write(dev, tp->phys[0], 0,
				((option & 0x20) ? 0x2000 : 0) |   /* 100Mbps */
				((option & 0x10) ? 0x0100 : 0));   /* Full duplex */
	}

	netif_add( dev, NULL, NULL, NULL, dev->state, rtl8139_init_lwip, tcpip_input );

	return 0;
}


/* Function searches RTL 8139 interfaces */
int rtl8139too_init(void)
{
	int cards_found = 0;
	pci_device_t *pdev;
	unsigned int i;

	/* Find all RTL 8139 devices */
	for (i = 0; rtl8139_pci_tbl[i].vendor != 0; i++) {
		do{
			dev_pciAlloc(&rtl8139_pci_tbl[i], &pdev);
			if(pdev)
				if (rtl8139_init_one(pdev) == 0) {
					cards_found++;
					/* give up after the first NIC is initialized - workaround for #342 */
					break;
				}
		}while(pdev);
	}

	if (!cards_found)
		return -ENOMEM;
	return 0;
}


/* Initialize the Rx and Tx rings, along with various 'dev' bits */
static void rtl8139_init_ring(struct netif *dev)
{
	struct rtl8139_private *tp = dev->state;
	int i;

	tp->cur_rx = 0;
	tp->cur_tx = 0;
	tp->dirty_tx = 0;

	for (i = 0; i < NUM_TX_DESC; i++)
		tp->tx_buf[i] = &tp->tx_bufs[i * TX_BUF_SIZE];
}


static void rtl_check_media(struct netif *dev)
{
	struct rtl8139_private *tp = dev->state;

	if (tp->phys[0] >= 0) {
		u16 mii_lpa = mdio_read(dev, tp->phys[0], MII_LPA);

		if (mii_lpa == 0xffff);
		else if ((mii_lpa & LPA_100FULL) == LPA_100FULL || (mii_lpa & 0x00C0) == LPA_10FULL)
			tp->mii.full_duplex = 1;

		main_printf(ATTR_INFO, "%s: Setting %s%s-duplex based on auto-negotiated partner ability %x.\n",
				dev->name,
				mii_lpa == 0 ? "" : (mii_lpa & 0x0180) ? "100mbps " : "10mbps ",
				tp->mii.full_duplex ? "full" : "half", mii_lpa);
	}
}


/* Start the hardware at open or resume */
static void rtl8139_hw_start(struct netif *dev)
{
	struct rtl8139_private *tp = dev->state;
	void *ioaddr = tp->mmio_addr;
	u32 i;
	u8 tmp;

	/* Bring old chips out of low-power mode */
	if (rtl_chip_info[tp->chipset].flags & HasHltClk)
		RTL_W8 (HltClk, 'R');

	rtl8139_chip_reset(ioaddr);

	/* unlock Config[01234] and BMCR register writes */
	RTL_W8_F(Cfg9346, Cfg9346_Unlock);

	/* Restore our idea of the MAC address */
	RTL_W32_F(MAC0 + 0, cpu_to_le32 (*(u32 *) (dev->hwaddr + 0)));
	RTL_W32_F(MAC0 + 4, cpu_to_le32 (*(u32 *) (dev->hwaddr + 4)));

	/* Must enable Tx/Rx before setting transfer thresholds! */
	RTL_W8(ChipCmd, CmdRxEnb | CmdTxEnb);

	tp->rx_config = rtl8139_rx_config | AcceptBroadcast | AcceptMyPhys;
	RTL_W32(RxConfig, tp->rx_config);

	/* Check this value: the documentation for IFG contradicts ifself */
	RTL_W32(TxConfig, rtl8139_tx_config);

	tp->cur_rx = 0;

	rtl_check_media(dev);

	if (tp->chipset >= CH_8139B) {
		/*
		 * Disable magic packet scanning, which is enabled
		 * when PM is enabled in Config1.  It can be reenabled
		 * via ETHTOOL_SWOL if desired
		 */
		RTL_W8(Config3, RTL_R8 (Config3) & ~Cfg3_Magic);
	}

	/* Lock Config[01234] and BMCR register writes */
	RTL_W8(Cfg9346, Cfg9346_Lock);

	/* init Rx ring buffer DMA address */
	RTL_W32_F(RxBuf, tp->rx_ring_dma);

	/* init Tx buffer DMA addresses */
	for (i = 0; i < NUM_TX_DESC; i++)
		RTL_W32_F(TxAddr0 + (i * 4), tp->tx_bufs_dma + (tp->tx_buf[i] - tp->tx_bufs));

	RTL_W32(RxMissed, 0);

	/* no early-rx interrupts */
	RTL_W16(MultiIntr, RTL_R16 (MultiIntr) & MultiIntrClear);

	/* make sure RxTx has started */
	tmp = RTL_R8(ChipCmd);
	if ((!(tmp & CmdRxEnb)) || (!(tmp & CmdTxEnb)))
		RTL_W8(ChipCmd, CmdRxEnb | CmdTxEnb);

	/* Enable all known interrupts by setting the interrupt mask. */
	RTL_W16(IntrMask, rtl8139_intr_mask);
}


static int rtl8139_open(struct netif *dev)
{
	struct rtl8139_private *tp = dev->state;
	page_t *page;

	if (hal_interruptsSetHandler(tp->irq, rtl8139_interrupt, (void *)dev) < 0)
		return -EINVAL;
	page=vm_pageAlloc(TX_BUF_TOT_LEN/SIZE_PAGE+1,vm_pageAlloc);
	if (vm_kmap(page, PGHD_WRITE | PGHD_PRESENT, (void**) &(tp->tx_bufs) ) == EOK)
		vm_kmapResolve(tp->tx_bufs,&tp->tx_bufs_dma);

	page=NULL;
	page=vm_pageAlloc(RX_BUF_TOT_LEN/SIZE_PAGE+1,vm_pageAlloc);
	if (vm_kmap(page, PGHD_WRITE | PGHD_PRESENT, (void**) &(tp->rx_ring) ) == EOK)
		vm_kmapResolve(tp->rx_ring,&tp->rx_ring_dma);

	if (tp->tx_bufs == NULL || tp->rx_ring == NULL) {
		main_printf(ATTR_ERROR, "%s:-ENOMEM\n",__FUNCTION__);
		// TODO uninstall irq handler.
		//low_maskirq(dev->irq, 1);
		//irq_uninstall(dev->irq, rtl8139_interrupt, (void *)dev);
/* FIXME!
		if (tp->tx_bufs)
			vm_pageFree(tp->tx_bufs);

		if (tp->rx_ring)
			vm_pageFree(tp->rx_ring);
*/
		return -ENOMEM;
	}

	tp->mii.full_duplex = tp->mii.duplex_lock;
	tp->tx_flag = (TX_FIFO_THRESH << 11) & 0x003f0000;
	tp->twistie = 1;
	tp->time_to_die = 0;

	rtl8139_init_ring(dev);
	rtl8139_hw_start(dev);

	return 0;
}


#if 0
static void rtl8139_tx_clear (struct rtl8139_private *tp)
{
	tp->cur_tx = 0;
	tp->dirty_tx = 0;
}

static int rtl8139_close(struct netif *dev)
{
	struct rtl8139_private *tp = dev->state;
	void *ioaddr = tp->mmio_addr; 

	netif_stop_queue(dev);
	proc_spinlockSet(&tp->lock);

	/* Stop the chip's Tx and Rx DMA processes */
	RTL_W8 (ChipCmd, 0);

	/* Disable interrupts by clearing the interrupt mask */
	RTL_W16 (IntrMask, 0);

	/* Update the error counts */
	tp->stats.rx_missed_errors += RTL_R32 (RxMissed);
	RTL_W32 (RxMissed, 0);

	proc_spinlockClear(&tp->lock, sopGetCycles);

	//low_maskirq(dev->irq, 1);
	//irq_uninstall(dev->irq, rtl8139_interrupt, (void *)dev);

	rtl8139_tx_clear (tp);

	if (tp->tx_bufs)
		/*pci_free(tp->pci_dev, TX_BUF_TOT_LEN, tp->tx_bufs, tp->tx_bufs_dma);*/
		vm_pageFree(tp->tx_bufs);
	if (tp->rx_ring)
		vm_pageFree(tp->rx_ring);
	/*pci_free(tp->pci_dev, RX_BUF_TOT_LEN, tp->rx_ring, tp->rx_ring_dma);*/
	tp->rx_ring = NULL;
	tp->tx_bufs = NULL;

	/* Green! Put the chip in low-power mode */
	RTL_W8(Cfg9346, Cfg9346_Unlock);
	if (rtl_chip_info[tp->chipset].flags & HasHltClk)
		RTL_W8 (HltClk, 'H');	/* 'R' would leave the clock running. */
	return 0;
}
#endif

#if 0
static netdev_stat_t *rtl8139_getstats(struct netif *dev)
{
	struct rtl8139_private *tp = dev->state;
	void *ioaddr = tp->mmio_addr;

	proc_spinlockSet(&tp->lock);
	tp->stats.rx_missed_errors += RTL_R32(RxMissed);
	RTL_W32(RxMissed, 0);
	proc_spinlockClear(&tp->lock, sopGetCycles);
	return &tp->stats;
}
#endif

static err_t rtl8139_start_xmit(struct netif *dev, struct pbuf *skb)
{
	struct rtl8139_private *tp = dev->state;
	void *ioaddr = tp->mmio_addr;
	unsigned int entry;
	unsigned int len = skb->tot_len;
	
	proc_spinlockSet(&tp->xmit_lock);

	if (tp->stopped) {
		tp->stats.tx_dropped++;
		proc_spinlockClear(&tp->xmit_lock, sopGetCycles);
		main_printf(ATTR_ERROR, "xmit stopped. droping \n");
		return 0;
	}

	/* Calculate the next Tx descriptor entry */
	entry = tp->cur_tx % NUM_TX_DESC;

	int cnts=0;
	if (len < TX_BUF_SIZE) {
		int off=0;
		memset(tp->tx_buf[entry], 0, ETH_ZLEN); // FIXME
		while(skb){
			cnts++;
			hal_memcpy(tp->tx_buf[entry]+off, skb->payload, skb->len);
			off+=skb->len;
			skb=skb->next;
		}
	}
	else {
		tp->stats.tx_dropped++;
		proc_spinlockClear(&tp->xmit_lock, sopGetCycles);
		return 0;
	}

{ int k; __asm__ ("xorl %%eax, %%eax; xchgl %%eax, %0"::"m" (k):"memory","eax"); }
	/* Note: the chip doesn't have auto-pad! */
	RTL_W32_F(TxStatus0 + (entry * sizeof(u32)), tp->tx_flag | max(len, (unsigned int)ETH_ZLEN));
	tp->cur_tx++;

	if ((tp->cur_tx - NUM_TX_DESC) == tp->dirty_tx)
		tp->stopped=1;

//	debug(ATTR_DEBUG, "%s: QueuedA Tx packet size %d %d (%d skbufs) to slot %d.\n", 1?"someif":dev->name, len, ETH_ZLEN, cnts, entry);

	proc_spinlockClear(&tp->xmit_lock, sopGetCycles);

	return 0;
}


static void rtl8139_tx_interrupt(struct netif *dev, struct rtl8139_private *tp, void *ioaddr)
{
	unsigned long dirty_tx, tx_left;
	int entry;
	int txstatus;

	proc_spinlockSet(&tp->xmit_lock);

	dirty_tx = tp->dirty_tx;
	tx_left = tp->cur_tx - dirty_tx;


	while (tx_left > 0) {
		entry = dirty_tx % NUM_TX_DESC;
		txstatus = RTL_R32(TxStatus0 + (entry * sizeof (u32)));

		if (!(txstatus & (TxStatOK | TxUnderrun | TxAborted)))
			break;

		/* Note: TxCarrierLost is always asserted at 100mbps */
		if (txstatus & (TxOutOfWindow | TxAborted)) {
			tp->stats.tx_errors++;

			if (txstatus & TxAborted) {
				tp->stats.tx_aborted_errors++;
				RTL_W32(TxConfig, TxClearAbt);
				RTL_W16(IntrStatus, TxErr);
			}
			if (txstatus & TxCarrierLost)
				tp->stats.tx_carrier_errors++;
			if (txstatus & TxOutOfWindow)
				tp->stats.tx_window_errors++;
		}
		else {
			if (txstatus & TxUnderrun) {
				/* Add 64 to the Tx FIFO threshold */
				if (tp->tx_flag < 0x00300000)
					tp->tx_flag += 0x00020000;
				tp->stats.tx_fifo_errors++;
			}
			tp->stats.collisions += (txstatus >> 24) & 15;
			tp->stats.tx_bytes += txstatus & 0x7ff;
			tp->stats.tx_packets++;
		}
		dirty_tx++;
		tx_left--;
	}

	/* only wake the queue if we did work, and the queue is stopped */
	if (tp->dirty_tx != dirty_tx) {
		tp->dirty_tx = dirty_tx;
		tp->stopped=0;
	}
	proc_spinlockClear(&tp->xmit_lock, sopGetCycles);
}


/* TODO: clean this up!  Rx reset need not be this intensive */
static void rtl8139_rx_err (u32 rx_status, struct netif *dev, struct rtl8139_private *tp, void *ioaddr)
{
	u8 tmp8;

	tp->stats.rx_errors++;
	if (!(rx_status & RxStatusOK)) {
		if (rx_status & RxTooLong) {
			/* A.C.: The chip hangs here */
		}
		if (rx_status & (RxBadSymbol | RxBadAlign))
			tp->stats.rx_frame_errors++;
		if (rx_status & (RxRunt | RxTooLong))
			tp->stats.rx_length_errors++;
		if (rx_status & RxCRCErr)
			tp->stats.rx_crc_errors++;
	}
	else {
		tp->xstats.rx_lost_in_ring++;
	}

	tmp8 = RTL_R8(ChipCmd);
	RTL_W8(ChipCmd, tmp8 & ~CmdRxEnb);
	RTL_W8(ChipCmd, tmp8);
	RTL_W32(RxConfig, tp->rx_config);
	tp->cur_rx = 0;
}


static void rtl8139_rx_interrupt(struct netif *dev, struct rtl8139_private *tp, void *ioaddr)
{
	unsigned char *rx_ring;
	u16 cur_rx;
	int ring_offset, received = 0;
	u32 rx_status;
	unsigned int rx_size, pkt_size;
	struct pbuf *skb;

	rx_ring = tp->rx_ring;
	cur_rx = tp->cur_rx;

	if(0) debug(ATTR_INFO, "%s: In rtl8139_rx(), current %x BufAddr %x, free to %x, Cmd %x.\n",
			dev->name,  cur_rx, RTL_R16(RxBufAddr), RTL_R16(RxBufPtr), RTL_R8(ChipCmd));

	while ((RTL_R8(ChipCmd) & RxBufEmpty) == 0) {
		ring_offset = cur_rx % RX_BUF_LEN;

		/* read size+status of next frame from DMA ring buffer */
		rx_status = le32_to_cpu (*(u32 *) (rx_ring + ring_offset));
		rx_size = rx_status >> 16;
		pkt_size = rx_size - 4;

		if(0)debug(ATTR_INFO, "%s:  rtl8139_rx() status %x, size %x, cur %x.\n", dev->name, rx_status, rx_size, cur_rx );

		/*
		 * Packet copy from FIFO still in progress - theoretically, this should
		 * never happen since EarlyRx is disabled
		 */
		if (rx_size == 0xfff0) {
			tp->xstats.early_rx++;
			break;
		}

		/*
		 * If Rx err or invalid rx_size/rx_status received (which happens if we
		 * get lost in the ring), Rx process gets reset, so we abort any further
		 * Rx processing
		 */
		if ((rx_size > (MAX_ETH_FRAME_SIZE + 4)) || (rx_size < 8) || (!(rx_status & RxStatusOK))) {
			rtl8139_rx_err(rx_status, dev, tp, ioaddr);
			return;
		}

		/*
		 * Malloc up new buffer, compatible with net-2e.
		 * Omit the four octet CRC from the length.
		 */
		skb = pbuf_alloc(PBUF_RAW, pkt_size, PBUF_POOL);
		if (skb) {
			struct pbuf *q;
			int off=0;
			for(q=skb; q ; q=q->next){
				/*XXX might be better to use pbuf_ helper funcs instead*/
				hal_memcpy(q->payload, (&rx_ring[ring_offset + 4])+off, q->len);
				off+=q->len;
			}

			/* Process packets through the LWIP stack */
			dev->input(skb, dev);

			tp->stats.rx_bytes += pkt_size;
			tp->stats.rx_packets++;
		}
		else {
			main_printf(ATTR_ERROR, "%s: Memory squeeze, dropping packet.\n", dev->name);
			tp->stats.rx_dropped++;
		}
		received++;
		cur_rx = (cur_rx + rx_size + 4 + 3) & ~3;
		RTL_W16(RxBufPtr, cur_rx - 16);

		RTL_W16_F(IntrStatus, RxAckBits);
	}

	if(0)debug(ATTR_INFO, "%s: Done rtl8139_rx(), current %x BufAddr %x, free to %x, Cmd %x.\n",
			dev->name, cur_rx, RTL_R16 (RxBufAddr), RTL_R16 (RxBufPtr), RTL_R8 (ChipCmd));

	tp->cur_rx = cur_rx;

	if (!received || (rx_size == 0xfff0)) {
		RTL_W16_F(IntrStatus, RxAckBits);
	}
}


static void rtl8139_weird_interrupt(struct netif *dev, struct rtl8139_private *tp, void *ioaddr, int status, int link_changed)
{
	int lpar;
	int duplex;

	main_printf(ATTR_INFO, "%s: Abnormal interrupt, status %x.\n", dev->name, status);

	/* Update the error count */
	tp->stats.rx_missed_errors += RTL_R32 (RxMissed);
	RTL_W32 (RxMissed, 0);

	if ((status & RxUnderrun) && link_changed && (tp->drv_flags & HAS_LNK_CHNG)) {
		/* Really link-change on new chips */
		lpar = RTL_R16(NWayLPAR);

		duplex = (lpar & LPA_100FULL) || (lpar & 0x01C0) == 0x0040 || tp->mii.duplex_lock;
		if (tp->mii.full_duplex != duplex) {
			tp->mii.full_duplex = duplex;
		}
		status &= ~RxUnderrun;
	}

	/* XXX along with rtl8139_rx_err, are we double-counting errors? */
	if (status & (RxUnderrun | RxOverflow | RxErr | RxFIFOOver))
		tp->stats.rx_errors++;

	if (status & PCSTimeout)
		tp->stats.rx_length_errors++;
	if (status & (RxUnderrun | RxFIFOOver))
		tp->stats.rx_fifo_errors++;
	if (status & PCIErr)
		main_printf(ATTR_ERROR, "%s: PCI Bus error.\n" ,dev->name );
}


/* The interrupt handler does all of the Rx thread work and cleans up after the Tx thread */
static int rtl8139_interrupt(unsigned int irq,cpu_context_t *ctx, void *dev_instance)
{
	struct netif *dev = (struct netif *)dev_instance;
	struct rtl8139_private *tp = dev->state;
	void *ioaddr = tp->mmio_addr;
	int ackstat, status;
	int link_changed = 0;
	int res = IHRES_IGNORE;

	proc_spinlockSet(&tp->lock);

	status = RTL_R16(IntrStatus);

	if ((status & (PCIErr | PCSTimeout | RxUnderrun | RxOverflow | RxFIFOOver | TxErr | TxOK | RxErr | RxOK)) == 0) {
		proc_spinlockClear(&tp->lock, sopGetCycles);
		return res;
	}
	res = IHRES_HANDLED;

	/* h/w no longer present (hotplug?) or major error, bail */
	if (status == 0xFFFF) {
		proc_spinlockClear(&tp->lock, sopGetCycles);
		return res;
	}

	/*
	 * Acknowledge all of the current interrupt sources ASAP, but
	 * an first get an additional status bit from CSCR
	 */
	if (status & RxUnderrun)
		link_changed = RTL_R16(CSCR) & CSCR_LinkChangeBit;

	/*
	 * The chip takes special action when we clear RxAckBits,
	 * so we clear them later in rtl8139_rx_interrupt
	 */
	ackstat = status & ~(RxAckBits | TxErr);
	if (ackstat)
		RTL_W16(IntrStatus, ackstat);

	if(0)debug(ATTR_INFO, "%s: interrupt  status=%x ackstat=%x.\n",
			dev->name, ackstat, status);

	if (status & RxAckBits)
		rtl8139_rx_interrupt (dev, tp, ioaddr);

	/* Check uncommon events with one test */
	if (status & (PCIErr | PCSTimeout | RxUnderrun | /* RxOverflow | RxFIFOOver |*/ RxErr))
		rtl8139_weird_interrupt(dev, tp, ioaddr, status, link_changed);

	if (status & (TxOK | TxErr)) {
		rtl8139_tx_interrupt (dev, tp, ioaddr);
		if (status & TxErr)
			RTL_W16(IntrStatus, TxErr);
	}

	proc_spinlockClear(&tp->lock, sopGetCycles);
	return res;
}
