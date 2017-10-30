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
 * Author: Pawel. Pisarczyk, Pawel Kolodziej
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <config.h>
#include <hal/if.h>
#include <main/if.h>
#include <vm/if.h>
#include <dev/if.h>
#include <net/if.h>

#include <dev/eepro100/eepro100.h>

#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/ethip6.h"
#include "netif/etharp.h"
#include "lwip/tcpip.h"

#define EE_READ_CMD		(6)

/*  EEPROM_Ctrl bits */
#define EE_SHIFT_CLK   0x01              /* EEPROM shift clock. */
#define EE_CS          0x02              /* EEPROM chip select. */
#define EE_DATA_WRITE  0x04              /* EEPROM chip data in. */
#define EE_DATA_READ   0x08              /* EEPROM chip data out. */
#define EE_ENB         (0x4800 | EE_CS)
#define EE_WRITE_0     0x4802
#define EE_WRITE_1     0x4806
#define EE_OFFSET      SCBeeprom


/*
 * Set the copy breakpoint for the copy-only-tiny-buffer Rx method.
 * Lower values use more memory, but are faster.
 */
static int rx_copybreak = 200;


const pci_id_t eepro100_ids[] = {
	{ PCI_VENDOR_INTEL, 0x1209, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1229, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x2449, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1029, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1030, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1031, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1032, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1033, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1034, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1035, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1036, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1037, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1038, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1039, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x103A, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x103B, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x103C, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x103D, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x103E, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1227, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x1228, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x2449, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x2459, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x245D, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x5200, PCI_ANY, PCI_ANY, PCI_ANY },
	{ PCI_VENDOR_INTEL, 0x5201, PCI_ANY, PCI_ANY, PCI_ANY },
	{ 0, 0, 0, 0}
};


static void wait_for_cmd_done(void *cmd_ioaddr)
{
	int wait = 1000;

	while (hal_inb(cmd_ioaddr) && --wait >= 0);
}


static int mdio_read(void *iobase, int phy_id, int location)
{
	int val, boguscnt = 64*10;		/* <64 usec. to complete, typ 27 ticks */

	hal_outl(iobase + SCBCtrlMDI, 0x08000000 | (location << 16) | (phy_id << 21));

	do {
    val = hal_inl(iobase + SCBCtrlMDI);
		if (--boguscnt < 0) {
			main_printf(ATTR_ERROR, " mdio_read() timed out with val = %x.\n", val);
			break;
		}
	} while (! (val & 0x10000000));
	return val & 0xffff;
}


static int mdio_write(void *iobase, int phy_id, int location, int value)
{
	int val, boguscnt = 64 * 10;

	hal_outl(iobase + SCBCtrlMDI, 0x04000000 | (location << 16) | (phy_id << 21) | value);

	do {
		val = hal_inl(iobase + SCBCtrlMDI);
		if (--boguscnt < 0) {
			main_printf(ATTR_ERROR, " mdio_write() timed out with val = %x.\n", val);
			break;
		}
	} while (!(val & 0x10000000));
	return val & 0xffff;
}


#if 0
static void _eepro100_miiReset(netdev_t *dev)
{
	struct speedo_private *sp = (struct speedo_private *)dev->priv;
	void *ioaddr = dev->base;

	/* Reset the MII transceiver, suggested by Fred Young @ scalable.com */
	if ((sp->phy[0] & 0x8000) == 0) {
		int phy_addr = sp->phy[0] & 0x1f;
		int advertising = mdio_read(ioaddr, phy_addr, 4);
		int mii_bmcr = mdio_read(ioaddr, phy_addr, 0);
		
		mdio_write(ioaddr, phy_addr, 0, 0x0400);
		mdio_write(ioaddr, phy_addr, 1, 0x0000);
		mdio_write(ioaddr, phy_addr, 4, 0x0000);
		mdio_write(ioaddr, phy_addr, 0, 0x8000);

		mdio_read(ioaddr, phy_addr, 0);
		mdio_write(ioaddr, phy_addr, 0, mii_bmcr);
		mdio_write(ioaddr, phy_addr, 4, advertising);
	}
}
#endif


static void _eepro100_showState(struct netif *dev)
{
	struct speedo_private *sp = (struct speedo_private *)dev->state;
	int i;

	main_printf(ATTR_DEBUG, "%s: Tx ring dump,  Tx queue %d / %d:\n", dev->name, sp->cur_tx, sp->dirty_tx);
	
	for (i = 0; i < TX_RING_SIZE; i++)
		main_printf(ATTR_DEBUG, "%s: %c%c%d %x\n", dev->name,
			i == sp->dirty_tx % TX_RING_SIZE ? '*' : ' ', i == sp->cur_tx % TX_RING_SIZE ? '=' : ' ',
			i, sp->tx_ring[i].status);

	main_printf(ATTR_DEBUG, "\n%s: Printing Rx ring (next to receive into %d, dirty index %d)\n",
		dev->name, sp->cur_rx, sp->dirty_rx);

	for (i = 0; i < RX_RING_SIZE; i++)
		main_printf(ATTR_DEBUG, "%s: %c%c%c%d %x | ", dev->name,
			sp->rx_ringp[i] == sp->last_rxf ? 'l' : ' ',
			i == sp->dirty_rx % RX_RING_SIZE ? '*' : ' ',
			i == sp->cur_rx % RX_RING_SIZE ? '=' : ' ',
			i, (sp->rx_ringp[i] != NULL) ? (unsigned)sp->rx_ringp[i]->status : 0);
	
	main_printf(ATTR_DEBUG, "\n");

	return;
}

#if 0
static netif_stat_t *eepro100_getStats(struct netif *dev)
{
	struct speedo_private *sp = (struct speedo_private *)dev->state;
	void *ioaddr = sp->base;

	/* Update only if the previous dump finished */
	if (sp->lstats->done_marker == le32_to_cpu(0xA007)) {
		sp->stats.tx_aborted_errors += le32_to_cpu(sp->lstats->tx_coll16_errs);
		sp->stats.tx_window_errors += le32_to_cpu(sp->lstats->tx_late_colls);
		sp->stats.tx_fifo_errors += le32_to_cpu(sp->lstats->tx_underruns);
		sp->stats.tx_fifo_errors += le32_to_cpu(sp->lstats->tx_lost_carrier);		
		/*sp->stats.tx_deferred += le32_to_cpu(sp->lstats->tx_deferred);*/
		
		sp->stats.collisions += le32_to_cpu(sp->lstats->tx_total_colls);
		sp->stats.rx_crc_errors += le32_to_cpu(sp->lstats->rx_crc_errs);
		sp->stats.rx_frame_errors += le32_to_cpu(sp->lstats->rx_align_errs);
		sp->stats.rx_over_errors += le32_to_cpu(sp->lstats->rx_resource_errs);
		sp->stats.rx_fifo_errors += le32_to_cpu(sp->lstats->rx_overrun_errs);
		sp->stats.rx_length_errors += le32_to_cpu(sp->lstats->rx_runt_errs);
		sp->lstats->done_marker = 0x0000;

		wait_for_cmd_done(ioaddr + SCBCmd);
		hal_outb(ioaddr + SCBCmd, CUDumpStats);
	}
	return &sp->stats;
}
#endif

#if 0
static void _eepro100_txPurge(netdev_t *dev)
{
	struct speedo_private *sp = (struct speedo_private *)dev->priv;
	struct speedo_mc_block *t;
	int entry;

	while ((int)(sp->cur_tx - sp->dirty_tx) > 0) {
		entry = sp->dirty_tx % TX_RING_SIZE;
		if (sp->tx_skbuff[entry]) {
			sp->stats.tx_errors++;
			dev_kfree_skb(sp->tx_skbuff[entry]);
			sp->tx_skbuff[entry] = 0;
		}
		sp->dirty_tx++;
	}
	
	while (sp->mc_setup_head != NULL) {		
		t = sp->mc_setup_head->next;
		vm_kfree(sp->mc_setup_head);
		sp->mc_setup_head = t;
	}
	sp->mc_setup_tail = NULL;
	sp->tx_full = 0;

	netif_wake_queue(dev);
}
#endif


static struct RxFD *speedo_rx_alloc(struct netif *dev, int entry)
{
	struct speedo_private *sp = (struct speedo_private *)dev->state;
	struct RxFD *rxf;
	struct pbuf *skb;

	/* Get a fresh skbuff to replace the consumed one */
	skb = pbuf_alloc(PBUF_RAW, PKT_BUF_SZ + sizeof(struct RxFD), PBUF_RAM);

	sp->rx_skbuff[entry] = skb;
	if (skb == NULL) {
		sp->rx_ringp[entry] = NULL;
		return NULL;
	}

	rxf = sp->rx_ringp[entry] = (struct RxFD *)skb->payload;
	vm_kmapResolve(rxf, &sp->rx_ring_dma[entry]);

	pbuf_header(skb,(s16_t) -sizeof(struct RxFD));
	rxf->rx_buf_addr = 0xffffffff;

{ int k; __asm__ ("xorl %%eax, %%eax; xchgl %%eax, %0"::"m" (k):"memory","eax"); }
	/* pci_dmasync(sp->pdev, sp->rx_ring_dma[entry], sizeof(struct RxFD), PCI_DMA_TODEVICE); */
	return rxf;
}


static void speedo_rx_link(struct netif *dev, int entry, struct RxFD *rxf, addr_t rxf_dma)
{
	struct speedo_private *sp = (struct speedo_private *)dev->state;

	rxf->status = cpu_to_le32(0xC0000001); 	/* '1' for driver use only. */
	rxf->link = 0;
	
	rxf->count = cpu_to_le32(PKT_BUF_SZ << 16);
	sp->last_rxf->link = cpu_to_le32(rxf_dma);
	sp->last_rxf->status &= cpu_to_le32(~0xC0000000);

/*  pci_dmasync(sp->pdev, sp->last_rxf_dma, sizeof(struct RxFD), PCI_DMA_TODEVICE); */
{ int k; __asm__ ("xorl %%eax, %%eax; xchgl %%eax, %0"::"m" (k):"memory","eax"); }

	sp->last_rxf = rxf;
	sp->last_rxf_dma = rxf_dma;
}


static int speedo_refill_rx_buf(struct netif *dev, int force)
{
	struct speedo_private *sp = (struct speedo_private *)dev->state;
	int entry;
	struct RxFD *rxf;

	entry = sp->dirty_rx % RX_RING_SIZE;

	if (sp->rx_skbuff[entry] == NULL) {
		rxf = speedo_rx_alloc(dev, entry);
		if (rxf == NULL) {
			unsigned int forw;
			int forw_entry;
			main_printf(ATTR_ERROR, "%s: no mem, force=%d\n", __FUNCTION__, force );
			
			if (!(sp->rx_ring_state & RrOOMReported)) {
				main_printf(ATTR_ERROR, "%s: can't fill rx buffer (force %d)!\n", dev->name, force);
				_eepro100_showState(dev);
				sp->rx_ring_state |= RrOOMReported;
			}
			
			if (!force)
				return -1;

			/* Borrow an skb from one of next entries */
			for (forw = sp->dirty_rx + 1; forw != sp->cur_rx; forw++)
				if (sp->rx_skbuff[forw % RX_RING_SIZE] != NULL)
					break;
			if (forw == sp->cur_rx)
				return -1;

			forw_entry = forw % RX_RING_SIZE;
			sp->rx_skbuff[entry] = sp->rx_skbuff[forw_entry];
			sp->rx_skbuff[forw_entry] = NULL;
			rxf = sp->rx_ringp[forw_entry];
			sp->rx_ringp[forw_entry] = NULL;
			sp->rx_ringp[entry] = rxf;
		}
	}
	else {
		rxf = sp->rx_ringp[entry];
	}
	
	speedo_rx_link(dev, entry, rxf, sp->rx_ring_dma[entry]);
	sp->dirty_rx++;
	sp->rx_ring_state &= ~(RrNoMem | RrOOMReported); /* Mark the progress. */
	return 0;
}


static void speedo_refill_rx_buffers(struct netif *dev, int force)
{
	struct speedo_private *sp = (struct speedo_private *)dev->state;

	/* Refill the RX ring. */
	while ((int)(sp->cur_rx - sp->dirty_rx) > 0 && speedo_refill_rx_buf(dev, force) != -1);
}


static void speedo_tx_buffer_gc(struct netif *dev)
{
	unsigned int dirty_tx;
	struct speedo_private *sp = (struct speedo_private *)dev->state;

	dirty_tx = sp->dirty_tx;
	while ((int)(sp->cur_tx - dirty_tx) > 0) {
		int entry = dirty_tx % TX_RING_SIZE;
		int status = le32_to_cpu(sp->tx_ring[entry].status);

//		main_printf(ATTR_ERROR,"tx status %p\n", status);
 		/* It still hasn't been processed */
		if ((status & StatusComplete) == 0)
			break;
		if (status & TxUnderrun){
			main_printf(ATTR_ERROR, "tx unterrun\n");
			if (sp->tx_threshold < 0x01e08000) {
				sp->tx_threshold += 0x00040000;
			}
		}

		/* Free the original skb */
		if (sp->tx_skbuff[entry]) {
			sp->stats.tx_packets++;	/* Count only user packets */
			sp->stats.tx_bytes += sp->tx_skbuff[entry]->len;

			pbuf_free(sp->tx_skbuff[entry]);
			sp->tx_skbuff[entry] = 0;
		}
		dirty_tx++;
	}

	if ((int)(sp->cur_tx - dirty_tx) > TX_RING_SIZE) {
		main_printf(ATTR_ERROR, "out-of-sync dirty pointer, %d vs. %d, full=%d.\n", dirty_tx, sp->cur_tx, sp->tx_full);
		dirty_tx += TX_RING_SIZE;
	}

	while (sp->mc_setup_head != NULL && (int)(dirty_tx - sp->mc_setup_head->tx - 1) > 0) {
		struct speedo_mc_block *t;
		t = sp->mc_setup_head->next;
		vm_kfree(sp->mc_setup_head);
		sp->mc_setup_head = t;
	}
	if (sp->mc_setup_head == NULL)
		sp->mc_setup_tail = NULL;

	sp->dirty_tx = dirty_tx;
}


/* Set or clear the multicast filter for this adaptor */
static void set_rx_mode(struct netif *dev)
{
	struct speedo_private *sp = (struct speedo_private *)dev->state;
	void *ioaddr = sp->base;
	struct descriptor *last_cmd;
	char new_rx_mode;
	int entry;
	static int txfifo = 8;        /* Tx FIFO threshold in 4 byte units, 0-15 */
	static int rxfifo = 8;        /* Rx FIFO threshold, default 32 bytes. */
	
	/* Tx/Rx DMA burst length, 0-127, 0 == no preemption, x==128 -> disabled. */
	static int txdmacount = 128;
	static int rxdmacount = 0;
	
	new_rx_mode = 0;

	if ((int)(sp->cur_tx - sp->dirty_tx) > TX_RING_SIZE - TX_MULTICAST_SIZE) {
		sp->rx_mode = -1;
		return;
	}

	if (new_rx_mode != sp->rx_mode) {
		u8 *config_cmd_data;

		/* spin_lock_irqsave(&sp->lock, flags); */
		hal_cpuDisableInterrupts();

		entry = sp->cur_tx++ % TX_RING_SIZE;
		last_cmd = sp->last_cmd;
		sp->last_cmd = (struct descriptor *)&sp->tx_ring[entry];

		sp->tx_skbuff[entry] = 0;			/* Redundant */
		
		sp->tx_ring[entry].status = cpu_to_le32(CmdSuspend | CmdConfigure);
		sp->tx_ring[entry].link = cpu_to_le32(TX_RING_ELEM_DMA(sp, (entry + 1) % TX_RING_SIZE));

		config_cmd_data = (void *)&sp->tx_ring[entry].tx_desc_addr;

		/* Construct a full CmdConfig frame. */
		hal_memcpy(config_cmd_data, (void *)i82558_config_cmd, CONFIG_DATA_SIZE);
		config_cmd_data[1] = (txfifo << 4) | rxfifo;
		config_cmd_data[4] = rxdmacount;
		config_cmd_data[5] = txdmacount + 0x80;
		config_cmd_data[15] |= (new_rx_mode & 2) ? 1 : 0;

		/*
		 * 0x80 doesn't disable FC 0x84 does.
		 * Disable Flow control since we are not ACK-ing any FC interrupts
		 * for now. --Dragan
		 */
		config_cmd_data[19] = 0x84;
		config_cmd_data[19] |= sp->full_duplex ? 0x40 : 0;
		config_cmd_data[21] = (new_rx_mode & 1) ? 0x0D : 0x05;

		/* Use the AUI port instead. */
		if (sp->phy[0] & 0x8000) {
			config_cmd_data[15] |= 0x80;
			config_cmd_data[8] = 0;
		}

		/* Trigger the command unit resume. */
		wait_for_cmd_done(ioaddr + SCBCmd);
		clear_suspend(last_cmd);
		hal_outb(ioaddr + SCBCmd, CUResume);

		if ((int)(sp->cur_tx - sp->dirty_tx) >= TX_QUEUE_LIMIT) {
			sp->stopped=1;
			sp->tx_full = 1;
		}
		
		hal_cpuEnableInterrupts();
		/* spin_unlock_irqrestore(&sp->lock, flags); */
	}

	sp->rx_mode = new_rx_mode;
}


/* Start the chip hardware after a full reset */
static void speedo_resume(struct netif *dev)
{
	struct speedo_private *sp = (struct speedo_private *)dev->state;
	void *ioaddr = sp->base;

	/* Start with a Tx threshold of 256 (0x..20.... 8 byte units). */
	sp->tx_threshold = 0x01208000;

	/* Set the segment registers to '0'. */
	wait_for_cmd_done(ioaddr + SCBCmd);
	hal_outl(ioaddr + SCBPointer, 0);

	/* impose a delay to avoid a bug */
	hal_inl(ioaddr + SCBPointer);
	//udelay(10);
	
	{
		int k;
    	for (k = 0; k < 10000000; k++);
	}
  
	hal_outb(ioaddr + SCBCmd, RxAddrLoad);
	wait_for_cmd_done(ioaddr + SCBCmd);
	hal_outb(ioaddr + SCBCmd, CUCmdBase);

	/* Load the statistics block and rx ring addresses. */
	wait_for_cmd_done(ioaddr + SCBCmd);
	hal_outl(ioaddr + SCBPointer, sp->lstats_dma);
	hal_outb(ioaddr + SCBCmd, CUStatsAddr);
	sp->lstats->done_marker = 0;

	if (sp->rx_ringp[sp->cur_rx % RX_RING_SIZE] != NULL) {
		wait_for_cmd_done(ioaddr + SCBCmd);
		hal_outl(ioaddr + SCBPointer, sp->rx_ring_dma[sp->cur_rx % RX_RING_SIZE]);
		hal_outb(ioaddr + SCBCmd, RxStart);
	}

	wait_for_cmd_done(ioaddr + SCBCmd);
	hal_outb(ioaddr + SCBCmd, CUDumpStats);
  
	{
		int k;
		for (k = 0; k < 10000000; k++);
	}
	//udelay(30);

	/* Fill the first command with our physical address. */
	{
		struct descriptor *ias_cmd;

		ias_cmd = (struct descriptor *)&sp->tx_ring[sp->cur_tx++ % TX_RING_SIZE];

		/* Avoid a bug(?!) here by marking the command already completed. */
		ias_cmd->cmd = cpu_to_le16(0xa000);
		ias_cmd->status = 0x4001; /* CmdSuspend | CmdIASetup */

		ias_cmd->link = cpu_to_le32(TX_RING_ELEM_DMA(sp, sp->cur_tx % TX_RING_SIZE));
		hal_memcpy(ias_cmd->params, dev->hwaddr, 6);
		sp->last_cmd = ias_cmd;
	}

	/* Start the chip's Tx process and unmask interrupts. */
	wait_for_cmd_done(ioaddr + SCBCmd);

	hal_outl(ioaddr + SCBPointer, TX_RING_ELEM_DMA(sp, sp->dirty_tx % TX_RING_SIZE));
	hal_outw(ioaddr + SCBCmd, CUStart | SCBMaskEarlyRx | SCBMaskFlowCtl);
}


static void speedo_rx_soft_reset(struct netif *dev)
{
	struct speedo_private *sp = dev->state;
	struct RxFD *rfd;
	void *ioaddr;

	ioaddr = sp->base;
	wait_for_cmd_done(ioaddr + SCBCmd);
	if (hal_inb(ioaddr + SCBCmd) != 0) {
		main_printf(ATTR_DEBUG, "%s: previous command stalled\n", dev->name);
		return;
	}

	/* Put hardware into known state */
	hal_outb(ioaddr + SCBCmd, RxAbort);

	rfd = sp->rx_ringp[sp->cur_rx % RX_RING_SIZE];
	rfd->rx_buf_addr = 0xffffffff;

	wait_for_cmd_done(ioaddr + SCBCmd);

	if (hal_inb(ioaddr + SCBCmd) != 0) {
		main_printf(ATTR_DEBUG, "%s: RxAbort command stalled\n", dev->name);
		return;
	}
	hal_outl(ioaddr + SCBPointer, sp->rx_ring_dma[sp->cur_rx % RX_RING_SIZE]);
	hal_outb(ioaddr + SCBCmd, RxStart);
}


static err_t speedo_start_xmit(struct netif *dev, struct pbuf *skb)
{
	struct speedo_private *sp = (struct speedo_private *)dev->state;
	void *ioaddr = sp->base;
	int entry;

	proc_mutexLock(&sp->mutex);
//	main_printf(ATTR_INFO, "%s %s sending %d bytes\n", dev->name, __FUNCTION__, skb->tot_len);

	/* Check if there is enough space */
	if ((int)(sp->cur_tx - sp->dirty_tx) >= TX_QUEUE_LIMIT) {
		main_printf(ATTR_ERROR, "%s: incorrect tbusy state, fixed.\n", dev->name);
		sp->stopped = 1;
		sp->tx_full = 1;
		proc_mutexUnlock(&sp->mutex);
		return 1;
	}

	/* Calculate Tx descriptor entry */
	entry = sp->cur_tx++ % TX_RING_SIZE;

	sp->tx_skbuff[entry] = pbuf_alloc(PBUF_RAW, skb->tot_len, PBUF_RAM); 
	pbuf_copy(sp->tx_skbuff[entry], skb);
	sp->tx_ring[entry].status = cpu_to_le32(CmdSuspend | CmdTx | CmdTxFlex);

	if (!(entry & ((TX_RING_SIZE >> 2) - 1)))
		sp->tx_ring[entry].status |= cpu_to_le32(CmdIntr);

	sp->tx_ring[entry].link = cpu_to_le32(TX_RING_ELEM_DMA(sp, sp->cur_tx % TX_RING_SIZE));
	sp->tx_ring[entry].tx_desc_addr = cpu_to_le32(TX_RING_ELEM_DMA(sp, entry) + TX_DESCR_BUF_OFFSET);

	/* The data region is always in one buffer descriptor */
	sp->tx_ring[entry].count = cpu_to_le32(sp->tx_threshold);

	vm_kmapResolve(sp->tx_skbuff[entry]->payload, &sp->tx_ring[entry].tx_buf_addr0);

	sp->tx_ring[entry].tx_buf_addr0 = cpu_to_le32(sp->tx_ring[entry].tx_buf_addr0);
	sp->tx_ring[entry].tx_buf_size0 = cpu_to_le32(skb->tot_len);

#if 0
	unsigned i;
	for(i=0; i< skb->tot_len; i++)
	{
		main_printf(ATTR_INFO, "%x ", ((unsigned char*)skb->payload)[i]);
	}
	main_printf(ATTR_INFO, "---------------- %d\n", skb->tot_len);
#endif
	{ int k; __asm__ volatile  ("xorl %%eax, %%eax; xchgl %%eax, %0"::"m" (k):"memory","eax"); }
	/* workaround for hardware bug on 10 mbit half duplex */

	if ((sp->partner == 0) && (sp->chip_id == 1)) {
		wait_for_cmd_done(ioaddr + SCBCmd);
		hal_outb(ioaddr + SCBCmd, 0);
	}

	/* Trigger command unit resume */
	wait_for_cmd_done(ioaddr + SCBCmd);
	clear_suspend(sp->last_cmd);

	/*
	 * We want the time window between clearing suspend flag on the previous
	 * command and resuming CU to be as small as possible.
	 * Interrupts in between are very undesired.  --SAW
	 */
	hal_outb(ioaddr + SCBCmd, CUResume);
	sp->last_cmd = (struct descriptor *)&sp->tx_ring[entry];

	/*
	 * Leave room for set_rx_mode(). If there is no more space than reserved
	 * for multicast filter mark the ring as full.
	 */
	if ((int)(sp->cur_tx - sp->dirty_tx) >= TX_QUEUE_LIMIT) {
		sp->stopped = 1;
		main_printf(ATTR_ERROR, "%s: incorrect tbusy state, fixed.\n", dev->name);
		sp->tx_full = 1;
	}

	proc_mutexUnlock(&sp->mutex);

	return 0;
}


/* Function receives frames */
static int speedo_rx(struct netif *dev)
{
	struct speedo_private *sp = (struct speedo_private *)dev->state;
	int entry = sp->cur_rx % RX_RING_SIZE;
	int rx_work_limit = sp->dirty_rx + RX_RING_SIZE - sp->cur_rx;
	int alloc_ok = 1;

	/* If we own the next entry, it's a new packet. Send it up. */
	while (sp->rx_ringp[entry] != NULL) {
		int status;
		int pkt_len;

		status = le32_to_cpu(sp->rx_ringp[entry]->status);
		pkt_len = le32_to_cpu(sp->rx_ringp[entry]->count) & 0x3fff;

		if (!(status & RxComplete)) {
//			main_printf(ATTR_ERROR, "Rx not complete %x\n",status);
			break;
		}

		if (--rx_work_limit < 0){
//			main_printf(ATTR_ERROR, "work limit\n");
			break;
		}

		/*
		 * Check for a rare out-of-memory case: the current buffer is
		 * the last buffer allocated in the RX ring.  --SAW
		 */
		if (sp->last_rxf == sp->rx_ringp[entry]) {

			/*
			 * Postpone the packet.  It'll be reaped at an interrupt when this
			 * packet is no longer the last packet in the ring.
			 */
			sp->rx_ring_state |= RrPostponed;
			main_printf(ATTR_INFO, "postponing\n");
			break;
		}

		if ((status & (RxErrTooBig|RxOK|0x0f90)) != RxOK) {

			if (status & RxErrTooBig)
				main_printf(ATTR_ERROR, "%s: Ethernet frame overran the Rx buffer, status %x!\n", dev->name, status);
			else if (!(status & RxOK)) {

				/* There was a fatal error.  This *should* be impossible. */
				sp->stats.rx_errors++;
				main_printf(ATTR_ERROR, "%s: Anomalous event in speedo_rx(), status %x.\n", dev->name, status);
			}else
				main_printf(ATTR_ERROR, "%s: rx strange status %x\n", dev->name, status);
		}
		else {
			struct pbuf *skb=NULL;
//			main_printf(ATTR_INFO, "%s rx frame %d bytes\n", __FUNCTION__, pkt_len);

			/*
			 * Check if the packet is long enough to just accept without
			 * copying to a properly sized skbuff.
			 */
			if ((pkt_len < rx_copybreak) && ((skb = pbuf_alloc(PBUF_RAW, pkt_len, PBUF_RAM)) != NULL)) {
				/* pci_dmasync(sp->pdev, sp->rx_ring_dma[entry], sizeof(struct RxFD) + pkt_len, PCI_DMA_FROMDEVICE); */
				pbuf_take(skb, sp->rx_skbuff[entry]->payload, pkt_len);
//				main_printf(ATTR_ERROR, " %d/%d\n", sp->rx_skbuff[entry]->len, sp->rx_skbuff[entry]->tot_len);
			}
			else {

				/* Pass up the already-filled skbuff */
				skb = sp->rx_skbuff[entry];
				if (skb == NULL) {
					main_printf(ATTR_ERROR, "%s: Inconsistent Rx descriptor chain.\n", dev->name);
					break;
				}
				pbuf_realloc(skb, pkt_len);
				sp->rx_skbuff[entry] = NULL;
				sp->rx_ringp[entry] = NULL;

				/* pci_unmap(sp->pdev, sp->rx_ring_dma[entry], PKT_BUF_SZ + sizeof(struct RxFD), PCI_DMA_FROMDEVICE); */
			}

			/* Process packets through network stack */
			dev->input(skb, dev);

			sp->stats.rx_packets++;
			sp->stats.rx_bytes += pkt_len;
		}
		entry = (++sp->cur_rx) % RX_RING_SIZE;
		sp->rx_ring_state &= ~RrPostponed;

		/*
		 * Refill the recently taken buffers.
		 * Do it one-by-one to handle traffic bursts better.
		 */
		if (alloc_ok && speedo_refill_rx_buf(dev, 0) == -1)
			alloc_ok = 0;
	}

	/* Try hard to refill the recently taken buffers. */
	speedo_refill_rx_buffers(dev, 1);

	return 0;
}


int speedo_interrupt(unsigned int n, cpu_context_t *ctx, void *arg)
{
	struct netif *dev = (struct netif *)arg;
	struct speedo_private *sp;
	void *ioaddr;
	u16 status;
	sp = (struct speedo_private *)dev->state;
	ioaddr = sp->base;

	status = hal_inw(ioaddr + SCBStatus);

	if ((status & 0xfc00) == 0)
	{
		return IHRES_IGNORE;
	}
  
	/* Acknowledge all of the current interrupt sources ASAP */
	hal_outw(ioaddr + SCBStatus, status & 0xfc00);

	/* Packet received, or Rx error or need to gather the postponed packet. */
	if ((status & 0x5000) || (sp->rx_ring_state & (RrNoMem | RrPostponed)) == RrPostponed) {
		speedo_rx(dev);
	}

	/* Always check if all rx buffers are allocated.  --SAW */
	speedo_refill_rx_buffers(dev, 0);

	/* spin_lock(&sp->lock); */

	/*
	 * The chip may have suspended reception for various reasons.
	 * Check for that, and re-prime it should this be the case.
	 */
	switch ((status >> 2) & 0xf) {
	case 0:  /* Idle */
		break;
	case 1:  /* Suspended */     
	case 2:  /* No resources (RxFDs) */
	case 9:  /* Suspended with no more RBDs */
	case 10: /* No resources due to no RBDs */
	case 12: /* Ready with no RBDs */
		speedo_rx_soft_reset(dev);
		break;
	case 3:
	case 5:
	case 6:
	case 7:
	case 8:
	case 11:
	case 13:
	case 14:
	case 15:
		break;
	}

	/* User interrupt, Command/Tx unit interrupt or CU not active. */
	if (status & 0xA400) {
		speedo_tx_buffer_gc(dev);
		if (sp->tx_full && (int)(sp->cur_tx - sp->dirty_tx) < TX_QUEUE_UNFULL) {

			/* The ring is no longer full */
			sp->tx_full = 0;

			/* Attention: under a spinlock.  --SAW */
			sp->stopped = 0;
		}
	}

	/* spin_unlock(&sp->lock); */

	return IHRES_HANDLED;
}


/* Initialize the Rx and Tx rings, along with various 'dev' bits */
static void speedo_init_rx_ring(struct netif *dev)
{
	struct speedo_private *sp = (struct speedo_private *)dev->state;
	struct RxFD *rxf, *last_rxf = NULL;
	addr_t last_rxf_dma = 0;
	int i;

	sp->cur_rx = 0;

	for (i = 0; i < RX_RING_SIZE; i++) {
		struct pbuf *skb;
		skb = pbuf_alloc(PBUF_RAW, PKT_BUF_SZ + sizeof(struct RxFD), PBUF_RAM);
		sp->rx_skbuff[i] = skb;

		/* OK.  Just initially short of Rx bufs. */
		if (skb == NULL)
			break;

		rxf = (struct RxFD *)skb->payload;
		sp->rx_ringp[i] = rxf;
		
		vm_kmapResolve(rxf, &sp->rx_ring_dma[i]);		

		pbuf_header(skb, (s16_t) -sizeof(struct RxFD));

		if (last_rxf) {
			last_rxf->link = cpu_to_le32(sp->rx_ring_dma[i]);

			/* pci_dmasync(sp->pdev, last_rxf_dma, sizeof(struct RxFD), PCI_DMA_TODEVICE); */
{ int k; __asm__ ("xorl %%eax, %%eax; xchgl %%eax, %0"::"m" (k):"memory","eax"); }

		}

		last_rxf = rxf;
		last_rxf_dma = sp->rx_ring_dma[i];
		rxf->status = cpu_to_le32(0x00000001);	/* '1' is flag value only. */
		rxf->link = 0;

		/* This field unused by i82557 */
		rxf->rx_buf_addr = 0xffffffff;
		rxf->count = cpu_to_le32(PKT_BUF_SZ << 16);
{ int k; __asm__ ("xorl %%eax, %%eax; xchgl %%eax, %0"::"m" (k):"memory","eax"); }
		/* pci_dmasync(sp->pdev, sp->rx_ring_dma[i], sizeof(struct RxFD), PCI_DMA_TODEVICE); */
	}

	sp->dirty_rx = (unsigned int)(i - RX_RING_SIZE);

	/* Mark the last entry as end-of-list */
	last_rxf->status = cpu_to_le32(0xC0000002);

{ int k; __asm__ ("xorl %%eax, %%eax; xchgl %%eax, %0"::"m" (k):"memory","eax"); }
	/* pci_dmasync(sp->pdev, sp->rx_ring_dma[RX_RING_SIZE-1], sizeof(struct RxFD), PCI_DMA_TODEVICE); */

	sp->last_rxf = last_rxf;
	sp->last_rxf_dma = last_rxf_dma;
}


/* Function opens network device for use with UDP/IP */
static int eepro100_open(struct netif *dev)
{
	struct speedo_private *sp = (struct speedo_private *)dev->state;
	void *ioaddr = sp->base;

	/* Set up the Tx queue early */
	sp->cur_tx = 0;
	sp->dirty_tx = 0;
	sp->last_cmd = 0;
	sp->tx_full = 0;

	proc_spinlockCreate(&sp->lock,"eepro100");
	proc_mutexCreate(&sp->mutex);

	sp->in_interrupt = 0;

	speedo_init_rx_ring(dev);

	/* Install driver ISR */
	hal_interruptsSetHandler(sp->irq, speedo_interrupt, (void *)dev);
    

	/* Fire up the hardware */
	hal_outw(ioaddr + SCBCmd, SCBMaskAll);
	speedo_resume(dev);

	/* Setup the chip and configure the multicast list */
	sp->mc_setup_head = NULL;
	sp->mc_setup_tail = NULL;
	sp->flow_ctrl = sp->partner = 0;
	sp->rx_mode = -1;

	set_rx_mode(dev);

	if ((sp->phy[0] & 0x8000) == 0)
		sp->advertising = mdio_read(ioaddr, sp->phy[0] & 0x1f, 4);

	/* No need to wait for the command unit to accept here */
	if ((sp->phy[0] & 0x8000) == 0)
		mdio_read(ioaddr, sp->phy[0] & 0x1f, 0);

	return 0;
}

#if 0
static int eepro100_close(netdev_t *dev)
{
	void *ioaddr = dev->base;
	struct speedo_private *sp = (struct speedo_private *)dev->priv;
	int i;

	/* Shutting down the chip nicely fails to disable flow control. So.. */
	hal_outl(ioaddr + SCBPort, PortPartialReset);

	/* hal_interruptsRemoveHandler(dev->irq, speedo_interrupt, (void *)dev); */

	/* Free all the skbuffs in the Rx and Tx queues. */
	for (i = 0; i < RX_RING_SIZE; i++) {
		struct sk_buff *skb = sp->rx_skbuff[i];
		sp->rx_skbuff[i] = 0;

		/* Clear the Rx descriptors. */
		if (skb) {
			dev_kfree_skb(skb);
		}
	}

	for (i = 0; i < TX_RING_SIZE; i++) {
		struct sk_buff *skb = sp->tx_skbuff[i];
		sp->tx_skbuff[i] = 0;

		/* Clear the Tx descriptors. */
		if (skb) {
			dev_kfree_skb(skb);
		}
	}

	return 0;
}
#endif


static int eepro100_eepromCmd(void *ioaddr, int cmd, int cmd_len)
{
	unsigned retval = 0;
	void *ee_addr = ioaddr + SCBeeprom;
	u16 dataval;

	hal_outw(ee_addr, EE_ENB);
	hal_outw(ee_addr, EE_ENB | EE_SHIFT_CLK);

	/* Shift the command bits out */
	do {
		dataval = (cmd & (1 << cmd_len)) ? EE_WRITE_1 : EE_WRITE_0;
		hal_outw(ee_addr, dataval);
		hal_outw(ee_addr, dataval | EE_SHIFT_CLK);
		retval = (retval << 1) | ((hal_inw(ee_addr) & EE_DATA_READ) ? 1 : 0);
	} while (--cmd_len >= 0);

	hal_outw(ee_addr, EE_ENB);

	/* Terminate the EEPROM access */
	hal_outw(ee_addr, EE_ENB & ~EE_CS);
	return retval;
}


void eepro100_dumpEeprom(u16 *eeprom)
{
	char *product;
	unsigned int i;
  
	static const char *connectors[] = { " RJ45", " BNC", " AUI", " MII" };  
	static const char *phys[] = {
		"None", "i82553-A/B", "i82553-C", "i82503",
		"DP83840", "80c240", "80c24", "i82555",
		"unknown-8", "unknown-9", "DP83840A", "unknown-11",
		"unknown-12", "unknown-13", "unknown-14", "unknown-15",
	};

	if (eeprom[3] & 0x0100)
		product = "OEM i82557/i82558 10/100 Ethernet";
	else
		product = "<unknown Intel chip>";

	main_printf(ATTR_INFO, "%s\n", product);

	if ((eeprom[3] & 0x03) != 0x03)
		main_printf(ATTR_INFO, "Receiver lock-up bug exists -- enabling work-around.\n");

	main_printf(ATTR_INFO, "Board assembly %x%x%d, Physical connectors present: ", eeprom[8], eeprom[9] >> 8, eeprom[9] & 0xff);
	for (i = 0; i < 4; i++)
		if (eeprom[5] & (1 << i))
			main_printf(ATTR_INFO, "%s", connectors[i]);
	main_printf(ATTR_INFO, "\nPrimary interface chip %s PHY #%d.\n", phys[(eeprom[6] >> 8) & 15], eeprom[6] & 0x1f);

	if (eeprom[7] & 0x0700)
		main_printf(ATTR_INFO, "Secondary interface chip %s\n", phys[(eeprom[7] >> 8) & 7]);
	return; 
}


int eepro100_test(struct netif *dev)
{
	volatile s32 *results;
	int boguscnt = 16000;
	struct speedo_private *sp = (struct speedo_private *)dev->state;
	unsigned int i;

	/* Perform a system self-test. */
	results = (s32 *)((((long)sp->tx_ring) + 15) & ~0xf);
	results[0] = 0;
	results[1] = -1;
	hal_outl(sp->base + SCBPort, sp->tx_ring_dma | PortSelfTest);

	do {
		for (i = 0; i < 100000000; i++);

	} while (results[1] == -1 );

	if (boguscnt < 0) {
		main_printf(ATTR_ERROR, "Self test failed, status %x:\n", results[1]);
	}
	else {
		main_printf(ATTR_DEBUG, "General self-test: %s.\n"
		"Serial sub-system self-test: %s.\n"
		"Internal registers self-test: %s.\n"
		"ROM checksum self-test: %s (%x)\n",
		results[1] & 0x1000 ? "failed" : "passed",
		results[1] & 0x0020 ? "failed" : "passed",
		results[1] & 0x0008 ? "failed" : "passed",
		results[1] & 0x0004 ? "failed" : "passed",
		results[0]);
	}

	hal_outl(sp->base + SCBPort, PortReset);
	hal_inl(sp->base + SCBPort);
  
	for (i = 0; i < 10000000; i++);
  
	return EOK;
}



static err_t eepro100_init_lwip(struct netif *netdev)
{
	unsigned int i;
	int read_cmd, ee_size, j, mdi_reg23;
	u16 eeprom[0x100], sum, value;
	page_t *page;
	void *tx_ring_space;
	addr_t tx_ring_dma;
	struct speedo_private *sp = (struct speedo_private *)netdev->state;

	/* Prepare TX descriptors */
	if ((page = vm_pageAlloc(1, vm_pageAlloc)) == NULL)
		return -ENOMEM;
	if (vm_kmap(page, PGHD_WRITE | PGHD_PRESENT, (void **)&tx_ring_space) < 0) {
		vm_pageFree(page);
		return -ENOMEM;
	}  
	tx_ring_dma = page->addr;
 
	/* Read configuration data from EEPROM */
	if ((eepro100_eepromCmd(sp->base, EE_READ_CMD << 24, 27) & 0xffe0000) == 0xffe0000) {
		ee_size = 0x100;
		read_cmd = EE_READ_CMD << 24;
	}
	else {
		ee_size = 0x40;
		read_cmd = EE_READ_CMD << 22;
	}

	for (j = 0, i = 0, sum = 0; i < ee_size; i++) {
		value = eepro100_eepromCmd(sp->base, read_cmd | (i << 16), 27);
		eeprom[i] = value;
		sum += value;

		if (i < 3) {
			netdev->hwaddr[j++] = value;
			netdev->hwaddr[j++] = value >> 8;
		}
	}
	netdev->hwaddr_len = 6;
	if (sum != 0xBABA) {
		main_printf(ATTR_ERROR, "Invalid EEPROM checksum %x\n", sum);
		return -1;
	}

	/* Dump information from EEPROM */
	eepro100_dumpEeprom(eeprom);

	/* Reset chip */
	hal_outl(sp->base + SCBPort, PortReset);
	hal_inl(sp->base + SCBPort);

{
	int k;
  	for (k = 0; k < 10000000; k++);
}
    
	if (((eeprom[6] >> 8) & 0x3f) == DP83840 ||  ((eeprom[6] >> 8) & 0x3f) == DP83840A) {
		mdi_reg23 = mdio_read(sp->base, eeprom[6] & 0x1f, 23) | 0x0422;
		main_printf(ATTR_DEBUG, "DP83840 specific setup, setting register 23 to %x.\n", mdi_reg23);
		mdio_write(sp->base, eeprom[6] & 0x1f, 23, mdi_reg23);
	}
	
	sp->tx_ring = tx_ring_space;
	sp->tx_ring_dma = tx_ring_dma;
	sp->lstats = (struct speedo_stats *)(sp->tx_ring + TX_RING_SIZE*sizeof(struct TxFD));
	sp->lstats_dma = TX_RING_ELEM_DMA(sp, TX_RING_SIZE);
  
	sp->full_duplex = 1;
  
	sp->phy[0] = eeprom[6];
	sp->phy[1] = eeprom[7];
	sp->rx_bug = (eeprom[3] & 0x03) == 3 ? 0 : 1;

	if (sp->rx_bug)
		main_printf(ATTR_INFO, "Receiver lock-up workaround activated.\n");

	netdev->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_LINK_UP;
	netdev->linkoutput = speedo_start_xmit;
	netdev->output = etharp_output;
	netdev->mtu = 1500;
	strcpy(netdev->name,"eth");
    
	/* Perform interface test */
	eepro100_test(netdev);

	eepro100_open(netdev);
	return ERR_OK;
}
/* Function tests existence of Intel EtherExpres device */
int eepro100_initOne(pci_device_t *dev, int card)
{
	unsigned int i;
	struct netif *netdev;
	struct speedo_private *sp;


	/* Register network device */
	netdev = vm_kmalloc( sizeof(struct netif) + sizeof(struct speedo_private));
	if (netdev == NULL) {
		main_printf(ATTR_ERROR, "dev/eepro100: Can't register new network device!\n");
		return -ENOMEM;
	}  
	memset(netdev, 0, sizeof(struct netif) + sizeof(struct speedo_private));
	netdev->state = ((void*)netdev) + sizeof(struct netif);
	sp = (struct speedo_private *)netdev->state;

	/* Get device base address */
	for (i = 0; i < 6; i++) {
		if (dev->resources[i].base & 1) {
			sp->base = (void *)(dev->resources[i].base & ~1);
			break;
		}
	}
	sp->irq = dev->irq;
	sp->pcidev = dev;
	main_printf(ATTR_ERROR, " irq %d io %p\n", sp->irq, sp->base);
	if (((dev->device > 0x1030 && (dev->device < 0x103F))) ||
		(dev->device == 0x2449) || (dev->device == 0x2459) || (dev->device == 0x245D)) {
		sp->chip_id = 1;
	}

 	netif_add( netdev, NULL, NULL, NULL, netdev->state, eepro100_init_lwip, tcpip_input );

	return 0;
}


/* Function lookups for Intel EtherExpress100 interfaces */
int dev_eepro100Init(void)
{
	pci_device_t *dev;
       	unsigned i, cards=0;

	/* Find all EEPRO100 devices */
	for (i = 0; eepro100_ids[i].vendor != 0; i++) {
		do{
			dev_pciAlloc(&eepro100_ids[i], &dev);

			if (dev) {
				if (eepro100_initOne(dev, cards) == 0)
					cards++;
			}
		}while(dev);
	}
	return 0;
}
