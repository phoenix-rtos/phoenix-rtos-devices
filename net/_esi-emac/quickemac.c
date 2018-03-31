/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * eSi-EMAC Ethernet controller driver
 *
 * Copyright 2012 Phoenix Systems
 *
 * Author: Jacek Popko
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/if.h>
#include <lwip/opt.h>
#include <lwip/def.h>
#include <lwip/mem.h>
#include <lwip/pbuf.h>
#include <lwip/sys.h>
#include <lwip/stats.h>
#include <netif/etharp.h>
#include <lwip/tcpip.h>
#include <lib/assert.h>

/* eSi-RISC includes. */
#include <esirisc/emac.h>
#include <esirisc/device.h>
#include "ringbuf.h"

#define IFNAME "eth"
#define SIZE_ETH_FRAME  1514

/* Ethernet address */
#define emacETHADDR0 0x00
#define emacETHADDR1 0x17
#define emacETHADDR2 0xa9
#define emacETHADDR3 0x00
#define emacETHADDR4 0x00
#define emacETHADDR5 0x01

#define SP605_PHY_ADDR	0x07
#define C3DEVKIT_PHY_ADDR	0x12
#define MARVELL_PHY_ID 0x0141


typedef struct {
	esi_emac_t *regs;
	event_t rx_evt, tx_evt, txtrig_evt;
	semaphore_t tx_sem;
	ringbuf_t *rx_rb, *tx_rb;
	unsigned received, transmited, overflow;
} emac_state_t;

 emac_state_t *emacemac;

static void emac_phyInitSP605(esi_emac_t *emac_dev, int addr)
{
	unsigned short value;

	/* Disable 1000bps mode on 88e1111 as the MAC doesn't support it */
	value = esi_emac_mdio_read(emac_dev, addr, 9);
	value &= ~((1 << 9) | (1 << 8));
	esi_emac_mdio_write(emac_dev, addr, 9, value);

	/* Re-negotiate */
	value = esi_emac_mdio_read(emac_dev, addr, 0);
	value |= 1 << 9;
	esi_emac_mdio_write(emac_dev, addr, 0, value);
}


static void emac_phyInitC3DevKit(esi_emac_t *emac_dev, int addr)
{
	unsigned short value;

	/* Disable 1000bps mode on 88e1111 as the MAC doesn't support it */
	value = esi_emac_mdio_read(emac_dev, addr, 9);
	value &= ~((1 << 9) | (1 << 8));
	esi_emac_mdio_write(emac_dev, addr, 9, value);

	/* Set GMII to Copper */
	value = esi_emac_mdio_read(emac_dev, addr, 0x1b);
	value |= 0xf;
	esi_emac_mdio_write(emac_dev, addr, 0x1b, value);

	/* Disable Timing controls */
	value = esi_emac_mdio_read(emac_dev, addr, 0x14);
	value &= ~0x82;
	esi_emac_mdio_write(emac_dev, addr, 0x14, value);

	/* Enable auto-cross over. */
	esi_emac_mdio_write(emac_dev, addr, 16, 0x78);

	/* Reset */
	value = esi_emac_mdio_read(emac_dev, addr, 0);
	value |= 1 << 15;
	esi_emac_mdio_write(emac_dev, addr, 0, value);
}


/* List of possible PHY addresses terminated with '0' entry */
static struct {
	void (*init)(esi_emac_t *emac_dev, int addr);
	u8	addr;
} phyHandler[] = {
	{
		.init = emac_phyInitSP605,
		.addr = SP605_PHY_ADDR
	},
	{
		.init = emac_phyInitC3DevKit,
		.addr = C3DEVKIT_PHY_ADDR
	},
	{
		.init = NULL,
		.addr = 0
	}
};


/**
 * MAC and PHY initialization
 *
 * @param netdev the already initialized lwip network interface structure
 */
static err_t emac_hwInit(struct netif *netdev)
{
	esi_emac_t *emac_dev;
	emac_state_t *emac_state;
	int i;
	u16 phy_id;

	emac_state = (emac_state_t *)netdev->state;
	emac_dev = emac_state->regs;

	/* set MAC hardware address length */
	netdev->hwaddr_len = ETHARP_HWADDR_LEN;

	/* set MAC hardware address */
	netdev->hwaddr[0] = emacETHADDR0;
	netdev->hwaddr[1] = emacETHADDR1;
	netdev->hwaddr[2] = emacETHADDR2;
	netdev->hwaddr[3] = emacETHADDR3;
	netdev->hwaddr[4] = emacETHADDR4;
	netdev->hwaddr[5] = emacETHADDR5;

	/* maximum transfer unit */
	netdev->mtu = 1500;

	/* device capabilities */
	netdev->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

	/* Set MAC address */
	emac_dev->mac_address_0 = emacETHADDR0;
	emac_dev->mac_address_1 = emacETHADDR1;
	emac_dev->mac_address_2 = emacETHADDR2;
	emac_dev->mac_address_3 = emacETHADDR3;
	emac_dev->mac_address_4 = emacETHADDR4;
	emac_dev->mac_address_5 = emacETHADDR5;

	for (i = 0; phyHandler[i].addr != 0; i++)
	{
		phy_id = esi_emac_mdio_read(emac_dev, phyHandler[i].addr, 2);
		if (MARVELL_PHY_ID == phy_id) {
			phyHandler[i].init(emac_dev, phyHandler[i].addr);
			break;
		}
	}

	/* Enable EMAC and interrupts */
	emac_dev->status = ESI_EMAC_RX_COMPLETE | ESI_EMAC_TX_COMPLETE;
	emac_dev->control = ESI_EMAC_RX_INT_ENABLE | ESI_EMAC_TX_INT_ENABLE |
				ESI_EMAC_FILTER_LENGTH_ENABLE | ESI_EMAC_FILTER_FCS_ENABLE |
				ESI_EMAC_FILTER_ADDRESS_ENABLE | ESI_EMAC_FULL_DUPLEX | ESI_EMAC_ENABLE;

	return ERR_OK;
}


/** EMAC Interrupt Handler */
static int emac_isr(unsigned irq, cpu_context_t *ctx, void *arg)
{
	struct netif *netdev = (struct netif *)arg;
	emac_state_t *emac_state = (emac_state_t *)netdev->state;
	esi_emac_t *emac = emac_state->regs;

	if (emac->status & ESI_EMAC_TX_COMPLETE) {
		/* Acknowledge TX complete interrupt */
		emac->status = ESI_EMAC_TX_COMPLETE;
		emac_state->transmited++;
		proc_eventSignal(&emac_state->tx_evt);
	}

	while (emac->status & ESI_EMAC_RX_COMPLETE) {
		if (ringbuf_fill(emac_state->rx_rb, emac))
			emac_state->received++;
		else {
			emac_state->overflow++;
			emac->status = ESI_EMAC_RX_COMPLETE;
			break;
		}

		/* acknowledge that packet has been read */
		emac->status = ESI_EMAC_RX_COMPLETE;


		/* Indicate to the receive task that a frame is available */
		proc_eventSignal(&emac_state->rx_evt);
	}

	return IHRES_HANDLED;
}


static err_t emac_tx(struct netif *netif, struct pbuf *p)
{
	emac_state_t *emac_state = (emac_state_t *)netif->state;

#if ETH_PAD_SIZE
	pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

	proc_semaphoreDown(&emac_state->tx_sem);
	pbuf_ref(p);
	ringbuf_put(emac_state->tx_rb, p);
	proc_eventSignal(&emac_state->txtrig_evt);
	LINK_STATS_INC(link.xmit);

	return ERR_OK;
}


static int emac_txThread(void *arg)
{
	struct netif *netdev = (struct netif *)arg;
	emac_state_t *emac_state = (emac_state_t *)netdev->state;
	esi_emac_t *emac = emac_state->regs;


	for (;;) {
		struct pbuf *p;

		if (proc_eventWait(&emac_state->txtrig_evt, 0) != EOK)
			break;

		while ((p = ringbuf_get(emac_state->tx_rb))) {
			struct pbuf *q;
			int length = 0;
			int i;

			proc_semaphoreUp(&emac_state->tx_sem);
			if (proc_eventWait(&emac_state->tx_evt, 0) != EOK)
				break;

			for (q = p; q != NULL && length < p->tot_len; q = q->next) {
				/* Send the data from the pbuf to the interface, one pbuf at a
				   time. The size of the data in each pbuf is kept in the ->len
				   variable. */
				for (i = 0; i < q->len; i++)
					emac->tx_data = ((u8 *)q->payload)[i];
				length += q->len;
			}
			/* Pad frame to be at least 60-bytes. */
			while (length < 60) {
				emac->tx_data = 0;
				length++;
			}

			emac->tx_length = length;
#if ETH_PAD_SIZE
			pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
			pbuf_free(p);
		}
	}
	return 0;
}


static int emac_rx(struct netif *netif, struct pbuf *p)
{
	struct eth_hdr *ethhdr;
	int status = EOK;

	/* no packet could be read, silently ignore this */
	if (p == NULL) return -EINVAL;
	/* points to packet payload, which starts with an Ethernet header */
	ethhdr = p->payload;

	switch (ntohs(ethhdr->type)) {
			/* IP or ARP packet? */
		case ETHTYPE_IP:
		case ETHTYPE_ARP:
			/* full packet send to tcpip_thread to process */
			if (netif->input(p, netif) != ERR_OK)
			{
				LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
				status = -EBUSY;
			}
			break;

		default:
			status = -EINVAL;
			break;
	}
	return status;
}


/**
 * Receive task.
 *
 * @param netif the network interface to pass received frames to
 */
static int emac_rxThread(void *arg)
{
	struct netif *netdev = (struct netif *)arg;
	emac_state_t *emac_state = (emac_state_t *)netdev->state;

	for (;;)
	{
		struct pbuf *p;

		if (proc_eventWait(&emac_state->rx_evt, 0) != EOK)
			break;

		while(!ringbuf_empty(emac_state->rx_rb)) {
				p = pbuf_alloc(PBUF_RAW, SIZE_ETH_FRAME + ETH_PAD_SIZE, PBUF_POOL);
				assert(p != NULL);

#if ETH_PAD_SIZE
			pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

			p = ringbuf_drain(emac_state->rx_rb, p);
			assert(p != NULL);

#if ETH_PAD_SIZE
			pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
			LINK_STATS_INC(link.recv);

			/* process the frame */
			if (emac_rx(netdev, p) != EOK)
					pbuf_free(p);
		}
	}

	return 0;
}


static int emac_init_one(esi_device_info_t *device)
{
	struct netif *netdev;
	emac_state_t *emac_state;
	int status;
	int i;

	if (device->device_id != ESI_DID_ENSILICA_APB_EMAC) {
		return ERR_DEV_DETECT;
	}

	if ((netdev = vm_kmalloc(sizeof(struct netif) + sizeof(void *) + sizeof(emac_state_t))) == NULL) {
		main_printf(ATTR_ERROR, "Can't register new network device!\n");
		return -ENOMEM;
	}

	memset(netdev, 0 , sizeof(struct netif) + sizeof(void *) + sizeof(emac_state_t));

#if LWIP_NETIF_HOSTNAME
	/* Initialize interface hostname */
	netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

	NETIF_INIT_SNMP(netdev, snmp_ifType_ethernet_csmacd, 100);

	memcpy(netdev->name, IFNAME, sizeof(IFNAME));

	netdev->output = etharp_output;
	netdev->linkoutput = emac_tx;
	emac_state = (emac_state_t *)(netdev + 1);

	status = vm_iomap((addr_t)device->base_address, device->size, PGHD_KERNEL_RW, (void **)&emac_state->regs);
	assert(status == EOK);

	proc_eventCreate(&emac_state->rx_evt);
	proc_eventCreate(&emac_state->tx_evt);
	proc_eventSignal(&emac_state->tx_evt);
	proc_eventCreate(&emac_state->txtrig_evt);

	emac_state->tx_rb = ringbuf_alloc(8);
	proc_semaphoreCreate(&emac_state->tx_sem, emac_state->tx_rb->size_mask);

	emac_state->rx_rb = ringbuf_alloc(8);
	for (i = 0; i <= emac_state->rx_rb->size_mask; i++)
	{
		emac_state->rx_rb->data[i] = pbuf_alloc(PBUF_RAW, SIZE_ETH_FRAME + ETH_PAD_SIZE, PBUF_POOL);
#if ETH_PAD_SIZE
		pbuf_header(emac_state->rx_rb->data[i], -ETH_PAD_SIZE); /* drop the padding word */
#endif
	}

	netif_add(netdev, NULL, NULL, NULL, emac_state, emac_hwInit, tcpip_input);

	hal_interruptsSetHandler(device->irq, emac_isr, netdev);

	/* Start task to receive packets */
	proc_thread(NULL, emac_rxThread, NULL, 0, netdev, ttRegular);
	proc_thread(NULL, emac_txThread, NULL, 0, netdev, ttRegular);

	emacemac = emac_state;
	return ERR_OK;
}


int emac_init(void)
{
	esi_device_info_t *device;
	int result = 0;

	/* 0 in size field indicates end of device list. */
	for (device = esi_device_get_list(); (device != NULL) && (device->size != 0); device++) {
		if (device->device_id == ESI_DID_ENSILICA_APB_EMAC) {
			main_printf(ATTR_INFO, "dev: eSi-EMAC rev=%d, base=0x%p, cfg=0x%p, irq=%d\n",
				device->revision, device->base_address, device->config, device->irq);

			if (emac_init_one(device) != 0) {
				main_printf(ATTR_ERROR, "eSi-EMAC initialization failed\n");
				result++;
			}
		}
	}
	return result;
}

