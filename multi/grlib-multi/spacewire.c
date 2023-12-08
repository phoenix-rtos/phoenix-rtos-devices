/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <board_config.h>
#include <errno.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/debug.h>
#include <sys/minmax.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <posix/utils.h>

#if defined(__CPU_GR716)
#include <phoenix/arch/gr716.h>
#elif defined(__CPU_GR712RC)
#include <phoenix/arch/gr712rc.h>
#else
#error Unsupported target
#endif

#include <phoenix/arch/sparcv8leon3.h>

#include "spacewire.h"
#include "grlib-multi.h"

/* clang-format off */
#define TRACE(fmt, ...) do { if (0) { printf("%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } } while (0)
/* clang-format on */

#define PAGE_ALIGN(addr) (((addr_t)(addr) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1))

/* GRSPW2 registers */
#define SPW_CTRL      0
#define SPW_STATUS    1
#define SPW_NODE_ADDR 2
#define SPW_CLK_DIV   3
#define SPW_DST_KEY   4
#define SPW_TIME      5
#define DMA_CTRL      8
#define DMA_RX_LEN    9
#define DMA_TX_DESC   10
#define DMA_RX_DESC   11
#define DMA_ADDR      12

/* SPW CTRL bits */
#define SPW_CTRL_RA  (1u << 31) /* RMAP available */
#define SPW_CTRL_RX  (1u << 30) /* RX unaligned access available */
#define SPW_CTRL_RC  (1u << 29) /* RMAP CRC available */
#define SPW_CTRL_NCH (3u << 27) /* Number of DMA channels */
#define SPW_CTRL_PO  (1u << 26) /* Number of ports - 1 */
#define SPW_CTRL_PS  (1u << 21) /* Port select */
#define SPW_CTRL_NP  (1u << 20) /* Disable port force */
#define SPW_CTRL_RD  (1u << 17) /* RMAP buffer disable */
#define SPW_CTRL_RE  (1u << 16) /* RMAP enable */
#define SPW_CTRL_TR  (1u << 11) /* Time RX enable */
#define SPW_CTRL_TT  (1u << 10) /* Time TX enable */
#define SPW_CTRL_LI  (1u << 9)  /* Link error IRQ */
#define SPW_CTRL_TQ  (1u << 8)  /* Tick-out IRQ */
#define SPW_CTRL_RS  (1u << 6)  /* Reset */
#define SPW_CTRL_PM  (1u << 5)  /* Promiscuous mode */
#define SPW_CTRL_TI  (1u << 4)  /* Tick in */
#define SPW_CTRL_IE  (1u << 3)  /* Interrupt enable */
#define SPW_CTRL_AS  (1u << 2)  /* Autostart */
#define SPW_CTRL_LS  (1u << 1)  /* Link start */
#define SPW_CTRL_LD  (1u << 0)  /* Link disable */

/* DMA CTRL bits */
#define DMA_CTRL_LE  (1u << 16) /* Disable TX when link error occurs */
#define DMA_CTRL_SP  (1u << 15) /* Remove 2nd byte (protocol id) of each packet */
#define DMA_CTRL_SA  (1u << 14) /* Remove 1st byte (address) of each packet */
#define DMA_CTRL_ENA (1u << 13) /* Enable separate node address for channel */
#define DMA_CTRL_NS  (1u << 12) /* No spill */
#define DMA_CTRL_RD  (1u << 11) /* RX descriptors available */
#define DMA_CTRL_RX  (1u << 10) /* RX active (read only) */
#define DMA_CTRL_AT  (1u << 9)  /* Abort TX */
#define DMA_CTRL_RA  (1u << 8)  /* RX AHB error */
#define DMA_CTRL_TA  (1u << 7)  /* TX AHB error */
#define DMA_CTRL_PR  (1u << 6)  /* Packet received */
#define DMA_CTRL_PS  (1u << 5)  /* Packet sent */
#define DMA_CTRL_AI  (1u << 4)  /* AHB error IRQ */
#define DMA_CTRL_RI  (1u << 3)  /* RX IRQ (if set in corresponding descriptor) */
#define DMA_CTRL_TI  (1u << 2)  /* TX IRQ (if set in corresponding descriptor) */
#define DMA_CTRL_RE  (1u << 1)  /* Receiver enable */
#define DMA_CTRL_TE  (1u << 0)  /* Transmitter enable */

#define DMA_CTRL_USR_MSK (DMA_CTRL_LE | DMA_CTRL_SP | DMA_CTRL_SA | DMA_CTRL_ENA | DMA_CTRL_NS)

#define SPW_RX_DESC_CNT 128
#define SPW_TX_DESC_CNT 64

/* Sensible maximum value */
#define MAX_PACKET_LEN 1024


/* RX descriptor ctrl bits:
 * 31 - truncated
 * 30 - data crc error
 * 29 - header crc error
 * 28 - Error End of Packet (EEP) termination
 * 27 - IE - interrupt enable
 * 26 - WR - wrap - the next descriptor used will be the first one
 * 25 - EN - enable descriptor
 * 24:0 - packet length (valid after EN was set to 0, in bytes)
 *
 * Bits IE, WR, EN are set to 0 by the hardware after packet is received.
 * When EN is 0, status bits and packet length are valid and can be read.
 * CRC header calculation can only be used if packet is of type RMAP.
 * CRC data calculation can be checked provided that sender has calculated
 * CRC for the packet according to the RMAP standard.
 */

#define RX_DESC_TRUNC   (1u << 31)
#define RX_DESC_DCRC    (1u << 30)
#define RX_DESC_HCRC    (1u << 29)
#define RX_DESC_EEP     (1u << 28)
#define RX_DESC_IE      (1u << 27)
#define RX_DESC_WR      (1u << 26)
#define RX_DESC_EN      (1u << 25)
#define RX_DESC_LEN     (0x1ffffffu)
#define RX_DESC_USR_MSK (RX_DESC_TRUNC | RX_DESC_DCRC | RX_DESC_HCRC | RX_DESC_EEP | RX_DESC_LEN)


typedef struct {
	volatile uint32_t ctrl;
	uint32_t addr; /* RX buff phy addr - must be word aligned */
} spw_rxDesc_t;

/* TX descriptor ctrl bits:
 * 17 - append data CRC
 * 16 - append header CRC
 * 15 - link error occurred
 * 14 - IE - interrupt enable
 * 13 - WR - wrap - the next descriptor used will be the first one
 * 12 - EN - enable descriptor
 * 11:8 - Non CRC bytes - number of bytes from start that will not be included in CRC calculation
 * 7:0 - header length in bytes
 */

#define TX_DESC_DCRC    (1u << 17)
#define TX_DESC_HCRC    (1u << 16)
#define TX_DESC_LERR    (1u << 15)
#define TX_DESC_IE      (1u << 14)
#define TX_DESC_WR      (1u << 13)
#define TX_DESC_EN      (1u << 12)
#define TX_DESC_NON_CRC (0xfu << 8)
#define TX_DESC_HDR_LEN (0xffu)
#define TX_DESC_USR_MSK (TX_DESC_DCRC | TX_DESC_HCRC | TX_DESC_NON_CRC | TX_DESC_HDR_LEN)


typedef struct {
	volatile uint32_t ctrl;
	uint8_t *hdrAddr;   /* TX header buff phy addr - does not have to be word aligned */
	uint32_t packetLen; /* TX packet length in bytes (without header) */
	uint8_t *dataAddr;  /* TX data buff phy addr - does not have to be word aligned */
} spw_txDesc_t;


typedef struct {
	volatile uint32_t *vbase;
	uint8_t addr;
	uint8_t txDescFree;

	size_t sentDesc;
	size_t lastTxDesc;
	size_t nextRxDesc;

	bool txWaited[SPW_TX_DESC_CNT];
	bool rxAcknowledged[SPW_RX_DESC_CNT];

	handle_t ctrlLock;
	handle_t txLock;
	handle_t txIrqLock;
	handle_t rxLock;
	handle_t rxConfLock;
	handle_t cond;
	handle_t rxAckCond;

	volatile uint8_t (*txBuff)[MAX_PACKET_LEN];
	volatile uint8_t (*rxBuff)[MAX_PACKET_LEN];
	volatile spw_txDesc_t *txDesc;
	volatile spw_rxDesc_t *rxDesc;
} spw_dev_t;


static struct {
	volatile uint32_t *base;
	unsigned int irq;
	unsigned int active;
} spw_info[] = {
	{ .active = SPW0_ACTIVE },
	{ .active = SPW1_ACTIVE },
	{ .active = SPW2_ACTIVE },
	{ .active = SPW3_ACTIVE },
	{ .active = SPW4_ACTIVE },
	{ .active = SPW5_ACTIVE },
};


static struct {
	spw_dev_t dev[SPW_CNT];
} spw_common;


/* Auxiliary functions */


static int spw_buffersAlloc(spw_dev_t *dev)
{
	size_t descSz = PAGE_ALIGN(sizeof(spw_txDesc_t) * SPW_TX_DESC_CNT + sizeof(spw_rxDesc_t) * SPW_RX_DESC_CNT);
	dev->txDesc = mmap(NULL, descSz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if (dev->txDesc == MAP_FAILED) {
		return -ENOMEM;
	}

	dev->rxDesc = (void *)((addr_t)dev->txDesc + sizeof(spw_txDesc_t) * SPW_TX_DESC_CNT);

	dev->rxBuff = mmap(NULL, MAX_PACKET_LEN * SPW_RX_DESC_CNT, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if (dev->rxBuff == MAP_FAILED) {
		return -ENOMEM;
	}

	dev->txBuff = mmap(NULL, MAX_PACKET_LEN * SPW_TX_DESC_CNT, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if (dev->txBuff == MAP_FAILED) {
		return -ENOMEM;
	}

	return EOK;
}


static void spw_buffersFree(spw_dev_t *dev)
{
	if (dev->txBuff != MAP_FAILED) {
		(void)munmap((void *)dev->txBuff, MAX_PACKET_LEN * SPW_TX_DESC_CNT);
	}
	if (dev->rxBuff != MAP_FAILED) {
		(void)munmap((void *)dev->rxBuff, MAX_PACKET_LEN * SPW_RX_DESC_CNT);
	}
	if (dev->txDesc != MAP_FAILED) {
		size_t descSz = PAGE_ALIGN(sizeof(spw_txDesc_t) * SPW_TX_DESC_CNT + sizeof(spw_rxDesc_t) * SPW_RX_DESC_CNT);
		(void)munmap((void *)dev->txDesc, descSz);
	}
}


static size_t spw_txMsgToPacket(const uint8_t *buf, spw_txPacket_t *packet)
{
	packet->flags = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
	packet->dataLen = (buf[4] << 24) | (buf[5] << 16) | (buf[6] << 8) | buf[7];

	packet->hdr = &buf[SPW_TX_MIN_BUFSZ];
	packet->data = &buf[SPW_TX_MIN_BUFSZ] + (packet->flags & 0xff);

	return SPW_TX_MIN_BUFSZ + (packet->flags & 0xffu) + packet->dataLen;
}


static size_t spw_rxPacketToMsg(const uint32_t flags, const size_t rxLen, const uint8_t *rx, uint8_t *buf)
{
	buf[0] = flags >> 24;
	buf[1] = (flags >> 16) & 0xffu;
	buf[2] = (flags >> 8) & 0xffu;
	buf[3] = flags & 0xffu;

	memcpy(&buf[SPW_RX_MIN_BUFSZ], rx, rxLen);

	return SPW_RX_MIN_BUFSZ + rxLen;
}


/* Interrupt handling */


static int spw_irqHandler(unsigned int n, void *arg)
{
	(void)n;

	spw_dev_t *dev = (spw_dev_t *)arg;

	/* TX IRQ */
	while (dev->sentDesc != dev->lastTxDesc) {
		if (dev->txDesc[dev->sentDesc].ctrl & TX_DESC_EN) {
			/* Not sent yet */
			break;
		}

		dev->sentDesc = (dev->sentDesc + 1) % SPW_TX_DESC_CNT;
		dev->txDescFree++;
	}

	/* For RX IRQ only cond in kernel is signalled */

	return 1;
}


/* Operations on device */


/* Blocks execution until all packets are transmitted. */
static int spw_transmitWait(spw_dev_t *dev, const uint8_t *buf, const size_t nPackets)
{
	if (nPackets > SPW_TX_DESC_CNT) {
		return -EINVAL;
	}

	(void)mutexLock(dev->txLock);

	TRACE("nPackets: %d", nPackets);

	/* Setup descriptors */
	size_t firstDesc = dev->lastTxDesc;
	const size_t lastDesc = (dev->lastTxDesc + nPackets) % SPW_TX_DESC_CNT;
	bool wrapped = (lastDesc <= firstDesc);

	for (size_t cnt = 0; cnt < nPackets; cnt++) {
		(void)mutexLock(dev->txIrqLock);
		while ((dev->txDescFree == 0) || dev->txWaited[dev->lastTxDesc]) {
			/* Wait for free descriptor or for packet to be acknowledged */
			(void)condWait(dev->cond, dev->txIrqLock, 0);
		}
		dev->txDescFree--;
		(void)mutexUnlock(dev->txIrqLock);

		volatile spw_txDesc_t *desc = &dev->txDesc[dev->lastTxDesc];

		spw_txPacket_t packet;
		buf += spw_txMsgToPacket(buf, &packet);
		/* Interrupt after each packet transmitted */
		desc->ctrl = (packet.flags & TX_DESC_USR_MSK) | TX_DESC_IE;
		if (dev->lastTxDesc == SPW_TX_DESC_CNT - 1) {
			/* Wrap around */
			desc->ctrl |= TX_DESC_WR;
		}
		desc->hdrAddr = (void *)va2pa((void *)packet.hdr);
		desc->packetLen = packet.dataLen;
		desc->dataAddr = (void *)va2pa((void *)packet.data);

		/* Everything is set up, enable descriptor */
		desc->ctrl |= TX_DESC_EN;

		dev->txWaited[dev->lastTxDesc] = true;

		/* Start transmission */
		dev->vbase[DMA_CTRL] |= DMA_CTRL_TE;

		dev->lastTxDesc = (dev->lastTxDesc + 1) % SPW_TX_DESC_CNT;
	}

	TRACE("Packets set up");

	/* Wait for transmission to finish */
	while ((firstDesc <= lastDesc) || wrapped) {
		if ((dev->txDesc[firstDesc].ctrl & TX_DESC_EN) == 0) {
			dev->txWaited[firstDesc] = false;
			size_t next = (firstDesc + 1) % SPW_TX_DESC_CNT;
			if ((next == 0) && wrapped) {
				wrapped = false;
			}
			firstDesc = next;
		}
		else {
			(void)mutexLock(dev->txIrqLock);
			(void)condWait(dev->cond, dev->txIrqLock, 0);
			(void)mutexUnlock(dev->txIrqLock);
		}
	}

	TRACE("Packets sent");

	(void)mutexUnlock(dev->txLock);

	return nPackets;
}


/* Starts transmission and returns */
static int spw_transmitAsync(spw_dev_t *dev, const uint8_t *buf, const size_t nPackets)
{
	(void)mutexLock(dev->txLock);

	TRACE("nPackets: %d", nPackets);

	/* Setup descriptors */
	for (size_t cnt = 0; cnt < nPackets; cnt++) {
		(void)mutexLock(dev->txIrqLock);
		while ((dev->txDescFree == 0) || dev->txWaited[dev->lastTxDesc]) {
			/* Wait for free descriptor or for packet to be acknowledged */
			(void)condWait(dev->cond, dev->txIrqLock, 0);
		}
		dev->txDescFree--;
		(void)mutexUnlock(dev->txIrqLock);

		volatile spw_txDesc_t *desc = &dev->txDesc[dev->lastTxDesc];

		spw_txPacket_t packet;
		buf += spw_txMsgToPacket(buf, &packet);
		/* Interrupt after each packet transmitted */
		desc->ctrl = (packet.flags & TX_DESC_USR_MSK) | TX_DESC_IE;
		if (dev->lastTxDesc == SPW_TX_DESC_CNT - 1) {
			/* Wrap around */
			desc->ctrl |= TX_DESC_WR;
		}
		desc->packetLen = packet.dataLen;

		uint8_t hdrLen = packet.flags & TX_DESC_HDR_LEN;
		volatile void *txBuff = dev->txBuff[dev->lastTxDesc];

		memcpy((char *)txBuff, packet.hdr, hdrLen);
		memcpy((char *)txBuff + hdrLen, packet.data, packet.dataLen);

		desc->hdrAddr = (uint8_t *)va2pa((void *)txBuff);
		desc->dataAddr = (uint8_t *)va2pa((char *)txBuff + hdrLen);

		/* Everything is set up, enable descriptor */
		desc->ctrl |= TX_DESC_EN;

		dev->txWaited[dev->lastTxDesc] = false;

		/* Start transmission */
		dev->vbase[DMA_CTRL] |= DMA_CTRL_TE;

		dev->lastTxDesc = (dev->lastTxDesc + 1) % SPW_TX_DESC_CNT;
	}

	TRACE("Packets set up");

	(void)mutexUnlock(dev->txLock);

	return nPackets;
}


static int spw_transmit(spw_dev_t *dev, const uint8_t *buf, const size_t bufsz, const size_t nPackets, bool async)
{
	if ((buf == NULL) || (bufsz < SPW_TX_MIN_BUFSZ)) {
		return -EINVAL;
	}

	if (nPackets == 0) {
		return 0;
	}

	return async ? spw_transmitAsync(dev, buf, nPackets) : spw_transmitWait(dev, buf, nPackets);
}


/* Configure RX DMA descriptors */
static int spw_rxConfigure(spw_dev_t *dev, size_t *firstDesc, const size_t nPackets)
{
	if (nPackets > SPW_RX_DESC_CNT) {
		return -EINVAL;
	}

	(void)mutexLock2(dev->rxConfLock, dev->rxLock);

	TRACE("nPackets: %d", nPackets);

	for (size_t cnt = 0; cnt < nPackets; cnt++) {
		while (!dev->rxAcknowledged[dev->nextRxDesc]) {
			(void)condWait(dev->rxAckCond, dev->rxLock, 0);
		}

		dev->rxAcknowledged[dev->nextRxDesc] = false;

		volatile spw_rxDesc_t *desc = &dev->rxDesc[dev->nextRxDesc];

		/* Interrupt after each packet received */
		desc->ctrl = RX_DESC_IE;
		if (dev->nextRxDesc == SPW_RX_DESC_CNT - 1) {
			/* Wrap around */
			desc->ctrl |= RX_DESC_WR;
		}

		memset((void *)dev->rxBuff[dev->nextRxDesc], 0, sizeof(dev->rxBuff[dev->nextRxDesc]));
		desc->addr = va2pa((void *)dev->rxBuff[dev->nextRxDesc]);

		/* Everything is set up, enable descriptor */
		desc->ctrl |= RX_DESC_EN;

		dev->nextRxDesc = (dev->nextRxDesc + 1) % SPW_RX_DESC_CNT;
	}

	*firstDesc = (dev->nextRxDesc + SPW_RX_DESC_CNT - nPackets) % SPW_RX_DESC_CNT;

	/* Enable receiver */
	dev->vbase[DMA_CTRL] |= (DMA_CTRL_RE | DMA_CTRL_RD);

	(void)mutexUnlock(dev->rxLock);
	(void)mutexUnlock(dev->rxConfLock);

	return nPackets;
}


/* Read from RX buffers */
static int spw_rxRead(spw_dev_t *dev, size_t firstDesc, uint8_t *buf, size_t bufsz, const size_t nPackets)
{
	if (nPackets > SPW_RX_DESC_CNT) {
		return -EINVAL;
	}

	size_t cnt = 0;
	const size_t lastDesc = (firstDesc + nPackets) % SPW_RX_DESC_CNT;
	bool wrapped = (lastDesc <= firstDesc);

	TRACE("first: %u last: %u nPackets: %u", firstDesc, lastDesc, nPackets);

	(void)mutexLock(dev->rxLock);

	while ((firstDesc < lastDesc) || wrapped) {
		if ((dev->rxDesc[firstDesc].ctrl & RX_DESC_EN) == 0) {
			uint32_t flags = dev->rxDesc[firstDesc].ctrl & RX_DESC_USR_MSK;
			size_t rxLen = flags & RX_DESC_LEN;
			if ((rxLen + SPW_RX_MIN_BUFSZ) > bufsz) {
				/* Buffer too small */
				break;
			}
			/* Copy packet to user buffer */
			rxLen = spw_rxPacketToMsg(flags, rxLen, (const uint8_t *)dev->rxBuff[firstDesc], buf);
			dev->rxAcknowledged[firstDesc] = true;
			size_t next = (firstDesc + 1) % SPW_RX_DESC_CNT;
			if ((next == 0) && wrapped) {
				wrapped = false;
			}
			firstDesc = next;
			buf += rxLen;
			bufsz -= rxLen;
			cnt++;
			condSignal(dev->rxAckCond);
		}
		else {
			(void)condWait(dev->cond, dev->rxLock, 0);
		}
	}

	(void)mutexUnlock(dev->rxLock);

	return cnt;
}


static int spw_configure(spw_dev_t *dev, const spw_config_t *config)
{
	(void)mutexLock(dev->ctrlLock);

	dev->vbase[SPW_NODE_ADDR] = (config->node.mask << 8) | config->node.addr;
	dev->vbase[DMA_ADDR] = (config->dma.mask << 8) | config->dma.addr;
	dev->vbase[DMA_CTRL] = (dev->vbase[DMA_CTRL] & ~DMA_CTRL_USR_MSK) | (config->dma.flags & DMA_CTRL_USR_MSK);

	(void)mutexUnlock(dev->ctrlLock);

	return EOK;
}


/* Message handling */


static void spw_handleDevCtl(msg_t *msg, int dev)
{
	const multi_i_t *idevctl = (multi_i_t *)msg->i.raw;
	multi_o_t *odevctl = (multi_o_t *)msg->o.raw;
	spw_dev_t *spw = &spw_common.dev[dev];

	switch (idevctl->spw.type) {
		case spw_config:
			odevctl->err = spw_configure(spw, &idevctl->spw.task.config);
			break;

		case spw_rxConfig:
			odevctl->err = spw_rxConfigure(spw, &odevctl->val, idevctl->spw.task.rxConfig.nPackets);
			break;

		case spw_rx:
			odevctl->err = spw_rxRead(
				spw, idevctl->spw.task.rx.firstDesc, msg->o.data, msg->o.size, idevctl->spw.task.rx.nPackets);
			break;

		case spw_tx:
			odevctl->err = spw_transmit(
				spw, msg->i.data, msg->i.size, idevctl->spw.task.tx.nPackets, idevctl->spw.task.tx.async);
			break;

		default:
			odevctl->err = -EINVAL;
			break;
	}
}


void spw_handleMsg(msg_t *msg, int dev)
{
	dev -= id_spw0;
	switch (msg->type) {
		case mtDevCtl:
			spw_handleDevCtl(msg, dev);
			break;

		default:
			msg->o.io.err = -EINVAL;
			break;
	}
}


/* Initialization */


int spw_createDevs(oid_t *oid)
{
	for (unsigned int i = 0; i < SPW_CNT; i++) {
		if (spw_info[i].active == 0) {
			continue;
		}

		char buf[8];
		if (snprintf(buf, sizeof(buf), "spw%d", i) >= sizeof(buf)) {
			return -1;
		}

		if (create_dev(oid, buf) < 0) {
			return -1;
		}
	}
	return 0;
}


static int spw_cguInit(int dev)
{
#if defined(__CPU_GR712RC)
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_cguctrl,
		.cguctrl = {
			.state = enable,
			.cgudev = cgudev_spw0 + dev,
		}
	};
#elif defined(__CPU_GR716)
	(void)dev;
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_cguctrl,
		.cguctrl = {
			.state = enable,
			.cgu = cgu_secondary,
			.cgudev = cgudev_grspw,
		}
	};
#endif
	return platformctl(&pctl);
}


static void spw_defaultConfig(spw_dev_t *dev)
{
	dev->vbase[SPW_CTRL] |= SPW_CTRL_LS;
	dev->vbase[DMA_CTRL] |= DMA_CTRL_RI | DMA_CTRL_TI;
	dev->vbase[DMA_RX_LEN] = MAX_PACKET_LEN;
	dev->vbase[DMA_TX_DESC] = va2pa((void *)dev->txDesc);
	dev->vbase[DMA_RX_DESC] = va2pa((void *)dev->rxDesc);
}


static int spw_createResources(spw_dev_t *dev, addr_t pbase)
{
	dev->vbase = MAP_FAILED;
	dev->ctrlLock = (handle_t)-1;
	dev->txLock = (handle_t)-1;
	dev->rxLock = (handle_t)-1;
	dev->txIrqLock = (handle_t)-1;
	dev->rxConfLock = (handle_t)-1;
	dev->cond = (handle_t)-1;
	dev->rxAckCond = (handle_t)-1;
	for (size_t i = 0; i < SPW_RX_DESC_CNT; i++) {
		dev->rxAcknowledged[i] = true;
	}
	dev->rxBuff = MAP_FAILED;
	dev->txBuff = MAP_FAILED;
	dev->rxDesc = MAP_FAILED;
	dev->txDesc = MAP_FAILED;

	uintptr_t base = (pbase & ~(_PAGE_SIZE - 1));
	dev->vbase = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
	if (dev->vbase == MAP_FAILED) {
		return -1;
	}

	if (spw_buffersAlloc(dev) < 0) {
		return -1;
	}

	if (mutexCreate(&dev->ctrlLock) < 0) {
		return -1;
	}

	if (mutexCreate(&dev->txLock) < 0) {
		return -1;
	}

	if (mutexCreate(&dev->rxLock) < 0) {
		return -1;
	}

	if (mutexCreate(&dev->txIrqLock) < 0) {
		return -1;
	}

	if (mutexCreate(&dev->rxConfLock) < 0) {
		return -1;
	}

	if (condCreate(&dev->cond) < 0) {
		return -1;
	}

	if (condCreate(&dev->rxAckCond) < 0) {
		return -1;
	}

	dev->vbase += (pbase - base) / sizeof(uintptr_t);

	return 0;
}


static void spw_cleanupResources(spw_dev_t *dev)
{
	if (dev->rxAckCond != (handle_t)-1) {
		resourceDestroy(dev->rxAckCond);
	}

	if (dev->cond != (handle_t)-1) {
		resourceDestroy(dev->cond);
	}

	if (dev->rxConfLock != (handle_t)-1) {
		resourceDestroy(dev->rxConfLock);
	}

	if (dev->txIrqLock != (handle_t)-1) {
		resourceDestroy(dev->txIrqLock);
	}

	if (dev->rxLock != (handle_t)-1) {
		resourceDestroy(dev->rxLock);
	}

	if (dev->txLock != (handle_t)-1) {
		resourceDestroy(dev->txLock);
	}

	if (dev->ctrlLock != (handle_t)-1) {
		resourceDestroy(dev->ctrlLock);
	}

	spw_buffersFree(dev);

	if (dev->vbase != MAP_FAILED) {
		(void)munmap((void *)dev->vbase, _PAGE_SIZE);
	}
}


int spw_init(void)
{
	for (unsigned int i = 0; i < SPW_CNT; i++) {
		if (spw_info[i].active == 0) {
			continue;
		}

		unsigned int instance = i;
		ambapp_dev_t dev = { .devId = CORE_ID_GRSPW2 };
		platformctl_t pctl = {
			.action = pctl_get,
			.type = pctl_ambapp,
			.ambapp = {
				.dev = &dev,
				.instance = &instance,
			}
		};

		if (platformctl(&pctl) < 0) {
			return -1;
		}

		if (dev.bus != BUS_AMBA_APB) {
			/* GRSPW2 should be on APB bus */
			return -1;
		}

		spw_info[i].base = dev.info.apb.base;
		spw_info[i].irq = dev.irqn;

		if (spw_cguInit(i) < 0) {
			return -1;
		}

		if (spw_createResources(&spw_common.dev[i], (addr_t)spw_info[i].base) < 0) {
			spw_cleanupResources(&spw_common.dev[i]);
			return -1;
		}

		spw_common.dev[i].txDescFree = SPW_TX_DESC_CNT;

		(void)interrupt(spw_info[i].irq, spw_irqHandler, &spw_common.dev[i], spw_common.dev[i].cond, NULL);

		spw_defaultConfig(&spw_common.dev[i]);
	}
	return EOK;
}
