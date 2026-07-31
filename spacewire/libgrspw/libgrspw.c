/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Lukasz Leczkowski, Andrzej Tlomak
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <board_config.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <phoenix/gaisler/ambapp.h>
#include <inttypes.h>

#ifdef __CPU_GR765
#include <phoenix/arch/riscv64/riscv64.h>
#else
#include <phoenix/arch/sparcv8leon/sparcv8leon.h>
#endif

#include "libgrspw.h"

/* clang-format off */
#define TRACE(fmt, ...) do { if (0) { printf("%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } } while (0)
#define LOG(fmt, ...)       printf("spacewire: " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) fprintf(stderr, "spacewire: %s: " fmt "\n", __func__, ##__VA_ARGS__)
/* clang-format on */

#define PAGE_ALIGN(addr) (((addr_t)(addr) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1))

#define SPW_ADDR_MASK UINT64_C(0xffffffff)

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
#define SPW_CTRL_RA  (1U << 31) /* RMAP available */
#define SPW_CTRL_RX  (1U << 30) /* RX unaligned access available */
#define SPW_CTRL_RC  (1U << 29) /* RMAP CRC available */
#define SPW_CTRL_NCH (3U << 27) /* Number of DMA channels */
#define SPW_CTRL_PO  (1U << 26) /* Number of ports - 1 */
#define SPW_CTRL_PS  (1U << 21) /* Port select */
#define SPW_CTRL_NP  (1U << 20) /* Disable port force */
#define SPW_CTRL_RD  (1U << 17) /* RMAP buffer disable */
#define SPW_CTRL_RE  (1U << 16) /* RMAP enable */
#define SPW_CTRL_TR  (1U << 11) /* Time RX enable */
#define SPW_CTRL_TT  (1U << 10) /* Time TX enable */
#define SPW_CTRL_LI  (1U << 9)  /* Link error IRQ */
#define SPW_CTRL_TQ  (1U << 8)  /* Tick-out IRQ */
#define SPW_CTRL_RS  (1U << 6)  /* Reset */
#define SPW_CTRL_PM  (1U << 5)  /* Promiscuous mode */
#define SPW_CTRL_TI  (1U << 4)  /* Tick in */
#define SPW_CTRL_IE  (1U << 3)  /* Interrupt enable */
#define SPW_CTRL_AS  (1U << 2)  /* Autostart */
#define SPW_CTRL_LS  (1U << 1)  /* Link start */
#define SPW_CTRL_LD  (1U << 0)  /* Link disable */

/* DMA CTRL bits */
#define DMA_CTRL_LE  (1U << 16) /* Disable TX when link error occurs */
#define DMA_CTRL_SP  (1U << 15) /* Remove 2nd byte (protocol id) of each packet */
#define DMA_CTRL_SA  (1U << 14) /* Remove 1st byte (address) of each packet */
#define DMA_CTRL_ENA (1U << 13) /* Enable separate node address for channel */
#define DMA_CTRL_NS  (1U << 12) /* No spill */
#define DMA_CTRL_RD  (1U << 11) /* RX descriptors available */
#define DMA_CTRL_RX  (1U << 10) /* RX active (read only) */
#define DMA_CTRL_AT  (1U << 9)  /* Abort TX */
#define DMA_CTRL_RA  (1U << 8)  /* RX AHB error */
#define DMA_CTRL_TA  (1U << 7)  /* TX AHB error */
#define DMA_CTRL_PR  (1U << 6)  /* Packet received */
#define DMA_CTRL_PS  (1U << 5)  /* Packet sent */
#define DMA_CTRL_AI  (1U << 4)  /* AHB error IRQ */
#define DMA_CTRL_RI  (1U << 3)  /* RX IRQ (if set in corresponding descriptor) */
#define DMA_CTRL_TI  (1U << 2)  /* TX IRQ (if set in corresponding descriptor) */
#define DMA_CTRL_RE  (1U << 1)  /* Receiver enable */
#define DMA_CTRL_TE  (1U << 0)  /* Transmitter enable */

#define DMA_CTRL_USR_MSK (DMA_CTRL_LE | DMA_CTRL_SP | DMA_CTRL_SA | DMA_CTRL_ENA | DMA_CTRL_NS)


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

#define RX_DESC_TRUNC   (1U << 31)
#define RX_DESC_DCRC    (1U << 30)
#define RX_DESC_HCRC    (1U << 29)
#define RX_DESC_EEP     (1U << 28)
#define RX_DESC_IE      (1U << 27)
#define RX_DESC_WR      (1U << 26)
#define RX_DESC_EN      (1U << 25)
#define RX_DESC_LEN     (0x1ffffffU)
#define RX_DESC_USR_MSK (RX_DESC_TRUNC | RX_DESC_DCRC | RX_DESC_HCRC | RX_DESC_EEP | RX_DESC_LEN)


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

#define TX_DESC_DCRC    (1U << 17)
#define TX_DESC_HCRC    (1U << 16)
#define TX_DESC_LERR    (1U << 15)
#define TX_DESC_IE      (1U << 14)
#define TX_DESC_WR      (1U << 13)
#define TX_DESC_EN      (1U << 12)
#define TX_DESC_NON_CRC (0xfU << 8)
#define TX_DESC_HDR_LEN (0xffU)
#define TX_DESC_USR_MSK (TX_DESC_DCRC | TX_DESC_HCRC | TX_DESC_NON_CRC | TX_DESC_HDR_LEN)


/* Auxiliary functions */


static int spw_buffersAlloc(spw_dev_t *dev)
{
	size_t descSz = PAGE_ALIGN(sizeof(spw_txDesc_t) * SPW_TX_DESC_CNT + sizeof(spw_rxDesc_t) * SPW_RX_DESC_CNT);
	dev->txDesc = mmap(NULL, descSz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if (dev->txDesc == MAP_FAILED) {
		return -ENOMEM;
	}

	dev->rxDesc = (void *)((addr_t)dev->txDesc + sizeof(spw_txDesc_t) * SPW_TX_DESC_CNT);

	dev->rxBuff = mmap(NULL, SPW_MAX_PACKET_LEN * SPW_RX_DESC_CNT, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS | MAP_CONTIGUOUS, -1, 0);
	if (dev->rxBuff == MAP_FAILED) {
		return -ENOMEM;
	}

	dev->txBuff = mmap(NULL, SPW_MAX_PACKET_LEN * SPW_TX_DESC_CNT, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS | MAP_CONTIGUOUS, -1, 0);
	if (dev->txBuff == MAP_FAILED) {
		return -ENOMEM;
	}

	return 0;
}


static void spw_buffersFree(spw_dev_t *dev)
{
	if (dev->txBuff != MAP_FAILED) {
		(void)munmap((void *)dev->txBuff, SPW_MAX_PACKET_LEN * SPW_TX_DESC_CNT);
	}
	if (dev->rxBuff != MAP_FAILED) {
		(void)munmap((void *)dev->rxBuff, SPW_MAX_PACKET_LEN * SPW_RX_DESC_CNT);
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
	packet->data = &buf[SPW_TX_MIN_BUFSZ] + (packet->flags & 0xffU);

	return SPW_TX_MIN_BUFSZ + (packet->flags & 0xffU) + packet->dataLen;
}


static size_t spw_rxPacketToMsg(const uint32_t flags, const size_t rxLen, const uint8_t *rx, uint8_t *buf)
{
	buf[0] = flags >> 24;
	buf[1] = (flags >> 16) & 0xffU;
	buf[2] = (flags >> 8) & 0xffU;
	buf[3] = flags & 0xffU;

	memcpy(&buf[SPW_RX_MIN_BUFSZ], rx, rxLen);

	return SPW_RX_MIN_BUFSZ + rxLen;
}


/* Interrupt handling */


__attribute__((section(".interrupt"))) static int spw_irqHandler(unsigned int n, void *arg)
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


int spw_transmit(spw_dev_t *dev, const uint8_t *buf, size_t bufsz, const spw_tx_t *tx)
{
	if ((buf == NULL) || (bufsz < SPW_TX_MIN_BUFSZ)) {
		return -EINVAL;
	}

	if (!tx->async && (tx->nPackets > SPW_TX_DESC_CNT)) {
		return -EINVAL;
	}

	(void)mutexLock(dev->txLock);

	TRACE("nPackets: %zu", tx->nPackets);

	/* Setup descriptors */
	size_t firstDesc = dev->lastTxDesc;
	const size_t lastDesc = (dev->lastTxDesc + tx->nPackets) % SPW_TX_DESC_CNT;
	bool wrapped = (lastDesc <= firstDesc);

	for (size_t cnt = 0; cnt < tx->nPackets; cnt++) {
		(void)mutexLock(dev->txIrqLock);
		while (dev->txDescFree == 0) {
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

		uint8_t hdrLen = packet.flags & TX_DESC_HDR_LEN;
		volatile void *txBuff = dev->txBuff[dev->lastTxDesc];

		memcpy((char *)txBuff, packet.hdr, hdrLen);
		memcpy((char *)txBuff + hdrLen, packet.data, packet.dataLen);

		/* on riscv64 pa can exceed 32 bits */
		uintptr_t pa = va2pa((void *)txBuff);
		if ((pa & ~SPW_ADDR_MASK) != 0) {
			LOG_ERROR("DMA addr 0x%" PRIxPTR "exceeds 32-bit limit", pa);
			return -EINVAL;
		}
		desc->hdrAddr = pa;

		pa += hdrLen;
		if ((pa & ~SPW_ADDR_MASK) != 0) {
			LOG_ERROR("DMA addr 0x%" PRIxPTR "exceeds 32-bit limit", pa);
			return -EINVAL;
		}
		desc->dataAddr = pa;
		desc->packetLen = packet.dataLen;

		/* Everything is set up, enable descriptor */
		desc->ctrl |= TX_DESC_EN;


		/* Start transmission */
		dev->vbase[DMA_CTRL] |= DMA_CTRL_TE;

		dev->lastTxDesc = (dev->lastTxDesc + 1) % SPW_TX_DESC_CNT;
	}

	if (!tx->async) {
		/* Wait for transmission to finish */
		while ((firstDesc <= lastDesc) || wrapped) {
			if ((dev->txDesc[firstDesc].ctrl & TX_DESC_EN) == 0) {
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
	}

	TRACE("Packets set up");

	(void)mutexUnlock(dev->txLock);

	return tx->nPackets;
}


/* Configure RX DMA descriptors */
int spw_rxConfigure(spw_dev_t *dev, size_t *firstDesc, const size_t nPackets)
{
	if (nPackets > SPW_RX_DESC_CNT) {
		return -EINVAL;
	}

	(void)mutexLock2(dev->rxConfLock, dev->rxLock);

	TRACE("nPackets: %zu", nPackets);

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
		uintptr_t pa = va2pa((void *)dev->rxBuff[dev->nextRxDesc]);
		if ((pa & ~SPW_ADDR_MASK) != 0) {
			LOG_ERROR("DMA addr 0x%" PRIxPTR "exceeds 32-bit limit", pa);
			return -EINVAL;
		}
		desc->addr = pa;

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
int spw_rxRead(spw_dev_t *dev, uint8_t *buf, size_t bufsz, size_t *readCnt, const spw_rx_t *rx)
{
	size_t firstDesc = rx->firstDesc;
	const size_t nPackets = rx->nPackets;
	const uint32_t timeoutUs = rx->timeoutUs;
	int err = 0;

	if ((firstDesc >= SPW_RX_DESC_CNT) || (nPackets > SPW_RX_DESC_CNT)) {
		return -EINVAL;
	}

	size_t cnt = 0;
	const size_t lastDesc = (firstDesc + nPackets) % SPW_RX_DESC_CNT;
	bool wrapped = (lastDesc <= firstDesc);

	TRACE("first: %zu last: %zu nPackets: %zu", firstDesc, lastDesc, nPackets);

	(void)mutexLock(dev->rxLock);

	while ((firstDesc < lastDesc) || wrapped) {
		if ((dev->rxDesc[firstDesc].ctrl & RX_DESC_EN) == 0) {
			uint32_t flags = dev->rxDesc[firstDesc].ctrl & RX_DESC_USR_MSK;
			size_t rxLen = flags & RX_DESC_LEN;
			if ((rxLen + SPW_RX_MIN_BUFSZ) > bufsz) {
				LOG_ERROR("buffer too small: %zu < %zu", bufsz, (rxLen + SPW_RX_MIN_BUFSZ));
				err = -EINVAL;
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
		}
		else {
			condSignal(dev->rxAckCond);
			if (condWait(dev->cond, dev->rxLock, timeoutUs) == -ETIME) {
				TRACE("packet: %zu timeout: %uus", firstDesc, timeoutUs);

				dev->vbase[DMA_CTRL] &= ~DMA_CTRL_RE;

				/* ack unused descriptors */
				while ((firstDesc < lastDesc) || wrapped) {
					dev->rxAcknowledged[firstDesc] = true;
					dev->rxDesc[firstDesc].ctrl = 0;

					size_t next = (firstDesc + 1) % SPW_RX_DESC_CNT;
					if ((next == 0) && wrapped) {
						wrapped = false;
					}
					firstDesc = next;
				}

				/* move DMA pointer to skip timeouted descriptors */
				dev->vbase[DMA_RX_DESC] = va2pa((void *)&dev->rxDesc[lastDesc]);
				dev->vbase[DMA_CTRL] |= DMA_CTRL_RE;

				err = -ETIME;
				break;
			}
		}
	}

	(void)mutexUnlock(dev->rxLock);
	condSignal(dev->rxAckCond);

	*readCnt = cnt;
	return err;
}


int spw_configure(spw_dev_t *dev, const spw_config_t *config)
{
	(void)mutexLock(dev->ctrlLock);

	dev->vbase[SPW_NODE_ADDR] = (config->node.mask << 8) | config->node.addr;
	dev->vbase[DMA_ADDR] = (config->dma.mask << 8) | config->dma.addr;
	dev->vbase[DMA_CTRL] = (dev->vbase[DMA_CTRL] & ~DMA_CTRL_USR_MSK) | (config->dma.flags & DMA_CTRL_USR_MSK);

	(void)mutexUnlock(dev->ctrlLock);

	return 0;
}


ssize_t spw_xferOp(spw_dev_t *dev, const uint8_t *txbuf, size_t txbufsz, uint8_t *rxbuf, size_t rxbufsz, const spw_xfer_t *xfer, size_t *readCnt)
{
	if ((xfer->nRxPackets > SPW_RX_DESC_CNT) || (txbufsz < SPW_TX_MIN_BUFSZ)) {
		return -EINVAL;
	}

	/* Configure, transmit and receive in one operation */
	size_t firstDesc;
	int err = spw_rxConfigure(dev, &firstDesc, xfer->nRxPackets);
	if (err < 0) {
		return err;
	}

	spw_tx_t tx = { .nPackets = xfer->nTxPackets, .async = true };
	(void)spw_transmit(dev, txbuf, txbufsz, &tx);

	spw_rx_t rx = {
		.firstDesc = firstDesc,
		.nPackets = xfer->nRxPackets,
		.timeoutUs = xfer->rxTimeoutUs,
	};

	return spw_rxRead(dev, rxbuf, rxbufsz, readCnt, &rx);
}


static int spw_cguInit(int dev)
{
#if defined(__CPU_GR712RC)
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_cguctrl,
		.task.cguctrl = {
			.v.state = enable,
			.cgudev = cgudev_spw0 + dev,
		}
	};
	return platformctl(&pctl);
#elif defined(__CPU_GR716)
	(void)dev;
	platformctl_t pctl = {
		.action = pctl_set,
		.type = pctl_cguctrl,
		.task.cguctrl = {
			.v.state = enable,
			.cgu = cgu_secondary,
			.cgudev = cgudev_grspw,
		}
	};
	return platformctl(&pctl);
#else
	(void)dev;
	return 0;
#endif
}


static int spw_defaultConfig(spw_dev_t *dev)
{
	/* no effect on grspw2_dma core*/
	dev->vbase[SPW_CTRL] |= SPW_CTRL_LS;

	dev->vbase[DMA_CTRL] |= DMA_CTRL_RI | DMA_CTRL_TI;
	dev->vbase[DMA_RX_LEN] = SPW_MAX_PACKET_LEN;

	uintptr_t pa = va2pa((void *)dev->txDesc);
	if ((pa & ~SPW_ADDR_MASK) != 0) {
		LOG_ERROR("DMA addr 0x%" PRIxPTR "exceeds 32-bit limit", pa);
		return -EINVAL;
	}
	dev->vbase[DMA_TX_DESC] = (uint32_t)pa;

	pa = va2pa((void *)dev->rxDesc);
	if ((pa & ~SPW_ADDR_MASK) != 0) {
		LOG_ERROR("DMA addr 0x%" PRIxPTR "exceeds 32-bit limit", pa);
		return -EINVAL;
	}
	dev->vbase[DMA_RX_DESC] = (uint32_t)pa;

	return 0;
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

	dev->vbase = (void *)((uintptr_t)dev->vbase + (pbase - base));

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


int spw_initDev(unsigned int instance, spw_dev_t *spwdev)
{
	unsigned int ppInstance = instance;
	ambapp_dev_t dev = { .devId = CORE_ID_GRSPW2 };
	platformctl_t pctl = {
		.action = pctl_get,
		.type = pctl_ambapp,
		.task.ambapp = {
			.dev = &dev,
			.instance = &ppInstance,
		}
	};

	/* try DMA core (spwrtr) */
	if (platformctl(&pctl) < 0) {
		dev.devId = CORE_ID_GRSPW2_DMA;
		if (platformctl(&pctl) < 0) {
			return -1;
		}
		LOG("spw%d: grspw2_dma core found", instance);
	}
	else {
		LOG("spw%d: grspw2 core found", instance);
	}

	if (dev.bus != BUS_AMBA_APB) {
		/* GRSPW2/DMA should be on APB bus */
		return -1;
	}

	if (spw_cguInit(instance) < 0) {
		return -1;
	}

	if (spw_createResources(spwdev, (addr_t)dev.info.apb.base) < 0) {
		spw_cleanupResources(spwdev);
		return -1;
	}

	spwdev->txDescFree = SPW_TX_DESC_CNT;

	(void)interrupt(dev.irqn, spw_irqHandler, spwdev, spwdev->cond, NULL);

	if (spw_defaultConfig(spwdev) < 0) {
		spw_cleanupResources(spwdev);
		return -1;
	}
	return 0;
}
