/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceFibre driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Andrzej Tlomak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <board_config.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
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
#include <phoenix/gaisler/ambapp.h>
#include <sys/msg.h>
#include <inttypes.h>

#include "grspfi.h"

#ifdef __CPU_GR765
#include <phoenix/arch/riscv64/riscv64.h>
#else
#include <phoenix/arch/sparcv8leon/sparcv8leon.h>
#endif

/* clang-format off */
#define TRACE(fmt, ...) do { if (0) { printf("%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } } while (0)
#define LOG(fmt, ...)       printf("spacefibre: " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) fprintf(stderr, "spacefibre: " fmt "\n", ##__VA_ARGS__)
/* clang-format on */

#define PAGE_ALIGN(addr) (((addr_t)(addr) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1))

#define SPFI_ADDR_MASK UINT64_C(0xffffffff) /* mask to check DMA desc. address overflow */

#define SPFI_THREADS_NO 4
#define SPFI_STACKSZ    4096
#define SPFI_PRIO       3

#define GRSPFI_VC_NUM     32
#define GRSPFI_DMA_CH_NUM 8

struct grspfi_regs {
	uint32_t cdc_gen_cap;  /* Codec General Capabilities */
	uint32_t cdc_buf_cap;  /* Codec Buffers Capabilities */
	uint32_t cdc_ctrl;     /* Codec Control */
	uint32_t lane_status;  /* Lane Layer Status */
	uint32_t retry_status; /* Retry Layer Status */
	uint32_t default_addr; /* Default Address */
	uint32_t _rsv0[2];     /* 0x018–0x01C reserved */
	uint32_t ts_reg;       /* Time-slot */
	uint32_t ts_ctrl;      /* Time-slot Control */
	uint32_t ts_len;       /* Time-slot Length */
	uint32_t bc_cfg;       /* Broadcast Channel Config */
	uint32_t _rsv1[52];    /* 0x034–0x0FF reserved */

	/* VC registers, stride 0x20 */
	struct {
		uint32_t ctrl;
		uint32_t status;
		uint32_t rx_max_len;
		uint32_t addr; /* addr reg*/
		uint32_t ts1;  /* timeslot 1 reg */
		uint32_t ts2;  /* timeslot 2 reg */
		uint32_t dkey; /* destination key */
		uint32_t _rsv;
	} vc[32];

	uint32_t _rsv2[192]; /* 0x500–0x7FF reserved */

	uint32_t dma_cap;    /* DMA Layer Capabilities */
	uint32_t dma_ctrl;   /* DMA Layer Control */
	uint32_t dma_status; /* DMA Layer Status */
	uint32_t spf_enable; /* SpaceFibre Enable */
	uint32_t bc_tx_addr; /* Broadcast TX Address */
	uint32_t bc_tx_size; /* Broadcast TX Size */
	uint32_t bc_tx_wptr; /* Broadcast TX Write Ptr */
	uint32_t bc_tx_rptr; /* Broadcast TX Read Ptr */
	uint32_t bc_rx_addr; /* Broadcast RX Address */
	uint32_t bc_rx_size; /* Broadcast RX Size */
	uint32_t bc_rx_wptr; /* Broadcast RX Write Ptr */
	uint32_t bc_rx_rptr; /* Broadcast RX Read Ptr */
	uint32_t bc_map;     /* Broadcast Channel Mapping */
	uint32_t _rsv3[51];  /* 0x8FF reserved */

	/* VC descriptor regs, stride 0x10 */
	struct {
		uint32_t tx_desc_base;
		uint32_t rx_desc_base;
		uint32_t ctrl;
		uint32_t stats;
	} vcdesc[32];

	/* DMA channel regs, stride 0x10 */
	struct {
		uint32_t ctrl;
#define SPFI_DCCTRL_TS (1u << 3) /* TS - transmitter stop */
#define SPFI_DCCTRL_RS (1u << 2) /* TE - receiver stop */
#define SPFI_DCCTRL_TE (1u << 2) /* TE - transmitter enable */
#define SPFI_DCCTRL_RE (1u << 1) /* TE - receiver enable */
		uint32_t status;
		uint32_t vc_map;
		uint32_t irq_ctrl;
#define SPFI_DCICTRL_RMI (1u << 3) /* RMAP irq enable */
#define SPFI_DCICTRL_BEI (1u << 2) /* bus master error irq enable */
#define SPFI_DCICTRL_TI  (1u << 1) /* transmit irq enable */
#define SPFI_DCICTRL_RI  (1u << 0) /* receive irq enable */
		uint32_t ext_status;
	} dma_ch[8];
};

#define SPFI_RX_DESC_CNT 128
#define SPFI_TX_DESC_CNT 64

#define SPFI_MAX_PACKET_LEN 1024

#define SPFI_RX_DESC_DCRC    (1u << 31)   /* data crc error */
#define SPFI_RX_DESC_HCRC    (1u << 30)   /* header crc error */
#define SPFI_RX_DESC_TRUNC   (1u << 29)   /* packet truncated */
#define SPFI_RX_DESC_EEP     (1u << 28)   /* Error End of Packet termination */
#define SPFI_RX_DESC_IE      (1u << 27)   /* IE - interrupt enable */
#define SPFI_RX_DESC_WR      (1u << 26)   /* WR -wrap - the next descriptor used will be the first one */
#define SPFI_RX_DESC_EN      (1u << 25)   /* EN - enable descriptor*/
#define SPFI_RX_DESC_LEN     (0x1ffffffu) /* packet length */
#define SPFI_RX_DESC_USR_MSK (SPFI_RX_DESC_TRUNC | SPFI_RX_DESC_DCRC | SPFI_RX_DESC_HCRC | SPFI_RX_DESC_EEP | SPFI_RX_DESC_LEN)


typedef struct {
	volatile uint32_t ctrl;
	uint32_t addr; /* RX buff phy addr - must be word aligned */
} spfi_rxDesc_t;


#define SPFI_TX_DESC_DCRC    (1u << 17)   /* append data CRT */
#define SPFI_TX_DESC_HCRC    (1u << 16)   /* append header CRC */
#define SPFI_TX_DESC_NON_CRC (0xfu << 12) /* non-CRC bytes */
#define SPFI_TX_DESC_IE      (1u << 10)   /* IE -interrupt enable */
#define SPFI_TX_DESC_WR      (1u << 9)    /* WR -wrap - the next descriptor used will be the first one */
#define SPFI_TX_DESC_EN      (1u << 8)    /* EN -enable descriptor */
#define SPFI_TX_DESC_HDR_LEN (0xffu)      /* header length*/
#define SPFI_TX_DESC_USR_MSK (SPFI_TX_DESC_DCRC | SPFI_TX_DESC_HCRC | SPFI_TX_DESC_NON_CRC | SPFI_TX_DESC_HDR_LEN)


typedef struct {
	volatile uint32_t ctrl;
	uint32_t hdrAddr;   /* TX header buff phy addr - does not have to be word aligned */
	uint32_t packetLen; /* TX packet length in bytes (without header) */
	uint32_t dataAddr;  /* TX data buff phy addr - does not have to be word aligned */
} spfi_txDesc_t;


typedef struct {
	volatile struct grspfi_regs *mmio;
	uint8_t addr;
	uint8_t txDescFree;

	size_t sentDesc;
	size_t lastTxDesc;
	size_t nextRxDesc;

	bool txWaited[SPFI_TX_DESC_CNT];
	bool rxAcknowledged[SPFI_RX_DESC_CNT];

	handle_t ctrlLock;
	handle_t txLock;
	handle_t txIrqLock;
	handle_t rxLock;
	handle_t rxConfLock;
	handle_t cond;
	handle_t rxAckCond;

	volatile uint8_t (*txBuff)[SPFI_MAX_PACKET_LEN];
	volatile uint8_t (*rxBuff)[SPFI_MAX_PACKET_LEN];
	volatile spfi_txDesc_t *txDesc;
	volatile spfi_rxDesc_t *rxDesc;
} spfi_dev_t;


static struct {
	volatile uint32_t *base;
	unsigned int irq;
	unsigned int active;
} spfi_info[] = {
	{ .active = SPFI0_ACTIVE },
	{ .active = SPFI1_ACTIVE },
	{ .active = SPFI2_ACTIVE },
	{ .active = SPFI3_ACTIVE },
	{ .active = SPFI4_ACTIVE },
	{ .active = SPFI5_ACTIVE },
};


static struct {
	oid_t oid;
	spfi_dev_t dev[SPFI_CNT];
	char stack[SPFI_THREADS_NO][SPFI_STACKSZ];
} spfi_common;


static int spfi_set_vc(spfi_dev_t *dev, uint8_t dma, uint32_t vc)
{
	if (dma >= SPFI_DMA_CH) {
		return -EINVAL;
	}
	dev->mmio->dma_ch[dma].vc_map = vc;
	return 0;
}


/* Configure RX DMA descriptors */
static int spfi_rxConfigure(spfi_dev_t *dev, size_t *firstDesc, const size_t nPackets)
{
	if (nPackets > SPFI_RX_DESC_CNT) {
		return -EINVAL;
	}

	(void)mutexLock2(dev->rxConfLock, dev->rxLock);

	TRACE("nPackets: %zu", nPackets);

	for (size_t cnt = 0; cnt < nPackets; cnt++) {
		while (!dev->rxAcknowledged[dev->nextRxDesc]) {
			(void)condWait(dev->rxAckCond, dev->rxLock, 0);
		}

		dev->rxAcknowledged[dev->nextRxDesc] = false;

		volatile spfi_rxDesc_t *desc = &dev->rxDesc[dev->nextRxDesc];

		/* Interrupt after each packet received */
		desc->ctrl = SPFI_RX_DESC_IE;
		if (dev->nextRxDesc == SPFI_RX_DESC_CNT - 1) {
			/* Wrap around */
			desc->ctrl |= SPFI_RX_DESC_WR;
		}

		memset((void *)dev->rxBuff[dev->nextRxDesc], 0, sizeof(dev->rxBuff[dev->nextRxDesc]));
		uintptr_t pa = va2pa((void *)dev->rxBuff[dev->nextRxDesc]);
		if ((pa & ~SPFI_ADDR_MASK) != 0) {
			LOG_ERROR("DMA addr 0x%" PRIxPTR "exceeds 32-bit limit", pa);
			return -EINVAL;
		}
		desc->addr = pa;

		/* Everything is set up, enable descriptor */
		desc->ctrl |= SPFI_RX_DESC_EN;

		dev->nextRxDesc = (dev->nextRxDesc + 1) % SPFI_RX_DESC_CNT;
	}

	*firstDesc = (dev->nextRxDesc + SPFI_RX_DESC_CNT - nPackets) % SPFI_RX_DESC_CNT;

	/* Enable receiver */
	dev->mmio->vc[0].ctrl |= (SPFI_DCCTRL_RE);

	(void)mutexUnlock(dev->rxLock);
	(void)mutexUnlock(dev->rxConfLock);

	return nPackets;
}


static void spfi_handleDevCtl(msg_t *msg, int dev)
{
	spfi_t *ictl = (spfi_t *)msg->i.raw;

	int err = 0;
	switch (ictl->type) {
		case spfi_vc_set:
			err = spfi_set_vc(&spfi_common.dev[dev], ictl->task.dma_mapping.dma, ictl->task.dma_mapping.vc);
			break;
		default:
			err = -EINVAL;
			break;
	}
	msg->o.err = err;
}


static void spfi_dispatchMsg(msg_t *msg)
{
	id_t dev = msg->oid.id;

	switch (dev) {
		case id_spfi0:
		case id_spfi1:
		case id_spfi2:
		case id_spfi3:
		case id_spfi4:
		case id_spfi5:
			dev -= id_spfi0;
			spfi_handleDevCtl(msg, dev);
			break;

		default:
			msg->o.err = -EINVAL;
			break;
	}
}


void spfi_thread(void *arg)
{
	(void)arg;

	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		while (msgRecv(spfi_common.oid.port, &msg, &rid) < 0) {
		}

		switch (msg.type) {
			case mtDevCtl:
				spfi_dispatchMsg(&msg);
				break;

			case mtRead:
			case mtWrite:
			case mtOpen:
			case mtClose:
				msg.o.err = 0;
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(spfi_common.oid.port, &msg, rid);
	}
}


static int spfi_createResources(spfi_dev_t *dev, addr_t pbase)
{
	dev->mmio = MAP_FAILED;
	dev->ctrlLock = (handle_t)-1;
	dev->txLock = (handle_t)-1;
	dev->rxLock = (handle_t)-1;
	dev->txIrqLock = (handle_t)-1;
	dev->rxConfLock = (handle_t)-1;
	dev->cond = (handle_t)-1;
	dev->rxAckCond = (handle_t)-1;
	for (size_t i = 0; i < SPFI_RX_DESC_CNT; i++) {
		dev->rxAcknowledged[i] = true;
	}
	dev->rxBuff = MAP_FAILED;
	dev->txBuff = MAP_FAILED;
	dev->rxDesc = MAP_FAILED;
	dev->txDesc = MAP_FAILED;

	uintptr_t base = (pbase & ~(_PAGE_SIZE - 1));
	dev->mmio = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
	if (dev->mmio == MAP_FAILED) {
		return -1;
	}

	dev->mmio = (void *)((uintptr_t)dev->mmio + pbase - base);

	// if (spw_buffersAlloc(dev) < 0) {
	// 	return -1;
	// }

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

	return 0;
}


static void spfi_cleanupResources(spfi_dev_t *dev)
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

	// spw_buffersFree(dev);

	if (dev->mmio != MAP_FAILED) {
		(void)munmap((void *)dev->mmio, _PAGE_SIZE);
	}
}


static int spfi_init(void)
{
	for (unsigned int i = 0; i < SPFI_CNT; i++) {
		if (spfi_info[i].active == 0) {
			continue;
		}

		unsigned int instance = i;
		ambapp_dev_t dev = { .devId = CORE_ID_GRSPFI };
		platformctl_t pctl = {
			.action = pctl_get,
			.type = pctl_ambapp,
			.task.ambapp = {
				.dev = &dev,
				.instance = &instance,
			}
		};

		if (platformctl(&pctl) < 0) {
			LOG_ERROR("Failed to find grspfi core");
			return -1;
		}

		if (dev.bus != BUS_AMBA_AHB) {
			LOG_ERROR("Failed to find grspfi on AHB bus");
			return -1;
		}

		if (spfi_createResources(&spfi_common.dev[i], (addr_t)spfi_info[i].base) < 0) {
			LOG_ERROR("Failed to create resoureces");
			spfi_cleanupResources(&spfi_common.dev[i]);
			return -1;
		}
	}
	return 0;
}


static int spfi_createDevs(oid_t *oid)
{
	for (unsigned int i = 0; i < SPFI_CNT; i++) {
		if (spfi_info[i].active == 0) {
			continue;
		}

		char buf[9];
		if (snprintf(buf, sizeof(buf), "spfi%d", i) >= sizeof(buf)) {
			return -1;
		}

		oid->id = id_spfi0 + i;
		if (create_dev(oid, buf) < 0) {
			return -1;
		}
	}
	return 0;
}


int main(void)
{
	oid_t oid;

	if (portCreate(&spfi_common.oid.port) < 0) {
		LOG_ERROR("Failed to create port\n");
		return EXIT_FAILURE;
	}

	/* Wait for rootfs */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(100000);
	}

	if (spfi_init() < 0) {
		portDestroy(spfi_common.oid.port);
		LOG_ERROR("Failed to initialize SpaceFibre\n");
		return EXIT_FAILURE;
	}

	if (spfi_createDevs(&spfi_common.oid) < 0) {
		portDestroy(spfi_common.oid.port);
		LOG_ERROR("Failed to create SpaceFibre devices");
		return EXIT_FAILURE;
	}

	for (int i = 1; i < SPFI_THREADS_NO - 1; i++) {
		beginthread(spfi_thread, SPFI_PRIO, spfi_common.stack[i], SPFI_STACKSZ, NULL);
	}

	LOG("initialized");
	priority(SPFI_PRIO);
	spfi_thread(NULL);

	return 0;
}
