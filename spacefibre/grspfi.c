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
		uint32_t status;
		uint32_t vc_map;
		uint32_t irq_ctrl;
		uint32_t ext_status;
	} dma_ch[8];
};


typedef struct {
	volatile struct grspfi_regs *mmio;
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
	oid_t spwOid;
	spfi_dev_t dev[SPFI_CNT];
	char spfiStack[SPFI_THREADS_NO][SPFI_STACKSZ];
} spfi_common;


static int spfi_set_vc(spfi_dev_t *dev, uint8_t dma, uint32_t vc)
{
	if (dma >= SPFI_DMA_CH) {
		return -EINVAL;
	}
	dev->mmio->dma_ch[dma].vc_map = vc;
	return 0;
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
		while (msgRecv(spfi_common.spwOid.port, &msg, &rid) < 0) {
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

		msgRespond(spfi_common.spwOid.port, &msg, rid);
	}
}


static int spfi_createResources(spfi_dev_t *dev, addr_t pbase)
{
	dev->mmio = MAP_FAILED;

	uintptr_t base = (pbase & ~(_PAGE_SIZE - 1));
	dev->mmio = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
	if (dev->mmio == MAP_FAILED) {
		return -1;
	}

	dev->mmio = (void *)((uintptr_t)dev->mmio + pbase - base);

	return 0;
}


static void spfi_cleanupResources(spfi_dev_t *dev)
{
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

	if (portCreate(&spfi_common.spwOid.port) < 0) {
		LOG_ERROR("Failed to create port\n");
		return EXIT_FAILURE;
	}

	/* Wait for rootfs */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(100000);
	}

	if (spfi_init() < 0) {
		portDestroy(spfi_common.spwOid.port);
		LOG_ERROR("Failed to initialize SpaceFibre\n");
		return EXIT_FAILURE;
	}

	if (spfi_createDevs(&spfi_common.spwOid) < 0) {
		portDestroy(spfi_common.spwOid.port);
		LOG_ERROR("Failed to create SpaceFibre devices");
		return EXIT_FAILURE;
	}

	for (int i = 1; i < SPFI_THREADS_NO - 1; i++) {
		beginthread(spfi_thread, SPFI_PRIO, spfi_common.spfiStack[i], SPFI_STACKSZ, NULL);
	}

	LOG("initialized");
	priority(SPFI_PRIO);
	spfi_thread(NULL);

	return 0;
}
