/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Andrzej Tlomak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include "grspwrtr.h"
#include <sys/debug.h>
#include <board_config.h>
#include <stdio.h>
#include <sys/msg.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/threads.h>
#include <unistd.h>
#include <posix/utils.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <phoenix/gaisler/ambapp.h>

#ifdef __CPU_GR765
#include <phoenix/arch/riscv64/riscv64.h>
#else
#include <phoenix/arch/sparcv8leon/sparcv8leon.h>
#endif
#include <phoenix/gaisler/ambapp.h>

/* clang-format off */
#define TRACE(fmt, ...) do { if (0) { printf("%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } } while (0)
#define LOG(fmt, ...)       printf("spacewire-router: " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) fprintf(stderr, "spacewire-router: " fmt "\n", ##__VA_ARGS__)
/* clang-format on */

#define SPWRTR_PRIO  3
#define SPWRTR_PORTS 4

#define R_RESERVED(start, end) uint32_t rsvd_##start##_##end[(end - start) / sizeof(uint32_t)] /* reserved address */
#define BIT(i)                 (1u << (i))                                                     /* bitmask for single bit */
#define BITS(start, end)       (((1u << ((end) - (start) + 1u)) - 1u) << (start))              /* bitmask for bits in provided range (inclusive) */

struct spwrtr_regs {
	uint32_t rtr_rtpmap[256];  /* routing table port map (addr under index 0 is restricted) */
	uint32_t rtr_rtactrl[256]; /* routing table addr ctrl, (addr under index 0 is restricted) */
	union {
		uint32_t rtr_pctrlcfg;     /* port ctrl, port 0 */
#define SPWRTR_PCTRLCFG_PL BIT(17) /* packet len truncation */
#define SPWRTR_PCTRLCFG_TS BIT(16) /* time-code truncation */
#define SPWRTR_PCTRLCFG_TR BIT(9)  /* timer enable */
		uint32_t rtr_pctrl[32];    /* port ctrl, ports 1-31 */
#define SPWRTR_PCTRL_RD_SHIFT 24
#define SPWRTR_PCTRL_RD_MASK  BITS(SPWRTR_PCTRL_RD_SHIFT, 31) /* run state clock divisor */
#define SPWRTR_PCTRL_ST       BIT(21)                         /* status transmission enable */
#define SPWRTR_PCTRL_SR       BIT(20)                         /* status reception enable */
#define SPWRTR_PCTRL_AD       BIT(19)                         /* address detect enable */
#define SPWRTR_PCTRL_LR       BIT(18)                         /* link reset */
#define SPWRTR_PCTRL_PL       BIT(17)                         /* port link enable */
#define SPWRTR_PCTRL_TS       BIT(16)                         /* time-code send */
#define SPWRTR_PCTRL_IC       BIT(15)                         /* interrupt on connect */
#define SPWRTR_PCTRL_ET       BIT(14)                         /* error termination */
#define SPWRTR_PCTRL_NF       BIT(13)                         /* no flow-control */
#define SPWRTR_PCTRL_PS       BIT(12)                         /* port start */
#define SPWRTR_PCTRL_BE       BIT(11)                         /* broadcast enable */
#define SPWRTR_PCTRL_DI       BIT(10)                         /* disconnect */
#define SPWRTR_PCTRL_TR       BIT(9)                          /* time-code receive */
#define SPWRTR_PCTRL_PR       BIT(8)                          /* port reset */
#define SPWRTR_PCTRL_TF       BIT(7)                          /* transmit FIFO reset */
#define SPWRTR_PCTRL_RS       BIT(6)                          /* reset status */
#define SPWRTR_PCTRL_TE       BIT(5)                          /* transmit enable */
#define SPWRTR_PCTRL_CE       BIT(3)                          /* credit enable */
#define SPWRTR_PCTRL_AS       BIT(2)                          /* auto-start enable */
#define SPWRTR_PCTRL_LS       BIT(1)                          /* link start */
#define SPWRTR_PCTRL_LD       BIT(0)                          /* link disable */
	};
	union {
		uint32_t rtr_pstscfg;           /* port status, port 0 */
#define SPWRTR_PSTSCFG_EO       BIT(31) /* early EOP */
#define SPWRTR_PSTSCFG_EE       BIT(30) /* early EEP */
#define SPWRTR_PSTSCFG_PL       BIT(29) /* packet length truncation */
#define SPWRTR_PSTSCFG_TT       BIT(28) /* time-code */
#define SPWRTR_PSTSCFG_PT       BIT(27) /* packet type error */
#define SPWRTR_PSTSCFG_HC       BIT(26) /* header crc error */
#define SPWRTR_PSTSCFG_PI       BIT(25) /* protocol id error */
#define SPWRTR_PSTSCFG_CE       BIT(24) /* clear error code */
#define SPWRTR_PSTSCFG_EC_SHIFT 20
#define SPWRTR_PSTSCFG_EC_MASK  BITS(SPWRTR_PSTSCFG_EC_SHIFT, 23) /* error code */
#define SPWRTR_PSTSCFG_TS       BIT(18)                           /* timeout spill */
#define SPWRTR_PSTSCFG_ME       BIT(17)                           /* port buffer error */
#define SPWRTR_PSTSCFG_IP_SHIFT 7
#define SPWRTR_PSTSCFG_IP_MASK  BITS(SPWRTR_PSTSCFG_IP_SHIFT, 11) /* input port */
#define SPWRTR_PSTSCFG_CP       BIT(4)                            /* clear spw plug&play error code */
#define SPWRTR_PSTSCFG_PC_MASK  BITS(0, 11)                       /* spw plug&play error code */
		uint32_t rtr_psts[32];                                    /* port status, ports 1-31 */
#define SPWRTR_PSTS_PL       BIT(29)                              /* packet length truncation */
#define SPWRTR_PSTS_TT       BIT(28)                              /* time-code */
#define SPWRTR_PSTS_RS       BIT(27)                              /* RMAP/P&P spill */
#define SPWRTR_PSTS_SR       BIT(26)                              /* Spill-if-not-ready */
#define SPWRTR_PSTS_LR       BIT(22)                              /* link-start-on-request status */
#define SPWRTR_PSTS_SP       BIT(21)                              /* spill status */
#define SPWRTR_PSTS_AC       BIT(20)                              /* active status */
#define SPWRTR_PSTS_AP       BIT(19)                              /* active port */
#define SPWRTR_PSTS_TS       BIT(18)                              /* timeout spill */
#define SPWRTR_PSTS_ME       BIT(17)                              /* port buffer error */
#define SPWRTR_PSTS_TF       BIT(16)                              /* transmit FIFO full */
#define SPWRTR_PSTS_RE       BIT(15)                              /* receive FIFO empty */
#define SPWRTR_PSTS_LS_SHIFT 12
#define SPWRTR_PSTS_LS_MASK  BITS(SPWRTR_PSTS_LS_SHIFT, 14) /* link state */
#define SPWRTR_PSTS_IP_SHIFT 7
#define SPWRTR_PSTS_IP_MASK  BITS(SPWRTR_PSTS_IP_SHIFT, 11) /* input port */
#define SPWRTR_PSTS_PR       BIT(6)                         /* port receive busy */
#define SPWRTR_PSTS_PB       BIT(5)                         /* port transmit busy */
#define SPWRTR_PSTS_IA       BIT(4)                         /* invalid address */
#define SPWRTR_PSTS_CE       BIT(3)                         /* credit error */
#define SPWRTR_PSTS_ER       BIT(2)                         /* escape error */
#define SPWRTR_PSTS_DE       BIT(1)                         /* disconnect error */
#define SPWRTR_PSTS_PE       BIT(0)                         /* parity error */
	};
	uint32_t rtr_ptimer[32]; /* port timer reload */
	union {
		uint32_t rtr_pctrl2cfg;  /* port ctrl2, port 0 */
		uint32_t rtr_pctrl2[32]; /* port ctrl2, ports 1-31 */
	};
	uint32_t rtr_rtrcfg; /* router cfg/status */
#define SPWRTR_RTCFG_RS BIT(7)
	uint32_t rtr_tc;        /* time-code */
	uint32_t rtr_ver;       /* version/instance ID */
	uint32_t rtr_idiv;      /* init divisor */
	uint32_t rtr_cfgwe;     /* cfg write enable */
	uint32_t rtr_prescaler; /* timer prescaler reload */
	uint32_t rtr_imask;     /* interrupt mask */
	uint32_t rtr_ipmask;    /* interrupt port mask */
	uint32_t rtr_pip;       /* port interrupt pending */
	uint32_t rtr_icodegen;  /* interrupt code generation */
	uint32_t rtr_isr0;      /* int code distr ISR 0-31 */
	uint32_t rtr_isr1;      /* int code distr ISR 32-63 */
	uint32_t rtr_isrtimer;  /* ISR timer reload */
	uint32_t rtr_aitimer;   /* ACK-to-INT timer reload */
	uint32_t rtr_isrctimer; /* ISR change timer reload */
	R_RESERVED(0xA3C, 0xA40);
	uint32_t rtr_lrunstat; /* link running status */
	uint32_t rtr_cap;      /* capability */
	R_RESERVED(0xA48, 0xA50);
	uint32_t rtr_pnpvend;    /* PnP dev vendor/product ID */
	uint32_t rtr_pnpuvend;   /* PnP unit vendor/product ID */
	uint32_t rtr_pnpusn;     /* PnP unit serial number */
	uint32_t rtr_pnpnetdisc; /* PnP port net discovery enable */
	R_RESERVED(0xA60, 0xC10);
	uint32_t rtr_ocharcnt[31]; /* outgoing char counter */
	uint32_t rtr_icharcnt[31]; /* incoming char counter */
	uint32_t rtr_opktcnt[31];  /* outgoing packet counter */
	uint32_t rtr_ipktcnt[31];  /* incoming packet counter */
	uint32_t rtr_maxplen[32];  /* maximum packet length */
	uint32_t rtr_credcnt[31];  /* credit counter */
	uint32_t rtr_gpo[4];       /* general purpose out 128 bits */
	uint32_t rtr_gpi[4];       /* general purpose in 128 bits */
	R_RESERVED(0x1000, 0x1004);
	uint32_t rtr_rtcomb_phy[31];  /* combined rt, addr 1-31 */
	uint32_t rtr_rtcomb_log[224]; /* combined rt, addr 32-255 */
	R_RESERVED(0x1400, 0x2000);
	uint32_t rtr_apbarea[1024]; /* APB address area */
};


typedef struct {
	volatile struct spwrtr_regs *mmio;
} spwrtr_dev_t;


static struct {
	volatile uint32_t *base;
	unsigned int irq;
} spwrtr_info;


static struct {
	oid_t oid;
	spwrtr_dev_t dev;
} spwrtr_common;


/* Operations on device */


static int spwrtr_setPortMapping(spwrtr_dev_t *dev, uint8_t port, uint32_t enPorts)
{
	if (port == 0) {
		return -EINVAL;
	}

	if ((enPorts >> (SPWRTR_PORTS + 1)) > 0) {
		return -EINVAL;
	}

	dev->mmio->rtr_rtpmap[port] = enPorts;
	TRACE("port: %d enPorts: %08x", port, enPorts);

	return 0;
}


static int spwrtr_getPortMapping(spwrtr_dev_t *dev, uint8_t port, uint32_t *enPorts)
{
	if (port == 0) {
		return -EINVAL;
	}

	*enPorts = dev->mmio->rtr_rtpmap[port];
	TRACE("port: %d enPorts: %08x", port, *enPorts);

	return 0;
}


static int spwrtr_resetDevice(spwrtr_dev_t *dev)
{

	dev->mmio->rtr_rtrcfg = SPWRTR_RTCFG_RS;
	while ((dev->mmio->rtr_rtrcfg & SPWRTR_RTCFG_RS) != 0) { }

	TRACE("reset");

	return 0;
}


/* Message handling */


static void spwrtr_handleDevCtl(msg_t *msg)
{
	spwrtr_t *ictl = (spwrtr_t *)msg->i.raw;
	spwrtr_o_t *octl = (spwrtr_o_t *)msg->o.raw;

	int err;
	switch (ictl->type) {
		case spwrtr_pmap_set:
			err = spwrtr_setPortMapping(&spwrtr_common.dev, ictl->task.mapping.port, ictl->task.mapping.enPorts);
			break;

		case spwrtr_pmap_get:
			err = spwrtr_getPortMapping(&spwrtr_common.dev, ictl->task.mapping.port, &octl->val);
			break;

		case spwrtr_reset:
			err = spwrtr_resetDevice(&spwrtr_common.dev);
			break;

		default:
			err = -EINVAL;
			break;
	}
	msg->o.err = err;
}


static void spwrtr_msgLoop(void)
{
	msg_t msg;
	msg_rid_t rid;

	for (;;) {
		while (msgRecv(spwrtr_common.oid.port, &msg, &rid) < 0) {
		}
		switch (msg.type) {
			case mtDevCtl:
				spwrtr_handleDevCtl(&msg);
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

		msgRespond(spwrtr_common.oid.port, &msg, rid);
	}
}


/* Initialization */


static int spwrtr_createDevs(oid_t *oid, int n)
{
	char buf[11];
	if (snprintf(buf, sizeof(buf), "spwrtr%d", n) >= sizeof(buf)) {
		return -1;
	}

	if (create_dev(oid, buf) < 0) {
		return -1;
	}
	return 0;
}


static int spwrtr_init(int n)
{
	if (n < 0) {
		LOG_ERROR("Invalid instance number: %d", n);
		return -1;
	}
	unsigned int instance = n;

	ambapp_dev_t dev = { .devId = CORE_ID_GRSPWROUTER };
	platformctl_t pctl = {
		.action = pctl_get,
		.type = pctl_ambapp,
		.task.ambapp = {
			.dev = &dev,
			.instance = &instance,
		}
	};

	if (platformctl(&pctl) < 0) {
		LOG_ERROR("Failed to find grspwrouter core");
		return -1;
	}

	if (dev.bus != BUS_AMBA_AHB) {
		/* GRSPWROUTER should be on AHB bus */
		LOG_ERROR("Failed to find grspwrouter on AHB bus");
		return -1;
	}

	/* find AHB I/O space bar */
	for (int j = 0; j < 4; j++) {
		if (dev.info.ahb.type[j] == AMBA_TYPE_AHBIO) {
			spwrtr_info.base = dev.info.ahb.base[j];
			break;
		}
	}
	spwrtr_info.irq = dev.irqn;

	uintptr_t pbase = (uintptr_t)spwrtr_info.base;
	uintptr_t base = (pbase & ~(_PAGE_SIZE - 1));
	spwrtr_common.dev.mmio = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
	if (spwrtr_common.dev.mmio == MAP_FAILED) {
		LOG_ERROR("Failed to create resources");
		return -1;
	}

	spwrtr_common.dev.mmio = (void *)((uintptr_t)spwrtr_common.dev.mmio + pbase - base);

	return 0;
}


static void spwrtr_usage(const char *progname)
{
	printf("Usage: %s [options]\n", progname);
	printf("Options:\n");
	printf("\t-n <id> - spwrtr core id\n");
}


int main(int argc, char **argv)
{
	oid_t oid;
	int c;
	int spwrtrn = 0;

	if (argc > 1) {
		do {
			c = getopt(argc, argv, "n:");
			switch (c) {
				case 'n':
					spwrtrn = atoi(optarg);
					break;

				case -1:
					break;

				default:
					spwrtr_usage(argv[0]);
					return EXIT_FAILURE;
			}
		} while (c != -1);
	}

	if (portCreate(&spwrtr_common.oid.port) < 0) {
		LOG_ERROR("Failed to create port");
		return EXIT_FAILURE;
	}

	/* Wait for rootfs */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(100000);
	}

	if (spwrtr_init(spwrtrn) < 0) {
		portDestroy(spwrtr_common.oid.port);
		LOG_ERROR("Failed to initialize SpaceWire-router");
		return EXIT_FAILURE;
	}

	if (spwrtr_createDevs(&spwrtr_common.oid, spwrtrn) < 0) {
		portDestroy(spwrtr_common.oid.port);
		LOG_ERROR("Failed to create SpaceWire-router device");
		return EXIT_FAILURE;
	}

	LOG("initialized");
	priority(SPWRTR_PRIO);
	spwrtr_msgLoop();

	return 0;
}
