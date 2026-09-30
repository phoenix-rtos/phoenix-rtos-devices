/*
 * Phoenix-RTOS
 *
 * GRLIB SpaceWire Router driver
 *
 * Copyright 2026 Phoenix Systems
 * Author: Andrzej Tlomak, Lukasz Leczkowski
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <board_config.h>
#include <stdio.h>
#include <errno.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <phoenix/gaisler/ambapp.h>

#ifdef __CPU_GR765
#include <phoenix/arch/riscv64/riscv64.h>
#else
#include <phoenix/arch/sparcv8leon/sparcv8leon.h>
#endif

#include <libgrspwrtr.h>


/* clang-format off */
#define TRACE(fmt, ...) do { if (0) { printf("%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } } while (0)
#define LOG(fmt, ...)       printf("spacewire-router: " fmt "\n", ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) fprintf(stderr, "spacewire-router: %s: " fmt "\n", __func__, ##__VA_ARGS__)
/* clang-format on */

#define CEIL_PAGE(x) ((((size_t)(x)) + _PAGE_SIZE - 1U) & (~(_PAGE_SIZE - 1U)))

#define SPWRTR_MAX_PHY_PORT 31
#define SPWRTR_FIRST_LOG    32

#define BIT(i)           (1U << (i))                                        /* bitmask for single bit */
#define BITS(start, end) (((1U << ((end) - (start) + 1U)) - 1U) << (start)) /* bitmask for bits in provided range (inclusive) */

/* ECSS-E-ST-50-12C: link initialization at 10 Mbit/s, minimum run rate 2 Mbit/s */
#define SPWRTR_INIT_FREQ 10000000U
#define SPWRTR_MIN_FREQ  2000000U

#define SPWRTR_RESET_RETRIES 1000U

#define SPWRTR_LINK_POLL_US 100U


#define SPWRTR_RTPMAP_PD BIT(0) /* packet distribution (port bits start at bit 1) */

#define SPWRTR_RTACTRL_MASK BITS(0, 3)

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

#define SPWRTR_PSTS_PT_SHIFT 30
#define SPWRTR_PSTS_PT_MASK  BITS(SPWRTR_PSTS_PT_SHIFT, 31) /* port type */
#define SPWRTR_PSTS_TT       BIT(28)                        /* time-code */
#define SPWRTR_PSTS_LR       BIT(22)                        /* link-start-on-request status */
#define SPWRTR_PSTS_SP       BIT(21)                        /* spill status */
#define SPWRTR_PSTS_AC       BIT(20)                        /* active status */
#define SPWRTR_PSTS_AP       BIT(19)                        /* active port */
#define SPWRTR_PSTS_TF       BIT(16)                        /* transmit FIFO full */
#define SPWRTR_PSTS_RE       BIT(15)                        /* receive FIFO empty */
#define SPWRTR_PSTS_LS_SHIFT 12
#define SPWRTR_PSTS_LS_MASK  BITS(SPWRTR_PSTS_LS_SHIFT, 14) /* link state */
#define SPWRTR_PSTS_IP_SHIFT 7
#define SPWRTR_PSTS_IP_MASK  BITS(SPWRTR_PSTS_IP_SHIFT, 11) /* input port */
#define SPWRTR_PSTS_PR       BIT(6)                         /* port receive busy */
#define SPWRTR_PSTS_PB       BIT(5)                         /* port transmit busy */

#define SPWRTR_RTRCFG_SP_SHIFT 27
#define SPWRTR_RTRCFG_SP       BITS(SPWRTR_RTRCFG_SP_SHIFT, 31)
#define SPWRTR_RTRCFG_AP_SHIFT 22
#define SPWRTR_RTRCFG_AP       BITS(SPWRTR_RTRCFG_AP_SHIFT, 26)
#define SPWRTR_RTRCFG_FP_SHIFT 17
#define SPWRTR_RTRCFG_FP       BITS(SPWRTR_RTRCFG_FP_SHIFT, 21)
#define SPWRTR_RTRCFG_SR       BIT(15) /* static routing enable */
#define SPWRTR_RTRCFG_RE       BIT(7)  /* router reset */

#define SPWRTR_IDIV_MASK BITS(0, 7)

/* Nearest-rounded init divisor */
#define SPWRTR_IDIV ((SPWCLK_FREQ + (SPWRTR_INIT_FREQ / 2U)) / SPWRTR_INIT_FREQ - 1U)


struct spwrtr_regs {
	uint32_t rtpmap[256];  /* routing table port map (addr under index 0 is restricted) */
	uint32_t rtactrl[256]; /* routing table addr ctrl (addr under index 0 is restricted) */
	union {
		uint32_t pctrlcfg;  /* port ctrl, port 0 */
		uint32_t pctrl[32]; /* port ctrl, ports 1-31 */
	};
	union {
		uint32_t pstscfg;  /* port status, port 0 */
		uint32_t psts[32]; /* port status, ports 1-31 */
	};
	uint32_t ptimer[32]; /* port timer reload */
	union {
		uint32_t pctrl2cfg;  /* port ctrl2, port 0 */
		uint32_t pctrl2[32]; /* port ctrl2, ports 1-31 */
	};
	uint32_t rtrcfg;    /* router cfg/status */
	uint32_t tc;        /* Time-code */
	uint32_t ver;       /* Version/instance ID */
	uint32_t idiv;      /* Initialization divisor */
	uint32_t cfgwe;     /* Configuration port write enable */
	uint32_t prescaler; /* Timer prescaler reload */
	uint32_t imask;     /* Interrupt mask */
	uint32_t ipmask;    /* Interrupt port mask */
};


/* Auxiliary functions */


static inline bool spwrtr_isSpwPort(const spwrtr_dev_t *dev, uint8_t port)
{
	return (port != 0U) && (port <= dev->nSpwPorts);
}


/* Any port except the configuration port */
static inline bool spwrtr_isPort(const spwrtr_dev_t *dev, uint8_t port)
{
	return (port != 0U) && (port < dev->nPorts);
}


static int spwrtr_checkAddr(const spwrtr_dev_t *dev, uint8_t addr)
{
	if (addr == 0U) {
		/* Address 0 is the configuration port - not routable */
		return -EINVAL;
	}

	if ((addr <= SPWRTR_MAX_PHY_PORT) && (addr >= dev->nPorts)) {
		/* Physical address of a non-existent port */
		return -EINVAL;
	}

	return 0;
}


static uint32_t spwrtr_validPortMask(const spwrtr_dev_t *dev)
{
	/* Bits 1..nPorts-1, bit 0 is packet distribution */
	return BITS(1, dev->nPorts - 1U);
}


static const char *spwrtr_portTypeName(const spwrtr_dev_t *dev, uint8_t port)
{
	if (port == 0U) {
		return "cfg";
	}
	if (port <= dev->nSpwPorts) {
		return "spw";
	}
	if (port <= (dev->nSpwPorts + dev->nAmbaPorts)) {
		return "amba";
	}
	if (port < dev->nPorts) {
		return "fifo";
	}
	return "none";
}


/* Operations on device */


int spwrtr_ambaPort(const spwrtr_dev_t *dev, unsigned int n)
{
	if (n >= dev->nAmbaPorts) {
		return -ENODEV;
	}

	return (int)(dev->nSpwPorts + 1U + n);
}


int spwrtr_setPortMapping(spwrtr_dev_t *dev, uint8_t addr, uint32_t enPorts)
{
	int err = spwrtr_checkAddr(dev, addr);
	if (err < 0) {
		return err;
	}

	if ((enPorts & ~SPWRTR_RTPMAP_PD & ~spwrtr_validPortMask(dev)) != 0U) {
		return -EINVAL;
	}

	dev->regs->rtpmap[addr] = enPorts;
	TRACE("addr: %u enPorts: %08" PRIx32, addr, enPorts);

	return 0;
}


int spwrtr_getPortMapping(spwrtr_dev_t *dev, uint8_t addr, uint32_t *enPorts)
{
	int err = spwrtr_checkAddr(dev, addr);
	if (err < 0) {
		return err;
	}

	*enPorts = dev->regs->rtpmap[addr];
	TRACE("addr: %u enPorts: %08" PRIx32, addr, *enPorts);

	return 0;
}


int spwrtr_setAddrCtrl(spwrtr_dev_t *dev, uint8_t addr, uint32_t ctrl)
{
	int err = spwrtr_checkAddr(dev, addr);
	if (err < 0) {
		return err;
	}

	if ((ctrl & ~SPWRTR_RTACTRL_MASK) != 0U) {
		return -EINVAL;
	}

	dev->regs->rtactrl[addr] = ctrl;
	TRACE("addr: %u ctrl: %" PRIx32, addr, ctrl);

	return 0;
}


int spwrtr_getAddrCtrl(spwrtr_dev_t *dev, uint8_t addr, uint32_t *ctrl)
{
	int err = spwrtr_checkAddr(dev, addr);
	if (err < 0) {
		return err;
	}

	*ctrl = dev->regs->rtactrl[addr] & SPWRTR_RTACTRL_MASK;

	return 0;
}


int spwrtr_setRoute(spwrtr_dev_t *dev, uint8_t addr, uint32_t enPorts, uint32_t ctrl)
{
	if ((enPorts & ~SPWRTR_RTPMAP_PD) == 0U) {
		LOG_ERROR("addr %u: empty port map", addr);
		return -EINVAL;
	}

	ctrl |= SPWRTR_RTACTRL_EN;

	/* Disable the entry while it is modified, so that it is never enabled with a partial map */
	int err = spwrtr_setAddrCtrl(dev, addr, ctrl & ~SPWRTR_RTACTRL_EN);
	if (err < 0) {
		return err;
	}

	err = spwrtr_setPortMapping(dev, addr, enPorts);
	if (err < 0) {
		return err;
	}

	err = spwrtr_setAddrCtrl(dev, addr, ctrl);
	if (err < 0) {
		return err;
	}

	return 0;
}


int spwrtr_clearRoute(spwrtr_dev_t *dev, uint8_t addr)
{
	int err = spwrtr_setAddrCtrl(dev, addr, 0);
	if (err < 0) {
		return err;
	}

	dev->regs->rtpmap[addr] = 0;

	return 0;
}


int spwrtr_resetDevice(spwrtr_dev_t *dev)
{
	dev->regs->rtrcfg = SPWRTR_RTRCFG_RE;

	unsigned int retries = SPWRTR_RESET_RETRIES;
	while (((dev->regs->rtrcfg & SPWRTR_RTRCFG_RE) != 0U) && (retries-- > 0U)) {
		usleep(10);
	}

	if ((dev->regs->rtrcfg & SPWRTR_RTRCFG_RE) != 0U) {
		LOG_ERROR("reset did not complete");
		return -ETIMEDOUT;
	}

	/* Reset restores the default init divisor */
	dev->regs->idiv = SPWRTR_IDIV;

	TRACE("reset");

	return 0;
}


int spwrtr_setClockDiv(spwrtr_dev_t *dev, uint8_t port, uint32_t clkdiv)
{
	if (!spwrtr_isSpwPort(dev, port) || ((clkdiv & ~0xffU) != 0U)) {
		return -EINVAL;
	}

	uint32_t pctrl = dev->regs->pctrl[port];
	pctrl &= ~SPWRTR_PCTRL_RD_MASK;
	pctrl |= (clkdiv << SPWRTR_PCTRL_RD_SHIFT) & SPWRTR_PCTRL_RD_MASK;

	dev->regs->pctrl[port] = pctrl;

	TRACE("port: %u set clockdiv: %" PRIu32, port, clkdiv);

	return 0;
}


int spwrtr_getClockDiv(spwrtr_dev_t *dev, uint8_t port, uint32_t *clkdiv)
{
	if (!spwrtr_isSpwPort(dev, port)) {
		return -EINVAL;
	}

	*clkdiv = (dev->regs->pctrl[port] & SPWRTR_PCTRL_RD_MASK) >> SPWRTR_PCTRL_RD_SHIFT;
	TRACE("port: %u clockdiv: %" PRIu32, port, *clkdiv);

	return 0;
}


const char *spwrtr_linkStateName(uint8_t linkState)
{
	switch (linkState) {
		case SPWRTR_LS_ERR_RESET: return "error-reset";
		case SPWRTR_LS_ERR_WAIT: return "error-wait";
		case SPWRTR_LS_READY: return "ready";
		case SPWRTR_LS_STARTED: return "started";
		case SPWRTR_LS_CONNECTING: return "connecting";
		case SPWRTR_LS_RUN: return "run";
		default: return "unknown";
	}
}


int spwrtr_getLinkState(spwrtr_dev_t *dev, uint8_t port, uint8_t *linkState)
{
	if (!spwrtr_isSpwPort(dev, port)) {
		return -EINVAL;
	}

	uint32_t sts = dev->regs->psts[port];
	*linkState = (uint8_t)((sts & SPWRTR_PSTS_LS_MASK) >> SPWRTR_PSTS_LS_SHIFT);

	return 0;
}


int spwrtr_getPortStatus(spwrtr_dev_t *dev, uint8_t port, uint32_t *sts)
{
	if (!spwrtr_isPort(dev, port)) {
		return -EINVAL;
	}

	*sts = dev->regs->psts[port];

	return 0;
}


int spwrtr_clearPortErrors(spwrtr_dev_t *dev, uint8_t port, uint32_t *errs)
{
	if (!spwrtr_isPort(dev, port)) {
		return -EINVAL;
	}

	/* Write back only the error bits that were seen - W1C */
	uint32_t found = dev->regs->psts[port] & SPWRTR_PSTS_ERR_MASK;
	if (found != 0U) {
		dev->regs->psts[port] = found;
	}

	if (errs != NULL) {
		*errs = found;
	}

	return 0;
}


void spwrtr_logPortStatus(spwrtr_dev_t *dev, uint8_t port)
{
	if (!spwrtr_isPort(dev, port)) {
		LOG_ERROR("invalid port %u", port);
		return;
	}

	const uint32_t sts = dev->regs->psts[port];
	const uint32_t ctrl = dev->regs->pctrl[port];

	LOG("port %u (%s): ctrl=0x%08" PRIx32 " sts=0x%08" PRIx32 " type=%" PRIu32,
			port, spwrtr_portTypeName(dev, port), ctrl, sts, (sts & SPWRTR_PSTS_PT_MASK) >> SPWRTR_PSTS_PT_SHIFT);

	if (spwrtr_isSpwPort(dev, port)) {
		const uint8_t ls = (uint8_t)((sts & SPWRTR_PSTS_LS_MASK) >> SPWRTR_PSTS_LS_SHIFT);
		const uint32_t rd = (ctrl & SPWRTR_PCTRL_RD_MASK) >> SPWRTR_PCTRL_RD_SHIFT;
		LOG("  link: %s, run rate %" PRIu32 " kbit/s (div %" PRIu32 "), ld=%u ls=%u as=%u di=%u",
				spwrtr_linkStateName(ls), (uint32_t)(SPWCLK_FREQ / 1000U) / (rd + 1U), rd,
				(ctrl & SPWRTR_PCTRL_LD) != 0U, (ctrl & SPWRTR_PCTRL_LS) != 0U,
				(ctrl & SPWRTR_PCTRL_AS) != 0U, (ctrl & SPWRTR_PCTRL_DI) != 0U);
	}

	LOG("  flags:%s%s%s%s%s%s%s  input port: %" PRIu32,
			((sts & SPWRTR_PSTS_PR) != 0U) ? " rx-busy" : "",
			((sts & SPWRTR_PSTS_PB) != 0U) ? " tx-busy" : "",
			((sts & SPWRTR_PSTS_SP) != 0U) ? " spilling" : "",
			((sts & SPWRTR_PSTS_AC) != 0U) ? " active" : "",
			((sts & SPWRTR_PSTS_TF) != 0U) ? " txfifo-full" : "",
			((sts & SPWRTR_PSTS_RE) != 0U) ? " rxfifo-empty" : "",
			((sts & SPWRTR_PSTS_LR) != 0U) ? " link-start-req" : "",
			(sts & SPWRTR_PSTS_IP_MASK) >> SPWRTR_PSTS_IP_SHIFT);

	const uint32_t errs = sts & SPWRTR_PSTS_ERR_MASK;
	if (errs == 0U) {
		LOG("  errors: none");
	}
	else {
		LOG("  errors:%s%s%s%s%s%s%s%s%s%s",
				((errs & SPWRTR_PSTS_ERR_PE) != 0U) ? " parity" : "",
				((errs & SPWRTR_PSTS_ERR_DE) != 0U) ? " disconnect" : "",
				((errs & SPWRTR_PSTS_ERR_ER) != 0U) ? " escape" : "",
				((errs & SPWRTR_PSTS_ERR_CE) != 0U) ? " credit" : "",
				((errs & SPWRTR_PSTS_ERR_IA) != 0U) ? " invalid-address(no-route)" : "",
				((errs & SPWRTR_PSTS_ERR_ME) != 0U) ? " memory" : "",
				((errs & SPWRTR_PSTS_ERR_TS) != 0U) ? " timeout-spill" : "",
				((errs & SPWRTR_PSTS_ERR_SR) != 0U) ? " spill-not-ready" : "",
				((errs & SPWRTR_PSTS_ERR_RS) != 0U) ? " rmap-spill" : "",
				((errs & SPWRTR_PSTS_ERR_PL) != 0U) ? " length-truncation" : "");
	}
}


void spwrtr_logRoute(spwrtr_dev_t *dev, uint8_t addr)
{
	if (spwrtr_checkAddr(dev, addr) < 0) {
		LOG_ERROR("invalid address %u", addr);
		return;
	}

	const uint32_t map = dev->regs->rtpmap[addr];
	const uint32_t ctrl = dev->regs->rtactrl[addr] & SPWRTR_RTACTRL_MASK;

	char ports[3 * 32 + 1];
	size_t len = 0;
	ports[0] = '\0';
	for (uint8_t p = 1; p < dev->nPorts; p++) {
		if (((map & BIT(p)) != 0U) && (len < sizeof(ports))) {
			len += (size_t)snprintf(&ports[len], sizeof(ports) - len, " %u", p);
		}
	}

	LOG("route 0x%x (%s): map=0x%08" PRIx32 " ->%s%s ctrl=0x%" PRIx32 "%s%s%s%s",
			addr, (addr < SPWRTR_FIRST_LOG) ? "phy" : "log", map,
			(len != 0U) ? ports : " (none)",
			((map & SPWRTR_RTPMAP_PD) != 0U) ? " [distribution]" : "",
			ctrl,
			((ctrl & SPWRTR_RTACTRL_EN) != 0U) ? " EN" : " disabled",
			((ctrl & SPWRTR_RTACTRL_HD) != 0U) ? " HD" : "",
			((ctrl & SPWRTR_RTACTRL_PR) != 0U) ? " PR" : "",
			((ctrl & SPWRTR_RTACTRL_SR) != 0U) ? " SR" : "");
}


/* Initialization */


/* Configure SpW port run-state link frequency and start the link.
 *  port - SpW port number
 *  spwFreq - requested run-state link frequency in Hz
 */
int spwrtr_portStart(spwrtr_dev_t *rtrDev, uint8_t port, uint32_t spwFreq)
{
	if (!spwrtr_isSpwPort(rtrDev, port)) {
		LOG_ERROR("port %u is not a SpW port (SpW ports: 1..%u)", port, rtrDev->nSpwPorts);
		return -EINVAL;
	}

	if ((spwFreq == 0U) || (spwFreq > SPWCLK_FREQ)) {
		LOG_ERROR("port %u: invalid link frequency %" PRIu32 " Hz (max %u Hz)", port, spwFreq, (unsigned int)SPWCLK_FREQ);
		return -EINVAL;
	}

	uint32_t rd = (SPWCLK_FREQ - 1U) / spwFreq;
	if (rd > 255U) {
		rd = 255U;
	}

	const uint32_t actual = SPWCLK_FREQ / (rd + 1U);
	if (actual < SPWRTR_MIN_FREQ) {
		LOG_ERROR("port %u: link rate %" PRIu32 " Hz below 2 Mbit/s minimum", port, actual);
		return -EINVAL;
	}

	/* Drop stale errors so that the status after start reflects this start only */
	(void)spwrtr_clearPortErrors(rtrDev, port, NULL);

	/* Set divisor, enable autostart and start link */
	uint32_t pctrl = rtrDev->regs->pctrl[port];
	pctrl = (pctrl & ~SPWRTR_PCTRL_RD_MASK) | ((rd << SPWRTR_PCTRL_RD_SHIFT) & SPWRTR_PCTRL_RD_MASK);
	pctrl = pctrl & ~(SPWRTR_PCTRL_DI | SPWRTR_PCTRL_LD);
	pctrl |= SPWRTR_PCTRL_AS | SPWRTR_PCTRL_LS;
	rtrDev->regs->pctrl[port] = pctrl;

	LOG("port %u: run rate %" PRIu32 " kbit/s (div %" PRIu32 "), link start + autostart", port, actual / 1000U, rd);

	return 0;
}


int spwrtr_waitLinkRun(spwrtr_dev_t *dev, uint8_t port, uint32_t timeoutUs)
{
	uint8_t ls = SPWRTR_LS_ERR_RESET;
	uint32_t waited = 0;

	for (;;) {
		int err = spwrtr_getLinkState(dev, port, &ls);
		if (err < 0) {
			return err;
		}

		if (ls == SPWRTR_LS_RUN) {
			break;
		}

		if (waited >= timeoutUs) {
			LOG_ERROR("port %u: link not running after %" PRIu32 " us (state: %s)", port, waited, spwrtr_linkStateName(ls));
			spwrtr_logPortStatus(dev, port);
			return -ETIMEDOUT;
		}

		usleep(SPWRTR_LINK_POLL_US);
		waited += SPWRTR_LINK_POLL_US;
	}

	/* Disconnect/escape/parity errors are expected while the link is starting up */
	uint32_t errs = 0;
	(void)spwrtr_clearPortErrors(dev, port, &errs);

	LOG("port %u: link running", port);

	return 0;
}


int spwrtr_init(spwrtr_dev_t *rtrDev, unsigned int n)
{
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

	rtrDev->regs = MAP_FAILED;

	if (platformctl(&pctl) < 0) {
		return -ENODEV;
	}

	if (dev.bus != BUS_AMBA_AHB) {
		/* GRSPWROUTER should be on AHB bus */
		return -ENODEV;
	}

	uintptr_t pbase = 0;
	int found = 0;
	/* find AHB I/O space bar */
	for (int j = 0; j < 4; j++) {
		if (dev.info.ahb.type[j] == AMBA_TYPE_AHBIO) {
			pbase = (uintptr_t)dev.info.ahb.base[j];
			found = 1;
			break;
		}
	}

	if (found == 0) {
		return -ENODEV;
	}

	uintptr_t base = (pbase & ~(_PAGE_SIZE - 1));
	size_t regsSize = CEIL_PAGE(sizeof(struct spwrtr_regs) + (pbase - base));

	void *vbase = mmap(NULL, regsSize, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, (off_t)base);
	if (vbase == MAP_FAILED) {
		return -ENOMEM;
	}

	rtrDev->regs = (void *)((uintptr_t)vbase + (pbase - base));

	/* Read out configuration */
	const uint32_t cfg = rtrDev->regs->rtrcfg;
	rtrDev->nSpwPorts = ((cfg & SPWRTR_RTRCFG_SP) >> SPWRTR_RTRCFG_SP_SHIFT);
	rtrDev->nAmbaPorts = ((cfg & SPWRTR_RTRCFG_AP) >> SPWRTR_RTRCFG_AP_SHIFT);
	rtrDev->nFifoPorts = ((cfg & SPWRTR_RTRCFG_FP) >> SPWRTR_RTRCFG_FP_SHIFT);

	rtrDev->nPorts = 1U + rtrDev->nSpwPorts + rtrDev->nAmbaPorts + rtrDev->nFifoPorts;

	/* Set IDIV for 10 Mbit/s link initialization */
	rtrDev->regs->idiv = SPWRTR_IDIV;

	const uint32_t ver = rtrDev->regs->ver;
	LOG("grspwrouter %u @0x%" PRIxPTR ": ver %" PRIu32 ".%" PRIu32 " id %" PRIu32 ", ports: spw %u (1..%u), amba %u (%u..%u), fifo %u%s",
			n, pbase, (ver >> 24) & 0xffU, (ver >> 16) & 0xffU, ver & 0xffU,
			rtrDev->nSpwPorts, rtrDev->nSpwPorts,
			rtrDev->nAmbaPorts, rtrDev->nSpwPorts + 1U, (unsigned int)rtrDev->nSpwPorts + rtrDev->nAmbaPorts,
			rtrDev->nFifoPorts, ((cfg & SPWRTR_RTRCFG_SR) != 0U) ? ", static routing" : "");
	LOG("init rate %u kbit/s (idiv %u)", (unsigned int)(SPWCLK_FREQ / 1000U / (SPWRTR_IDIV + 1U)), (unsigned int)SPWRTR_IDIV);

	return 0;
}
