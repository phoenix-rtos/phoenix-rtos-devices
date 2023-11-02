/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL SDMA driver
 *
 * Copyright 2018 Phoenix Systems
 * Author: Krystian Wasik
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <syslog.h>
#include <fcntl.h>

#include <sys/stat.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/file.h>
#include <posix/utils.h>

#include <phoenix/arch/imx6ull.h>

#include "sdma-api.h"

#if 0
#define COL_RED     "\033[1;31m"
#define COL_CYAN    "\033[1;36m"
#define COL_YELLOW  "\033[1;33m"
#define COL_NORMAL  "\033[0m"
#else
#define COL_RED
#define COL_CYAN
#define COL_YELLOW
#define COL_NORMAL
#endif

#define LOG_IDENT "imx6ull-sdma"
#define LOG_TAG LOG_IDENT": "
#define log_debug(fmt, ...)     do { log_printf(LOG_DEBUG, fmt "\n", ##__VA_ARGS__); } while (0)
#define log_info(fmt, ...)      do { log_printf(LOG_INFO, fmt "\n", ##__VA_ARGS__); } while (0)
#define log_warn(fmt, ...)      do { log_printf(LOG_WARNING, COL_YELLOW fmt COL_NORMAL "\n", ##__VA_ARGS__); } while (0)
#define log_error(fmt, ...)     do { log_printf(LOG_ERR, COL_RED fmt COL_NORMAL "\n", ##__VA_ARGS__); } while (0)

#define NUM_OF_SDMA_CHANNELS    (32)
#define NUM_OF_SDMA_REQUESTS    (48)

#define NUM_OF_WORKER_THREADS   4 //(NUM_OF_SDMA_CHANNELS)
#define WORKER_THD_PRIO         (3)
#define WORKER_THD_STACK        (4096)

#define STATS_THD_PRIO          (4)
#define STATS_THD_STACK         (4096)

#define MAIN_THD_PRIO           (2)

/* Buffer Descriptor Commands for Bootload scripts */
#define SDMA_CMD_C0_SET_DM                      (0x1)
#define SDMA_CMD_C0_GET_DM                      (0x2)
#define SDMA_CMD_C0_SET_PM                      (0x4)
#define SDMA_CMD_C0_GET_PM                      (0x6)
#define SDMA_CMD_C0_SETCTX(channel)             (0x07 | (channel << 3))
#define SDMA_CMD_C0_GETCTX(channel)             (0x03 | (channel << 3))

#define INTR_CHANNEL_TIMEOUT_US (35 * 1000 * 1000)

/* Channel Control Block */
struct __attribute__((packed)) sdma_channel_ctrl_s {
	addr_t current_bd; /* Current buffer descriptor pointer */
	addr_t base_bd; /* Base buffer descriptor pointer */
	uint32_t channel_desc;
	uint32_t status;
};

typedef struct sdma_channel_ctrl_s sdma_channel_ctrl_t;

typedef struct __attribute__((packed)) sdma_arm_regs_s {
	uint32_t MC0PTR;
	uint32_t INTR;
	uint32_t STOP_STAT;
	uint32_t HSTART;
	uint32_t EVTOVR;
	uint32_t DSPOVR;
	uint32_t HOSTOVR;
	uint32_t EVTPEND;
	uint32_t reserved0;
	uint32_t RESET;
	uint32_t EVTERR;
	uint32_t INTRMASK;
	uint32_t PSW;
	uint32_t EVTERRDBG;
	uint32_t CONFIG;
	uint32_t SDMA_LOCK;
	uint32_t ONCE_ENB;
	uint32_t ONCE_DATA;
	uint32_t ONCE_INSTR;
	uint32_t ONCE_STAT;
	uint32_t ONCE_CMD;
	uint32_t reserved1;
	uint32_t ILLINSTRADDR;
	uint32_t CHN0ADDR;
	uint32_t EVT_MIRROR;
	uint32_t EVT_MIRROR2;
	uint32_t reserved2[2];
	uint32_t XTRIG_CONF1;
	uint32_t XTRIG_CONF2;
	uint32_t reserved3[34];
	uint32_t SDMA_CHNPRI[32];
	uint32_t reserved4[32];
	uint32_t CHNENBL[48];
} sdma_arm_regs_t;

/* Context Switching Mode */
typedef enum {
	sdma_csm__static                = 0x0,
	sdma_csm__dynamic_low_power     = 0x1,
	sdma_csm__dynamic_no_loop       = 0x2,
	sdma_csm__dynamic               = 0x3,
} sdma_csm_t;

typedef struct {
	int active;
	int auto_bd_done;

	sdma_buffer_desc_t *bd;
	addr_t bd_paddr;

	id_t file_id;

	handle_t intr_cond;
	unsigned intr_cnt;
	unsigned missed_intr_cnt;

	unsigned read_cnt;
} sdma_channel_t;

struct driver_common_s
{
	char worker_thd_stack[NUM_OF_WORKER_THREADS][WORKER_THD_STACK] __attribute__ ((aligned(8)));
	char stats_thd_stack[STATS_THD_STACK] __attribute__ ((aligned(8)));

	uint32_t port;

	sdma_channel_t channel[NUM_OF_SDMA_CHANNELS];
	sdma_channel_ctrl_t *ccb; /* Pointer to channel control block array */
	addr_t ccb_paddr;

	/* Temporary buffer (uncached, with known physical address) for
	 * loading/dumping contexts, scripts etc. */
	size_t tmp_size;
	void *tmp;
	addr_t tmp_paddr;

	volatile sdma_arm_regs_t *regs;

	handle_t intr_cond;
	handle_t lock;

	addr_t ocram_next;

	int stats_period_s;
	int use_syslog;
	int initialized;

	int broken;

	const char *dump_dir;

	uint32_t active_mask;
} common;

static void log_printf(int lvl, const char* fmt, ...)
{
	va_list arg;

	va_start(arg, fmt);

	/* Don't use syslog until initialized */
	if (!common.use_syslog || !common.initialized) {
		printf("%s", LOG_TAG);
		vprintf(fmt, arg);
	} else {
		vsyslog(lvl, fmt, arg);
	}

	va_end(arg);
}

#define SDMA_CONFIG_CSM_MASK                    (0b11)

static void sdma_set_context_switching_mode(sdma_csm_t csm)
{
	common.regs->CONFIG &= ~SDMA_CONFIG_CSM_MASK;
	common.regs->CONFIG |= csm;
}

static void sdma_enable_channel(uint8_t channel_id)
{
	common.channel[channel_id].active = 1;
	common.active_mask |= (1 << channel_id);
	common.regs->HSTART = (1 << channel_id);
}

static void sdma_disable_channel(uint8_t channel_id)
{
	common.regs->HSTART &= ~(1 << channel_id);
	common.active_mask &= ~(1 << channel_id);
	common.channel[channel_id].active = 0;
}

static int sdma_run_channel0_cmd(uint16_t count,
								 uint8_t command,
								 uint32_t buffer_addr,
								 uint32_t ext_buffer_addr)
{
	if (common.regs->STOP_STAT & 1)
		return -EBUSY;

	common.channel[0].bd->count = count;
	common.channel[0].bd->flags = SDMA_BD_DONE | SDMA_BD_WRAP;
	common.channel[0].bd->command = command;
	common.channel[0].bd->buffer_addr = buffer_addr;
	common.channel[0].bd->ext_buffer_addr = ext_buffer_addr;

	sdma_enable_channel(0);

	/* Wait until HE bit is cleared */
	unsigned tries = 100;
	while (common.regs->STOP_STAT & 1) {
		if (--tries == 0)
			return -ETIME;
		usleep(1000);
	}

	/* After channel 0 was used at least once, we can enable dynamic context
	 * switching */
	sdma_set_context_switching_mode(sdma_csm__dynamic);

	return EOK;
}

#define CHNPRIn_PRIORITY_MASK                   (0b111)

static void sdma_set_channel_priority(uint8_t channel_id, uint8_t priority)
{
	common.regs->SDMA_CHNPRI[channel_id] &= ~CHNPRIn_PRIORITY_MASK;
	common.regs->SDMA_CHNPRI[channel_id] |= priority & CHNPRIn_PRIORITY_MASK;
}

static int sdma_context_load(uint8_t channel_id, sdma_context_t *context)
{
	memcpy(common.tmp, context, sizeof(sdma_context_t));
	return sdma_run_channel0_cmd(sizeof(sdma_context_t)/4,
								 SDMA_CMD_C0_SETCTX(channel_id),
								 common.tmp_paddr,
								 0);
}

static int sdma_context_dump(uint8_t channel_id, sdma_context_t *context)
{
	int res;

	// Fill buffer with recognizable pattern for debugging purposes
	memset(common.tmp, 0xa5, sizeof(sdma_context_t));

	res = sdma_run_channel0_cmd(sizeof(sdma_context_t)/4,
								SDMA_CMD_C0_GETCTX(channel_id),
								common.tmp_paddr,
								0);
	if (res < 0)
		return res;

	memcpy(context, common.tmp, sizeof(sdma_context_t));

	return EOK;
}

#define OCRAM_BASE              (0x900000)
#define OCRAM_END               (0x920000)

static addr_t sdma_ocram_alloc(size_t size)
{
	unsigned n = (size + _PAGE_SIZE - 1)/_PAGE_SIZE;
	addr_t paddr = 0;

	if (common.ocram_next + n*_PAGE_SIZE <= OCRAM_END) {
		paddr = common.ocram_next;
		common.ocram_next += n*_PAGE_SIZE;
	}

	return paddr;
}

static void *sdma_alloc_uncached(size_t size, addr_t *paddr, int ocram)
{
	uint32_t n = (size + _PAGE_SIZE - 1)/_PAGE_SIZE;
	int flags = MAP_UNCACHED | MAP_ANONYMOUS;
	addr_t _paddr = 0;

	if (ocram) {
		flags |= MAP_PHYSMEM;
		_paddr = sdma_ocram_alloc(n*_PAGE_SIZE);
		if (!_paddr)
			return NULL;
	}

	void *vaddr = mmap(NULL, n * _PAGE_SIZE, PROT_READ | PROT_WRITE, flags, -1, _paddr);
	if (vaddr == MAP_FAILED)
		return NULL;

	if (!ocram)
		_paddr = va2pa(vaddr);

	if (paddr != NULL)
		*paddr = _paddr;

	return vaddr;
}

static int sdma_free_uncached(void *vaddr, size_t size)
{
	unsigned n = (size + _PAGE_SIZE - 1)/_PAGE_SIZE;

	return munmap(vaddr, n*_PAGE_SIZE);
}

static int __attribute__((unused)) sdma_program_memory_dump(uint16_t addr,
															addr_t buffer,
															size_t size)
{
	return sdma_run_channel0_cmd(size, SDMA_CMD_C0_GET_PM, buffer, addr);
}

static int __attribute__((unused)) sdma_program_memory_write(uint16_t addr,
															 addr_t buffer,
															 size_t size)
{
	return sdma_run_channel0_cmd(size, SDMA_CMD_C0_SET_PM, buffer, addr);
}

static int sdma_data_memory_dump(uint16_t addr,
								 addr_t buffer,
								 size_t size)
{
	return sdma_run_channel0_cmd(size, SDMA_CMD_C0_GET_DM, buffer, addr);
}

static int sdma_data_memory_write(uint16_t addr,
								  addr_t buffer,
								  size_t size)
{
	return sdma_run_channel0_cmd(size, SDMA_CMD_C0_SET_DM, buffer, addr);
}

static int sdma_intr(unsigned int intr, void *arg)
{
	uint32_t _INTR;
	struct driver_common_s *cmn = (struct driver_common_s*)arg;

	while ((_INTR = cmn->regs->INTR) != 0) {

		/* Clear interrupt flags */
		cmn->regs->INTR = _INTR;

		unsigned i;
		for (i = 1; i < NUM_OF_SDMA_CHANNELS; i++) {

			/* Check if channel is active and it's interrupt flag is set */
			if (_INTR & (1 << i) && cmn->channel[i].active) {

				/* Set BD_DONE in all buffer descriptors */
				sdma_buffer_desc_t *current = cmn->channel[i].bd;
				do {
					if (!(current->flags & SDMA_BD_DONE))
						current->flags |= SDMA_BD_DONE;
				} while (!((current++)->flags & SDMA_BD_WRAP));

				/* Increase interrupt count to notify dispatcher that interrupt for
				 * this channel occurred */
				cmn->channel[i].intr_cnt++;
			}
		}
	}

	return 0;
}

static int sdma_init_structs(void)
{
	unsigned i;

	for (i = 0; i < NUM_OF_SDMA_CHANNELS; i++)
		common.channel[i].active = 0;

	common.ccb = sdma_alloc_uncached(sizeof(sdma_channel_ctrl_t) * NUM_OF_SDMA_CHANNELS, &common.ccb_paddr, 1);
	if (common.ccb == NULL)
		goto fail;

	memset(common.ccb, 0, sizeof(sdma_channel_ctrl_t) * NUM_OF_SDMA_CHANNELS);

	common.channel[0].bd = sdma_alloc_uncached(sizeof(sdma_buffer_desc_t), &common.channel[0].bd_paddr, 0);
	if (common.channel[0].bd == NULL)
		goto fail;

	common.tmp_size = _PAGE_SIZE;
	common.tmp = sdma_alloc_uncached(common.tmp_size, &common.tmp_paddr, 0);
	if (common.tmp == NULL)
		goto fail;

	return 0;

fail:
	if (common.ccb != NULL) sdma_free_uncached(common.ccb, sizeof(sdma_channel_ctrl_t) * NUM_OF_SDMA_CHANNELS);
	if (common.channel[0].bd != NULL) sdma_free_uncached(common.channel[0].bd, sizeof(sdma_buffer_desc_t));
	if (common.tmp != NULL) sdma_free_uncached(common.tmp, _PAGE_SIZE);

	return -ENOMEM;
}

#define SDMA_RESET_BIT		(1 << 0)
#define SDMA_RESCHED_BIT	(1 << 1)

static int sdma_reset_core(void)
{
	unsigned tries = 100;
	while (common.regs->RESET & SDMA_RESET_BIT) {
		if (--tries == 0)
			return -ETIME;
		usleep(1000);
	}

	return 0;
}

static void sdma_init_core(void)
{
	unsigned i, status;

	/* Clear channel interrupt status */
	status = common.regs->INTR;
	common.regs->INTR = status;

	/* Clear channel stop status */
	status = common.regs->STOP_STAT;
	common.regs->STOP_STAT = status;

	common.regs->EVTOVR = 0;
	common.regs->HOSTOVR = 0;
	common.regs->DSPOVR = 0xffffffff;

	/* Clear channel pending status */
	common.regs->EVTPEND = common.regs->EVTPEND;

	common.regs->INTRMASK = 0;

	/* Initialize DMA request-channels matrix with zeros */
	for (i = 0; i < NUM_OF_SDMA_REQUESTS; i++)
		common.regs->CHNENBL[i] = 0;

	/* Set the priority of each channel to zero */
	for (i = 0; i < NUM_OF_SDMA_CHANNELS; i++)
		common.regs->SDMA_CHNPRI[i] = 0;

	/* Static context switching */
	common.regs->CONFIG = 0;

	/* Set physical address of channel control blocks */
	common.regs->MC0PTR = common.ccb_paddr;

	/* Set context size to 32 bytes (set SMSZ bit) */
	common.regs->CHN0ADDR |= 1 << 14;
}

static void sdma_init_channel0(void)
{
	common.ccb[0].base_bd = common.channel[0].bd_paddr;
	common.ccb[0].current_bd = common.channel[0].bd_paddr;

	/* Ignore DMA requests for channel 0 (channel 0 will be triggered by setting HE[0] bit) */
	common.regs->HOSTOVR = 0;
	common.regs->EVTOVR = 1;

	sdma_set_channel_priority(0, SDMA_CHANNEL_PRIORITY_MAX);
}

static int sdma_init(void)
{
	int res;
	unsigned handle;

	const addr_t sdma_paddr = 0x20ec000;
	common.regs = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, sdma_paddr);
	if (common.regs == MAP_FAILED) {
		log_error("sdma_init: mmap failed");
		return -errno;
	}

	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.dev = pctl_clk_sdma;
	pctl.devclock.state = 0b11;
	platformctl(&pctl);

	if ((res = sdma_init_structs()) < 0)
		return res;

	if ((res = sdma_reset_core()) < 0)
		return res;

	sdma_init_core();

	sdma_init_channel0();

	interrupt(32 + 2, sdma_intr, &common, common.intr_cond, &handle);

	return 0;
}

static int sdma_set_bd_array(uint8_t channel_id, addr_t paddr, unsigned cnt)
{
	sdma_buffer_desc_t *bd;

	size_t size = cnt * sizeof(sdma_buffer_desc_t);
	unsigned n = (size + _PAGE_SIZE - 1)/_PAGE_SIZE;
	bd = mmap(NULL, n * _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, paddr);
	if (bd == MAP_FAILED) {
		log_error("sdma_set_bd_array: mmap failed");
		return -errno;
	}

	common.channel[channel_id].bd = bd;
	common.channel[channel_id].bd_paddr = paddr;

	common.ccb[channel_id].base_bd = paddr;
	common.ccb[channel_id].current_bd = paddr;

	/* TODO: mmap buffers */

	/* TODO: Error handlig (unmap what was already mapped) */

	return 0;
}

static int sdma_channel_configure(uint8_t channel_id, sdma_channel_config_t *cfg)
{
	int res;

	if (cfg->priority >= SDMA_CHANNEL_PRIORITY_MAX) {
		log_error("unsupported channel priority");
		return -1;
	}

	common.regs->DSPOVR |= 1 << channel_id;

	if (cfg->trig == sdma_trig__event) {
		if (cfg->event >= NUM_OF_SDMA_REQUESTS) {
			log_error("event number is too high (%d)", cfg->event);
			return -1;
		}

		common.regs->CHNENBL[cfg->event] |= 1 << channel_id;

		common.regs->EVTOVR &= ~(1 << channel_id);
		common.regs->HOSTOVR |= 1 << channel_id;

	} else {
		common.regs->EVTOVR |= 1 << channel_id;
		common.regs->HOSTOVR &= ~(1 << channel_id);
	}

	sdma_set_channel_priority(channel_id, cfg->priority);

	if ((res = sdma_set_bd_array(channel_id, cfg->bd_paddr, cfg->bd_cnt)) < 0) {
		log_error("failed to set buffer descriptor array (%d)", res);
		return -1;
	}

	return 0;
}

static int dev_init(void)
{
	int i, res;
	oid_t dev;
	char filename[10];

	res = portCreate(&common.port);
	if (res != EOK) {
		log_error("could not create port: %d", res);
		return -1;
	}

	/* Start from channel 1. Channel 0 is used for loading/dumping context,
	 * scripts etc. */
	for (i = 1; i < NUM_OF_SDMA_CHANNELS; i++) {

		res = snprintf(filename, sizeof(filename), "sdma/ch%02u", (unsigned)i);

		dev.port = common.port;
		dev.id = i;

		if ((res = create_dev(&dev, filename)) != EOK) {
			log_error("could not create %s (res=%d)", filename, res);
			return -1;
		}

		common.channel[i].file_id = dev.id;
	}

	log_info("initialized");

	return 0;
}

static int dev_open(oid_t *oid, int flags)
{
	return EOK;
}

static int dev_close(oid_t *oid, int flags)
{
	/* TODO: unmap buffer descriptor array and buffers */

	return EOK;
}

static int oid_to_channel(oid_t *oid)
{
	return oid->id;
}

static int dev_read(oid_t *oid, void *data, size_t size)
{
	int channel = oid_to_channel(oid);
	unsigned intr_cnt;
	int res;

	mutexLock(common.lock);

	res = condWait(common.channel[channel].intr_cond, common.lock, INTR_CHANNEL_TIMEOUT_US);
	if (res == -ETIME) {
		mutexUnlock(common.lock);
		log_error("dev_read: timeout");
		return -EIO;
	}

	intr_cnt = common.channel[channel].intr_cnt;

	mutexUnlock(common.lock);

	common.channel[channel].read_cnt++;

	if (data != NULL && size == sizeof(unsigned)) {
		memcpy(data, &intr_cnt, sizeof(unsigned));
	} else if (data != NULL) {
		log_error("dev_read: invalid size");
		return -EIO;
	}

	return EOK;
}

static int dev_ctl(msg_t *msg)
{
	int channel, res;
	sdma_dev_ctl_t dev_ctl;
	sdma_context_t *context;

	memcpy(&dev_ctl, msg->o.raw, sizeof(sdma_dev_ctl_t));

	if ((channel = oid_to_channel(&dev_ctl.oid)) < 0) {
		log_error("dev_ctl: failed to get channel corresponding to this oid");
		return -EIO;
	}

	switch (dev_ctl.type) {
		case sdma_dev_ctl__channel_cfg:
			if ((res = sdma_channel_configure(channel, &dev_ctl.cfg)) < 0) {
				log_error("dev_ctl: an error occurred while configuring channel %d (%d)", channel, res);
				return -EIO;
			}
			return EOK;

		case sdma_dev_ctl__data_mem_write:
			if (msg->o.size != dev_ctl.mem.len || msg->o.size > common.tmp_size) {
				log_error("dev_ctl: invalid size");
				return -EIO;
			}
			memcpy(common.tmp, msg->o.data, msg->o.size);
			return sdma_data_memory_write(dev_ctl.mem.addr, common.tmp_paddr, dev_ctl.mem.len);

		case sdma_dev_ctl__data_mem_read:
			if (msg->o.size != dev_ctl.mem.len || msg->o.size > common.tmp_size) {
				log_error("dev_ctl: invalid size");
				return -EIO;
			}
			res = sdma_data_memory_dump(dev_ctl.mem.addr, common.tmp_paddr, dev_ctl.mem.len);
			memcpy(msg->o.data, common.tmp, msg->o.size);
			return res;

		case sdma_dev_ctl__context_dump:
			if (msg->o.size != sizeof(sdma_context_t)) {
				log_error("dev_ctl: can't dump context of channel %d (invalid size)", channel);
				return -EIO;
			}
			context = (sdma_context_t*)msg->o.data;
			return sdma_context_dump(channel, context);

		case sdma_dev_ctl__context_set:
			if (msg->o.size != sizeof(sdma_context_t)) {
				log_error("dev_ctl: can't set context for channel %d (invalid size)", channel);
				return -EIO;
			}
			context = (sdma_context_t*)msg->o.data;
			return sdma_context_load(channel, context);

		case sdma_dev_ctl__enable:
			sdma_enable_channel(channel);
			return EOK;

		case sdma_dev_ctl__disable:
			sdma_disable_channel(channel);
			return EOK;

		case sdma_dev_ctl__ocram_alloc:
			dev_ctl.alloc.paddr = sdma_ocram_alloc(dev_ctl.alloc.size);
			memcpy(msg->o.raw, &dev_ctl, sizeof(sdma_dev_ctl_t));
			return EOK;

		default:
			log_error("dev_ctl: unknown type (%d)", dev_ctl.type);
			return -ENOSYS;
	}

	return EOK;
}

static void worker_thread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;

	(void)arg;

	while (1) {
		if (msgRecv(common.port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
			case mtOpen:
				msg.o.io.err = dev_open(&msg.i.openclose.oid, msg.i.openclose.flags);
				break;

			case mtClose:
				msg.o.io.err = dev_close(&msg.i.openclose.oid, msg.i.openclose.flags);
				break;

			case mtRead:
				msg.o.io.err = dev_read(&msg.i.io.oid, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.io.err = -ENOSYS;
				break;

			case mtDevCtl:
				mutexLock(common.lock);
				msg.o.io.err = dev_ctl(&msg);
				mutexUnlock(common.lock);
				break;
		}

		msgRespond(common.port, &msg, rid);
	}
}

static void stats_thread(void *arg)
{
	int i;
	unsigned intr_cnt, read_cnt, missed_cnt;

	while (1) {

		sleep(common.stats_period_s);

		/* Skip channel 0, since it's only used for configuration */
		for (i = 1; i < NUM_OF_SDMA_CHANNELS; i++) {
			if (!common.channel[i].active)
				continue;

			intr_cnt = common.channel[i].intr_cnt;
			missed_cnt = common.channel[i].missed_intr_cnt;
			read_cnt = common.channel[i].read_cnt;

			log_info("ch#%u stats: %u interrupts; %u missed; %u reads", i, intr_cnt, missed_cnt, read_cnt);
		}
	}
}

static int init(void)
{
	int res, i;

	common.ocram_next = OCRAM_BASE;

	if (common.use_syslog)
		openlog("sdma-driver", LOG_NDELAY, LOG_DAEMON);

	if (mutexCreate(&common.lock) != EOK) {
		log_error("failed to create mutex");
		return -1;
	}

	if (condCreate(&common.intr_cond) != EOK) {
		log_error("failed to create conditional variable");
		return -1;
	}

	for (i = 0; i < NUM_OF_SDMA_CHANNELS; i++) {
		common.channel[i].intr_cnt = 0;
		common.channel[i].read_cnt = 0;
		common.channel[i].missed_intr_cnt = 0;
		if (condCreate(&common.channel[i].intr_cond) != EOK) {
			log_error("failed to create conditional variable for channel %d", i);
			return -1;
		}
	}

	if ((res = sdma_init()) < 0) {
		log_error("SDMA initialization failed (%d [%s])", res, strerror(res));
		return res;
	}

	if ((res = dev_init()) < 0) {
		log_error("device initialization failed (%d)", res);
		return res;
	}

	for (i = 0; i < NUM_OF_WORKER_THREADS; i++)
		beginthread(worker_thread, WORKER_THD_PRIO, common.worker_thd_stack[i], WORKER_THD_STACK, NULL);

	if (common.stats_period_s > 0) {
		beginthread(stats_thread, STATS_THD_PRIO, common.stats_thd_stack, STATS_THD_STACK, NULL);
	}

	common.initialized = 1;

	return 0;
}

static FILE *create_dump_file(void)
{
	char path[256];

	snprintf(path, sizeof(path), "%s/sdma_dump", common.dump_dir);

	unsigned i = 0;
	while (access(path, F_OK) == 0) {
		snprintf(path, sizeof(path), "%s/sdma_dump.%u", common.dump_dir, i);
		if (++i > 16)
			return NULL;
	}

	log_warn("Creating debug info dump (%s)", path);

	return fopen(path, "w");
}

static int create_flag_file(const char *path)
{
	int res;

	res = open(path, O_WRONLY | O_CREAT | O_TRUNC);
	if (res < 0) {
		log_error("create_flag_file: open failed (res=%d, errno=%s)", res, strerror(errno));
		return -1;
	}

	return close(res);
}

static void dump_debug_info(void)
{
	int res;
	FILE *f;

	f = create_dump_file();
	if (f == NULL) {
		log_error("Failed to create file for debug info dump");
		return;
	}

	fprintf(f, "SDMA regs:\n\n");
	fprintf(f, "EVTPEND      = 0x%x\n", common.regs->EVTPEND);
	fprintf(f, "EVT_MIRROR   = 0x%x\n", common.regs->EVT_MIRROR);
	fprintf(f, "EVT_MIRROR2  = 0x%x\n", common.regs->EVT_MIRROR2);
	fprintf(f, "EVTERRDBG    = 0x%x\n", common.regs->EVTERRDBG);
	fprintf(f, "INTR         = 0x%x\n", common.regs->INTR);
	fprintf(f, "INTRMASK     = 0x%x\n", common.regs->INTRMASK);
	fprintf(f, "PSW          = 0x%x\n", common.regs->PSW);
	fprintf(f, "ONCE_STAT    = 0x%x\n", common.regs->ONCE_STAT);
	fprintf(f, "HSTART       = 0x%x\n", common.regs->HSTART);
	fprintf(f, "STOP_STAT    = 0x%x\n\n", common.regs->STOP_STAT);

	/* Disable all channels */
	fprintf(f, "Disabling all channels and forcing reschedule\n\n");
	common.regs->STOP_STAT = common.regs->STOP_STAT;
	common.regs->RESET |= SDMA_RESCHED_BIT;

	unsigned i;
	for (i = 1; i < NUM_OF_SDMA_CHANNELS; i++) {
		if (!common.channel[i].active)
			continue;

		fprintf(f, "Channel %u is active\n", i);

		fprintf(f, "intr_cnt   = %u\n", common.channel[i].intr_cnt);
		fprintf(f, "missed_cnt = %u\n", common.channel[i].missed_intr_cnt);
		fprintf(f, "priority   = %u\n\n", common.regs->SDMA_CHNPRI[i]);

		int context_dumped = 1;
		sdma_context_t context;

		fprintf(f, "Dumping context...\n");
		res = sdma_context_dump(i, &context);
		if (res != EOK) {

			fprintf(f, "Failed to dump channel context (%d [%s])\n", res, strerror(-res));
			fprintf(f, "Reinitializing SDMA core...\n");

			res = sdma_reset_core();
			if (res != EOK) {
				fprintf(f, "Initialization failed (%d)\n", res);
			}

			sdma_init_core();

			sdma_init_channel0();

			fprintf(f, "Retrying to dump context...\n");
			res = sdma_context_dump(i, &context);
			if (res != EOK) {
				context_dumped = 0;
				fprintf(f, "Failed to dump channel context (%d [%s])\n", res, strerror(-res));
			}
		}

		if (context_dumped) {
			unsigned pc = context.state[0] & SDMA_CONTEXT_PC_MASK;
			fprintf(f, "pc           = 0x%x (%u)\n", pc, pc);
			fprintf(f, "state[0]     = 0x%x\n", context.state[0]);
			fprintf(f, "state[1]     = 0x%x\n", context.state[1]);
			fprintf(f, "gr[0]        = 0x%x\n", context.gr[0]);
			fprintf(f, "gr[1]        = 0x%x\n", context.gr[1]);
			fprintf(f, "gr[2]        = 0x%x\n", context.gr[2]);
			fprintf(f, "gr[3]        = 0x%x\n", context.gr[3]);
			fprintf(f, "gr[4]        = 0x%x\n", context.gr[4]);
			fprintf(f, "gr[5]        = 0x%x\n", context.gr[5]);
			fprintf(f, "gr[6]        = 0x%x\n", context.gr[6]);
			fprintf(f, "gr[7]        = 0x%x\n", context.gr[7]);
			fprintf(f, "mda          = 0x%x\n", context.mda);
			fprintf(f, "msa          = 0x%x\n", context.msa);
			fprintf(f, "ms           = 0x%x\n", context.ms);
			fprintf(f, "md           = 0x%x\n", context.md);
			fprintf(f, "pda          = 0x%x\n", context.pda);
			fprintf(f, "psa          = 0x%x\n", context.psa);
			fprintf(f, "ps           = 0x%x\n", context.ps);
			fprintf(f, "pd           = 0x%x\n", context.pd);
			fprintf(f, "ca           = 0x%x\n", context.ca);
			fprintf(f, "cs           = 0x%x\n", context.cs);
			fprintf(f, "dda          = 0x%x\n", context.dda);
			fprintf(f, "dsa          = 0x%x\n", context.dsa);
			fprintf(f, "ds           = 0x%x\n", context.ds);
			fprintf(f, "dd           = 0x%x\n", context.dd);
			fprintf(f, "scratch[0]   = 0x%x\n", context.scratch[0]);
			fprintf(f, "scratch[1]   = 0x%x\n", context.scratch[1]);
			fprintf(f, "scratch[2]   = 0x%x\n", context.scratch[2]);
			fprintf(f, "scratch[3]   = 0x%x\n", context.scratch[3]);
			fprintf(f, "scratch[4]   = 0x%x\n", context.scratch[4]);
			fprintf(f, "scratch[5]   = 0x%x\n", context.scratch[5]);
			fprintf(f, "scratch[6]   = 0x%x\n", context.scratch[6]);
			fprintf(f, "scratch[7]   = 0x%x\n\n", context.scratch[7]);
		}

		unsigned j = 0;
		sdma_buffer_desc_t *current = common.channel[i].bd;
		if (current != NULL) {
			do {
				fprintf(f, "bd[%u].flags = 0x%x\n", j, current->flags);
				fprintf(f, "\tSDMA_BD_DONE is %s\n", (current->flags & SDMA_BD_DONE) ? "SET" : "CLEARED");
				fprintf(f, "\tSDMA_BD_WRAP is %s\n", (current->flags & SDMA_BD_WRAP) ? "SET" : "CLEARED");
				fprintf(f, "\tSDMA_BD_INTR is %s\n\n", (current->flags & SDMA_BD_INTR) ? "SET" : "CLEARED");
				j++;
			} while (!((current++)->flags & SDMA_BD_WRAP));
		}

		fprintf(f, "Dumping CCB...\n");
		fprintf(f, "common.ccb[i].base_bd         = 0x%x\n", common.ccb[i].base_bd);
		fprintf(f, "common.ccb[i].current_bd      = 0x%x\n\n", common.ccb[i].current_bd);
	}

	fclose(f);
}

#define INTR_WAIT_TIMEOUT_US	(30*1000*1000)
#define SDMA_BROKEN_FILE		"/var/run/sdma_broken"

int main(int argc, char *argv[])
{
	int res, display_usage = 0;
	oid_t root;

	priority(MAIN_THD_PRIO);

	common.use_syslog = 0;
	common.stats_period_s = 0; /* Don't print stats by default */
	common.initialized = 0;
	common.active_mask = 0;
	common.dump_dir = "/var/run";
	common.broken = 0;

	while ((res = getopt(argc, argv, "S:sd:")) >= 0) {
		switch (res) {
		case 'S':
			common.stats_period_s = (int)strtol(optarg, NULL, 0);
			break;
		case 's':
			common.use_syslog = 1;
			break;
		case 'd':
			common.dump_dir = optarg;
			break;
		default:
			display_usage = 1;
			break;
		}
	}

	if (display_usage) {
		printf("Usage: sdma-driver [-s] [-S period] [-d path]\n\r");
		printf("    -S period    Print stats with given period (in seconds)\n\r");
		printf("    -s           Output logs to syslog instead of stdout\n\r");
		printf("    -d path      Set directory for debug info dump (default: %s)\n\r", common.dump_dir);
		return 1;
	}

	/* Wait for the filesystem */
	while (lookup("/", NULL, &root) < 0)
		usleep(10000);

	if (init())
		return -EIO;

	unsigned i, intr_cnt[NUM_OF_SDMA_CHANNELS], cnt;
	memset(intr_cnt, 0, sizeof(intr_cnt));

	while (1) {
		mutexLock(common.lock);
		res = condWait(common.intr_cond, common.lock, INTR_WAIT_TIMEOUT_US);

		if (res == -ETIME) {

			/* If any channel is active (except channel 0) dump debug info.
			   The channel nr 0 is used to configure SDMA, we don't expect interrupt event. */
			if ((common.active_mask & ~0x1) && !common.broken) {

				create_flag_file(SDMA_BROKEN_FILE);

				log_warn("Timed out waiting for interrupts");
				dump_debug_info();

				common.broken = 1;
			}

			mutexUnlock(common.lock);
			continue;
		}

		for (i = 0; i < NUM_OF_SDMA_CHANNELS; i++) {
			cnt = common.channel[i].intr_cnt;

			if (intr_cnt[i] == cnt) /* No interrupts for this channel */
				continue;

			if ((intr_cnt[i] + 1) != cnt) { /* More than one interrupt */
				common.channel[i].missed_intr_cnt += cnt - intr_cnt[i] - 1;
#if 0
				/* Enable only for debugging purposes. Printing here makes us miss even more interrupts. */
				log_warn("missed interrupt for channel %d (%u vs %u)", i, intr_cnt[i], cnt);
#endif
			}

			condSignal(common.channel[i].intr_cond);
			intr_cnt[i] = cnt;
		}

		mutexUnlock(common.lock);
	}

	/* Should never be reached */
	log_error("Exiting!");
	return 0;
}
