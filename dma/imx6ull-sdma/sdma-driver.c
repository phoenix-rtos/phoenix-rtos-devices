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

#include <sys/stat.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/file.h>

#include <phoenix/arch/imx6ull.h>

#include "sdma-api.h"

#define COL_RED     "\033[1;31m"
#define COL_CYAN    "\033[1;36m"
#define COL_YELLOW  "\033[1;33m"
#define COL_NORMAL  "\033[0m"

#define LOG_TAG "sdma-drv: "
#define log_debug(fmt, ...)     do { printf(LOG_TAG fmt "\n", ##__VA_ARGS__); } while (0)
#define log_info(fmt, ...)      do { printf(COL_CYAN LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
#define log_warn(fmt, ...)      do { printf(COL_YELLOW LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)
#define log_error(fmt, ...)     do { printf(COL_RED  LOG_TAG fmt "\n" COL_NORMAL, ##__VA_ARGS__); } while (0)

#define NUM_OF_SDMA_CHANNELS    (32)
#define NUM_OF_SDMA_REQUESTS    (48)

#define NUM_OF_WORKER_THREADS   (NUM_OF_SDMA_CHANNELS)
#define WORKER_THD_PRIO         (2)
#define WORKER_THD_STACK        (4096)

/* Buffer Descriptor Commands for Bootload scripts */
#define SDMA_CMD_C0_SET_DM                      (0x1)
#define SDMA_CMD_C0_GET_DM                      (0x2)
#define SDMA_CMD_C0_SET_PM                      (0x4)
#define SDMA_CMD_C0_GET_PM                      (0x6)
#define SDMA_CMD_C0_SETCTX(channel)             (0x07 | (channel << 3))
#define SDMA_CMD_C0_GETCTX(channel)             (0x03 | (channel << 3))

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

typedef struct {
    int active;
    int auto_bd_done;

    sdma_buffer_desc_t *bd;
    addr_t bd_paddr;

    id_t file_id;

    handle_t intr_cond;
    unsigned intr_cnt;
} sdma_channel_t;

struct driver_common_s
{
    char stack[NUM_OF_WORKER_THREADS][WORKER_THD_STACK] __attribute__ ((aligned(8)));

    uint32_t port;

    sdma_channel_t channel[NUM_OF_SDMA_CHANNELS];
    sdma_channel_ctrl_t *ccb; /* Pointer to channel control block array */

    /* Temporary buffer (uncached, with known physical address) for
     * loading/dumping contexts, scripts etc. */
    size_t tmp_size;
    void *tmp;
    addr_t tmp_paddr;

    volatile sdma_arm_regs_t *regs;

    handle_t intr_cond;
    handle_t lock;
} common;

static void sdma_enable_channel(u8 channel_id)
{
    common.channel[channel_id].active = 1;
    common.regs->HSTART = (1 << channel_id);
}

static void __attribute__((unused)) sdma_disable_channel(u8 channel_id)
{
    common.regs->HSTART &= ~(1 << channel_id);
    common.channel[channel_id].active = 0;
}

static void sdma_run_channel0_cmd(uint16_t count,
                                  uint8_t command,
                                  uint32_t buffer_addr,
                                  uint32_t ext_buffer_addr)
{
    common.channel[0].bd->count = count;
    common.channel[0].bd->flags = SDMA_BD_DONE | SDMA_BD_WRAP;
    common.channel[0].bd->command = command;
    common.channel[0].bd->buffer_addr = buffer_addr;
    common.channel[0].bd->ext_buffer_addr = ext_buffer_addr;

    sdma_enable_channel(0);

    /* Wait until HE bit is cleared */
    while (common.regs->STOP_STAT & 1);
}

#define CHNPRIn_PRIORITY_MASK                   (0b111)

static void sdma_set_channel_priority(uint8_t channel_id, uint8_t priority)
{
    common.regs->SDMA_CHNPRI[channel_id] &= ~CHNPRIn_PRIORITY_MASK;
    common.regs->SDMA_CHNPRI[channel_id] |= priority & CHNPRIn_PRIORITY_MASK;
}

static void sdma_context_load(uint8_t channel_id, sdma_context_t *context)
{
    memcpy(common.tmp, context, sizeof(sdma_context_t));
    sdma_run_channel0_cmd(sizeof(sdma_context_t)/4,
                          SDMA_CMD_C0_SETCTX(channel_id),
                          common.tmp_paddr,
                          0);
}

static void sdma_context_dump(uint8_t channel_id, sdma_context_t *context)
{
    sdma_run_channel0_cmd(sizeof(sdma_context_t)/4,
                          SDMA_CMD_C0_GETCTX(channel_id),
                          common.tmp_paddr,
                          0);
    memcpy(context, common.tmp, sizeof(sdma_context_t));
}

static void *sdma_alloc_unchaced(size_t size, addr_t *paddr)
{
    uint32_t n = (size + SIZE_PAGE - 1)/SIZE_PAGE;

    void *vaddr = mmap(NULL, n*SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);
    if (vaddr == MAP_FAILED)
        return NULL;

    if (paddr) {
        addr_t page_addr = va2pa(vaddr - (addr_t)vaddr % SIZE_PAGE);
        *paddr = page_addr + (addr_t)vaddr % SIZE_PAGE;
    }

    return vaddr;
}

static int sdma_free_uncached(void *vaddr, size_t size)
{
    unsigned n = (size + SIZE_PAGE - 1)/SIZE_PAGE;

    return munmap(vaddr, n*SIZE_PAGE);
}

static void __attribute__((unused)) sdma_program_memory_dump(uint16_t addr,
                                                             addr_t buffer,
                                                             size_t size)
{
    sdma_run_channel0_cmd(size, SDMA_CMD_C0_GET_PM, buffer, addr);
}

static void __attribute__((unused)) sdma_program_memory_write(uint16_t addr,
                                                              addr_t buffer,
                                                              size_t size)
{
    sdma_run_channel0_cmd(size, SDMA_CMD_C0_SET_PM, buffer, addr);
}

static void sdma_data_memory_dump(uint16_t addr,
                                  addr_t buffer,
                                  size_t size)
{
    sdma_run_channel0_cmd(size, SDMA_CMD_C0_GET_DM, buffer, addr);
}

static void sdma_data_memory_write(uint16_t addr,
                                   addr_t buffer,
                                   size_t size)
{
    sdma_run_channel0_cmd(size, SDMA_CMD_C0_SET_DM, buffer, addr);
}

static void sdma_reset(void)
{
    uint32_t i, status;

    common.regs->MC0PTR = 0;

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
}

static int sdma_intr(unsigned int intr, void *arg)
{
    struct driver_common_s *cmn = (struct driver_common_s*)arg;

    uint32_t _INTR = cmn->regs->INTR;
    cmn->regs->INTR = _INTR;

    unsigned i;
    for (i = 0; i < NUM_OF_SDMA_CHANNELS; i++) {

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

    return 0;
}

static int sdma_init(void)
{
    common.ccb = NULL;
    common.channel[0].bd = NULL;
    common.tmp = NULL;

    const addr_t sdma_paddr = 0x20ec000;
    common.regs = mmap(NULL, SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_DEVICE, OID_PHYSMEM, sdma_paddr);
    if (common.regs == MAP_FAILED)
        goto fail;

    platformctl_t pctl;
    pctl.action = pctl_set;
    pctl.type = pctl_devclock;
    pctl.devclock.dev = pctl_clk_sdma;
    pctl.devclock.state = 0b11;
    platformctl(&pctl);

    sdma_reset();

    /* Static context switching */
    common.regs->CONFIG = 0;

    /* Set context size to 32 bytes (set SMSZ bit) */
    common.regs->CHN0ADDR |= 1 << 14;

    addr_t ccb_paddr;
    common.ccb = sdma_alloc_unchaced(sizeof(sdma_channel_ctrl_t) * NUM_OF_SDMA_CHANNELS, &ccb_paddr);
    if (common.ccb == NULL)
        goto fail;

    memset(common.ccb, 0, sizeof(sdma_channel_ctrl_t) * NUM_OF_SDMA_CHANNELS);

    common.channel[0].bd = sdma_alloc_unchaced(sizeof(sdma_buffer_desc_t), &common.channel[0].bd_paddr);
    if (common.channel[0].bd == NULL)
        goto fail;

    common.ccb[0].base_bd = common.channel[0].bd_paddr;
    common.ccb[0].current_bd = common.channel[0].bd_paddr;

    common.regs->MC0PTR = ccb_paddr;

    /* Ignore DMA requests for channel 0 (channel 0 will be triggered by setting HE[0] bit) */
    common.regs->HOSTOVR = 0;
    common.regs->EVTOVR = 1;

    sdma_set_channel_priority(0, SDMA_CHANNEL_PRIORITY_MAX);

    common.tmp_size = SIZE_PAGE;
    common.tmp = sdma_alloc_unchaced(common.tmp_size, &common.tmp_paddr);
    if (common.tmp == NULL)
        goto fail;

    unsigned i;
    for (i = 0; i < NUM_OF_SDMA_CHANNELS; i++)
        common.channel[i].active = 0;

    unsigned handle;
    interrupt(32 + 2, sdma_intr, &common, common.intr_cond, &handle);

    return 0;

fail:

    if (common.ccb != NULL) sdma_free_uncached(common.ccb, sizeof(sdma_channel_ctrl_t) * NUM_OF_SDMA_CHANNELS);
    if (common.channel[0].bd != NULL) sdma_free_uncached(common.channel[0].bd, sizeof(sdma_buffer_desc_t));
    if (common.tmp != NULL) sdma_free_uncached(common.tmp, SIZE_PAGE);

    return -1;
}

static int sdma_set_bd_array(uint8_t channel_id, addr_t paddr, unsigned cnt)
{
    sdma_buffer_desc_t *bd;

    size_t size = cnt * sizeof(sdma_buffer_desc_t);
    unsigned n = (size + SIZE_PAGE - 1)/SIZE_PAGE;
    bd = mmap(NULL, n*SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_DEVICE, OID_PHYSMEM, paddr);
    if (bd == MAP_FAILED)
        return -1;

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

static int dev_init(oid_t root)
{
    int i, res;
    oid_t dir;
    msg_t msg;
    const char *dirname = "/dev/sdma";
    char filename[5];

    res = portCreate(&common.port);
    if (res != EOK) {
        log_error("could not create port: %d", res);
        return -1;
    }

    res = mkdir("/dev", 0);
    if (res < 0 && res != -EEXIST) {
        log_error("mkdir /dev failed (%d)", res);
        return -1;
    }

    res = mkdir(dirname, 0);
    if (res < 0 && res != -EEXIST) {
        log_error("mkdir %s failed (%d)", dirname, res);
        return -1;
    }

    if ((res = lookup(dirname, &dir)) < 0) {
        log_error("%s lookup failed (%d)", dirname, res);
        return -1;
    }

    /* Start from channel 1. Channel 0 is used for loading/dumping context,
     * scripts etc. */
    for (i = 1; i < NUM_OF_SDMA_CHANNELS; i++) {

        res = snprintf(filename, sizeof(filename), "ch%02u", (unsigned)i);

        msg.type = mtCreate;
        msg.i.create.type = otDev;
        msg.i.create.mode = 0;
        msg.i.create.dev.port = common.port;
        msg.i.create.dev.id = i;
        msg.i.create.dir = dir;
        msg.i.data = filename;
        msg.i.size = strlen(filename) + 1;

        if ((res = msgSend(root.port, &msg)) < 0 || msg.o.create.err != EOK) {
            log_error("could not create %s/%s (res=%d, err=%d)", dirname, filename, res, msg.o.create.err);
            return -1;
        }

        common.channel[i].file_id = msg.i.create.dev.id;
    }

    log_info("device initialized");

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

    mutexLock(common.lock);
    condWait(common.channel[channel].intr_cond, common.lock, 0);

    intr_cnt = common.channel[channel].intr_cnt;

    mutexUnlock(common.lock);

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
        sdma_data_memory_write(dev_ctl.mem.addr, common.tmp_paddr, dev_ctl.mem.len);
        return EOK;

    case sdma_dev_ctl__data_mem_read:
        if (msg->o.size != dev_ctl.mem.len || msg->o.size > common.tmp_size) {
            log_error("dev_ctl: invalid size");
            return -EIO;
        }
        sdma_data_memory_dump(dev_ctl.mem.addr, common.tmp_paddr, dev_ctl.mem.len);
        memcpy(msg->o.data, common.tmp, msg->o.size);
        return EOK;

    case sdma_dev_ctl__context_dump:
        if (msg->o.size != sizeof(sdma_context_t)) {
            log_error("dev_ctl: can't dump context of channel %d (invalid size)", channel);
            return -EIO;
        }
        context = (sdma_context_t*)msg->o.data;
        sdma_context_dump(channel, context);
        return EOK;

    case sdma_dev_ctl__context_set:
        if (msg->o.size != sizeof(sdma_context_t)) {
            log_error("dev_ctl: can't set context for channel %d (invalid size)", channel);
            return -EIO;
        }
        context = (sdma_context_t*)msg->o.data;
        sdma_context_load(channel, context);
        return EOK;

    case sdma_dev_ctl__enable:
        sdma_enable_channel(channel);
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
    unsigned rid;

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

static int init(oid_t root)
{
    int res, i;

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
        if (condCreate(&common.channel[i].intr_cond) != EOK) {
            log_error("failed to create conditional variable for channel %d", i);
            return -1;
        }
    }

    if ((res = sdma_init()) < 0) {
        log_error("SDMA initialization failed");
        return res;
    }

    if ((res = dev_init(root)) < 0) {
        log_error("device initialization failed");
        return res;
    }

    for (i = 0; i < NUM_OF_WORKER_THREADS; i++)
        beginthread(worker_thread, WORKER_THD_PRIO, common.stack[i], WORKER_THD_STACK, NULL);

    return 0;
}

int main(void)
{
    oid_t root;

    /* Wait for the filesystem */
    while (lookup("/", &root) < 0)
        usleep(10000);

    if (init(root))
        return -EIO;

    unsigned i, intr_cnt[NUM_OF_SDMA_CHANNELS], cnt;
    memset(intr_cnt, 0, sizeof(intr_cnt));

    while (1) {
        mutexLock(common.lock);
        condWait(common.intr_cond, common.lock, 0);

        for (i = 0; i < NUM_OF_SDMA_CHANNELS; i++) {
            cnt = common.channel[i].intr_cnt;

            if (intr_cnt[i] == cnt) /* No interrupts for this channel */
                continue;

            if ((intr_cnt[i] + 1) != cnt) /* More than one interrupt */
                log_warn("missed interrupt for channel %d (%u vs %u)", i, intr_cnt[i], cnt);

            condSignal(common.channel[i].intr_cond);
            intr_cnt[i] = cnt;
        }

        mutexUnlock(common.lock);
    }

    /* Should never be reached */
    log_error("Exiting!");
    return 0;
}
