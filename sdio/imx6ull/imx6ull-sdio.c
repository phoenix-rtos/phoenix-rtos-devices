/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL SDIO driver
 *
 * Copyright 2022 Phoenix Systems
 * Author: Ziemowit Leszczynski, Artur Miller
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <sys/mman.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include <phoenix/arch/imx6ull.h>

#include <sdio.h>


/* SDIO standard commands */
#define SDIO_CMD_STATE_IDLE     0  /* set the card into idle state       */
#define SDIO_CMD_RELATIVE_ADDR  3  /* set card's relative address        */
#define SDIO_CMD_SEND_OP_COND   5  /* inquire about operating conditions */
#define SDIO_CMD_SELECT_CARD    7  /* select a card by address           */
#define SDIO_CMD_STATE_INACTIVE 15 /* set the card into inactive state   */
#define SDIO_CMD_RW_DIRECT      52 /* direct register I/O operation      */
#define SDIO_CMD_RW_EXTENDED    53 /* bulk register I/O operation        */

/* SDIO event status bits */
#define SDIO_STATUS_CMD_DONE       (1UL << 0)  /* command complete         */
#define SDIO_STATUS_RW_DONE        (1UL << 1)  /* transfer complete        */
#define SDIO_STATUS_BLOCK_GAP      (1UL << 2)  /* transfer was stopped     */
#define SDIO_STATUS_DMA_DONE       (1UL << 3)  /* DMA transfer success     */
#define SDIO_STATUS_RW_WRITE_READY (1UL << 4)  /* write buffer ready       */
#define SDIO_STATUS_RW_READ_READY  (1UL << 5)  /* read buffer ready        */
#define SDIO_STATUS_CARD_IN        (1UL << 6)  /* card inserted            */
#define SDIO_STATUS_CARD_OUT       (1UL << 7)  /* card removed             */
#define SDIO_STATUS_CARD_IRQ       (1UL << 8)  /* card requested interrupt */
#define SDIO_STATUS_TUNING_RETUNE  (1UL << 12) /* retuning required soon   */
#define SDIO_STATUS_CMD_TIMEOUT    (1UL << 16) /* command timed out        */
#define SDIO_STATUS_CMD_CRC        (1UL << 17) /* command CRC value error  */
#define SDIO_STATUS_CMD_ENDBIT     (1UL << 18) /* command end bit error    */
#define SDIO_STATUS_CMD_INDEX      (1UL << 19) /* command index error      */
#define SDIO_STATUS_DATA_TIMEOUT   (1UL << 20) /* data timed out           */
#define SDIO_STATUS_DATA_CRC       (1UL << 21) /* data CRC value error     */
#define SDIO_STATUS_DATA_ENDBIT    (1UL << 22) /* data end bit error       */
#define SDIO_STATUS_TUNING_ERROR   (1UL << 26) /* tuning error occurred    */
#define SDIO_STATUS_DMA_ERROR      (1UL << 28) /* DMA transfer failed      */

/* configuration values */
#define DMA_BUFFER_SIZE   2048
#define THREAD_STACK_SIZE 1024
#define SDHC_RETRIES      10

/* hardware platform specific */
#define USDHC2_ADDR             0x2194000
#define USDHC2_IRQ              (32 + 23)
#define SDHC_SYSCTL_RESET_ALL   (1UL << 24)
#define SDHC_SYSCTL_RESET_CMD   (1UL << 25)
#define SDHC_SYSCTL_RESET_DATA  (1UL << 26)
#define SDHC_SYSCTL_CLK_DIV_8   (0x04 << 8)
#define SDHC_SYSCTL_CLK_DIV_4   (0x02 << 8)
#define SDHC_SYSCTL_CLK_DEFAULT 0x8010
#define SDHC_SYSCTL_RESERVED    0xf
#define SDHC_INTERRUPT_DEFAULTS 0x107f000f
#define SDHC_CMD_ERROR          0x107f0000

#define ARRAY_LENGTH(array) (sizeof(array) / sizeof(array[0]))


static const uint32_t sdioEvents[] = { SDIO_STATUS_CARD_IN, SDIO_STATUS_CARD_OUT, SDIO_STATUS_CARD_IRQ };

/* clang-format off */
enum { reg_ds_addr = 0, reg_blk_att, reg_cmd_arg, reg_cmd_xfer_typ, reg_cmd_rsp0, reg_cmd_rsp1,
	reg_cmd_rsp2, reg_cmd_rsp3, reg_data_buff_acc_port, reg_pres_state, reg_prot_ctrl, reg_sys_ctrl,
	reg_int_status, reg_int_status_en, reg_int_signal_en, reg_autocmd12_err_status, reg_host_ctrl_cap,
	reg_wtmk_lvl, reg_mix_ctrl, reg_force_event = 20, reg_adma_err_status, reg_adma_sys_addr,
	reg_dll_ctrl = 24, reg_dll_status, reg_clk_tune_ctrl_status, reg_vend_spec = 48, reg_mmc_boot,
	reg_vend_spec2, reg_tuning_ctrl };
/* clang-format on */

static struct {
	volatile uint32_t *base;
	handle_t cmdLock;
	uint16_t blocksz;

	/* DMA */
	void *dmaptr;
	addr_t dmaphys;

	/* API flags */
	int sdioInitialized;
	int eventThreadStarted;

	/* event service */
	uint32_t enabledEvents;
	handle_t isrHandle;
	sdio_event_handler_t eventHandlers[ARRAY_LENGTH(sdioEvents)];
	void *eventHandlerArgs[ARRAY_LENGTH(sdioEvents)];
	uint8_t stack[THREAD_STACK_SIZE] __attribute__((aligned(8)));
	handle_t eventLock;
	handle_t eventCond;
} sdio_common;


static int platform_setMux(int mux, char mode)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_iomux;
	ctl.iomux.mux = mux;
	ctl.iomux.sion = 0;
	ctl.iomux.mode = mode;

	return platformctl(&ctl);
}


static int platform_setSel(int isel, char daisy)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_ioisel;
	ctl.ioisel.isel = isel;
	ctl.ioisel.daisy = daisy;

	return platformctl(&ctl);
}


static int platform_setPad(int pad, char hys, char pus, char pue, char pke, char ode, char speed, char dse, char sre)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_iopad;
	ctl.iopad.pad = pad;
	ctl.iopad.hys = hys;
	ctl.iopad.pus = pus;
	ctl.iopad.pue = pue;
	ctl.iopad.pke = pke;
	ctl.iopad.ode = ode;
	ctl.iopad.speed = speed;
	ctl.iopad.dse = dse;
	ctl.iopad.sre = sre;

	return platformctl(&ctl);
}


static int platform_setDevClk(int dev, unsigned int state)
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_devclock;
	ctl.devclock.dev = dev;
	ctl.devclock.state = state;

	return platformctl(&ctl);
}


static int platform_configure(void)
{
	uint8_t i;
	static const int muxes[] = { pctl_mux_csi_vsync, pctl_mux_csi_hsync,
		pctl_mux_csi_d0, pctl_mux_csi_d1, pctl_mux_csi_d2, pctl_mux_csi_d3 };
	static const int sels[] = { pctl_isel_usdhc2_clk, pctl_isel_usdhc2_cmd,
		pctl_isel_usdhc2_d0, pctl_isel_usdhc2_d1, pctl_isel_usdhc2_d3 };
	static const int pads[] = { pctl_pad_csi_hsync, pctl_pad_csi_d0,
		pctl_pad_csi_d1, pctl_pad_csi_d2, pctl_pad_csi_d3 };

	for (i = 0; i < ARRAY_LENGTH(muxes); i++) {
		if (platform_setMux(muxes[i], 1) < 0) {
			return -1;
		}
	}
	for (i = 0; i < ARRAY_LENGTH(sels); i++) {
		if (platform_setSel(sels[i], 0) < 0) {
			return -1;
		}
	}
	if (platform_setSel(pctl_isel_usdhc2_d2, 2) < 0) {
		return -1;
	}
	for (i = 0; i < ARRAY_LENGTH(pads); i++) {
		if (platform_setPad(pads[i], 0, 2, 1, 1, 0, 2, 1, 0) < 0) {
			return -1;
		}
	}
	if (platform_setPad(pctl_pad_csi_vsync, 0, 0, 0, 0, 0, 2, 1, 0) < 0) {
		return -1;
	}
	if (platform_setDevClk(pctl_clk_usdhc2, 3) < 0) {
		return -1;
	}

	return 0;
}


static int sdhc_reset(uint32_t mode)
{
	uint8_t i;
	uint32_t val;

	if (mode == SDHC_SYSCTL_RESET_ALL) {
		*(sdio_common.base + reg_sys_ctrl) = SDHC_SYSCTL_CLK_DEFAULT | SDHC_SYSCTL_RESERVED;
	}
	*(sdio_common.base + reg_sys_ctrl) |= mode;

	for (i = 0; i < SDHC_RETRIES; ++i) {
		usleep(10);
		val = *(sdio_common.base + reg_sys_ctrl) & mode;
		if (val == 0) {
			return 0;
		}
	}

	return -1;
}


static int sdhc_cmdWait(uint32_t flags, uint32_t wait_us, uint32_t max_cnt)
{
	uint32_t i, val;

	for (i = 0; i < max_cnt; ++i) {
		val = *(sdio_common.base + reg_int_status);

		if (val & SDHC_CMD_ERROR) {
			return -1;
		}

		if ((val & flags) == flags) {
			return 0;
		}

		usleep(wait_us);
	}

	return -1;
}


static int sdio_isr(unsigned int n, void *arg)
{
	/* clear Card Interrupt signal enable only
	 * i.MX 6ULL documentation, page 4042 */
	if (*(sdio_common.base + reg_int_status) & SDIO_STATUS_CARD_IRQ) {
		*(sdio_common.base + reg_int_signal_en) &= ~SDIO_STATUS_CARD_IRQ;
	}

	return 0;
}


static void *sdio_dmammap(void)
{
	void *p;
	size_t sz = (DMA_BUFFER_SIZE + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1);

	p = mmap(NULL, sz, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_UNCACHED, -1, 0);
	if (p == MAP_FAILED) {
		return NULL;
	}

	return p;
}


static addr_t sdio_mphys(void *p, size_t *psz)
{
	size_t sz;
	addr_t pa;
	addr_t npa;

	pa = va2pa(p);
	sz = _PAGE_SIZE - (pa & (_PAGE_SIZE - 1));

	while (sz < *psz) {
		npa = va2pa(p + sz);
		if (npa != pa + sz) {
			break;
		}

		sz += _PAGE_SIZE;
	}

	if (sz < *psz) {
		*psz = sz;
	}

	return pa;
}


static int sdio_allocDMA(void)
{
	size_t psize = DMA_BUFFER_SIZE;

	sdio_common.dmaptr = sdio_dmammap();
	if (sdio_common.dmaptr == NULL) {
		return -1;
	}

	sdio_common.dmaphys = sdio_mphys(sdio_common.dmaptr, &psize);

	if (DMA_BUFFER_SIZE != psize) {
		munmap(sdio_common.dmaptr, DMA_BUFFER_SIZE);
		sdio_common.dmaptr = NULL;
		return -1;
	}

	if (sdio_common.dmaphys & 3) {
		munmap(sdio_common.dmaptr, DMA_BUFFER_SIZE);
		sdio_common.dmaptr = NULL;
		return -1;
	}

	return 0;
}


static void sdio_eventThread(void *arg)
{
	uint8_t i;
	uint32_t val;

	while (1) {
		mutexLock(sdio_common.eventLock);
		condWait(sdio_common.eventCond, sdio_common.eventLock, 0);

		val = *(sdio_common.base + reg_int_status) & sdio_common.enabledEvents;
		for (i = 0; i < ARRAY_LENGTH(sdioEvents); i++) {
			/* special treatment of Card Interrupt
			 * i.MX 6ULL documentation, pages 4038, 4042 */
			if ((sdioEvents[i] & val) != 0) {
				if (sdioEvents[i] == SDIO_STATUS_CARD_IRQ) {
					*(sdio_common.base + reg_int_status_en) &= ~SDIO_STATUS_CARD_IRQ;
				}
				*(sdio_common.base + reg_int_status) |= sdioEvents[i];
				if (sdio_common.eventHandlers[i] != NULL) {
					sdio_common.eventHandlers[i](sdio_common.eventHandlerArgs[i]);
				}
				if (sdioEvents[i] == SDIO_STATUS_CARD_IRQ) {
					*(sdio_common.base + reg_int_status_en) |= SDIO_STATUS_CARD_IRQ;
					*(sdio_common.base + reg_int_signal_en) |= SDIO_STATUS_CARD_IRQ;
				}
			}
		}

		mutexUnlock(sdio_common.eventLock);
	}
}


static int _sdio_cmdSend(uint8_t cmd, uint32_t arg, uint32_t *res)
{
	uint32_t val, cmdFrame;

	if (res != NULL) {
		*res = 0;
	}

	val = *(sdio_common.base + reg_pres_state);
	if (val & 0x7) {
		return -EBUSY;
	}

	cmdFrame = (cmd & 0x3f) << 24;

	/* set response type */
	switch (cmd) {
		case SDIO_CMD_STATE_IDLE:
			/* no response */
			break;
		case SDIO_CMD_RELATIVE_ADDR:
			cmdFrame |= 1UL << 20; /* CICEN=1 */
			cmdFrame |= 1UL << 19; /* CCCEN=1 */
			cmdFrame |= 0x2 << 16; /* RSPTYP=2 */
			break;
		case SDIO_CMD_SEND_OP_COND:
			cmdFrame |= 0x2 << 16; /* RSPTYP=2 */
			break;
		case SDIO_CMD_SELECT_CARD:
			cmdFrame |= 1UL << 20; /* CICEN=1 */
			cmdFrame |= 1UL << 19; /* CCCEN=1 */
			cmdFrame |= 0x3 << 16; /* RSPTYP=3 */
			break;
		case SDIO_CMD_STATE_INACTIVE:
			/* no response */
			break;
		case SDIO_CMD_RW_DIRECT:
			cmdFrame |= 1UL << 20; /* CICEN=1 */
			cmdFrame |= 1UL << 19; /* CCCEN=1 */
			cmdFrame |= 0x2 << 16; /* RSPTYP=2 */
			break;
		default:
			return -EINVAL;
	}

	*(sdio_common.base + reg_mix_ctrl) = (1UL << 31);
	*(sdio_common.base + reg_blk_att) = 0;
	*(sdio_common.base + reg_cmd_arg) = arg;
	*(sdio_common.base + reg_cmd_xfer_typ) = cmdFrame;

	/* wait 1 ms max */
	if (sdhc_cmdWait(0x1, 10, 100) < 0) {
		sdhc_reset(SDHC_SYSCTL_RESET_CMD);
		return -ETIMEDOUT;
	}

	/* clear status flags */
	*(sdio_common.base + reg_int_status) = 0x1; /* CC=1 */

	/* retrieve response */
	switch (cmd) {
		case SDIO_CMD_STATE_IDLE:
			val = 0;
			break;
		case SDIO_CMD_RELATIVE_ADDR:
			val = *(sdio_common.base + reg_cmd_rsp0);
			break;
		case SDIO_CMD_SEND_OP_COND:
			val = *(sdio_common.base + reg_cmd_rsp0);
			break;
		case SDIO_CMD_SELECT_CARD:
			val = *(sdio_common.base + reg_cmd_rsp3);
			break;
		case SDIO_CMD_STATE_INACTIVE:
			val = 0;
			break;
		case SDIO_CMD_RW_DIRECT:
			val = *(sdio_common.base + reg_cmd_rsp0);
			break;
		default:
			return -EINVAL;
	}

	if (res != NULL) {
		*res = val;
	}

	return 0;
}


static int sdio_cmdSend(uint8_t cmd, uint32_t arg, uint32_t *res)
{
	int rslt;

	mutexLock(sdio_common.cmdLock);
	rslt = _sdio_cmdSend(cmd, arg, res);
	mutexUnlock(sdio_common.cmdLock);

	return rslt;
}


static int sdio_cardInit(void)
{
	uint32_t cardRCA;

	/* SDIO card init and selection sequence,
	 * SDIO simplified specification v3.00,
	 * page 26 */
	if (sdio_cmdSend(SDIO_CMD_STATE_IDLE, 0, NULL) < 0) {
		return -1;
	}
	if (sdio_cmdSend(SDIO_CMD_SEND_OP_COND, 0, NULL) < 0) {
		return -1;
	}
	if (sdio_cmdSend(SDIO_CMD_RELATIVE_ADDR, 0, &cardRCA) < 0) {
		return -1;
	}
	if (sdio_cmdSend(SDIO_CMD_SELECT_CARD, cardRCA, NULL) < 0) {
		return -1;
	}

	return 0;
}


static int sdio_startEventThread(void)
{
	if (sdio_common.eventThreadStarted == 1) {
		return 0;
	}

	sdio_common.eventLock = -1;
	if (mutexCreate(&sdio_common.eventLock) < 0) {
		return -1;
	}
	sdio_common.eventCond = -1;
	if (condCreate(&sdio_common.eventCond) < 0) {
		resourceDestroy(sdio_common.eventLock);
		return -1;
	}
	interrupt(USDHC2_IRQ, sdio_isr, NULL, sdio_common.eventCond, &sdio_common.isrHandle);
	if (beginthread(sdio_eventThread, 4, &sdio_common.stack, THREAD_STACK_SIZE, NULL) < 0) {
		resourceDestroy(sdio_common.eventLock);
		resourceDestroy(sdio_common.eventCond);
		return -1;
	}

	sdio_common.eventThreadStarted = 1;
	return 0;
}


static void _sdio_free(void)
{
	sdhc_reset(SDHC_SYSCTL_RESET_ALL);

	/*
	 * FIXME: the following resources:
	 * isrHandle, eventLock, eventCond
	 * are not destroyed since they are used
	 * by running event service thread.
	 * The same applies for hardware register
	 * base address which remains mapped.
	 * this is due to supposed problems with
	 * threadJoin function which would allow
	 * to stop the thread properly.
	 */

	sdio_common.sdioInitialized = 0;
	sdio_common.blocksz = 0;

	if (sdio_common.cmdLock >= 0) {
		resourceDestroy(sdio_common.cmdLock);
		sdio_common.cmdLock = (handle_t)NULL;
	}
	if (sdio_common.dmaptr != NULL) {
		munmap(sdio_common.dmaptr, DMA_BUFFER_SIZE);
		sdio_common.dmaptr = NULL;
		sdio_common.dmaphys = (addr_t)NULL;
	}

	*(sdio_common.base + reg_int_status_en) = 0;
	*(sdio_common.base + reg_int_signal_en) = 0;

	memset(&sdio_common.eventHandlers, 0, sizeof(sdio_common.eventHandlers));
	memset(&sdio_common.eventHandlerArgs, 0, sizeof(sdio_common.eventHandlerArgs));
}


void sdio_free(void)
{
	if (sdio_common.sdioInitialized == 0) {
		return;
	}

	mutexLock(sdio_common.eventLock);
	_sdio_free();
	mutexUnlock(sdio_common.eventLock);
}


int sdio_init(void)
{
	uint8_t i;
	int rslt;
	void *ptr;

	if (sdio_common.sdioInitialized != 0) {
		return 0;
	}

	ptr = mmap(NULL, _PAGE_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, USDHC2_ADDR);
	if (ptr == MAP_FAILED) {
		_sdio_free();
		return -EIO;
	}
	sdio_common.base = ptr;
	if (sdio_allocDMA() < 0) {
		_sdio_free();
		return -ENOMEM;
	}

	sdio_common.cmdLock = -1;
	if (mutexCreate(&sdio_common.cmdLock) < 0) {
		_sdio_free();
		return -ENOMEM;
	}

	if (platform_configure() < 0) {
		_sdio_free();
		return -EIO;
	}

	if (sdhc_reset(SDHC_SYSCTL_RESET_ALL) < 0) {
		_sdio_free();
		return -EIO;
	}

	/* FIXME: Change card detection pin polarity to make
	 * the embedded module work on dataro. CD_POL bit is
	 * marked as "Only for debug". This needs changing.
	 * i.MX 6ULL Reference manual rev.1, page 4072. */
	*(sdio_common.base + reg_vend_spec) |= (1UL << 5);

	*(sdio_common.base + reg_int_status_en) = SDHC_INTERRUPT_DEFAULTS;

	for (i = 0; i < 5; i++) {
		rslt = sdio_cardInit();
		if (rslt == 0) {
			break;
		}
		usleep(1000);
	}
	if (rslt < 0) {
		_sdio_free();
		return -EIO;
	}

	if (sdio_startEventThread() < 0) {
		_sdio_free();
		return -ENOMEM;
	}

	sdio_common.sdioInitialized = 1;
	return 0;
}


int sdio_config(uint32_t freq, uint16_t blocksz)
{
	uint8_t i;
	uint32_t val;

	if (freq != 25000000 && freq != 50000000) {
		return -EINVAL;
	}

	sdio_common.blocksz = blocksz;

	*(sdio_common.base + reg_prot_ctrl) |= (1UL << 1);  /* set 4-bit mode */
	*(sdio_common.base + reg_vend_spec) &= ~(1UL << 8); /* FRC_SDCLK_ON=0 */
	/* "Before changing clock divisor [...] Host Driver
	 * should make sure the SDSTB bit is high"
	 * i.MX 6ULL Reference Manual Rev.1, page 4028 */
	for (i = 0; i < SDHC_RETRIES; i++) {
		val = *(sdio_common.base + reg_pres_state) & (1UL << 3);
		if (val != 0) {
			break;
		}
		usleep(10);
	}
	if (val == 0) {
		return -EIO;
	}

	if (freq == 50000000) {
		*(sdio_common.base + reg_sys_ctrl) = SDHC_SYSCTL_CLK_DIV_4 | SDHC_SYSCTL_RESERVED;
	}
	else {
		*(sdio_common.base + reg_sys_ctrl) = SDHC_SYSCTL_CLK_DIV_8 | SDHC_SYSCTL_RESERVED;
	}

	/* Send 80 initialization clock cycles
	 * to the card, and wait until it finishes.
	 * Should be done in ~3us on 25MHz clock */
	*(sdio_common.base + reg_sys_ctrl) |= (1UL << 27);
	for (i = 0; i < SDHC_RETRIES; i++) {
		val = *(sdio_common.base + reg_sys_ctrl) & (1UL << 27);
		if (val == 0) {
			break;
		}
		usleep(10);
	}
	if (val != 0) {
		return -ETIMEDOUT;
	}

	return 0;
}


int sdio_transferDirect(sdio_dir_t dir, uint32_t address, uint8_t area, uint8_t *data)
{
	int rslt = 0;
	uint32_t response, arg = 0;

	arg |= ((address & 0x1ffff) << 9);
	arg |= ((area & 0x7) << 28);
	arg |= (dir << 31);

	/* On direct write transfers data is part of argument */
	if (dir == sdio_write) {
		arg |= *data;
	}

	rslt = sdio_cmdSend(SDIO_CMD_RW_DIRECT, arg, &response);

	/* Data is part of response on read transfers
	 * and also on writes with RaW (read after write)
	 * flag set - SDIO simplified spec v3.00, page 36 */
	if (dir == sdio_read) {
		*data = response & 0xff;
	}

	return rslt;
}


static int _sdio_transferBulk(sdio_dir_t dir, int blockMode, uint32_t address,
	uint8_t area, uint8_t *data, size_t len)
{
	uint32_t val, cmd, mix, blk;
	uint32_t arg = 0;
	uint16_t count;

	count = (blockMode != 0) ? (len / sdio_common.blocksz) : len;

	/* construct argument */
	arg |= (dir << 31);
	arg |= ((area & 0x7) << 28);
	arg |= ((address & 0x1ffff) << 9);
	arg |= (count & 0x1ff);
	arg |= ((blockMode != 0) << 27);
	arg |= (1UL << 26);

	val = *(sdio_common.base + reg_pres_state);
	if (val & 0x7) {
		return -EBUSY;
	}

	val = *(sdio_common.base + reg_int_status);
	if (val & 0x2) {
		return -EIO;
	}

	if (dir == sdio_write) {
		memcpy(sdio_common.dmaptr, data, len);
	}

	cmd = SDIO_CMD_RW_EXTENDED << 24;
	cmd |= 1UL << 21; /* DPSEL=1 */
	cmd |= 1UL << 20; /* CICEN=1 */
	cmd |= 1UL << 19; /* CCCEN=1 */
	cmd |= 0x2 << 16; /* RSPTYP=2 */

	mix = (1U << 31) | (1UL << 0); /* DMAEN=1 */
	if (dir == sdio_read) {
		mix |= 1UL << 4; /* DTDSEL=1 */
	}

	if (blockMode != 0) {
		mix |= (1UL << 5); /* MSBSEL=1 */
		mix |= (1UL << 1); /* BCEN=1 */

		blk = (count << 16) | sdio_common.blocksz;
	}
	else {
		blk = (1UL << 16) | len;
	}

	*(sdio_common.base + reg_mix_ctrl) = mix;
	*(sdio_common.base + reg_blk_att) = blk;
	*(sdio_common.base + reg_ds_addr) = sdio_common.dmaphys;
	*(sdio_common.base + reg_cmd_arg) = arg;
	*(sdio_common.base + reg_cmd_xfer_typ) = cmd;

	/* wait 1 ms max */
	if (sdhc_cmdWait(0xb, 10, 100) < 0) {
		sdhc_reset(SDHC_SYSCTL_RESET_CMD);
		sdhc_reset(SDHC_SYSCTL_RESET_DATA);
		return -ETIMEDOUT;
	}

	/* clear status flags */
	*(sdio_common.base + reg_int_status) = 0xb; /* DINT=1 TC=1 CC=1 */

	if (dir == sdio_read) {
		memcpy(data, sdio_common.dmaptr, len);
	}

	/* retrieve response */
	val = *(sdio_common.base + reg_cmd_rsp0);

	return 0;
}


int sdio_transferBulk(sdio_dir_t dir, int blockMode, uint32_t address,
	uint8_t area, uint8_t *data, size_t len)
{
	int rslt;

	if (len > DMA_BUFFER_SIZE) {
		return -EINVAL;
	}
	if (blockMode != 0 && (sdio_common.blocksz == 0 || len % sdio_common.blocksz != 0)) {
		return -EINVAL;
	}

	mutexLock(sdio_common.cmdLock);
	rslt = _sdio_transferBulk(dir, blockMode, address, area, data, len);
	mutexUnlock(sdio_common.cmdLock);

	return rslt;
}


int sdio_eventRegister(uint8_t event, sdio_event_handler_t handler, void *arg)
{
	if (event >= ARRAY_LENGTH(sdioEvents)) {
		return -EINVAL;
	}

	mutexLock(sdio_common.eventLock);
	sdio_common.eventHandlers[event] = handler;
	sdio_common.eventHandlerArgs[event] = arg;
	mutexUnlock(sdio_common.eventLock);

	return 0;
}


int sdio_eventEnable(uint8_t event, int enabled)
{
	uint32_t val;

	if (event >= ARRAY_LENGTH(sdioEvents)) {
		return -EINVAL;
	}

	mutexLock(sdio_common.eventLock);
	val = *(sdio_common.base + reg_int_status_en);
	if (enabled != 0) {
		val |= sdioEvents[event];
	}
	else {
		val &= ~sdioEvents[event];
	}
	*(sdio_common.base + reg_int_status_en) = val;

	val = *(sdio_common.base + reg_int_signal_en);
	if (enabled != 0) {
		val |= sdioEvents[event];
	}
	else {
		val &= ~sdioEvents[event];
	}
	*(sdio_common.base + reg_int_signal_en) = val;
	sdio_common.enabledEvents = val;
	mutexUnlock(sdio_common.eventLock);

	return 0;
}
