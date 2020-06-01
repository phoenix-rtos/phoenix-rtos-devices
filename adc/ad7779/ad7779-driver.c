/*
 * Phoenix-RTOS
 *
 * i.MX RT1064 AD7779 driver.
 *
 * Copyright 2018, 2020 Phoenix Systems
 * Author: Krystian Wasik, Marcin Baran
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/msg.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include <phoenix/arch/imxrt.h>

#include <edma.h>

#include "ad7779.h"
#include "adc-api.h"

#define COL_RED     "\033[1;31m"
#define COL_CYAN    "\033[1;36m"
#define COL_NORMAL  "\033[0m"

#define LOG_TAG "ad7779-drv: "
#define log_debug(fmt, ...)     do { printf(LOG_TAG fmt "\n", ##__VA_ARGS__); } while (0)
#define log_info(fmt, ...)      do { printf(LOG_TAG COL_CYAN fmt COL_NORMAL "\n", ##__VA_ARGS__); } while (0)
#define log_error(fmt, ...)     do { printf(LOG_TAG COL_RED fmt COL_NORMAL "\n", ##__VA_ARGS__); } while (0)

#define SAI1_RX_DMA_REQUEST         (19)
#define SAI1_RX_DMA_CHANNEL         (7)

#define ADC_BUFFER_SIZE             (4 * AD7779_NUM_OF_CHANNELS * _PAGE_SIZE)
#define NUM_OF_BUFFERS              (16)

typedef volatile struct {
	uint32_t VERID; //0x00
	uint32_t PARAM; //0x04
	uint32_t TCSR;  //0x08
	uint32_t TCR1;  //0x0C
	uint32_t TCR2;  //0x10
	uint32_t TCR3;  //0x14
	uint32_t TCR4;  //0x18
	uint32_t TCR5;  //0x1C
	uint32_t TDR0;  //0x20
	uint32_t TDR1;  //0x24
	uint32_t TDR2;  //0x28
	uint32_t TDR3;  //0x2C
	uint32_t reserved0[4];  //0x30-0x3C
	uint32_t TFR0;  //0x40
	uint32_t TFR1;  //0x44
	uint32_t TFR2;  //0x48
	uint32_t TFR3;  //0x4C
	uint32_t reserved1[4];  //0x50-0x5C
	uint32_t TMR;  //0x60
	uint32_t reserved2[9];  //0x64-0x84
	uint32_t RCSR;  //0x88
	uint32_t RCR1;  //0x8C
	uint32_t RCR2;  //0x90
	uint32_t RCR3;  //0x94
	uint32_t RCR4;  //0x98
	uint32_t RCR5;  //0x9C
	uint32_t RDR0;  //0xA0
	uint32_t RDR1;  //0xA4
	uint32_t RDR2;  //0xA8
	uint32_t RDR3;  //0xAC
	uint32_t reserved3[4];  //0xB0-0xBC
	uint32_t RFR0;  //0xC0
	uint32_t RFR1;  //0xC4
	uint32_t RFR2;  //0xC8
	uint32_t RFR3;  //0xCC
	uint32_t reserved4[4];  //0xD0-0xDC
	uint32_t RMR;  //0xE0
} sai_t;

struct driver_common_s
{
	uint32_t port;
	volatile uint32_t current;

	volatile struct edma_tcd_s
		__attribute__((aligned(32))) tcds[NUM_OF_BUFFERS];

	addr_t buffer_paddr;

	sai_t *sai;
	addr_t sai_paddr;

	addr_t ocram_ptr;

	handle_t irq_cond, irq_lock, irq_handle;

	int enabled;
} common;

#define SAI_FIFO_WATERMARK          (8)

#define SAI_RCR3_RCE_BIT            (1 << 16)
#define SAI_RCSR_RE_BIT             (1 << 31)
#define SAI_RCSR_SR_BIT             (1 << 24)
#define SAI_RCSR_FRDE_BIT           (1 << 0)

#define TCD_CSR_INTMAJOR_BIT        (1 << 1)
#define TCD_CSR_ESG_BIT             (1 << 4)

static int sai_init(void)
{
	common.sai_paddr = 0x40384000; /* SAI1 */
	common.sai = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_DEVICE, OID_PHYSMEM, common.sai_paddr);
	if (common.sai == MAP_FAILED)
		return -1;

	platformctl_t pctl;
	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.dev = pctl_clk_sai1;
	pctl.devclock.state = 0b11;
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_b0_14;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 3; /* ALT3 (SAI1_RX_SYNC) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai1_rx_sync;
	pctl.ioisel.daisy = 2; /* Select GPIO_B0_14 pad */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_b0_15;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 3; /* ALT3 (SAI1_RX_BCLK) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai1_rx_bclk;
	pctl.ioisel.daisy = 2; /* Select GPIO_B0_15 pad */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_b1_00;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 3; /* ALT3 (SAI1_RX_DATA0) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai1_rx_data0;
	pctl.ioisel.daisy = 2; /* Select GPIO_B1_00 pad */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_b0_10;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 3; /* ALT3 (SAI1_RX_DATA1) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai1_rx_data1;
	pctl.ioisel.daisy = 1; /* Select GPIO_B0_10 pad */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_b0_11;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 3; /* ALT3 (SAI1_RX_DATA2) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai1_rx_data2;
	pctl.ioisel.daisy = 1; /* Select GPIO_B0_11 pad */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_b0_12;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 3; /* ALT3 (SAI1_RX_DATA3) */
	platformctl(&pctl);

	pctl.action = pctl_set;
	pctl.type = pctl_ioisel;
	pctl.ioisel.isel = pctl_isel_sai1_rx_data3;
	pctl.ioisel.daisy = 1; /* Select GPIO_B0_12 pad */
	platformctl(&pctl);

	/* Initialize SAI */
	common.sai->RCR1 = SAI_FIFO_WATERMARK;
	common.sai->RCR2 = 0x0; /* External bit clock (slave mode) */
	common.sai->RCR3 |= SAI_RCR3_RCE_BIT;
	common.sai->RCR4 = 0x00070018; /* FRSZ=7, SYWD=0, MF=1, FSE=1, FSP=0, FSD=0 */
	common.sai->RCR5 = 0x1f1f1f00; /* WNW=31, WOW=31, FBT=31 */
	common.sai->RMR = 0x0; /* No words masked */
	common.sai->RCSR |= SAI_RCSR_FRDE_BIT; /* FIFO Request DMA Enable */

	return 0;
}

static void sai_rx_enable(void)
{
	common.sai->RCSR |= SAI_RCSR_RE_BIT;
}

static void sai_rx_disable(void)
{
	common.sai->RCSR &= ~(SAI_RCSR_RE_BIT);
	common.sai->RCSR &= ~(SAI_RCSR_SR_BIT);
}

static addr_t sai_get_rx_fifo_ptr(void)
{
	return (addr_t)(&(((sai_t*)common.sai_paddr)->RDR0));
}

static int edma_error_handler(unsigned int n, void *arg)
{
	/* TODO: Store some info for debugging? Notify about it somehow? */
	sai_rx_disable();
	common.enabled = 0;
	edma_clear_error(SAI1_RX_DMA_CHANNEL);

	return 0;
}

static int edma_irq_handler(unsigned int n, void *arg)
{
	common.current += 1;

	edma_clear_interrupt(SAI1_RX_DMA_CHANNEL);
	return 0;
}


#define OCRAM2_BASE              (0x20200000)
#define OCRAM2_END               (0x2027FFFF)

/* Allocating OCRAM2 from the end */
static addr_t ocram_alloc(size_t size)
{
	unsigned n = (size + _PAGE_SIZE - 1)/_PAGE_SIZE;
	addr_t paddr = 0;

	if (common.ocram_ptr + n*_PAGE_SIZE <= OCRAM2_END) {
		paddr = common.ocram_ptr;
		common.ocram_ptr += n*_PAGE_SIZE;
	}

	return paddr;
}

static void *alloc_uncached(size_t size, addr_t *paddr, int ocram)
{
	uint32_t n = (size + _PAGE_SIZE - 1)/_PAGE_SIZE;
	oid_t *oid = OID_NULL;
	addr_t _paddr = 0;

	if (ocram) {
		oid = OID_PHYSMEM;
		_paddr = ocram_alloc(n*_PAGE_SIZE);
		if (!_paddr)
			return NULL;
	}

	void *vaddr = mmap(NULL, n*_PAGE_SIZE,
		PROT_READ | PROT_WRITE, MAP_UNCACHED, oid, _paddr);
	if (vaddr == MAP_FAILED)
		return NULL;

	if (!ocram)
		_paddr = va2pa(vaddr);

	if (paddr != NULL)
		*paddr = _paddr;

	return vaddr;
}

static int free_uncached(void *vaddr, size_t size)
{
	uint32_t n = (size + _PAGE_SIZE - 1)/_PAGE_SIZE;

	return munmap(vaddr, n*_PAGE_SIZE);
}

static int edma_configure(void)
{
	int res;

	if((res = mutexCreate(&common.irq_lock) != EOK)) {
		log_error("mutex resource creation failed");
		return res;
	}

	if((res = condCreate(&common.irq_cond)) != EOK) {
		log_error("conditional resource creation failed");
		resourceDestroy(common.irq_lock);
		return res;
	}

	void *buf = alloc_uncached(ADC_BUFFER_SIZE, &common.buffer_paddr, 0);

	if (buf == NULL) {
		log_error("edma buffers allocation failed");
		return -ENOMEM;
	}

	memset(buf, 0, ADC_BUFFER_SIZE);

	uint8_t xfer_size = sizeof(uint32_t);

	common.tcds[0].soff = 0;
	common.tcds[0].attr = (edma_get_tcd_attr_xsize(xfer_size) << 8) |
							edma_get_tcd_attr_xsize(xfer_size);

	/* Number of bytes per minor loop iteration */
	common.tcds[0].nbytes_mlnoffno = xfer_size * AD7779_NUM_OF_CHANNELS;
	common.tcds[0].slast = 0;
	common.tcds[0].doff = xfer_size;
	common.tcds[0].dlast_sga = (uint32_t)&common.tcds[1];

	/* Number of major loop iterations */
	common.tcds[0].biter_elinkno =
		ADC_BUFFER_SIZE / common.tcds[0].nbytes_mlnoffno / NUM_OF_BUFFERS;
	common.tcds[0].citer_elinkno = common.tcds[0].biter_elinkno;

	/* Set addrs for the TCD. */
	common.tcds[0].saddr = sai_get_rx_fifo_ptr();
	common.tcds[0].daddr = (uint32_t)buf;

	/* Enable major loop finish interrupt and scatter-gather */
	common.tcds[0].csr = TCD_CSR_INTMAJOR_BIT | TCD_CSR_ESG_BIT;

	int i;

	for (i = 1; i < NUM_OF_BUFFERS; ++i) {
		edma_copy_tcd(&common.tcds[0], &common.tcds[i]);
		common.tcds[i].daddr = (uint32_t)buf + i * ADC_BUFFER_SIZE / NUM_OF_BUFFERS;
		common.tcds[i].dlast_sga = (uint32_t)&common.tcds[(i + 1) % NUM_OF_BUFFERS];
	}

	if ((res = edma_install_tcd(&common.tcds[0], SAI1_RX_DMA_CHANNEL)) != 0)
		return res;

	interrupt(EDMA_CHANNEL_IRQ(SAI1_RX_DMA_CHANNEL),
		edma_irq_handler, NULL, common.irq_cond, &common.irq_handle);

	dmamux_set_source(SAI1_RX_DMA_CHANNEL, SAI1_RX_DMA_REQUEST);
	dmamux_channel_enable(SAI1_RX_DMA_CHANNEL);
	edma_channel_enable(SAI1_RX_DMA_CHANNEL);

	return 0;
}

static int dev_init(void)
{
	int res;
	oid_t dir;
	msg_t msg;

	res = portCreate(&common.port);
	if (res != EOK) {
		log_error("could not create port: %d", res);
		return -1;
	}

	res = mkdir(ADC_DEVICE_DIR, 0);
	if (res < 0 && errno != EEXIST) {
		log_error("mkdir /dev failed (%d)", -errno);
		return -1;
	}

	if ((res = lookup(ADC_DEVICE_DIR, NULL, &dir)) < 0) {
		log_error("%s lookup failed (%d)", ADC_DEVICE_DIR, res);
		return -1;
	}

	msg.type = mtCreate;
	msg.i.create.type = otDev;
	msg.i.create.mode = 0;
	msg.i.create.dev.port = common.port;
	msg.i.create.dev.id = 0;
	msg.i.create.dir = dir;
	msg.i.data = ADC_DEVICE_FILE_NAME;
	msg.i.size = sizeof(ADC_DEVICE_FILE_NAME);
	msg.o.data = NULL;
	msg.o.size = 0;

	if ((res = msgSend(dir.port, &msg)) < 0 || msg.o.create.err != EOK) {
		log_error("could not create %s (res=%d, err=%d)",
			ADC_DEVICE_FILE_NAME, res, msg.o.create.err);
		return -1;
	}

	log_info("device initialized");

	return 0;
}

static int dev_open(oid_t *oid, int flags)
{
	(void)oid;
	(void)flags;

	return EOK;
}

static int dev_close(oid_t *oid, int flags)
{
	(void)oid;
	(void)flags;

	return EOK;
}

static int dev_read(void *data, size_t size)
{
	if (data != NULL && size != sizeof(unsigned))
		return -EIO;

	mutexLock(common.irq_lock);

	condWait(common.irq_cond, common.irq_lock, 0);

	*(uint32_t **)data = &common.current;

	mutexUnlock(common.irq_lock);

	return EOK;
}

static int dev_ctl(msg_t *msg)
{
	int res;
	adc_dev_ctl_t dev_ctl;

	memcpy(&dev_ctl, msg->o.raw, sizeof(adc_dev_ctl_t));

	switch (dev_ctl.type) {
		case adc_dev_ctl__enable:
			common.enabled = 1;
			common.current = 0;
			edma_channel_enable(SAI1_RX_DMA_CHANNEL);
			sai_rx_enable();
			return EOK;

		case adc_dev_ctl__disable:
			sai_rx_disable();
			edma_channel_disable(SAI1_RX_DMA_CHANNEL);
			common.enabled = 0;
			common.current = 0;
			return EOK;

		case adc_dev_ctl__set_adc_mux:
			res = ad7779_set_adc_mux((dev_ctl.mux >> 6),
				(dev_ctl.mux >> 2) & 0b1111);
			if (res != AD7779_OK)
				return -EIO;
			return EOK;

		case adc_dev_ctl__set_config:
			if (common.enabled)
				return -EBUSY;
			res = ad7779_set_sampling_rate(dev_ctl.config.sampling_rate);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			return EOK;

		case adc_dev_ctl__get_config:
			res = ad7779_get_sampling_rate(&dev_ctl.config.sampling_rate);
			if (res != AD7779_OK)
				return -EIO;
			dev_ctl.config.channels = AD7779_NUM_OF_CHANNELS;
			dev_ctl.config.bits = AD7779_NUM_OF_BITS;
			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));
			return EOK;

		case adc_dev_ctl__get_buffers:
			dev_ctl.buffers.paddr = common.buffer_paddr;
			dev_ctl.buffers.num = NUM_OF_BUFFERS;
			dev_ctl.buffers.size = ADC_BUFFER_SIZE;
			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));
			return EOK;

		case adc_dev_ctl__set_channel_config:
		{
			res = ad7779_set_channel_gain(dev_ctl.ch_config.channel,
				dev_ctl.ch_config.gain);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;

			ad7779_chmode_t mode = (dev_ctl.ch_config.ref_monitor_mode << 1) |
				dev_ctl.ch_config.meter_rx_mode;

			res = ad7779_set_channel_mode(dev_ctl.ch_config.channel, mode);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			return EOK;
		}

		case adc_dev_ctl__get_channel_config:
		{
			uint8_t gain;
			res = ad7779_get_channel_gain(dev_ctl.ch_config.channel, &gain);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;

			dev_ctl.ch_config.gain = gain;

			ad7779_chmode_t mode;
			res = ad7779_get_channel_mode(dev_ctl.ch_config.channel, &mode);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;

			dev_ctl.ch_config.meter_rx_mode = 0;
			dev_ctl.ch_config.ref_monitor_mode = 0;
			if (mode == ad7779_chmode__ref_monitor)
				dev_ctl.ch_config.ref_monitor_mode = 1;
			else if (mode == ad7779_chmode__meter_rx)
				dev_ctl.ch_config.meter_rx_mode = 1;

			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));
			return EOK;
		}

		case adc_dev_ctl__set_channel_gain:
			res = ad7779_set_channel_gain(dev_ctl.gain.channel, dev_ctl.gain.val);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			return EOK;

		case adc_dev_ctl__get_channel_gain:
			res = ad7779_get_channel_gain(dev_ctl.gain.channel, &dev_ctl.gain.val);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));
			return EOK;

		case adc_dev_ctl__set_channel_calib:
			res = ad7779_set_channel_offset(dev_ctl.calib.channel, dev_ctl.calib.offset);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			res = ad7779_set_channel_gain_correction(dev_ctl.calib.channel, dev_ctl.calib.gain);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			return EOK;

		case adc_dev_ctl__get_channel_calib:
			res = ad7779_get_channel_offset(dev_ctl.calib.channel, &dev_ctl.calib.offset);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			res = ad7779_get_channel_gain_correction(dev_ctl.calib.channel, &dev_ctl.calib.gain);
			if (res == AD7779_ARG_ERROR)
				return -EINVAL;
			if (res != AD7779_OK)
				return -EIO;
			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));
			return EOK;

		default:
			log_error("dev_ctl: unknown type (%d)", dev_ctl.type);
			return -ENOSYS;
	}

	return EOK;
}

static void msg_loop(void)
{
	msg_t msg;
	unsigned rid;

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
				msg.o.io.err = dev_read(msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.io.err = -ENOSYS;
				break;

			case mtDevCtl:
				msg.o.io.err = dev_ctl(&msg);
				break;
		}

		msgRespond(common.port, &msg, rid);
	}
}

static void ad7779_enableCache(unsigned char enable)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_devcache;
	pctl.devcache.state = !!enable;

	platformctl(&pctl);
}

static int init(void)
{
	int res;

	common.enabled = 0;
	common.current = 0;
	common.ocram_ptr = OCRAM2_BASE;

	if ((res = sai_init()) < 0) {
		log_error("failed to initialize sai");
		return res;
	}

	if ((res = edma_init(edma_error_handler)) != 0) {
		log_error("failed to initialize edma");
		return res;
	}

	if((res = edma_configure()) != 0) {
		log_error("failed to configure edma");
		return res;
	}

	if ((res = ad7779_init()) < 0) {
		log_error("failed to initialize ad7779 (%d)", res);
		return res;
	}

	if ((res = dev_init()) < 0) {
		log_error("device initialization failed");
		return res;
	}

	return 0;
}

int main(void)
{
	oid_t root;

	/* Temporary fix for OCRAM allocation */
	ad7779_enableCache(1);

	/* Wait for the filesystem */
	while (lookup("/", NULL, &root) < 0)
		usleep(10000);

	if (init())
		return -EIO;

	msg_loop();

	/* Should never be reached */
	log_error("Exiting!");
	return 0;
}
