/*
 * Phoenix-RTOS
 *
 * i.MX RT1170 ADE7913 test application
 *
 * Copyright 2021 Phoenix Systems
 * Author: Marcin Baran
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <sys/file.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/stat.h>
#include <sys/threads.h>

#include <imxrt-multi.h>
#ifdef TARGET_IMXRT1170
#include <phoenix/arch/imxrt1170.h>
#endif

#include <edma.h>

#include "gpio.h"
#include "ade7913.h"
#include "adc-api.h"

#define COL_RED    "\033[1;31m"
#define COL_CYAN   "\033[1;36m"
#define COL_NORMAL "\033[0m"

#define LOG_TAG "ade7913: "

#ifdef NDEBUG
#define log_debug(fmt, ...)
#else
#define log_debug(fmt, ...) \
	do { \
		printf(LOG_TAG fmt "\n", ##__VA_ARGS__); \
	} while (0)
#endif

#define log_info(fmt, ...) \
	do { \
		printf(LOG_TAG COL_CYAN fmt COL_NORMAL "\n", ##__VA_ARGS__); \
	} while (0)
#define log_error(fmt, ...) \
	do { \
		printf(LOG_TAG COL_RED fmt COL_NORMAL "\n", ##__VA_ARGS__); \
	} while (0)


/*
 * The buffer needs to be aligned with:
 * num_of_devices * bytes_per_device * num_of_buffers
 * so that in each filled buffer there will be same amount
 * of samples for each device.
 */
#define ADC_BUFFER_SIZE             (16 * _PAGE_SIZE)
#define NUM_OF_BUFFERS              (4)

#define ADE7913_NUM_OF_BITS         24

#define MAX_INIT_TRIES              (4)

#define DREADY_DMA_CHANNEL          5
#define GPIO_PIN_INTERRUPT          105 + 16
#define SPI_RCV_DMA_CHANNEL         6
#define SPI_RCV_DMA_REQUEST         36
#define TCD_CSR_INTMAJOR_BIT        (1 << 1)
#define TCD_CSR_ESG_BIT             (1 << 4)

enum { gpio_dr = 0, gpio_gdir, gpio_psr, gpio_icr1, gpio_icr2, gpio_imr, gpio_isr, gpio_edge_sel };

enum { spi_verid = 0, spi_param, spi_cr = 0x4, spi_sr, spi_ier, spi_der, spi_cfgr0, spi_cfgr1, spi_dmr0 = 0xc,
	   spi_dmr1, spi_ccr = 0x10, spi_fcr = 0x16, spi_fsr, spi_tcr, spi_tdr, spi_rsr = 0x1c, spi_rdr };


static const uint32_t adc_read_cmd_lookup[4] = { (0x4 << 24) | 0xFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };

static uint32_t spi_write_cmd_lookup[4] = { (spi_mode_3 << 30) | (1 << 27) | (0 << 24) | (spi_msb << 23) | (16 * 8 - 1),
	(spi_mode_3 << 30) | (1 << 27) | (1 << 24) | (spi_msb << 23) | (16 * 8 - 1),
	(spi_mode_3 << 30) | (1 << 27) | (2 << 24) | (spi_msb << 23) | (16 * 8 - 1),
	(spi_mode_3 << 30) | (1 << 27) | (3 << 24) | (spi_msb << 23) | (16 * 8 - 1) };


struct {
	int spi;
	int devcnt;
	oid_t ade7913_spi;
	const char *order;

	uint32_t *buff;
	volatile uint32_t *gpio3_ptr;
	volatile uint32_t *spi1_ptr;
	uint32_t port;


	volatile uint32_t edma_transfers;
	addr_t buffer_paddr;

	volatile struct edma_tcd_s tcds[8 + NUM_OF_BUFFERS];

	handle_t gpio_irq_cond, gpio_irq_lock, gpio_irq_handle;
	handle_t edma_spi_rcv_cond, edma_spi_rcv_lock, edma_spi_ch_handle;
	handle_t dready_cond, dready_handle;

	int enabled;
} common;


static void gpio_clear_interrupt(int pin)
{
	if (pin > 31)
		return;
	*(common.gpio3_ptr + gpio_isr) |= (0b1 << pin);
}


static int gpio_irq_handler(unsigned int n, void *arg)
{
	gpio_clear_interrupt(20);
	edma_software_request_start(DREADY_DMA_CHANNEL);

	return 0;
}

static int edma_spi_tx_irq_handler(unsigned int n, void *arg)
{
	edma_clear_interrupt(DREADY_DMA_CHANNEL);
	edma_software_request_start(DREADY_DMA_CHANNEL);

	return 0;
}


static int edma_spi_rcv_irq_handler(unsigned int n, void *arg)
{
	++common.edma_transfers;

	edma_clear_interrupt(SPI_RCV_DMA_CHANNEL);
	return 0;
}


static int edma_error_handler(unsigned int n, void *arg)
{
	edma_clear_error(DREADY_DMA_CHANNEL);

	return 0;
}


static int gpio_init(void)
{
	int res;
	platformctl_t pctl;

	gpio_setDir(&common.ade7913_spi, id_gpio3, 20, 0);

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_ad_21;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 5;
	platformctl(&pctl);

	if ((res = mutexCreate(&common.gpio_irq_lock)) != 0) {
		log_error("Mutex resource creation failed");
		return res;
	}

	if ((res = condCreate(&common.gpio_irq_cond)) != 0) {
		log_error("Conditional resource creation failed");
		resourceDestroy(common.gpio_irq_lock);
		return res;
	}

	/* Set dready GPIO interrupt */
	*(common.gpio3_ptr + gpio_icr2) |= (0b11 << 8);

	interrupt(GPIO_PIN_INTERRUPT,
		gpio_irq_handler, NULL, common.gpio_irq_cond, &common.gpio_irq_handle);

	return 0;
}


static int spi_init(oid_t *device, int spi)
{
	msg_t msg;
	multi_i_t *imsg = (multi_i_t *)msg.i.raw;
	char name[10];

	snprintf(name, sizeof(name), "/dev/spi%d", spi);

	while (lookup(name, NULL, device) < 0)
		usleep(5000);

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg->id = id_spi1 + spi - 1;
	imsg->spi.type = spi_config;
	imsg->spi.config.cs = 0;
	imsg->spi.config.mode = spi_mode_3;
	imsg->spi.config.endian = spi_msb;
	imsg->spi.config.sckDiv = 0;
	imsg->spi.config.prescaler = 1;

	return msgSend(device->port, &msg);
}


static int edma_configure(void)
{
	int devnum, res, i;
	size_t size = ADC_BUFFER_SIZE;
	oid_t *oid = OID_NULL;

	if ((res = mutexCreate(&common.edma_spi_rcv_lock)) != 0) {
		log_error("Mutex resource creation failed");
		return res;
	}

	if ((res = condCreate(&common.edma_spi_rcv_cond)) != 0) {
		log_error("Conditional resource creation failed");
		resourceDestroy(common.edma_spi_rcv_lock);
		return res;
	}

	if ((res = condCreate(&common.dready_cond)) != 0) {
		log_error("Conditional resource creation failed");
		resourceDestroy(common.edma_spi_rcv_lock);
		resourceDestroy(common.edma_spi_rcv_cond);
		return res;
	}

	common.buff = mmap(NULL, (size + _PAGE_SIZE - 1) / _PAGE_SIZE * _PAGE_SIZE,
		PROT_READ | PROT_WRITE, MAP_UNCACHED, oid, common.buffer_paddr);

	if (common.buff == MAP_FAILED) {
		log_error("Edma buffers allocation failed");
		resourceDestroy(common.edma_spi_rcv_lock);
		resourceDestroy(common.edma_spi_rcv_lock);
		resourceDestroy(common.dready_cond);

		return -1;
	}

	/* Set request commands order */
	for (i = 0; i < common.devcnt; ++i) {
		devnum = (int)(common.order[i] - '0');
		spi_write_cmd_lookup[i] = (spi_write_cmd_lookup[i] & ~(0x3000000)) | (devnum << 24);
	}

	common.buffer_paddr = va2pa(common.buff);

	memset(common.buff, 0, size);

	uint8_t xfer_size = sizeof(uint32_t);

	common.tcds[0].soff = 0;
	common.tcds[0].attr = (edma_get_tcd_attr_xsize(xfer_size) << 8) |
		edma_get_tcd_attr_xsize(xfer_size);

	/* Number of bytes per minor loop iteration */
	common.tcds[0].nbytes_mlnoffno = xfer_size;
	common.tcds[0].slast = 0;
	common.tcds[0].doff = 0;
	common.tcds[0].dlast_sga = (uint32_t)&common.tcds[1];

	/* Number of major loop iterations */
	common.tcds[0].biter_elinkno = 1;
	common.tcds[0].citer_elinkno = common.tcds[0].biter_elinkno;

	/* Set addrs for the TCD. */
	common.tcds[0].saddr = (uint32_t)&spi_write_cmd_lookup[0];
	common.tcds[0].daddr = (uint32_t)(common.spi1_ptr + spi_tcr);

	/* Enable major loop finish interrupt and scatter-gather */
	common.tcds[0].csr = TCD_CSR_ESG_BIT | TCD_CSR_INTMAJOR_BIT;

	common.tcds[1].soff = xfer_size;
	common.tcds[1].attr = (edma_get_tcd_attr_xsize(sizeof(uint32_t)) << 8) |
		edma_get_tcd_attr_xsize(sizeof(uint32_t));

	/* Number of bytes per minor loop iteration */
	common.tcds[1].nbytes_mlnoffno = 4 * xfer_size;
	common.tcds[1].slast = -xfer_size;
	common.tcds[1].doff = 0;
	common.tcds[1].dlast_sga = (uint32_t)&common.tcds[2];

	/* Number of major loop iterations */
	common.tcds[1].biter_elinkno = 1;
	common.tcds[1].citer_elinkno = common.tcds[1].biter_elinkno;

	/* Set addrs for the TCD. */
	common.tcds[1].saddr = (uint32_t)adc_read_cmd_lookup;
	common.tcds[1].daddr = (uint32_t)(common.spi1_ptr + spi_tdr);

	/* Enable major loop finish interrupt and scatter-gather */
	common.tcds[1].csr = TCD_CSR_ESG_BIT | TCD_CSR_INTMAJOR_BIT;

	for (i = 0; i < common.devcnt - 1; ++i) {
		edma_copy_tcd(&common.tcds[0], &common.tcds[2 * i + 2]);
		common.tcds[2 * i + 2].dlast_sga = (uint32_t)(&common.tcds[2 * i + 3]);
		common.tcds[2 * i + 2].saddr = (uint32_t)&spi_write_cmd_lookup[i + 1];

		edma_copy_tcd(&common.tcds[1], &common.tcds[2 * i + 3]);
		common.tcds[2 * i + 3].dlast_sga = (uint32_t)(&common.tcds[2 * i + 4]);
	}

	common.tcds[2 * common.devcnt - 1].dlast_sga = (uint32_t)(&common.tcds[0]);
	common.tcds[2 * common.devcnt - 1].csr = TCD_CSR_ESG_BIT;

	common.tcds[8].soff = 0;
	common.tcds[8].attr = (edma_get_tcd_attr_xsize(xfer_size) << 8) |
		edma_get_tcd_attr_xsize(xfer_size);
	common.tcds[8].saddr = (uint32_t)(common.spi1_ptr + spi_rdr);
	common.tcds[8].soff = 0;
	common.tcds[8].slast = 0;
	common.tcds[8].nbytes_mlnoffno = xfer_size;
	common.tcds[8].doff = xfer_size;
	common.tcds[8].daddr = (uint32_t)common.buff;
	common.tcds[8].dlast_sga = (uint32_t)&common.tcds[9];
	common.tcds[8].biter_elinkno =
		ADC_BUFFER_SIZE / common.tcds[8].nbytes_mlnoffno / NUM_OF_BUFFERS;
	common.tcds[8].citer_elinkno = common.tcds[8].biter_elinkno;
	common.tcds[8].csr = TCD_CSR_INTMAJOR_BIT | TCD_CSR_ESG_BIT;

	for (i = 1; i < NUM_OF_BUFFERS; ++i) {
		edma_copy_tcd(&common.tcds[8], &common.tcds[8 + i]);
		common.tcds[8 + i].daddr = (uint32_t)common.buff + i * ADC_BUFFER_SIZE / NUM_OF_BUFFERS;
		common.tcds[8 + i].dlast_sga = (uint32_t)&common.tcds[8 + ((i + 1) % NUM_OF_BUFFERS)];
	}

	if ((res = edma_install_tcd(&common.tcds[0], DREADY_DMA_CHANNEL)) != 0 ||
		(res = edma_install_tcd(&common.tcds[8], SPI_RCV_DMA_CHANNEL)) != 0) {
		log_error("Edma buffer installation failed");
		munmap(common.buff, (size + _PAGE_SIZE - 1) / _PAGE_SIZE * _PAGE_SIZE);

		resourceDestroy(common.edma_spi_rcv_lock);
		resourceDestroy(common.edma_spi_rcv_lock);
		resourceDestroy(common.dready_cond);

		return res;
	}

	interrupt(EDMA_CHANNEL_IRQ(SPI_RCV_DMA_CHANNEL),
		edma_spi_rcv_irq_handler, NULL, common.edma_spi_rcv_cond, &common.edma_spi_ch_handle);

	interrupt(EDMA_CHANNEL_IRQ(DREADY_DMA_CHANNEL),
		edma_spi_tx_irq_handler, NULL, common.dready_cond, &common.dready_handle);

	dmamux_set_source(SPI_RCV_DMA_CHANNEL, SPI_RCV_DMA_REQUEST);
	dmamux_channel_enable(SPI_RCV_DMA_CHANNEL);
	edma_channel_enable(SPI_RCV_DMA_CHANNEL);
	edma_channel_enable(DREADY_DMA_CHANNEL);

	return 0;
}


static int adc_init(int hard)
{
	int i, res, devnum;

	/* Start init from device with xtal */
	for (i = 0; i < common.devcnt; ++i) {
		devnum = (int)(common.order[i] - '0');

		log_info("Configuring ADE7913 device nr %d", devnum);
		while (ade7913_init(&common.ade7913_spi, devnum,
				   devnum == common.order[common.devcnt - 1] - '0' ? 0 : 1) < 0) {
			log_error("Failed to initialize ADE7913 nr %d", devnum);
			usleep(500000);
		}

		if (ade7913_enable(&common.ade7913_spi, devnum) < 0)
			log_error("Could not enable ADE7913 nr %d", devnum);

		/* Wait for next adc to start */
		usleep(50000);
	}

	if ((res = ade7913_sync(&common.ade7913_spi, common.order, common.devcnt, 0)) < 0) {
		log_error("Could not synchronize ADE7913 devices: %s", strerror(res));
		return -1;
	}

	for (i = 0; i < common.devcnt; ++i) {
		devnum = (int)(common.order[i] - '0');

		if (ade7913_lock(&common.ade7913_spi, devnum) < 0)
			log_error("Could not lock ADE7913 nr %d", devnum);
	}

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
	int res = 0;

	if (data != NULL && size != sizeof(unsigned))
		return -EIO;

	mutexLock(common.edma_spi_rcv_lock);
	if ((res = condWait(common.edma_spi_rcv_cond, common.edma_spi_rcv_lock, 1000000)) == 0)
		*(uint32_t **)data = (uint32_t *)&common.edma_transfers;
	mutexUnlock(common.edma_spi_rcv_lock);

	return res;
}

static int dev_ctl(msg_t *msg)
{
	int devnum, res, i;
	adc_dev_ctl_t dev_ctl;

	memcpy(&dev_ctl, msg->o.raw, sizeof(adc_dev_ctl_t));

	switch (dev_ctl.type) {
		case adc_dev_ctl__enable:
			common.enabled = 1;
			edma_channel_enable(SPI_RCV_DMA_CHANNEL);
			edma_channel_enable(DREADY_DMA_CHANNEL);
			return EOK;

		case adc_dev_ctl__disable:
			common.enabled = 0;
			edma_channel_disable(DREADY_DMA_CHANNEL);
			edma_channel_disable(SPI_RCV_DMA_CHANNEL);
			return EOK;

		case adc_dev_ctl__reset:
			return adc_init(dev_ctl.reset_hard);

		case adc_dev_ctl__set_config:
			if (common.enabled)
				return -EBUSY;

			for (i = 0; i < common.devcnt; ++i) {
				devnum = (int)(common.order[i] - '0');

				if (ade7913_unlock(&common.ade7913_spi, devnum) < 0)
					return -EAGAIN;

				res = ade7913_set_sampling_rate(&common.ade7913_spi, devnum, (int)dev_ctl.config.sampling_rate);

				if (ade7913_lock(&common.ade7913_spi, devnum) < 0)
					return -EIO;

				if (res < 0)
					return -EINVAL;
			}

			return EOK;

		case adc_dev_ctl__get_config:
			res = ade7913_get_sampling_rate(&common.ade7913_spi,
				(int)(common.order[0] - '0'), (int *)&dev_ctl.config.sampling_rate);

			dev_ctl.config.bits = ADE7913_NUM_OF_BITS;

			if (res < 0)
				return -EIO;

			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));

			return EOK;

		case adc_dev_ctl__get_buffers:
			dev_ctl.buffers.paddr = common.buffer_paddr;
			dev_ctl.buffers.num = NUM_OF_BUFFERS;
			dev_ctl.buffers.size = ADC_BUFFER_SIZE;
			memcpy(msg->o.raw, &dev_ctl, sizeof(adc_dev_ctl_t));
			return EOK;

		case adc_dev_ctl__set_channel_config:
		case adc_dev_ctl__get_channel_config:
		case adc_dev_ctl__set_channel_gain:
		case adc_dev_ctl__get_channel_gain:
		case adc_dev_ctl__set_channel_calib:
		case adc_dev_ctl__get_channel_calib:
		case adc_dev_ctl__status:
		case adc_dev_ctl__set_adc_mux:
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
	unsigned long rid;

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


static int init(void)
{
	int res;

	common.enabled = 0;
	common.edma_transfers = 0;

	do {
		if ((res = spi_init(&common.ade7913_spi, common.spi)) < 0) {
			log_error("failed to initialize spi%d", 1);
			break;
		}

		if ((res = adc_init(0)) < 0) {
			log_error("failed to initialize ade7193: (%d)", res);
			break;
		}

		if ((res = gpio_init()) < 0) {
			log_error("failed to initialize gpio");
			resourceDestroy(common.gpio_irq_lock);
			resourceDestroy(common.gpio_irq_cond);

			break;
		}

		if ((res = edma_init(edma_error_handler)) != 0) {
			log_error("failed to initialize edma");
			resourceDestroy(common.gpio_irq_lock);
			resourceDestroy(common.gpio_irq_cond);

			break;
		}

		if ((res = edma_configure()) != 0) {
			resourceDestroy(common.gpio_irq_lock);
			resourceDestroy(common.gpio_irq_cond);

			log_error("failed to configure edma");
			break;
		}

		if ((res = dev_init()) < 0) {
			log_error("device initialization failed");
			resourceDestroy(common.gpio_irq_lock);
			resourceDestroy(common.gpio_irq_cond);
			resourceDestroy(common.edma_spi_rcv_lock);
			resourceDestroy(common.edma_spi_rcv_lock);
			resourceDestroy(common.dready_cond);
			munmap(common.buff, (ADC_BUFFER_SIZE + _PAGE_SIZE - 1) / _PAGE_SIZE * _PAGE_SIZE);

			break;
		}

		/* Enable SPI edma receive request */
		*(common.spi1_ptr + spi_der) = (1 << 1);
		/* Enable GPIO DREADY interrupt */
		*(common.gpio3_ptr + gpio_imr) |= (0b1 << 20);
	} while (0);

	return res;
}


int main(int argc, char **argv)
{
	int i;
	oid_t root;

	common.gpio3_ptr = (void *)0x40134000;
	common.spi1_ptr = (void *)0x40114000;

	if (argc != 3) {
		log_error("No device list or no SPI device given");
		return -EINVAL;
	}
	else {
		common.order = argv[1];
		common.devcnt = strlen(argv[1]);
		common.spi = atoi(argv[2]);

		for (i = 0; i < common.devcnt; ++i) {
			if ((int)(common.order[i] - '0') >= common.devcnt || (int)(common.order[i] - '0') < 0) {
				log_error("Wrong order format provided");
				return -1;
			}
		}

		if (common.spi < 0) {
			log_error("Wrong spi number provided");
			return -1;
		}

		log_info("Device order: %s", common.order);
	}

	if (common.devcnt > 4) {
		log_error("Incorrect ADE7913 device count (4 max)");
		return -EINVAL;
	}

	/* Wait for the filesystem */
	while (lookup("/", NULL, &root) < 0)
		usleep(10000);

	for (i = 0; i < MAX_INIT_TRIES; ++i) {
		if (!init())
			break;

		usleep(100000);
	}

	if (i == MAX_INIT_TRIES) {
		log_error("could not init driver.");
		return -EIO;
	}

	msg_loop();

	/* Should never be reached */
	log_error("Exiting!");
	return 0;
}
