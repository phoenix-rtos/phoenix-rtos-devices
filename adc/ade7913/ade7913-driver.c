/*
 * Phoenix-RTOS
 *
 * i.MX RT1170 ADE7913 polyphase ADC driver
 *
 * Copyright 2021-2022 Phoenix Systems
 * Author: Marcin Baran, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <sys/msg.h>
#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/platform.h>
#include <sys/interrupt.h>
#include <posix/utils.h>

#include <imxrt-multi.h>
#include <phoenix/arch/imxrt1170.h>

#include <edma.h>
#include <board_config.h>

#include "ade7913.h"
#include "adc-api-ade7913.h"
#include "flexpwm.h"


#define COL_RED    "\033[1;31m"
#define COL_CYAN   "\033[1;36m"
#define COL_NORMAL "\033[0m"

#define LOG_TAG "ade7913: "

/* clang-format off */
#ifdef NDEBUG
#define log_debug(fmt, ...)
#else
#define log_debug(fmt, ...) do { printf(LOG_TAG fmt "\n", ##__VA_ARGS__); } while (0)
#endif

#define log_info(fmt, ...)  do { printf(LOG_TAG COL_CYAN fmt COL_NORMAL "\n", ##__VA_ARGS__); } while (0)
#define log_error(fmt, ...) do { printf(LOG_TAG COL_RED fmt COL_NORMAL "\n", ##__VA_ARGS__); } while (0)
/* clang-format on */


#ifndef ADE7913_PRIO
#define ADE7913_PRIO 4
#endif

/* ADE7913_BUF_NUM - number of dma buffers, needs to be power of two */
#ifndef ADE7913_BUF_NUM
#define ADE7913_BUF_NUM 4
#endif

/*
 * The buffer needs to be aligned with:
 * num_of_devices * bytes_per_device * num_of_buffers
 * so that in each filled buffer there will be same amount
 * of samples for each device.
 */
#define ADC_BUFFER_SIZE (4 * (ADE7913_BUF_NUM) * _PAGE_SIZE)

#define DREADY_DMA_CHANNEL  5
#define SPI_RCV_DMA_CHANNEL 6
#define SPI_SND_DMA_CHANNEL 7
#define SEQ_DMA_CHANNEL     8
#define SPI_RCV_DMA_REQUEST 36

#define PWM4SM3_READ_DMA_REQUEST 112
#define PWM4_BASE_ADDR           ((void *)0x40198000)

#define TCD_CSR_INTMAJOR_BIT     (1 << 1)
#define TCD_CSR_INTHALF_BIT      (1 << 2)
#define TCD_CSR_DREQ_BIT         (1 << 3)
#define TCD_CSR_ESG_BIT          (1 << 4)
#define TCD_CSR_MAJORELINK_BIT   (1 << 5)
#define TCD_CSR_MAJORLINK_CH(ch) ((((ch)&0x1f) << 8) | TCD_CSR_MAJORELINK_BIT)
#define E_LINK_BIT               (1 << 15)
#define E_LINK_CH(ch)            ((((ch)&0x1f) << 9) | E_LINK_BIT)

/* clang-format off */

/* GPIO */
enum { gpio_dr = 0, gpio_gdir, gpio_psr, gpio_icr1, gpio_icr2, gpio_imr, gpio_isr, gpio_edge_sel, gpio_dr_set,
	gpio_dr_clear, gpio_dr_toggle };


/* SPI */
enum { spi_verid = 0, spi_param, spi_cr = 0x4, spi_sr, spi_ier, spi_der, spi_cfgr0, spi_cfgr1, spi_dmr0 = 0xc,
	spi_dmr1, spi_ccr = 0x10, spi_fcr = 0x16, spi_fsr, spi_tcr, spi_tdr, spi_rsr = 0x1c, spi_rdr };

static const uint32_t spi_base[] = { 0x40114000, 0x40118000, 0x4011c000, 0x40120000, 0x40c2c000, 0x40c30000 };

static const int spi_clk[] = { pctl_clk_lpspi1, pctl_clk_lpspi2, pctl_clk_lpspi3, pctl_clk_lpspi4,
	pctl_clk_lpspi5, pctl_clk_lpspi6 };

/* clang-format on */


/* ADE7913 commands LUT used by DMA */
static const uint32_t adc_read_cmd_lookup[4] = { 0xFFFFFF04, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };

static uint32_t spi_write_cmd_lookup[4] = {
	((uint32_t)spi_mode_3 << 30) | (1 << 27) | (0 << 24) | (spi_msb << 23) | (1 << 22) | (16 * 8 - 1),
	((uint32_t)spi_mode_3 << 30) | (1 << 27) | (1 << 24) | (spi_msb << 23) | (1 << 22) | (16 * 8 - 1),
	((uint32_t)spi_mode_3 << 30) | (1 << 27) | (2 << 24) | (spi_msb << 23) | (1 << 22) | (16 * 8 - 1),
	((uint32_t)spi_mode_3 << 30) | (1 << 27) | (3 << 24) | (spi_msb << 23) | (1 << 22) | (16 * 8 - 1)
};


struct {
	int spi;
	int devcnt;
	oid_t ade7913_spi;
	const char *order;

	uint32_t *buff;
	volatile uint32_t *spi_ptr;
	volatile uint32_t *gpio3_ptr;
	volatile uint32_t *iomux_ptr[4];
	volatile struct edma_tcd_s *tcd_dready_ptr;
	volatile struct edma_tcd_s *tcd_spisnd_ptr;
	volatile struct edma_tcd_s *tcd_spircv_ptr;
	volatile struct edma_tcd_s *tcd_seq_ptr;

	volatile struct edma_tcd_s tcds[5 + 4 * 4 + ADE7913_BUF_NUM];
	volatile uint32_t edma_transfers;
	addr_t buffer_paddr;

	handle_t edma_spi_rcv_cond, edma_spi_rcv_lock, edma_spi_ch_handle;
	handle_t dready_cond;
	oid_t oid;

	int enabled;
} common;


static int edma_spi_rcv_irq_handler(unsigned int n, void *arg)
{
	struct edma_tcd_s tcd;
	uint32_t reg, mux_copy[4];
	int i, notsync = 0;

	/*
	 * If there is still some data to be sent by the SPI module (while we should already
	 * received the last frame) it is a sign that DMA TCD has been shifted by interference
	 * (EFT/Burst testing) possibly due to /DREADY signal loss, DMA TCD needs to be
	 * adjusted again. Before sending SYNC broadcast, wait until SPI module is idle.
	 */
	while (*(common.spi_ptr + spi_sr) & (1 << 24)) {
		notsync = 1;
	}

	/* Get SPI muxes */
	for (i = 0; i < common.devcnt; ++i) {
		mux_copy[i] = *common.iomux_ptr[i];
	}

	/* Set CS GPIOs to inactive */
	*(common.gpio3_ptr + gpio_gdir) |= (1 << 17) | (1 << 18) | (1 << 19) | (1 << 28);

	/* Set CS GPIOs to active */
	reg = *(common.gpio3_ptr + gpio_dr);
	*(common.gpio3_ptr + gpio_dr) = reg & ~((1 << 17) | (1 << 18) | (1 << 19) | (1 << 28));

	/* Set muxes alt. mode to gpio */
	for (i = 0; i < common.devcnt; ++i) {
		*common.iomux_ptr[i] = 5;
	}

	/* Send SYNC broadcast */
	*(common.spi_ptr + spi_tcr) = ((uint32_t)spi_mode_3 << 30) | (1 << 27) | (1 << 26) | (spi_msb << 23) | (1 << 22) | (1 << 19) | (2 * 8 - 1);
	*(common.spi_ptr + spi_tdr) = 0x0258ffff;

	/* wait until broadcast is transmitted */
	while ((*(common.spi_ptr + spi_fsr) & 0x1f) || (*(common.spi_ptr + spi_sr) & (1 << 24)))
		;

	/* Set muxes mode back to previous mode (SPI CS) */
	for (i = 0; i < common.devcnt; ++i) {
		*common.iomux_ptr[i] = mux_copy[i];
	}

	/* Re-enable /DREADY after SYNC broadcast */
	edma_install_tcd(common.tcd_dready_ptr, DREADY_DMA_CHANNEL);
	edma_channel_enable(DREADY_DMA_CHANNEL);

	if (notsync == 0) {
		++common.edma_transfers;
	}
	else {
		/* If not in-sync adjust DMA TCD (as described above) */
		edma_install_tcd(common.tcd_spircv_ptr, SPI_RCV_DMA_CHANNEL);
		edma_channel_enable(SPI_RCV_DMA_CHANNEL);
		edma_read_tcd(&tcd, SPI_RCV_DMA_CHANNEL);

		common.edma_transfers += ADE7913_BUF_NUM;
		common.edma_transfers &= ~(ADE7913_BUF_NUM - 1);
		common.edma_transfers += ((((tcd.daddr + tcd.doff) - (uint32_t)common.buff) & ~(ADC_BUFFER_SIZE / (ADE7913_BUF_NUM) - 1)) >> 11);
	}

	edma_clear_interrupt(SPI_RCV_DMA_CHANNEL);

	return 0;
}


static int edma_error_handler(unsigned int n, void *arg)
{
	const uint32_t mask = 1U << DREADY_DMA_CHANNEL;

	if (edma_error_channel() & mask) {
		edma_clear_error(DREADY_DMA_CHANNEL);
	}

	return EOK;
}


static int pwm_init(void)
{
	int res;
	platformctl_t pctl = { 0 };

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;
	pctl.iomux.mux = pctl_mux_gpio_ad_21;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 11; /* FLEXPWM4 (PWM3_X) */

	res = platformctl(&pctl);
	if (res < 0) {
		log_error("Unable to set IOMUX");
		return res;
	}

	flexpwm_init(PWM4_BASE_ADDR);

	/* Enable PWM in input capture mode to trigger on /DREADY signal */
	res = flexpwm_input_capture(3, FLEXPWM_CAP_EDGE_FALLING, FLEXPWM_CAP_DISABLED);
	if (res < 0) {
		log_error("Unable to set input capture");
	}

	return res;
}


static int common_setClock(int clock, int div, int mux, int mfd, int mfn, int state)
{
	platformctl_t pctl = {
		.action = pctl_get,
		.type = pctl_devclock,
		.devclock = { .dev = clock },
	};

	int res = platformctl(&pctl);
	if (res < 0) {
		return res;
	}

	pctl.action = pctl_set;

	if (div >= 0) {
		pctl.devclock.div = div;
	}

	if (mux >= 0) {
		pctl.devclock.mux = mux;
	}

	if (mfd >= 0) {
		pctl.devclock.mfd = mfd;
	}

	if (mfn >= 0) {
		pctl.devclock.mfn = mfn;
	}

	if (state >= 0) {
		pctl.devclock.state = state;
	}

	return platformctl(&pctl);
}


static int spi_init(oid_t *device, int spi)
{
	int res;
	msg_t msg;
	char name[10];
	multi_i_t *imsg = (multi_i_t *)msg.i.raw;

	if (spi < 1 || spi > sizeof(spi_clk) / sizeof(spi_clk[0])) {
		return -ENXIO;
	}

	snprintf(name, sizeof(name), "/dev/spi%d", spi);

	while (lookup(name, NULL, device) < 0) {
		usleep(100 * 1000);
	}

	spi--;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg->id = id_spi1 + spi;
	imsg->spi.type = spi_config;
	imsg->spi.config.cs = 0;
	imsg->spi.config.mode = spi_mode_3;
	imsg->spi.config.endian = spi_msb;
	imsg->spi.config.sckDiv = 0;
	imsg->spi.config.prescaler = 1;

	res = msgSend(device->port, &msg);
	if (res < 0) {
		return res;
	}

	/*
	 * Set SCLK to 5.33 MHz (max allowed is 5.6MHz), assuming maximum of 4 ADC @ 8kHz sampling rate:
	 * 4 x (128bit burst packet with 4us spacing) + (16bit broadcast packet with 4us spacing)= ~5.29MHz
	 */
	return common_setClock(spi_clk[spi], 26, 6, -1, -1, 1);
}


static int adc_init(int hard)
{
	int i, res, devnum, isclkout;

	for (i = 0; hard && i < common.devcnt; ++i) {
		devnum = (int)(common.order[i] - '0');

		if (ade7913_reset_hard(&common.ade7913_spi, devnum) < 0) {
			log_error("Could reset ADE7913 no. %d", devnum);
		}
	}

	/* Start init from device with xtal */
	for (i = 0; i < common.devcnt; ++i) {
		devnum = (int)(common.order[i] - '0');
		/* the last ADC is DREADY and the other CLOCK clock output (daisy chain) */
		isclkout = (i != common.devcnt - 1);

		log_info("Configuring ADE7913 device no. %d", devnum);
		while (ade7913_init(&common.ade7913_spi, devnum, isclkout) < 0) {
			log_error("Failed to initialize ADE7913 no. %d", devnum);
			usleep(500 * 1000);
		}

		if (ade7913_enable(&common.ade7913_spi, devnum) < 0) {
			log_error("Could not enable ADE7913 no. %d", devnum);
			return -1;
		}

		/* V2 channel on neutral wire is used to measure ADE7913 temperature */
		if ((devnum == 0) && (ade7913_temperature(&common.ade7913_spi, devnum) < 0)) {
			log_error("Could not enable temperature channel no. %d", devnum);
			return -1;
		}

		if (ade7913_emi(&common.ade7913_spi, devnum, (devnum % 2 ? 0xaa : 0x55)) < 0) {
			log_error("Could not set EMI reg of ADE7913 no. %d", devnum);
			return -1;
		}

		/* Wait for next ADC to start */
		usleep(50 * 1000);
	}

	res = ade7913_sync(&common.ade7913_spi, common.order, common.devcnt, 0);
	if (res < 0) {
		log_error("Could not synchronize ADE7913 devices (%d)", res);
		return -1;
	}

	for (i = 0; i < common.devcnt; ++i) {
		devnum = (int)(common.order[i] - '0');

		if (ade7913_lock(&common.ade7913_spi, devnum) < 0) {
			log_error("Could not lock ADE7913 no. %d", devnum);
			return -1;
		}
	}

	return EOK;
}


static void dma_stop(void)
{
	/* Disable SPI eDMA receive request */
	*(common.spi_ptr + spi_der) &= ~(1 << 1);

	dmamux_channel_disable(DREADY_DMA_CHANNEL);
	dmamux_channel_disable(SEQ_DMA_CHANNEL);
	dmamux_channel_disable(SPI_SND_DMA_CHANNEL);
	dmamux_channel_disable(SPI_RCV_DMA_CHANNEL);

	edma_channel_disable(DREADY_DMA_CHANNEL);
	edma_channel_disable(SEQ_DMA_CHANNEL);
	edma_channel_disable(SPI_SND_DMA_CHANNEL);
	edma_channel_disable(SPI_RCV_DMA_CHANNEL);
}

static void dma_start(void)
{
	dmamux_set_source(SPI_RCV_DMA_CHANNEL, SPI_RCV_DMA_REQUEST);
	dmamux_set_source(DREADY_DMA_CHANNEL, PWM4SM3_READ_DMA_REQUEST);

	dmamux_channel_enable(SPI_RCV_DMA_CHANNEL);
	dmamux_channel_enable(SPI_SND_DMA_CHANNEL);
	dmamux_channel_enable(SEQ_DMA_CHANNEL);
	dmamux_channel_enable(DREADY_DMA_CHANNEL);

	edma_channel_enable(SPI_RCV_DMA_CHANNEL);
	edma_channel_enable(SPI_SND_DMA_CHANNEL);
	edma_channel_enable(SEQ_DMA_CHANNEL);
	edma_channel_enable(DREADY_DMA_CHANNEL);

	/* Enable SPI eDMA receive request */
	*(common.spi_ptr + spi_der) |= (1 << 1);
}


static int dma_setup_tcds(void)
{
	int cs_seq, res, i, tcd_count;

	common.tcds[0].soff = 0;
	common.tcds[0].attr = (edma_get_tcd_attr_xsize(sizeof(uint32_t)) << 8) |
		edma_get_tcd_attr_xsize(sizeof(uint32_t));

	/* Number of bytes per minor loop iteration */
	common.tcds[0].nbytes_mlnoffno = sizeof(uint32_t);
	common.tcds[0].slast = 0;
	common.tcds[0].doff = 0;
	common.tcds[0].dlast_sga = (uint32_t)&common.tcds[1];

	/* Number of major loop iterations */
	common.tcds[0].biter_elinkno = 1;
	common.tcds[0].citer_elinkno = common.tcds[0].biter_elinkno;

	/* Set TCD addresses of the LUT for SPI transmit commands */
	common.tcds[0].saddr = (uint32_t)&spi_write_cmd_lookup[0];
	common.tcds[0].daddr = (uint32_t)(common.spi_ptr + spi_tcr);

	/* Enable scatter-gather and link with SPI send channel at major loop end */
	common.tcds[0].csr = TCD_CSR_ESG_BIT | TCD_CSR_MAJORLINK_CH(SPI_SND_DMA_CHANNEL);

	common.tcds[4].soff = sizeof(uint32_t);
	common.tcds[4].attr = (edma_get_tcd_attr_xsize(sizeof(uint32_t)) << 8) |
		edma_get_tcd_attr_xsize(sizeof(uint32_t));

	/* Number of bytes per minor loop iteration */
	common.tcds[4].nbytes_mlnoffno = 4 * sizeof(uint32_t);
	common.tcds[4].slast = -sizeof(uint32_t);
	common.tcds[4].doff = 0;
	common.tcds[4].dlast_sga = (uint32_t)&common.tcds[4];

	/* Number of major loop iterations */
	common.tcds[4].biter_elinkno = 1;
	common.tcds[4].citer_elinkno = common.tcds[4].biter_elinkno;

	/* Set TCD addresses of the LUT for ADE7913 burst read command */
	common.tcds[4].saddr = (uint32_t)adc_read_cmd_lookup;
	common.tcds[4].daddr = (uint32_t)(common.spi_ptr + spi_tdr);

	/* Enable scatter-gather */
	common.tcds[4].csr = TCD_CSR_ESG_BIT;

	/* Clone the above setup for each of the ADE7913 devices creating a ring */
	for (i = 1; i < common.devcnt; ++i) {
		edma_copy_tcd(&common.tcds[i], &common.tcds[0]);
		common.tcds[i].dlast_sga = (uint32_t)(&common.tcds[i + 1]);
		common.tcds[i].saddr = (uint32_t)&spi_write_cmd_lookup[i];
	}

	/* ... and close the ring */
	common.tcds[common.devcnt - 1].dlast_sga = (uint32_t)(&common.tcds[0]);

	/* Setup SPI receive buffers (for samples) */
	common.tcds[5].soff = 0;
	common.tcds[5].saddr = (uint32_t)(common.spi_ptr + spi_rdr);
	common.tcds[5].slast = 0;
	common.tcds[5].doff = sizeof(uint32_t);
	common.tcds[5].daddr = (uint32_t)common.buff;
	common.tcds[5].dlast_sga = (uint32_t)&common.tcds[6];

	common.tcds[5].nbytes_mlnoffno = sizeof(uint32_t);
	common.tcds[5].attr = (edma_get_tcd_attr_xsize(sizeof(uint32_t)) << 8) | edma_get_tcd_attr_xsize(sizeof(uint32_t));

	common.tcds[5].biter_elinkyes = ADC_BUFFER_SIZE / common.tcds[5].nbytes_mlnoffno / (ADE7913_BUF_NUM);
	common.tcds[5].biter_elinkyes &= ~E_LINK_CH(0xff);
	common.tcds[5].biter_elinkyes |= E_LINK_CH(SEQ_DMA_CHANNEL);
	common.tcds[5].citer_elinkyes = common.tcds[5].biter_elinkyes;
	common.tcds[5].csr = TCD_CSR_INTMAJOR_BIT | TCD_CSR_ESG_BIT | TCD_CSR_MAJORLINK_CH(SEQ_DMA_CHANNEL);

	/* Clone SPI receive buffer setup and make it a ring buffer */
	for (i = 1; i < ADE7913_BUF_NUM; ++i) {
		edma_copy_tcd(&common.tcds[5 + i], &common.tcds[5]);
		common.tcds[5 + i].daddr = (uint32_t)common.buff + i * ADC_BUFFER_SIZE / (ADE7913_BUF_NUM);
		common.tcds[5 + i].dlast_sga = (uint32_t)&common.tcds[5 + ((i + 1) % (ADE7913_BUF_NUM))];
	}

	/* Create chip-select sequencer triggered by /DREADY signal
	 *
	 * /DREADY                                                                             /DREADY          - DREADY_DMA_CHANNEL
	 * |                                                                                   |
	 * CS1  .    .    .     CS2  .    .    .     CS3  .    .    .     CS4  .    .    .     CS1              - SEQ_DMA_CHANNEL
	 * |                    |                    |                    |                    |
	 * .CMD FFFFFFFFFFFFFFF .CMD FFFFFFFFFFFFFFF .CMD FFFFFFFFFFFFFFF .CMD FFFFFFFFFFFFFFF .CMD FFFFF .... <- SPI_SND (burst read command)
	 * \.DAT .DAT .DAT .DAT \.DAT .DAT .DAT .DAT \.DAT .DAT .DAT .DAT \.DAT .DAT .DAT .DAT \.DAT .DAT .... -> SPI_RCV (to sample buffer)
	 *                                                                                    |
	 *                                                                                    !STOP            -! SEQ_DMA_CHANNEL
	 * (devcnt=4, three-phase meter L1,L2,L3,N)
	 *
	 */

	cs_seq = 5 + i;

	common.tcds[cs_seq].soff = 0;
	common.tcds[cs_seq].saddr = 0;
	common.tcds[cs_seq].slast = 0;
	common.tcds[cs_seq].doff = 0;
	common.tcds[cs_seq].daddr = 1;
	common.tcds[cs_seq].dlast_sga = (uint32_t)&common.tcds[cs_seq + 1];
	common.tcds[cs_seq].nbytes_mlnoffno = 1;
	common.tcds[cs_seq].attr = (edma_get_tcd_attr_xsize(1) << 8) | edma_get_tcd_attr_xsize(1);
	common.tcds[cs_seq].biter_elinkno = 1;
	common.tcds[cs_seq].citer_elinkno = common.tcds[cs_seq].biter_elinkno;
	common.tcds[cs_seq].csr = TCD_CSR_ESG_BIT;

	tcd_count = 4 * common.devcnt;
	for (i = 1; i < tcd_count; ++i) {
		edma_copy_tcd(&common.tcds[cs_seq + i], &common.tcds[cs_seq]);
		common.tcds[cs_seq + i].daddr = 0;
		common.tcds[cs_seq + i].dlast_sga = (uint32_t)&common.tcds[cs_seq + ((i + 1) % tcd_count)];

		if ((i + 1) != tcd_count && (i + 1) % 4 == 0) {
			common.tcds[cs_seq + i].biter_elinkyes = E_LINK_CH(DREADY_DMA_CHANNEL) | 1;
			common.tcds[cs_seq + i].citer_elinkyes = common.tcds[cs_seq + i].biter_elinkyes;
			common.tcds[cs_seq + i].csr = TCD_CSR_ESG_BIT | TCD_CSR_MAJORLINK_CH(DREADY_DMA_CHANNEL);
		}
	}
	/* end of sequencer */

	common.tcd_dready_ptr = &common.tcds[0];
	common.tcd_spisnd_ptr = &common.tcds[4];
	common.tcd_spircv_ptr = &common.tcds[5];
	common.tcd_seq_ptr = &common.tcds[cs_seq];

	common.edma_transfers = 0;

	res = edma_install_tcd(common.tcd_dready_ptr, DREADY_DMA_CHANNEL);
	if (res != 0) {
		return res;
	}

	res = edma_install_tcd(common.tcd_spisnd_ptr, SPI_SND_DMA_CHANNEL);
	if (res != 0) {
		return res;
	}

	res = edma_install_tcd(common.tcd_spircv_ptr, SPI_RCV_DMA_CHANNEL);
	if (res != 0) {
		return res;
	}

	res = edma_install_tcd(common.tcd_seq_ptr, SEQ_DMA_CHANNEL);
	if (res != 0) {
		return res;
	}

	return EOK;
}


int restart_sampling(void)
{
	int res;

	dma_stop();

	memset(common.buff, 0, ADC_BUFFER_SIZE);

	res = spi_init(&common.ade7913_spi, common.spi);
	if (res < 0) {
		log_error("Failed to initialize SPI #%d", common.spi);
		return res;
	}

	res = adc_init(1);
	if (res < 0) {
		log_error("Failed to re-initialize ADE7913");
		return res;
	}
	res = pwm_init();
	if (res < 0) {
		log_error("Failed to init PWM input capture");
		return res;
	}
	res = dma_setup_tcds();
	if (res < 0) {
		log_error("Failed to setup DMA TCDs");
		return res;
	}

	dma_start();

	return EOK;
}


static void edma_destroy(void)
{
	if (common.edma_spi_ch_handle != (handle_t)-1) {
		resourceDestroy(common.edma_spi_ch_handle);
	}
	if (common.dready_cond != (handle_t)-1) {
		resourceDestroy(common.dready_cond);
	}
	if (common.edma_spi_rcv_cond != (handle_t)-1) {
		resourceDestroy(common.edma_spi_rcv_cond);
	}
	if (common.edma_spi_rcv_lock != (handle_t)-1) {
		resourceDestroy(common.edma_spi_rcv_lock);
	}
	if (common.buff != MAP_FAILED) {
		munmap(common.buff, (ADC_BUFFER_SIZE + _PAGE_SIZE - 1) / _PAGE_SIZE * _PAGE_SIZE);
	}
}


static int edma_configure(void)
{
	int devnum, res, i;

	common.edma_spi_ch_handle = (handle_t)-1;
	common.edma_spi_rcv_lock = (handle_t)-1;
	common.edma_spi_rcv_cond = (handle_t)-1;
	common.dready_cond = (handle_t)-1;
	common.buff = MAP_FAILED;

	res = edma_init(edma_error_handler);
	if (res < 0) {
		log_error("Failed to initialize eDMA");
		return res;
	}

	res = mutexCreate(&common.edma_spi_rcv_lock);
	if (res < 0) {
		log_error("Mutex resource creation failed");
		return res;
	}

	res = condCreate(&common.edma_spi_rcv_cond);
	if (res < 0) {
		log_error("Conditional resource creation failed");
		return res;
	}

	res = condCreate(&common.dready_cond);
	if (res < 0) {
		log_error("Conditional resource creation failed");
		return res;
	}

	common.buff = mmap(NULL, (ADC_BUFFER_SIZE + _PAGE_SIZE - 1) / _PAGE_SIZE * _PAGE_SIZE,
		PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, -1, common.buffer_paddr);

	if (common.buff == MAP_FAILED) {
		log_error("eDMA buffer allocation failed");
		return -ENOMEM;
	}

	memset(common.buff, 0, ADC_BUFFER_SIZE);
	common.buffer_paddr = va2pa(common.buff);

	/* Set request commands order */
	for (i = 0; i < common.devcnt; ++i) {
		devnum = (int)(common.order[i] - '0');
		spi_write_cmd_lookup[i] = (spi_write_cmd_lookup[i] & ~(0x3000000)) | (devnum << 24);
	}

	res = dma_setup_tcds();
	if (res < 0) {
		log_error("Failed to init TCDs");
		return res;
	}

	interrupt(EDMA_CHANNEL_IRQ(SPI_RCV_DMA_CHANNEL),
		edma_spi_rcv_irq_handler, NULL, common.edma_spi_rcv_cond, &common.edma_spi_ch_handle);

	dma_start();

	return EOK;
}


static int dev_init(const char *devname)
{
	int res;

	common.oid.port = (uint32_t)-1;

	res = portCreate(&common.oid.port);
	if (res != EOK) {
		log_error("Could not create port: %d", res);
		return res;
	}

	res = create_dev(&common.oid, devname);
	if (res < 0) {
		log_error("Could not create %s (res=%d)", devname, res);
		return res;
	}

	log_info("Device initialized");

	return EOK;
}


static void dev_destroy(void)
{
	if (common.oid.port != (uint32_t)-1) {
		portDestroy(common.oid.port);
	}
}


static int dev_read(void *data, size_t size)
{
	int res = 0;

	if (data == NULL || size != sizeof(unsigned)) {
		return -EIO;
	}

	mutexLock(common.edma_spi_rcv_lock);
	res = condWait(common.edma_spi_rcv_cond, common.edma_spi_rcv_lock, 1000000);
	if (res == 0) {
		*(uint32_t *)data = common.edma_transfers;
	}
	mutexUnlock(common.edma_spi_rcv_lock);

	return res;
}


static int dev_ctl(msg_t *msg)
{
	ade7913_dev_ctl_t dev_ctl;
	int devnum, res, i;

	memcpy(&dev_ctl, msg->o.raw, sizeof(dev_ctl));

	switch (dev_ctl.type) {
		case ade7913_dev_ctl__enable:
			common.enabled = 1;
			dma_start();
			return EOK;

		case ade7913_dev_ctl__disable:
			common.enabled = 0;
			dma_stop();
			return EOK;

		case ade7913_dev_ctl__reset:
			return restart_sampling();

		case ade7913_dev_ctl__set_config:
			if (common.enabled) {
				return -EBUSY;
			}

			for (i = 0; i < common.devcnt; ++i) {
				devnum = (int)(common.order[i] - '0');

				if (ade7913_unlock(&common.ade7913_spi, devnum) < 0) {
					return -EAGAIN;
				}

				res = ade7913_set_sampling_rate(&common.ade7913_spi,
					devnum, (int)dev_ctl.config.sampling_rate);

				if (ade7913_lock(&common.ade7913_spi, devnum) < 0) {
					return -EIO;
				}

				if (res < 0) {
					return -EINVAL;
				}
			}

			return EOK;

		case ade7913_dev_ctl__get_config:
			dev_ctl.config.bits = 24; /* device constant */
			dev_ctl.config.devices = common.devcnt;
			res = ade7913_get_sampling_rate(&common.ade7913_spi,
				(int)(common.order[0] - '0'), (int *)&dev_ctl.config.sampling_rate);

			if (res < 0) {
				return -EIO;
			}

			memcpy(msg->o.raw, &dev_ctl, sizeof(ade7913_dev_ctl_t));

			return EOK;

		case ade7913_dev_ctl__get_buffers:
			dev_ctl.buffers.paddr = common.buffer_paddr;
			dev_ctl.buffers.num = ADE7913_BUF_NUM;
			dev_ctl.buffers.size = ADC_BUFFER_SIZE;
			memcpy(msg->o.raw, &dev_ctl, sizeof(ade7913_dev_ctl_t));
			return EOK;

		case ade7913_dev_ctl__status:
			return EOK;

		default:
			log_error("dev_ctl: unknown type (%d)", dev_ctl.type);
			return -ENOSYS;
	}

	return EOK;
}


static int msg_loop(void)
{
	msg_t msg;
	msg_rid_t rid;
	int err = EOK;

	for (;;) {
		err = msgRecv(common.oid.port, &msg, &rid);
		if (err < 0) {
			if (err == -EINTR) {
				continue;
			}
			break;
		}

		switch (msg.type) {
			case mtOpen:
				msg.o.io.err = EOK;
				break;

			case mtClose:
				msg.o.io.err = EOK;
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

		msgRespond(common.oid.port, &msg, rid);
	}

	return err;
}


static int init(void)
{
	int res;

	do {
		res = dev_init(ADC_DEVICE_FILE_NAME);
		if (res < 0) {
			log_error("Device initialization failed");
			break;
		}

		res = spi_init(&common.ade7913_spi, common.spi);
		if (res < 0) {
			log_error("Failed to initialize SPI #%d", common.spi);
			break;
		}

		res = adc_init(0);
		if (res < 0) {
			log_error("Failed to initialize ADE7193: (%d)", res);
			break;
		}

		res = pwm_init();
		if (res < 0) {
			log_error("Failed to initialize PWM");
			break;
		}

		res = edma_configure();
		if (res < 0) {
			log_error("Failed to configure eDMA");
			break;
		}
	} while (0);

	if (res < 0) {
		dev_destroy();
		edma_destroy();
	}

	return res;
}


static void usage(char *name)
{
	printf(
		"Usage: %s <order> <nspi>\n"
		"\torder - chip select order (device list)\n"
		"\tnspi  - SPI device number\n"
		"\tExample: %s 1230 1",
		name, name);
}


int main(int argc, char **argv)
{
	oid_t tmp_oid;
	int i, err, devnum;

	priority(ADE7913_PRIO);

	memset(&common, 0, sizeof(common));

	if (argc != 3) {
		log_error("No device list or no SPI device given");
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	common.spi = atoi(argv[2]);
	if (common.spi < 1 || common.spi > sizeof(spi_base) / sizeof(spi_base[0])) {
		log_error("Wrong spi number provided");
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	common.spi_ptr = (uint32_t *)spi_base[common.spi - 1];
	common.gpio3_ptr = (uint32_t *)0x40134000;
	common.iomux_ptr[0] = (uint32_t *)0x400e8000 + 4 + pctl_mux_gpio_ad_18;
	common.iomux_ptr[1] = (uint32_t *)0x400e8000 + 4 + pctl_mux_gpio_ad_19;
	common.iomux_ptr[2] = (uint32_t *)0x400e8000 + 4 + pctl_mux_gpio_ad_20;
	common.iomux_ptr[3] = (uint32_t *)0x400e8000 + 4 + pctl_mux_gpio_ad_29;

	common.order = argv[1];
	common.devcnt = strlen(common.order);

	if (common.devcnt < 1 || common.devcnt > 4) {
		log_error("Incorrect ADE7913 device count (1 min, 4 max)");
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	for (i = 0; i < common.devcnt; ++i) {
		devnum = common.order[i] - '0';
		if (devnum >= common.devcnt || devnum < 0) {
			log_error("Wrong order format provided");
			usage(argv[0]);
			return EXIT_FAILURE;
		}
	}

	log_info("Device order: %s", common.order);

	/* Wait for the filesystem */
	while (lookup("/", NULL, &tmp_oid) < 0) {
		usleep(10 * 1000);
	}

	if (init() < 0) {
		log_error("Could not init driver.");
		return EXIT_FAILURE;
	}

	err = msg_loop();
	if (err < 0) {
		/* FIXME: this is critical error (how to handle it: log than reboot?) */
		log_error("Message loop failed (err=%d)", err);
		return EXIT_FAILURE;
	}

	log_error("Exiting!");
	return EXIT_SUCCESS;
}
