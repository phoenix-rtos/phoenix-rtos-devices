/*
 * Phoenix-RTOS
 *
 * i.MX RT SPI driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <sys/interrupt.h>
#include <sys/threads.h>
#include <sys/pwman.h>
#include <sys/msg.h>
#include <unistd.h>
#include <errno.h>


#include "imxrt-multi.h"
#include "common.h"
#include "config.h"
#include "spi.h"

#define SPI1_POS 0
#define SPI2_POS (SPI1_POS + SPI1)
#define SPI3_POS (SPI2_POS + SPI2)
#define SPI4_POS (SPI3_POS + SPI3)

#define SPI_CNT (SPI1 + SPI2 + SPI3 + SPI4)

#define MAX_FRAME_SZ 0x1000
#define WORD_SIZE sizeof(uint32_t)
#define MAX_FIFOSZ_BYTES 16 * WORD_SIZE


enum { spi_verid = 0, spi_param, spi_cr = 0x4, spi_sr, spi_ier, spi_der, spi_cfgr0, spi_cfgr1, spi_dmr0 = 0xc,
	   spi_dmr1, spi_ccr = 0x10, spi_fcr = 0x16, spi_fsr, spi_tcr, spi_tdr, spi_rsr = 0x1c, spi_rdr };


struct {
	handle_t cond;
	handle_t mutex;
	handle_t inth;
	handle_t irqLock;

	int irq;
	volatile int ready;
	volatile uint32_t *base;

	uint32_t tcr;
} spi_common[SPI_CNT];


static const int spiConfig[] = { SPI1, SPI2, SPI3, SPI4 };


static const int spiPos[] = { SPI1_POS, SPI2_POS, SPI3_POS, SPI4_POS };


static inline uint32_t spi_deserializeWord(const uint8_t *buff)
{
	uint32_t word = 0;

	word = buff[3] | (buff[2] << 8) | (buff[1] << 16) | (buff[0] << 24);

	return word;
}


static inline void spi_serializeWord(uint8_t *buff, uint32_t word, int8_t start)
{
	for (int i = start - 1; i >= 0; --i)
		*buff++ = (word >> (i * 8)) & 0xff;
}


static int spi_irqHandler(unsigned int n, void *arg)
{
	int spi = (int)arg;

	*(spi_common[spi].base + spi_ier) &= ~(1 << 0);
	spi_common[spi].ready = 1;

	return 0;
}


static void spi_initTransmition(int spi)
{
	mutexLock(spi_common[spi].irqLock);
	spi_common[spi].ready = 0;

	*(spi_common[spi].base + spi_ier) |= 1;

	while (!spi_common[spi].ready)
		condWait(spi_common[spi].cond, spi_common[spi].irqLock, 0);

	mutexUnlock(spi_common[spi].irqLock);
}


static void spi_txBytes(int spi, const uint8_t *txBuff, int bytesNumber)
{
	uint32_t word;

	while (bytesNumber / WORD_SIZE) {
		*(spi_common[spi].base + spi_tdr) = spi_deserializeWord(txBuff);
		txBuff += WORD_SIZE;
		bytesNumber -= WORD_SIZE;
	}

	if (bytesNumber) {
		word = 0;
		while (bytesNumber--)
			word = (word << 8) | *txBuff++;
		*(spi_common[spi].base + spi_tdr) = word;
	}
}


static void spi_rxBytes(int spi, uint8_t *rxBuff, int bytesNumber)
{
	uint32_t word;

	/* Get data from RX Fifo */
	while (bytesNumber / WORD_SIZE) {
		word = *(spi_common[spi].base + spi_rdr);
		spi_serializeWord(rxBuff, word, WORD_SIZE);
		rxBuff += WORD_SIZE;
		bytesNumber -= WORD_SIZE;
	}

	if (bytesNumber) {
		word = *(spi_common[spi].base + spi_rdr);
		spi_serializeWord(rxBuff, word, bytesNumber);
		rxBuff += bytesNumber;
	}
}


static int spi_performTranscation(int spi, unsigned char cs, const uint8_t *txBuff, uint8_t *rxBuff, int len)
{
	int size = len;
	int rxTotalBytes = 0;
	int txFifoBytes, rxFifoBytes;
	uint32_t txWordsCnt, rxWordsCnt;

	if (!spiConfig[spi])
		return -EINVAL;

	if ((len * 8) > MAX_FRAME_SZ)
		return -EINVAL;

	spi = spiPos[spi];

	mutexLock(spi_common[spi].mutex);

	/* Initialize Transmit Command Register */
	*(spi_common[spi].base + spi_tcr) = (spi_common[spi].tcr & ~(0x7ff)) | ((cs & 0x3) << 24) | (len * 8 - 1);

	/* Wait until Transmit Command will be taken from  TX fifo */
	txWordsCnt = *(spi_common[spi].base + spi_fsr) & 0x1f;
	while (txWordsCnt)
		txWordsCnt = ((*(spi_common[spi].base + spi_fsr)) & 0x1f);

	while (size > 0) {
		if (size <= MAX_FIFOSZ_BYTES)
			txFifoBytes = size;
		else
			txFifoBytes = MAX_FIFOSZ_BYTES;

		/* Fill transmit FIFO */
		spi_txBytes(spi, txBuff, txFifoBytes);
		txBuff += txFifoBytes;
		size -= txFifoBytes;

		/* Transfer data into slave */
		txWordsCnt = *(spi_common[spi].base + spi_fsr) & 0x1f;
		spi_initTransmition(spi);

		/* Wait on transmit data.
		 * Omit this step if Transmit Command is sent in transfers bigger than MAX_FIFOSZ_BYTES */
		rxWordsCnt = (*(spi_common[spi].base + spi_fsr) >> 16) & 0x1f;
		while ((rxWordsCnt != txWordsCnt) & (len <= MAX_FIFOSZ_BYTES))
			rxWordsCnt = (*(spi_common[spi].base + spi_fsr) >> 16) & 0x1f;

		/* Get data from receive FIFO */
		rxFifoBytes = rxWordsCnt * WORD_SIZE;
		if ((rxFifoBytes + rxTotalBytes) > len)
			rxFifoBytes = len - rxTotalBytes;

		spi_rxBytes(spi, rxBuff, rxFifoBytes);
		rxBuff += rxFifoBytes;
		rxTotalBytes += rxFifoBytes;
	}

	/*
	 * If data length is a multiplication of MAX_FIFOSZ_BYTES, Transmit Command moves one word out of FIFO.
	 * The last word is received after trasnsmission.
	 */
	rxFifoBytes = len - rxTotalBytes;

	if (rxFifoBytes) {
		rxWordsCnt = (*(spi_common[spi].base + spi_fsr) >> 16) & 0x1f;
		if (rxWordsCnt) {
			spi_rxBytes(spi, rxBuff, rxFifoBytes);
			rxTotalBytes += rxFifoBytes;
		}
	}

	mutexUnlock(spi_common[spi].mutex);

	return rxTotalBytes;
}


static int spi_configure(uint32_t spi, uint32_t bdiv, uint32_t prescaler, uint32_t endian, uint32_t mode, uint32_t cs)
{
	int i;

	if (!spiConfig[spi])
		return -EINVAL;

	if (bdiv > 255)
		return -EINVAL;

	if (prescaler >= 8)
		return -EINVAL;

	i = spiPos[spi];

	mutexLock(spi_common[i].mutex);

	/* Disable module */
	*(spi_common[i].base + spi_cr) = 0;

	/* Software reset */
	*(spi_common[i].base + spi_cr) |= 1 << 1;
	*(spi_common[i].base + spi_cr) = 0;

	/* Set master mode as default */
	*(spi_common[i].base + spi_cfgr1) |= 1;

	/* CS polarity: 0 - low */
	*(spi_common[i].base + spi_cfgr1) &= ~(0xf << 8);

	/* Set pin configuration */
	*(spi_common[i].base + spi_cfgr1) &= ~(1 << 26) & ~(3 << 24);

	/* Enable Module */
	*(spi_common[i].base + spi_cr) |= 1;

	/* Set baudrate divider */
	*(spi_common[i].base + spi_ccr) = (*(spi_common[i].base + spi_ccr) & ~0xff) | bdiv;

	/* Set watermarks */
	*(spi_common[i].base + spi_fcr) &=  ~(0xf << 16) & ~0xf;

	/* Set transmit command register */
	*(spi_common[i].base + spi_tcr) &= ~(1 << 26); /* reset register */
	spi_common[i].tcr = *(spi_common[i].base + spi_tcr) | ((mode & 0x3) << 30) | ((cs & 0x3) << 24) | ((endian & 0x1) << 23) | (prescaler << 27);

	mutexUnlock(spi_common[i].mutex);

	return EOK;
}


static void spi_handleDevCtl(msg_t *msg, int dev)
{
	multi_i_t *idevctl = (multi_i_t *)msg->i.raw;
	multi_o_t *odevctl = (multi_o_t *)msg->o.raw;

	dev -= id_spi1;

	uint8_t *rxBuff = (uint8_t *)msg->o.data;
	const uint8_t *txBuff = (const uint8_t *)msg->i.data;

	switch (idevctl->spi.type) {
		case spi_config:
			odevctl->err = spi_configure(dev, idevctl->spi.config.sckDiv, idevctl->spi.config.prescaler, idevctl->spi.config.endian, idevctl->spi.config.mode, idevctl->spi.config.cs);
			break;

		case spi_transaction:
			odevctl->err = spi_performTranscation(dev, idevctl->spi.transaction.cs, txBuff, rxBuff, idevctl->spi.transaction.frameSize);
			break;

		default:
			odevctl->err = -ENOSYS;
			break;
	}
}


int spi_handleMsg(msg_t *msg, int dev)
{
	switch (msg->type) {
		case mtGetAttr:
		case mtOpen:
		case mtClose:
		case mtSetAttr :
		case mtWrite:
		case mtRead:
			msg->o.io.err = EOK;
			break;

		case mtDevCtl:
			spi_handleDevCtl(msg, dev);
			break;
	}

	return EOK;
}


static int spi_getIsel(int mux, int *isel, int *val)
{
	switch (mux) {
		case pctl_mux_gpio_sd_b0_01 :  *isel = pctl_isel_lpspi1_pcs0; *val = 0; break;
		case pctl_mux_gpio_emc_30 :    *isel = pctl_isel_lpspi1_pcs0; *val = 1; break;
		case pctl_mux_gpio_emc_27 :    *isel = pctl_isel_lpspi1_sck; *val = 0; break;
		case pctl_mux_gpio_sd_b0_00 :  *isel = pctl_isel_lpspi1_sck; *val = 1; break;
		case pctl_mux_gpio_emc_29 :    *isel = pctl_isel_lpspi1_sdi; *val = 0; break;
		case pctl_mux_gpio_sd_b0_03 :  *isel = pctl_isel_lpspi1_sdi; *val = 1; break;
		case pctl_mux_gpio_emc_28 :    *isel = pctl_isel_lpspi1_sdo; *val = 0; break;
		case pctl_mux_gpio_sd_b0_02 :  *isel = pctl_isel_lpspi1_sdo; *val = 1; break;
		case pctl_mux_gpio_sd_b1_06 :  *isel = pctl_isel_lpspi2_pcs0; *val = 0; break;
		case pctl_mux_gpio_emc_01 :    *isel = pctl_isel_lpspi2_pcs0; *val = 1; break;
		case pctl_mux_gpio_sd_b1_07 :  *isel = pctl_isel_lpspi2_sck; *val = 0; break;
		case pctl_mux_gpio_emc_00 :    *isel = pctl_isel_lpspi2_sck; *val = 1; break;
		case pctl_mux_gpio_sd_b1_09 :  *isel = pctl_isel_lpspi2_sdi; *val = 0; break;
		case pctl_mux_gpio_emc_03 :    *isel = pctl_isel_lpspi2_sdi; *val = 1; break;
		case pctl_mux_gpio_sd_b1_08 :  *isel = pctl_isel_lpspi2_sdo; *val = 0; break;
		case pctl_mux_gpio_emc_02 :    *isel = pctl_isel_lpspi2_sdo; *val = 1; break;
		case pctl_mux_gpio_ad_b0_03 :  *isel = pctl_isel_lpspi3_pcs0; *val = 0; break;
		case pctl_mux_gpio_ad_b1_12 :  *isel = pctl_isel_lpspi3_pcs0; *val = 1; break;
		case pctl_mux_gpio_ad_b0_00 :  *isel = pctl_isel_lpspi3_sck; *val = 0; break;
		case pctl_mux_gpio_ad_b1_15 :  *isel = pctl_isel_lpspi3_sck; *val = 1; break;
		case pctl_mux_gpio_ad_b1_13 :  *isel = pctl_isel_lpspi3_sdi; *val = 0; break;
		case pctl_mux_gpio_ad_b0_02 :  *isel = pctl_isel_lpspi3_sdi; *val = 1; break;
		case pctl_mux_gpio_ad_b1_14 :  *isel = pctl_isel_lpspi3_sdo; *val = 0; break;
		case pctl_mux_gpio_ad_b0_01 :  *isel = pctl_isel_lpspi3_sdo; *val = 1; break;
		case pctl_mux_gpio_b0_00 :     *isel = pctl_isel_lpspi4_pcs0; *val = 0; break;
		case pctl_mux_gpio_b1_04 :     *isel = pctl_isel_lpspi4_pcs0; *val = 1; break;
		case pctl_mux_gpio_b0_03 :     *isel = pctl_isel_lpspi4_sck; *val = 0; break;
		case pctl_mux_gpio_b1_07 :     *isel = pctl_isel_lpspi4_sck; *val = 1; break;
		case pctl_mux_gpio_b0_01 :     *isel = pctl_isel_lpspi4_sdi; *val = 0; break;
		case pctl_mux_gpio_b1_05 :     *isel = pctl_isel_lpspi4_sdi; *val = 1; break;
		case pctl_mux_gpio_b0_02 :     *isel = pctl_isel_lpspi4_sdo; *val = 0; break;
		case pctl_mux_gpio_b1_06 :     *isel = pctl_isel_lpspi4_sdo; *val = 1; break;
		default: return -1;
	}

	return 0;
}


static int spi_muxVal(int mux)
{
	switch (mux) {
		case pctl_mux_gpio_b1_04 :
		case pctl_mux_gpio_b1_05 :
		case pctl_mux_gpio_b1_06 :
		case pctl_mux_gpio_b1_07 :
			return 1;

		case pctl_mux_gpio_ad_b1_12 :
		case pctl_mux_gpio_ad_b1_13 :
		case pctl_mux_gpio_ad_b1_14 :
		case pctl_mux_gpio_ad_b1_15 :
		case pctl_mux_gpio_emc_00 :
		case pctl_mux_gpio_emc_01 :
		case pctl_mux_gpio_emc_02 :
		case pctl_mux_gpio_emc_03 :
		case pctl_mux_gpio_emc_40 :
		case pctl_mux_gpio_emc_41 :
		case pctl_mux_gpio_b1_02 :
		case pctl_mux_gpio_b1_03 :
			return 2;

		case pctl_mux_gpio_b0_00 :
		case pctl_mux_gpio_b0_01 :
		case pctl_mux_gpio_b0_02 :
		case pctl_mux_gpio_b0_03 :
		case pctl_mux_gpio_emc_27 :
		case pctl_mux_gpio_emc_28 :
		case pctl_mux_gpio_emc_29 :
		case pctl_mux_gpio_emc_30 :
		case pctl_mux_gpio_emc_31 :
			return 3;

		case pctl_mux_gpio_b1_11 :
			return 6;

		case pctl_mux_gpio_ad_b0_00 :
		case pctl_mux_gpio_ad_b0_01 :
		case pctl_mux_gpio_ad_b0_02 :
		case pctl_mux_gpio_ad_b0_03 :
		case pctl_mux_gpio_ad_b0_04 :
		case pctl_mux_gpio_ad_b0_05 :
		case pctl_mux_gpio_ad_b0_06 :
			return 7;

		default :
			return 4;
	}
}


static void spi_initPins(void)
{
	int i, isel, val;
	static const int muxes[] = {
#if SPI1
		PIN2MUX(SPI1_SCK), PIN2MUX(SPI1_SD0), PIN2MUX(SPI1_SDI),
#ifdef SPI1_PCS0
		PIN2MUX(SPI1_PCS0),
#endif
#ifdef SPI1_PCS1
		PIN2MUX(SPI1_PCS1),
#endif
#ifdef SPI1_PCS2
		PIN2MUX(SPI1_PCS2),
#endif
#ifdef SPI1_PCS3
		PIN2MUX(SPI1_PCS3),
#endif
#endif

#if SPI2
		PIN2MUX(SPI2_SCK), PIN2MUX(SPI2_SD0), PIN2MUX(SPI2_SDI),
#ifdef SPI2_PCS0
		PIN2MUX(SPI2_PCS0),
#endif
#ifdef SPI2_PCS1
		PIN2MUX(SPI2_PCS1),
#endif
#ifdef SPI2_PCS2
		PIN2MUX(SPI2_PCS2),
#endif
#ifdef SPI2_PCS3
		PIN2MUX(SPI2_PCS3),
#endif
#endif		

#if SPI3
		PIN2MUX(SPI3_SCK), PIN2MUX(SPI3_SD0), PIN2MUX(SPI3_SDI),
#ifdef SPI3_PCS0
		PIN2MUX(SPI3_PCS0),
#endif
#ifdef SPI3_PCS1
		PIN2MUX(SPI3_PCS1),
#endif
#ifdef SPI3_PCS2
		PIN2MUX(SPI3_PCS2),
#endif
#ifdef SPI3_PCS3
		PIN2MUX(SPI3_PCS3),
#endif
#endif

#if SPI4
		PIN2MUX(SPI4_SCK), PIN2MUX(SPI4_SD0), PIN2MUX(SPI4_SDI),
#ifdef SPI4_PCS0
		PIN2MUX(SPI4_PCS0),
#endif
#ifdef SPI4_PCS1
		PIN2MUX(SPI4_PCS1),
#endif
#ifdef SPI4_PCS2
		PIN2MUX(SPI4_PCS2),
#endif
#ifdef SPI4_PCS3
		PIN2MUX(SPI4_PCS3),
#endif
#endif
	};

	for (i = 0; i < sizeof(muxes) / sizeof(muxes[0]); ++i) {
		common_setMux(muxes[i], 0, spi_muxVal(muxes[i]));

		if (spi_getIsel(muxes[i], &isel, &val) < 0)
			continue;
		common_setInput(isel, val);
	}
}


int spi_init(void)
{
	int i, spi;

	static const struct {
		volatile uint32_t *base;
		int clk;
		int irq;
	} spiInfo[] = {
		{ LPSPI1_BASE, LPSPI1_CLK, LPSPI1_IRQ },
		{ LPSPI2_BASE, LPSPI2_CLK, LPSPI2_IRQ },
		{ LPSPI3_BASE, LPSPI3_CLK, LPSPI3_IRQ },
		{ LPSPI4_BASE, LPSPI4_CLK, LPSPI4_IRQ }
	};

	spi_initPins();

	for (i = 0, spi = 0; spi < sizeof(spiInfo) / sizeof(spiInfo[0]); ++spi) {
		if (!spiConfig[spi])
			continue;

		if (common_setClock(spiInfo[spi].clk, clk_state_run) < 0)
			return -EFAULT;

		if (condCreate(&spi_common[i].cond) != EOK)
			return -ENOENT;

		if (mutexCreate(&spi_common[i].mutex) != EOK) {
			resourceDestroy(spi_common[i].cond);
			return -ENOENT;
		}

		if (mutexCreate(&spi_common[i].irqLock) != EOK) {
			resourceDestroy(spi_common[i].cond);
			resourceDestroy(spi_common[i].mutex);
			return -ENOENT;
		}

		spi_common[i].ready = 1;
		spi_common[i].irq = spiInfo[spi].irq;
		spi_common[i].base = (void *)spiInfo[spi].base;

		interrupt(spi_common[i].irq, spi_irqHandler, (void *)i, spi_common[i].cond, &spi_common[i].inth);

		/* Disable module */
		*(spi_common[i].base + spi_cr) = 0;
		++i;
	}

	return 0;
}
