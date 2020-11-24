/*
 * Phoenix-RTOS
 *
 * i.MX RT1064 AD7779 driver.
 *
 * Copyright 2018, 2019, 2020 Phoenix Systems
 * Author: Krystian Wasik, Aleksander Kaminski, Hubert Buczynski, Marcin Baran
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>

#include <phoenix/arch/imxrt.h>
#include <imxrt-multi.h>

#include "ad7779.h"

#define COL_RED	 "\033[1;31m"
#define COL_CYAN	"\033[1;36m"
#define COL_NORMAL  "\033[0m"

#define LOG_TAG "ad7779-drv: "

#define log_info(fmt, ...)	  do { printf(COL_CYAN LOG_TAG fmt COL_NORMAL "\n", ##__VA_ARGS__); } while (0)
#define log_error(fmt, ...)	 do { printf(COL_RED  LOG_TAG fmt COL_NORMAL "\n", ##__VA_ARGS__); } while (0)

#if 1
#define log_debug(fmt, ...)	 do { printf(LOG_TAG fmt "\n", ##__VA_ARGS__); } while (0)
#else
#define log_debug(fmt, ...)
#endif

#define AD7779_CHn_CONFIG(n)              (0x00 + n)
#define CHn_GAIN_SHIFT                    (6)

#define AD7779_CH_DISABLE                 (0x08)

#define AD7779_GENERAL_USER_CONFIG_1      (0x11)
#define POWERMODE_BIT                     (1 << 6)
#define PDB_REFOUT_BUF                    (1 << 4)

#define AD7779_GENERAL_USER_CONFIG_2      (0x12)
#define SPI_SYNC                          (1 << 0)

#define AD7779_GENERAL_USER_CONFIG_3      (0x13)

#define AD7779_DOUT_FORMAT                (0x14)

#define AD7779_ADC_MUX_CONFIG             (0x15)

#define AD7779_BUFFER_CONFIG_1            (0x19)
#define AD7779_BUFFER_CONFIG_2            (0x1A)

#define AD7779_CHn_OFFSET_UBYTE(n)        (0x1C + 6*n)
#define AD7779_CHn_OFFSET_MBYTE(n)        (0x1D + 6*n)
#define AD7779_CHn_OFFSET_LBYTE(n)        (0x1E + 6*n)

#define AD7779_CHn_GAIN_UBYTE(n)          (0x1F + 6*n)
#define AD7779_CHn_GAIN_MBYTE(n)          (0x20 + 6*n)
#define AD7779_CHn_GAIN_LBYTE(n)          (0x21 + 6*n)

#define AD7779_CHn_ERR_REG(n)             (0x4C + n)
#define AD7779_CH0_1_SAT_ERR              (0x54)
#define AD7779_CH2_3_SAT_ERR              (0x55)
#define AD7779_CH4_5_SAT_ERR              (0x56)
#define AD7779_CH6_7_SAT_ERR              (0x57)
#define AD7779_GEN_ERR_REG_1              (0x59)
#define AD7779_GEN_ERR_REG_2              (0x5B)
#define AD7779_STATUS_REG_1               (0x5D)
#define AD7779_STATUS_REG_2               (0x5E)
#define AD7779_STATUS_REG_3               (0x5F)

#define AD7779_SRC_N_MSB                  (0x60)
#define AD7779_SRC_N_LSB                  (0x61)
#define AD7779_SRC_IF_MSB                 (0x62)
#define AD7779_SRC_IF_LSB                 (0x63)

#define AD7779_SRC_UPDATE                 (0x64)
#define SRC_LOAD_SOURCE_BIT               (1 << 7)
#define SRC_LOAD_UPDATE_BIT               (1 << 0)

#define AD7779_MCLK_FREQ                  ((uint32_t)8192*1000)


static struct {
	oid_t multidrv;
} ad7779_common;


/* Pin config:
 * /START -> GPIO_B0_04 (ALT5 GPIO2_IO04)
 * /RESET -> GPIO_B0_05 (ALT5 GPIO2_IO05)
 * /DRDY -> GPIO_B0_14 (ALT3 SAI1_RX_SYNC)
 * DCLK -> GPIO_B0_15 (ALT3 SAI1_RX_BCLK)
 * DOUT3 -> GPIO_B0_12 (ALT3 SAI1_RX_DATA03)
 * DOUT2 -> GPIO_B0_11 (ALT3 SAI1_RX_DATA02)
 * DOUT1 -> GPIO_B0_10 (ALT3 SAI1_RX_DATA01)
 * DOUT0 -> GPIO_B1_00 (ALT3 SAI1_RX_DATA00)
 * /CS -> GPIO_B0_00 (managed by Multidrv)
 * SCLK -> GPIO_B0_03 (managed by Multidrv)
 * SDO -> GPIO_B0_01 (managed by Multidrv)
 * SDI -> GPIO_B0_02 (managed by Multidrv)
 * CLK_SEL -> GPIO_B0_06 (ALT5 GPIO2_IO06)
 */


static void gpio_setPin(int gpio, int pin, int state)
{
	msg_t msg;
	multi_i_t *imsg = NULL;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg = (multi_i_t *)msg.i.raw;

	imsg->id = gpio;
	imsg->gpio.type = gpio_set_port;
	imsg->gpio.port.val = !!state << pin;
	imsg->gpio.port.mask = 1 << pin;

	msgSend(ad7779_common.multidrv.port, &msg);
}


static void gpio_setDir(int gpio, int pin, int dir)
{
	msg_t msg;
	multi_i_t *imsg = NULL;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg = (multi_i_t *)msg.i.raw;

	imsg->id = gpio;
	imsg->gpio.type = gpio_set_dir;
	imsg->gpio.dir.val = !!dir << pin;
	imsg->gpio.dir.mask = 1 << pin;

	msgSend(ad7779_common.multidrv.port, &msg);
}


static int lpspi_transaction(char *buff, size_t bufflen)
{
	msg_t msg;
	multi_i_t *imsg = (multi_i_t *)msg.i.raw;
	multi_o_t *omsg = (multi_o_t *)msg.o.raw;

	msg.type = mtDevCtl;
	msg.i.data = buff;
	msg.i.size = bufflen;
	msg.o.data = buff;
	msg.o.size = bufflen;

	imsg->id = id_spi4;
	imsg->spi.type = spi_transaction;
	imsg->spi.transaction.cs = 0;
	imsg->spi.transaction.frameSize = bufflen;

	msgSend(ad7779_common.multidrv.port, &msg);

	return omsg->err;
}


static int lpspi_config(void)
{
	msg_t msg;
	multi_i_t *imsg = (multi_i_t *)msg.i.raw;

	msg.type = mtDevCtl;
	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = NULL;
	msg.o.size = 0;

	imsg->id = id_spi4;
	imsg->spi.type = spi_config;
	imsg->spi.config.cs = 0;
	imsg->spi.config.mode = spi_mode_0;
	imsg->spi.config.endian = spi_msb;
	imsg->spi.config.sckDiv = 4;
	imsg->spi.config.prescaler = 3;

	return msgSend(ad7779_common.multidrv.port, &msg);
}


#define AD7779_READ_BIT		 (0x80)

static int ad7779_read(uint8_t addr, uint8_t *data, uint8_t len)
{
	uint8_t buff[len + 1];

	if (len == 0)
		return AD7779_OK;

	buff[0] = addr | AD7779_READ_BIT;
	memset(buff + 1, 0, len);
	if (lpspi_transaction((void *)buff, len + 1) < 0)
		return AD7779_CTRL_IO_ERROR;

	memcpy(data, buff + 1, len);

	if (buff[0] != 0x20)
		return AD7779_CTRL_HEADER_ERROR;

	return AD7779_OK;
}


static int __attribute__((unused)) ad7779_read_reg(uint8_t addr, uint8_t *val)
{
	return ad7779_read(addr, val, sizeof(uint8_t));
}


static int ad7779_write(uint8_t addr, const uint8_t *data, uint8_t len)
{
	uint8_t buff[len + 1];

	if (len == 0)
		return AD7779_OK;

	buff[0] = addr;
	memcpy(buff + 1, data, len);

	if (lpspi_transaction((void *)buff, len + 1) < 0)
		return AD7779_CTRL_IO_ERROR;

	if (buff[0] != 0x20)
		return AD7779_CTRL_HEADER_ERROR;

	return AD7779_OK;
}


static int ad7779_write_reg(uint8_t addr, uint8_t val)
{
	return ad7779_write(addr, &val, sizeof(val));
}


static int ad7779_set_clear_bits(uint8_t addr, uint8_t set, uint8_t clear)
{
	int res;
	uint8_t reg_val, reg_val_ver;

	res = ad7779_read(addr, &reg_val, sizeof(reg_val));
	if (res < 0)
		return res;

	reg_val |= set;
	reg_val &= ~clear;

	res = ad7779_write(addr, &reg_val, sizeof(reg_val));
	if (res < 0)
		return res;

	res = ad7779_read(addr, &reg_val_ver, sizeof(reg_val_ver));
	if (res < 0)
		return res;

	if (reg_val != reg_val_ver)
		return AD7779_VERIFY_FAILED;

	return 0;
}

int ad7779_set_adc_mux(ad7779_ref_t ref, ad7779_meter_t meter)
{
	return ad7779_write_reg(AD7779_ADC_MUX_CONFIG, (ref << 6) | (meter << 2));
}

int ad7779_get_mode(ad7779_mode_t *mode)
{
	int res;
	uint8_t reg;

	if (mode == NULL)
		return AD7779_ARG_ERROR;

	res = ad7779_read_reg(AD7779_GENERAL_USER_CONFIG_1, &reg);
	if (res != AD7779_OK)
		return res;

	if (reg & POWERMODE_BIT) {
		*mode = ad7779_mode__high_resolution;
	} else {
		*mode = ad7779_mode__low_power;
	}

	return AD7779_OK;
}

int ad7779_set_mode(ad7779_mode_t mode)
{
	if (mode == ad7779_mode__high_resolution)
		return ad7779_set_clear_bits(AD7779_GENERAL_USER_CONFIG_1, POWERMODE_BIT, 0);
	else
		return ad7779_set_clear_bits(AD7779_GENERAL_USER_CONFIG_1, 0, POWERMODE_BIT);
}

int ad7779_get_sampling_rate(uint32_t *fs)
{
	int res;

	if (fs == NULL)
		return AD7779_ARG_ERROR;

	ad7779_mode_t mode;
	if ((res = ad7779_get_mode(&mode)) < 0)
		return res;

	uint32_t base = AD7779_MCLK_FREQ/4;
	if (mode == ad7779_mode__low_power) base = base/2;

	uint8_t SRC_N_MSB, SRC_N_LSB, SRC_IF_MSB, SRC_IF_LSB;
	if ((res = ad7779_read_reg(AD7779_SRC_N_MSB, &SRC_N_MSB)) < 0)
		return res;
	if ((res = ad7779_read_reg(AD7779_SRC_N_LSB, &SRC_N_LSB)) < 0)
		return res;
	if ((res = ad7779_read_reg(AD7779_SRC_IF_MSB, &SRC_IF_MSB)) < 0)
		return res;
	if ((res = ad7779_read_reg(AD7779_SRC_IF_LSB, &SRC_IF_LSB)) < 0)
		return res;

	uint16_t SRC_N = ((uint16_t)SRC_N_MSB) << 8 | SRC_N_LSB; /* Decimation rate */
	uint16_t SRC_IF = ((uint16_t)SRC_IF_MSB) << 8 | SRC_IF_LSB; /* Interpolation factor */

	*fs = (((uint64_t)base) << 16)/(SRC_IF + (((uint64_t)SRC_N) << 16));

	log_debug("current sampling rate is %u (SRC_N=%u, SRC_IF=%u)", *fs, SRC_N, SRC_IF);

	return AD7779_OK;
}

int ad7779_set_sampling_rate(uint32_t fs)
{
	int res;

	ad7779_mode_t mode;
	if ((res = ad7779_get_mode(&mode)) < 0)
		return res;

	uint32_t base = AD7779_MCLK_FREQ/4;
	if (mode == ad7779_mode__low_power) base = base/2;

	/* Sanity check */
	if (mode == ad7779_mode__low_power && fs > AD7779_MAX_SAMPLE_RATE_LP) {
		log_debug("sampling rate too high (low power mode)");
		return AD7779_ARG_ERROR;
	}
	if (mode == ad7779_mode__high_resolution && fs > AD7779_MAX_SAMPLE_RATE_HR) {
		log_debug("sampling rate too high (high resolution mode)");
		return AD7779_ARG_ERROR;
	}

	uint16_t SRC_N = base/fs; /* Decimation rate */
	uint16_t SRC_IF = (base%fs << 16)/fs; /* Interpolation factor */

	log_debug("setting sampling rate to %u (SRC_N=%u, SRC_IF=%u)", fs, SRC_N, SRC_IF);

	log_debug("clearing SRC_LOAD_UPDATE bit");
	if ((res = ad7779_set_clear_bits(AD7779_SRC_UPDATE, 0, SRC_LOAD_UPDATE_BIT)) < 0)
		return res;

	if ((res = ad7779_write_reg(AD7779_SRC_N_MSB,  (SRC_N  >> 8) & 0xff)) < 0)
		return res;
	if ((res = ad7779_write_reg(AD7779_SRC_N_LSB,   SRC_N       & 0xff)) < 0)
		return res;
	if ((res = ad7779_write_reg(AD7779_SRC_IF_MSB, (SRC_IF >> 8) & 0xff)) < 0)
		return res;
	if ((res = ad7779_write_reg(AD7779_SRC_IF_LSB,  SRC_IF     & 0xff)) < 0)
		return res;

	/* Trigger ODR update (by setting SRC_LOAD_UPDATE bit) */
	log_debug("triggering ODR update by setting SRC_LOAD_UPDATE_BIT");
	if ((res = ad7779_set_clear_bits(AD7779_SRC_UPDATE, SRC_LOAD_UPDATE_BIT, 0)) < 0)
		return res;

	/* Reset internal logic */
	log_debug("reseting internal logic");
	if ((res = ad7779_set_clear_bits(AD7779_GENERAL_USER_CONFIG_2, 0, SPI_SYNC)) < 0)
		return res;
	if ((res = ad7779_set_clear_bits(AD7779_GENERAL_USER_CONFIG_2, SPI_SYNC, 0)) < 0)
		return res;

	return AD7779_OK;
}

int ad7779_get_enabled_channels(uint8_t *ch)
{
	return ad7779_read_reg(AD7779_CH_DISABLE, ch);
}

int ad7779_set_enabled_channels(uint8_t ch)
{
	return ad7779_write_reg(AD7779_CH_DISABLE, ~ch);
}

int ad7779_get_channel_mode(uint8_t channel, ad7779_chmode_t* mode)
{
	int res;
	uint8_t reg;

	if (channel >= AD7779_NUM_OF_CHANNELS || mode == NULL)
		return AD7779_ARG_ERROR;

	if ((res = ad7779_read_reg(AD7779_CHn_CONFIG(channel), &reg)) < 0)
		return res;

	*mode = (reg >> 4) & 0b11;

	return AD7779_OK;
}

int ad7779_set_channel_mode(uint8_t channel, ad7779_chmode_t mode)
{
	if (channel >= AD7779_NUM_OF_CHANNELS || mode > ad7779_chmode__ref_monitor)
		return AD7779_ARG_ERROR;

	if (mode == ad7779_chmode__normal)
		return ad7779_set_clear_bits(AD7779_CHn_CONFIG(channel),
			0, (0b11 << 4));
	else if (mode == ad7779_chmode__meter_rx)
		return ad7779_set_clear_bits(AD7779_CHn_CONFIG(channel),
			(0b01 << 4), (0b10 << 4));
	else
		return ad7779_set_clear_bits(AD7779_CHn_CONFIG(channel),
			(0b10 << 4), (0b01 << 4));
}

int ad7779_get_channel_gain(uint8_t channel, uint8_t *gain)
{
	int res;
	uint8_t reg;

	if (channel >= AD7779_NUM_OF_CHANNELS)
		return AD7779_ARG_ERROR;

	if ((res = ad7779_read_reg(AD7779_CHn_CONFIG(channel), &reg)) < 0)
		return res;

	if (gain != NULL)
		*gain = 1 << (reg >> CHn_GAIN_SHIFT);

	log_debug("current gain for channel %u is %u", channel, *gain);

	return AD7779_OK;
}

int ad7779_set_channel_gain(uint8_t channel, uint8_t gain)
{
	if (channel >= AD7779_NUM_OF_CHANNELS)
		return AD7779_ARG_ERROR;

	log_debug("setting gain for channel %u to %u", channel, gain);

	switch (gain) {
		case 1:
			return ad7779_set_clear_bits(AD7779_CHn_CONFIG(channel),
				0b00 << CHn_GAIN_SHIFT, 0);
			break;
		case 2:
			return ad7779_set_clear_bits(AD7779_CHn_CONFIG(channel),
				0b01 << CHn_GAIN_SHIFT, 0);
			break;
		case 4:
			return ad7779_set_clear_bits(AD7779_CHn_CONFIG(channel),
				0b10 << CHn_GAIN_SHIFT, 0);
			break;
		case 8:
			return ad7779_set_clear_bits(AD7779_CHn_CONFIG(channel),
				0b11 << CHn_GAIN_SHIFT, 0);
		default:
			return AD7779_ARG_ERROR;
	}
}

int ad7779_get_channel_offset(uint8_t channel, uint32_t *offset)
{
	int res;
	uint8_t reg;
	uint32_t val = 0;

	if (channel >= AD7779_NUM_OF_CHANNELS)
		return AD7779_ARG_ERROR;

	if ((res = ad7779_read_reg(AD7779_CHn_OFFSET_LBYTE(channel), &reg)) < 0)
		return res;
	val |= reg;
	if ((res = ad7779_read_reg(AD7779_CHn_OFFSET_MBYTE(channel), &reg)) < 0)
		return res;
	val |= (reg << 8);
	if ((res = ad7779_read_reg(AD7779_CHn_OFFSET_UBYTE(channel), &reg)) < 0)
		return res;
	val |= (reg << 16);
	*offset = val;

	// log_debug("current offset for channel %u is 0x%x", channel, *offset);

	return AD7779_OK;
}

int ad7779_set_channel_offset(uint8_t channel, uint32_t offset)
{
	int res;

	if (channel >= AD7779_NUM_OF_CHANNELS)
		return AD7779_ARG_ERROR;

	// log_debug("setting offset for channel %u to 0x%x", channel, offset);

	if ((res = ad7779_write_reg(AD7779_CHn_OFFSET_LBYTE(channel),
			offset & 0xff)) < 0)
		return res;
	if ((res = ad7779_write_reg(AD7779_CHn_OFFSET_MBYTE(channel),
			(offset >> 8) & 0xff)) < 0)
		return res;
	if ((res = ad7779_write_reg(AD7779_CHn_OFFSET_UBYTE(channel),
			(offset >> 16) & 0xff)) < 0)
		return res;

	return AD7779_OK;
}

int ad7779_get_channel_gain_correction(uint8_t channel, uint32_t *gain)
{
	int res;
	uint8_t reg;
	uint32_t val = 0;

	if (channel >= AD7779_NUM_OF_CHANNELS)
		return AD7779_ARG_ERROR;

	if ((res = ad7779_read_reg(AD7779_CHn_GAIN_LBYTE(channel), &reg)) < 0)
		return res;
	val |= reg;
	if ((res = ad7779_read_reg(AD7779_CHn_GAIN_LBYTE(channel), &reg)) < 0)
		return res;
	val |= (reg << 8);
	if ((res = ad7779_read_reg(AD7779_CHn_GAIN_LBYTE(channel), &reg)) < 0)
		return res;
	val |= (reg << 16);
	*gain = val;

	// log_debug("current gain correction for channel %u is 0x%x", channel, *gain);

	return AD7779_OK;
}

int ad7779_set_channel_gain_correction(uint8_t channel, uint32_t gain)
{
	int res;

	if (channel >= AD7779_NUM_OF_CHANNELS)
		return AD7779_ARG_ERROR;

	// log_debug("setting gain correction for channel %u to 0x%x", channel, gain);

	if ((res = ad7779_write_reg(AD7779_CHn_GAIN_LBYTE(channel),
			gain & 0xff)) < 0)
		return res;
	if ((res = ad7779_write_reg(AD7779_CHn_GAIN_MBYTE(channel),
			(gain >> 8) & 0xff)) < 0)
		return res;
	if ((res = ad7779_write_reg(AD7779_CHn_GAIN_UBYTE(channel),
			(gain >> 16) & 0xff)) < 0)
		return res;

	return AD7779_OK;
}

int ad7779_get_status(uint8_t *status_buf)
{
	uint8_t i, k = 0;

	if (status_buf == NULL)
		return AD7779_ARG_ERROR;

	for (i = 0; i < 8; ++i) {
		if (ad7779_read_reg(AD7779_CHn_ERR_REG(i), &status_buf[k++]))
			return AD7779_CTRL_IO_ERROR;
	}

	for (i = 0; i < 4; ++i) {
		if (ad7779_read_reg(AD7779_CH0_1_SAT_ERR + i, &status_buf[k++]))
			return AD7779_CTRL_IO_ERROR;
	}

	for (i = 0; i < 2; ++i) {
		if (ad7779_read_reg(AD7779_GEN_ERR_REG_1 + i * 2, &status_buf[k++]))
			return AD7779_CTRL_IO_ERROR;
	}

	for (i = 0; i < 3; ++i) {
		if (ad7779_read_reg(AD7779_STATUS_REG_1 + i, &status_buf[k++]))
			return AD7779_CTRL_IO_ERROR;
	}

	return AD7779_OK;
}

int ad7779_print_status(void)
{
	uint8_t i, tmp;

	for (i = 0; i < AD7779_NUM_OF_CHANNELS; ++i) {
		ad7779_read_reg(AD7779_CHn_ERR_REG(i), &tmp);
		log_info("AD7779_CH%d_ERR_REG=0x%x", i , tmp);
	}

	ad7779_read_reg(AD7779_CH0_1_SAT_ERR, &tmp);
	log_info("AD7779_CH0_1_SAT_ERR=0x%x", tmp);

	ad7779_read_reg(AD7779_CH2_3_SAT_ERR, &tmp);
	log_info("AD7779_CH2_3_SAT_ERR=0x%x", tmp);

	ad7779_read_reg(AD7779_CH4_5_SAT_ERR, &tmp);
	log_info("AD7779_CH4_5_SAT_ERR=0x%x", tmp);

	ad7779_read_reg(AD7779_CH6_7_SAT_ERR, &tmp);
	log_info("AD7779_CH6_7_SAT_ERR=0x%x", tmp);

	ad7779_read_reg(AD7779_GEN_ERR_REG_1, &tmp);
	log_info("AD7779_GEN_ERR_REG_1=0x%x", tmp);

	ad7779_read_reg(AD7779_GEN_ERR_REG_2, &tmp);
	log_info("AD7779_GEN_ERR_REG_2=0x%x", tmp);

	ad7779_read_reg(AD7779_STATUS_REG_1, &tmp);
	log_info("AD7779_STATUS_REG_1=0x%x", tmp);

	ad7779_read_reg(AD7779_STATUS_REG_2, &tmp);
	log_info("AD7779_STATUS_REG_2=0x%x", tmp);

	ad7779_read_reg(AD7779_STATUS_REG_3, &tmp);
	log_info("AD7779_STATUS_REG_3=0x%x", tmp);

	return 0;
}


static int ad7779_gpio_init(void)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_iomux;

	pctl.iomux.sion = 0;
	pctl.iomux.mode = 5;

	/* Configure as GPIOs */

	/* /START */
	pctl.iomux.mux = pctl_mux_gpio_b0_04;
	platformctl(&pctl);
	/* /RESET */
	pctl.iomux.mux = pctl_mux_gpio_b0_05;
	platformctl(&pctl);
	/* /CLK_SEL */
	pctl.iomux.mux = pctl_mux_gpio_b0_06;
	platformctl(&pctl);
	/* Hardware reset */
	pctl.iomux.mux = pctl_mux_gpio_emc_23;
	platformctl(&pctl);

	pctl.type = pctl_iopad;
	pctl.iopad.hys = 0;
	pctl.iopad.pus = 0b11;
	pctl.iopad.pue = 0;
	pctl.iopad.pke = 1;
	pctl.iopad.ode = 0;
	pctl.iopad.speed = 2;
	pctl.iopad.dse = 1;
	pctl.iopad.sre = 0;

	/* /START */
	pctl.iopad.pad = pctl_pad_gpio_b0_04;
	platformctl(&pctl);
	/* /RESET */
	pctl.iopad.pad = pctl_pad_gpio_b0_05;
	platformctl(&pctl);
	/* /CLK_SEL */
	pctl.iopad.pad = pctl_pad_gpio_b0_06;
	platformctl(&pctl);
	/* Hardware reset */
	pctl.iopad.pad = pctl_pad_gpio_emc_23;
	platformctl(&pctl);

	/* Set states */
	gpio_setDir(id_gpio2, 4, 1);
	gpio_setDir(id_gpio2, 5, 1);
	gpio_setDir(id_gpio2, 6, 1);
	gpio_setDir(id_gpio4, 23, 1);

	gpio_setPin(id_gpio2, 4, 1);
	gpio_setPin(id_gpio2, 5, 1);
	gpio_setPin(id_gpio2, 6, 1);
	gpio_setPin(id_gpio4, 23, 0);

	return AD7779_OK;
}


static int ad7779_reset(int hard)
{
	int i;
	uint8_t status[17];

	/* Hardware reset */
	if (hard) {
		gpio_setPin(id_gpio4, 23, 1);
		usleep(200000);
		gpio_setPin(id_gpio4, 23, 0);
	}

	/* Software reset */
	gpio_setPin(id_gpio2, 4, 0);
	usleep(10000);
	gpio_setPin(id_gpio2, 5, 0);
	usleep(200000);
	gpio_setPin(id_gpio2, 4, 1);
	usleep(100000);
	gpio_setPin(id_gpio2, 5, 1);

	memset(status, 0, sizeof(status));

	for (i = 0; i < 4; ++i) {
		if (!ad7779_get_status(status)) {
			if (status[16] & 0x10)
				return AD7779_OK;
		}

		usleep(100000);
	}

	return AD7779_CTRL_IO_ERROR;
}


int ad7779_init(int hard)
{
	int res;

	while (lookup("/dev/gpio1", NULL, &ad7779_common.multidrv) < 0)
		usleep(100 * 1000);

	if ((res = lpspi_config()) < 0)
		return res;

	if ((res = ad7779_gpio_init()) < 0)
		return res;

	if ((res = ad7779_reset(hard)) < 0)
		return res;

	if ((res = ad7779_write_reg(AD7779_CH_DISABLE, 0xFF)) < 0)
		return res;

	for (int i = 0; i < 8; ++i)
		if ((res = ad7779_set_channel_mode(i, ad7779_chmode__normal)) < 0)
			return res;


	/* Use external reference with RES_OUT connected to REFx+/REFx- */
	/* Set meter mux to 280mV */
	log_debug("setting external reference");
	if ((res = ad7779_set_adc_mux(ad7779_ref__ext, ad7779_meter__280mV)) < 0)
		return res;

	/* Power up internal reference */
	ad7779_set_clear_bits(AD7779_GENERAL_USER_CONFIG_1, PDB_REFOUT_BUF, 0);

	log_debug("switching to high resolution mode");
	if ((res = ad7779_set_mode(ad7779_mode__high_resolution)) < 0)
		return res;

	/* Use one DOUTx line; DCLK_CLK_DIV = 1 */
	log_debug("setting DOUT_FORMAT");
	if ((res = ad7779_write_reg(AD7779_DOUT_FORMAT, 0xe0)) < 0)
		return res;

	/* Make sure SRC_LOAD_SOURCE bit is cleared */
	log_debug("clearing SRC_LOAD_SOURCE bit");
	if ((res = ad7779_set_clear_bits(AD7779_SRC_UPDATE, 0, SRC_LOAD_SOURCE_BIT)) < 0)
		return res;

	log_debug("setting sampling rate");
	if ((res = ad7779_set_sampling_rate(AD7779_MAX_SAMPLE_RATE_HR)) < 0)
		return res;

	return AD7779_OK;
}
