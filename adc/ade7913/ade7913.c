/*
 * Phoenix-RTOS
 *
 * i.MX RT1176 ADE7913 API
 *
 * Copyright 2021-2022 Phoenix Systems
 * Author: Marcin Baran, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <sys/msg.h>
#include <sys/platform.h>

#include <imxrt-multi.h>

#ifdef __CPU_IMXRT117X
#include <phoenix/arch/imxrt1170.h>
#endif

#include "gpio.h"
#include "ade7913.h"


/* ADE7913 registers address */
enum { ade7913_iwv = 0, ade7913_v1wv, ade7913_v2wv, ade7913_adc_crc = 4, ade7913_ctrl_crc,
	ade7913_cnt_snapshot = 7, ade7913_config, ade7913_status0, ade7913_lock_reg, ade7913_sync_snap,
	ade7913_counter0, ade7913_counter1, ade7913_emi_ctrl, ade7913_status1, ade7913_tempos = 24 };

/* ADE7913 registers bits */
#define ADE7913_CONFIG_CLKOUT_EN (1 << 0)
#define ADE7913_CONFIG_PWRDWN_EN (1 << 2)
#define ADE7913_CONFIG_TEMP_EN   (1 << 3)
#define ADE7913_CONFIG_ADC_FREQ  (0x4)
#define ADE7913_CONFIG_SWRST     (1 << 6)
#define ADE7913_CONFIG_BW        (1 << 7)

#define ADE7913_STATUS0_RESET_ON (1 << 0)
#define ADE7913_STATUS0_CRC_STAT (1 << 1)
#define ADE7913_STATUS0_IC_PROT  (1 << 2)

#define ADE7913_SYNC_SNAP_SYNC (1 << 0)
#define ADE7913_SYNC_SNAP_SNAP (1 << 1)

#define ADE7913_EMI_CTRLS(X) (1 << X)

#define ADE7913_STATUS1_VERSION (1 << 0)
#define ADE7913_STATUS1_ADC_NA  (1 << 3)

/* ADE7913 communication utils */
#define ADE7913_READ_BIT         (1 << 2)
#define ADE7913_ADDR_OFFS        0x3
#define ADE7913_LOCK_SEQ         0xca
#define ADE7913_UNLOCK_SEQ       0x9c
#define ADE7913_CONFIG_FREQ_8KHZ (0b00 << ADE7913_CONFIG_ADC_FREQ)
#define ADE7913_CONFIG_FREQ_4KHZ (0b01 << ADE7913_CONFIG_ADC_FREQ)
#define ADE7913_CONFIG_FREQ_2KHZ (0b10 << ADE7913_CONFIG_ADC_FREQ)
#define ADE7913_CONFIG_FREQ_1KHZ (0b11 << ADE7913_CONFIG_ADC_FREQ)


static int lpspi_transaction(oid_t *device, int cs,
	uint8_t *buff, size_t size)
{
	msg_t msg;
	multi_i_t *imsg = (multi_i_t *)msg.i.raw;
	multi_o_t *omsg = (multi_o_t *)msg.o.raw;

	msg.type = mtDevCtl;
	msg.i.data = buff;
	msg.i.size = size;
	msg.o.data = buff;
	msg.o.size = size;

	imsg->id = device->id;
	imsg->spi.type = spi_transaction;
	imsg->spi.transaction.cs = cs;
	imsg->spi.transaction.frameSize = size;

	msgSend(device->port, &msg);

	return omsg->err;
}


static int ade7913_read_reg(oid_t *device, int cs,
	uint8_t addr, uint8_t *data)
{
	uint8_t buff[2];

	if (addr > ade7913_tempos)
		return -EINVAL;

	buff[0] = (addr << ADE7913_ADDR_OFFS) | ADE7913_READ_BIT;

	if (lpspi_transaction(device, cs, buff, sizeof(buff)) < 0)
		return -EIO;

	*data = buff[1];

	return EOK;
}


/* Only 1 byte write at a time possible */
static int ade7913_write_reg(oid_t *device, int cs,
	uint8_t addr, uint8_t value)
{
	int res = 0;
	uint8_t buff[2] = { (addr << ADE7913_ADDR_OFFS), value };

	/* Check if register is writtable */
	if ((addr >= ade7913_iwv && addr <= ade7913_cnt_snapshot) ||
			addr > ade7913_emi_ctrl || addr == ade7913_status0)
		return -EINVAL;

	if ((res = lpspi_transaction(device, cs, buff, sizeof(buff))) < 0)
		return res;

	if (addr == ade7913_lock_reg)
		return EOK;

	/* Check written register */
	buff[0] = (addr << ADE7913_ADDR_OFFS) | ADE7913_READ_BIT;

	if ((res = lpspi_transaction(device, cs, buff, sizeof(buff))) < 0)
		return res;

	return value == buff[1] ? EOK : -EAGAIN;
}


static int ade7913_set_clear_bits(oid_t *device, int cs,
	uint8_t addr, uint8_t set, uint8_t clear)
{
	int res;
	uint8_t val;

	if ((res = ade7913_read_reg(device, cs, addr, &val)) < 0)
		return res;

	val |= set;
	val &= ~clear;

	if ((res = ade7913_write_reg(device, cs, addr, val)) < 0)
		return res;

	return EOK;
}


int ade7913_ready(oid_t *device, int cs)
{
	uint8_t reg;

	if (ade7913_read_reg(device, cs, ade7913_status0, &reg))
		return -EIO;

	if (!(reg & ADE7913_STATUS0_RESET_ON))
		return 1;

	return 0;
}


int ade7913_init(oid_t *device, int cs, int clkout)
{
	int i = 0;
	int set = 0x0;

	while (!ade7913_ready(device, cs)) {
		if (++i > 4)
			return -EBUSY;

		usleep(10000);
	}

	if (ade7913_unlock(device, cs) < 0)
		return -EAGAIN;

	if (clkout)
		set = ADE7913_CONFIG_CLKOUT_EN;

	if (ade7913_set_clear_bits(device, cs, ade7913_config, set | ADE7913_CONFIG_FREQ_8KHZ,
		    ADE7913_CONFIG_BW | ADE7913_CONFIG_TEMP_EN | ADE7913_CONFIG_PWRDWN_EN) < 0)
		return -EIO;

	return EOK;
}

int ade7913_set_sampling_rate(oid_t *device, int cs, int freq_hz)
{
	int set;

	if (freq_hz == 8000)
		set = ADE7913_CONFIG_FREQ_8KHZ;
	else if (freq_hz == 4000)
		set = ADE7913_CONFIG_FREQ_4KHZ;
	else if (freq_hz == 2000)
		set = ADE7913_CONFIG_FREQ_2KHZ;
	else if (freq_hz == 1000)
		set = ADE7913_CONFIG_FREQ_1KHZ;
	else
		return -EINVAL;

	if (ade7913_set_clear_bits(device, cs, ade7913_config, set, 0) < 0)
		return -EIO;

	return EOK;
}

int ade7913_get_sampling_rate(oid_t *device, int cs, int *freq_hz)
{
	uint8_t config_freq;

	if (ade7913_read_reg(device, cs, ade7913_config, &config_freq) < 0)
		return -EAGAIN;

	config_freq = (config_freq & ADE7913_CONFIG_FREQ_1KHZ) >> ADE7913_CONFIG_ADC_FREQ;

	if (config_freq == ADE7913_CONFIG_FREQ_8KHZ)
		*freq_hz = 8000;
	else if (config_freq == ADE7913_CONFIG_FREQ_4KHZ)
		*freq_hz = 4000;
	else if (config_freq == ADE7913_CONFIG_FREQ_2KHZ)
		*freq_hz = 2000;
	else if (config_freq == ADE7913_CONFIG_FREQ_1KHZ)
		*freq_hz = 1000;
	else
		return -EIO;

	return EOK;
}

/* TODO: make this function generic */
int ade7913_sync(oid_t *device, const char *cs, int devcnt, int snap)
{
	int i, res;
	int modes[4];
	oid_t gpiodev;
	platformctl_t pctl;

	int muxes[4] = { pctl_mux_gpio_ad_18, pctl_mux_gpio_ad_19,
		pctl_mux_gpio_ad_20, pctl_mux_gpio_ad_29 };

	int pins[4] = { 17, 18, 19, 28 };

	uint8_t buff[2] = { (ade7913_sync_snap << ADE7913_ADDR_OFFS), ADE7913_SYNC_SNAP_SYNC };

	if (snap)
		buff[1] = ADE7913_SYNC_SNAP_SNAP;

	if (lookup("/dev/gpio3", NULL, &gpiodev))
		return -ENODEV;

	if (devcnt <= 0 || cs == NULL)
		return -EINVAL;

	for (i = 0; i < devcnt; ++i) {
		if ((int)(cs[i] - '0') >= 4 || (int)(cs[i] - '0') < 0)
			return -EINVAL;
	}

	/* Get SPI muxes */
	pctl.action = pctl_get;
	pctl.type = pctl_iomux;

	for (i = 0; i < devcnt; ++i) {
		pctl.iomux.mux = muxes[(int)(cs[i] - '0')];
		platformctl(&pctl);
		modes[(int)(cs[i] - '0')] = pctl.iomux.mode;
	}

	/* Set CS GPIOs to inactive */
	for (i = 0; i < devcnt; ++i) {
		gpio_setDir(&gpiodev, id_gpio3, pins[(int)(cs[i] - '0')], 1);
		gpio_setPin(&gpiodev, id_gpio3, pins[(int)(cs[i] - '0')], 1);
	}

	/* Set muxes to gpio */
	pctl.action = pctl_set;
	pctl.iomux.sion = 0;
	pctl.iomux.mode = 5;

	for (i = 0; i < 4; ++i) {
		pctl.iomux.mux = muxes[(int)(cs[i] - '0')];
		platformctl(&pctl);
	}

	/* Set CS GPIOs to active */
	for (i = 0; i < devcnt; ++i) {
		gpio_setPin(&gpiodev, id_gpio3, pins[(int)(cs[i] - '0')], 0);
	}

	/* Send broadcast */
	if ((res = lpspi_transaction(device, 0, buff, sizeof(buff))) < 0)
		return res;

	/* Set CS GPIOs to inactive */
	for (i = 0; i < devcnt; ++i) {
		gpio_setPin(&gpiodev, id_gpio3, pins[(int)(cs[i] - '0')], 1);
	}

	/* Set muxes back to SPI CS */
	pctl.action = pctl_set;
	pctl.iomux.sion = 0;

	for (i = 0; i < devcnt; ++i) {
		pctl.iomux.mux = muxes[(int)(cs[i] - '0')];
		pctl.iomux.mode = modes[(int)(cs[i] - '0')];
		platformctl(&pctl);
	}

	return EOK;
}


int ade7913_version(oid_t *device, int cs)
{
	uint8_t reg;

	if (ade7913_read_reg(device, cs, ade7913_status1, &reg) < 0)
		return -EIO;

	return (reg & (0b11 << ADE7913_STATUS1_VERSION));
}


int ade7913_sample_lost(oid_t *device, int cs)
{
	uint8_t reg;

	if (ade7913_read_reg(device, cs, ade7913_status1, &reg) < 0)
		return -EIO;

	return (reg & ADE7913_STATUS1_ADC_NA);
}


/* Read from IWV register initializes burst read */
int ade7913_sample_regs_read(oid_t *device, int cs,
	ade7913_burst_reg_t *reg)
{
	uint8_t buff[sizeof(ade7913_burst_reg_t) + 1];

	buff[0] = (ade7913_iwv << ADE7913_ADDR_OFFS) | ADE7913_READ_BIT;

	/* Next 8 sent bytes cannot be 0 so as not to reset ade7193 */
	memset(buff + 1, 0xff, sizeof(ade7913_burst_reg_t));

	if (lpspi_transaction(device, cs, buff, sizeof(ade7913_burst_reg_t) + 1) < 0)
		return -EIO;

	memcpy(reg, buff + 1, sizeof(ade7913_burst_reg_t));

	return EOK;
}


int ade7913_lock(oid_t *device, int cs)
{
	uint8_t reg;

	if (ade7913_write_reg(device, cs, ade7913_lock_reg, ADE7913_LOCK_SEQ) < 0)
		return -EIO;

	if (ade7913_read_reg(device, cs, ade7913_status0, &reg) < 0)
		return -EIO;

	if (!(reg & ADE7913_STATUS0_IC_PROT))
		return -EAGAIN;

	return EOK;
}


int ade7913_unlock(oid_t *device, int cs)
{
	uint8_t reg;

	if (ade7913_write_reg(device, cs, ade7913_lock_reg, ADE7913_UNLOCK_SEQ) < 0)
		return -EIO;

	if (ade7913_read_reg(device, cs, ade7913_status0, &reg) < 0)
		return -EIO;

	if (reg & ADE7913_STATUS0_IC_PROT)
		return -EAGAIN;

	return EOK;
}


int ade7913_emi(oid_t *device, int cs, uint8_t val)
{
	return ade7913_write_reg(device, cs,
		ade7913_emi_ctrl, val);
}


int ade7913_enable(oid_t *device, int cs)
{
	return ade7913_set_clear_bits(device, cs,
		ade7913_config, 0, ADE7913_CONFIG_PWRDWN_EN);
}


int ade7913_disable(oid_t *device, int cs)
{
	return ade7913_set_clear_bits(device, cs,
		ade7913_config, ADE7913_CONFIG_PWRDWN_EN, 0);
}


int ade7913_temperature(oid_t *device, int cs)
{
	return ade7913_set_clear_bits(device, cs,
		ade7913_config, ADE7913_CONFIG_TEMP_EN, 0);
}


int ade7913_voltage(oid_t *device, int cs)
{
	return ade7913_set_clear_bits(device, cs,
		ade7913_config, 0, ADE7913_CONFIG_TEMP_EN);
}


int ade7913_reset_soft(oid_t *device, int cs)
{
	return ade7913_set_clear_bits(device, cs,
		ade7913_config, ADE7913_CONFIG_SWRST, 0);
}


/* 8 consecutive writes valued 0 causes hardware reset */
int ade7913_reset_hard(oid_t *device, int cs)
{
	uint8_t buff[8] = { 0x0 };

	return lpspi_transaction(device, cs, buff, sizeof(buff));
}
