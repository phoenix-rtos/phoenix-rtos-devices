/*
 * Phoenix-RTOS
 *
 * i.MX RT1176 ADE7913 API
 *
 * Copyright 2021 Phoenix Systems
 * Author: Marcin Baran
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

#include <imxrt-multi.h>

#include "ade7913.h"


/* ADE7913 registers address */
enum { ade7913_iwv = 0, ade7913_v1wv, ade7913_v2wv, ade7913_adc_crc = 4, ade7913_ctrl_crc,
	ade7913_cnt_snapshot = 7, ade7913_config, ade7913_status0, ade7913_lock_reg, ade7913_sync_snap,
	ade7913_counter0, ade7913_counter1, ade7913_emi_ctrl, ade7913_status1, ade7913_tempos = 24 };

/* ADE7913 registers bits */
#define ADE7913_CONFIG_CLKOUT_EN (1 << 0)
#define ADE7913_CONFIG_PWRDWN_EN (1 << 2)
#define ADE7913_CONFIG_TEMP_EN   (1 << 3)
#define ADE7913_CONFIG_ADC_FREQ  (1 << 4)
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
#define ADE7913_READ_BIT         0x4
#define ADE7913_ADDR_OFFS        0x3
#define ADE7913_LOCK_SEQ         0xca
#define ADE7913_UNLOCK_SEQ       0x9c
#define ADE7913_CONFIG_FREQ_8KHZ (0x00 << ADE7913_CONFIG_ADC_FREQ)
#define ADE7913_CONFIG_FREQ_4KHZ (0x01 << ADE7913_CONFIG_ADC_FREQ)
#define ADE7913_CONFIG_FREQ_2KHZ (0x10 << ADE7913_CONFIG_ADC_FREQ)
#define ADE7913_CONFIG_FREQ_1KHZ (0x11 << ADE7913_CONFIG_ADC_FREQ)
#define ADE7913_READY            1


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
	uint8_t addr, uint8_t *data, size_t size)
{
	uint8_t buff[size + 1];

	if (size == 0)
		return EOK;

	if (addr > ade7913_tempos)
		return -EINVAL;

	buff[0] = (addr << ADE7913_ADDR_OFFS) | ADE7913_READ_BIT;

	if (lpspi_transaction(device, cs, buff, size) < 0)
		return -EIO;

	memcpy(data, &buff[1], size);

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

	if ((res = lpspi_transaction(device, cs, buff, 2)) < 0)
		return res;

	if (addr == ade7913_lock_reg)
		return EOK;

	/* Check written register */
	buff[0] = (addr << ADE7913_ADDR_OFFS) | ADE7913_READ_BIT;

	if ((res = lpspi_transaction(device, cs, buff, 2)) < 0)
		return res;

	return value == buff[1] ? EOK : -EAGAIN;
}


static int ade7913_set_clear_bits(oid_t *device, int cs,
	uint8_t addr, uint8_t set, uint8_t clear)
{
	int res;
	uint8_t val;

	if ((res = ade7913_read_reg(device, cs, addr, &val, sizeof(val))) < 0)
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

	if (ade7913_read_reg(device, cs, ade7913_status0, &reg, sizeof(reg)))
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


int ade7913_version(oid_t *device, int cs)
{
	uint8_t reg;

	if (ade7913_read_reg(device, cs, ade7913_status1, &reg, sizeof(reg)) < 0)
		return -EIO;

	return (reg & (0b11 << ADE7913_STATUS1_VERSION));
}


int ade7913_sample_lost(oid_t *device, int cs)
{
	uint8_t reg;

	if (ade7913_read_reg(device, cs, ade7913_status1, &reg, sizeof(reg)) < 0)
		return -EIO;

	return (reg & ADE7913_STATUS1_ADC_NA);
}


/* Read from IWV register initializes burst read */
int ade7913_sample_regs_read(oid_t *device, int cs,
	ade7913_burst_reg_t *reg)
{
	return ade7913_read_reg(device, cs, ade7913_iwv, (uint8_t *)reg,
		sizeof(ade7913_burst_reg_t));
}


int ade7913_lock(oid_t *device, int cs)
{
	uint8_t reg;

	if (ade7913_write_reg(device, cs, ade7913_lock_reg, ADE7913_LOCK_SEQ) < 0)
		return -EIO;

	if (ade7913_read_reg(device, cs, ade7913_status0, &reg, sizeof(reg)) < 0)
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

	if (ade7913_read_reg(device, cs, ade7913_status0, &reg, sizeof(reg)) < 0)
		return -EIO;

	if (reg & ADE7913_STATUS0_IC_PROT)
		return -EAGAIN;

	return EOK;
}


int ade7913_enable(oid_t *device, int cs)
{
	return ade7913_set_clear_bits(device, cs,
		ade7913_config, ADE7913_CONFIG_PWRDWN_EN, 0);
}


int ade7913_disable(oid_t *device, int cs)
{
	return ade7913_set_clear_bits(device, cs,
		ade7913_config, 0, ADE7913_CONFIG_PWRDWN_EN);
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
