/*
 * Phoenix-RTOS
 *
 * i.MX RT1064 AD7779 test
 *
 * Copyright 2019, 2020 Phoenix Systems
 * Author: Aleksander Kaminski, Marcin Baran
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <stdio.h>
#include <string.h>
#include <sys/msg.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>

#include "adc-api-ad7779.h"

#define MIDSCALE        0x800000
#define FULLSCALE       0xFFFFFF
#define GAIN_BASE       0x555555

#define VREF_2_5V       2.5
#define VREF_280mV      0.28

#define VREF            VREF_2_5V

#define VSCALE          0.000875
#define CSCALE          0.00489

#define GAIN            4

#define NUM_CHANNELS    8


static int adc_send_msg(oid_t adc_dev, adc_dev_ctl_t* ctl)
{
	msg_t msg;
	msg.type = mtDevCtl;
	memcpy(msg.o.raw, ctl, sizeof(adc_dev_ctl_t));

	int res;
	if ((res = msgSend(adc_dev.port, &msg)) < 0 || msg.o.io.err != EOK) {
		printf("adc command error from %s (res=%d, err=%d)\n",
			ADC_DRIVER, res, msg.o.io.err);
		return res;
	}

	memcpy(ctl, msg.o.raw, sizeof(adc_dev_ctl_t));
	return EOK;
};

static int adc_get_buffers(oid_t adc_dev, uint32_t** buf0, size_t *buf_num, size_t *size)
{
	adc_dev_ctl_t adc_ctl;
	adc_ctl.type = adc_dev_ctl__get_buffers;

	if (adc_send_msg(adc_dev, &adc_ctl) != EOK)
		return -1;

	*buf0 = (uint32_t *)adc_ctl.buffers.paddr;
	*buf_num= (size_t)adc_ctl.buffers.num;
	*size = (size_t)adc_ctl.buffers.size;

	return 0;
};

static int adc_set_ch_gain(oid_t adc_dev, uint8_t channel, uint8_t gain, uint8_t mode)
{
	adc_dev_ctl_t adc_ctl;
	adc_ctl.type = adc_dev_ctl__set_channel_config;
	adc_ctl.ch_config.gain = gain;
	adc_ctl.ch_config.channel = channel;
	adc_ctl.ch_config.ref_monitor_mode = (mode >> 1) & 0b1;
	adc_ctl.ch_config.meter_rx_mode = mode & 0b1;

	return adc_send_msg(adc_dev, &adc_ctl);
};

static int adc_set_ch_calib(oid_t adc_dev, uint8_t channel,
	uint32_t gain, uint32_t offset)
{
	adc_dev_ctl_t adc_ctl;
	adc_ctl.type = adc_dev_ctl__set_channel_calib;
	adc_ctl.calib.channel = channel;
	adc_ctl.calib.gain = gain;
	adc_ctl.calib.offset = offset;

	return adc_send_msg(adc_dev, &adc_ctl);
};

static int adc_enable(oid_t adc_dev, uint8_t is_enabled)
{
	adc_dev_ctl_t adc_ctl;
	if(is_enabled)
		adc_ctl.type = adc_dev_ctl__enable;
	else
		adc_ctl.type = adc_dev_ctl__disable;

	return adc_send_msg(adc_dev, &adc_ctl);
};

static int adc_set_adc_mux(oid_t adc_dev, uint8_t mux)
{
	adc_dev_ctl_t adc_ctl;
	adc_ctl.type = adc_dev_ctl__set_adc_mux;
	adc_ctl.mux = mux;

	return adc_send_msg(adc_dev, &adc_ctl);
}

static inline uint32_t adc_convert(uint32_t value)
{
	value &= FULLSCALE;
	if (value > 1 << 23)
		value -= 2*(1 << 23);

	return value;
};

static inline double adc_raw2meas(uint32_t value, double gain)
{
	return (((double)value)/MIDSCALE) * VREF / gain;
}

static inline uint32_t adc_meas2raw(double value, double gain)
{
	return (value * gain / VREF) * MIDSCALE;
}

static void adc_calib_gain_offset(double ref1, double meas1,
	double ref2, double meas2, double gain,
	uint32_t *gain_err, uint32_t *offset_err)
{
	double g = ((meas1 - meas2) / (VREF_2_5V - VREF_280mV));
	*gain_err = (uint32_t)(GAIN_BASE / g);
	double err = (meas1/g - VREF_2_5V) / VREF;
	err *= MIDSCALE * g * 0.75;
	if (err < 0)
		*offset_err = (uint32_t)(FULLSCALE + err + 1);
	else
		*offset_err = (uint32_t)err;
}


int main(int argc, char *argv[])
{
	sleep(2);

	printf("\nStarting adc test measurements\n");

	oid_t adc_dev;

	if (lookup(ADC_DRIVER, NULL, &adc_dev) < 0)
		return -ENODEV;

	uint32_t *buf = NULL;
	size_t num_bufs = 0;
	size_t buf_size = 0;

	if (adc_get_buffers(adc_dev, &buf, &num_bufs, &buf_size) != EOK)
		return -1;

	buf_size = buf_size / sizeof(uint32_t);

	printf("Measuring reference values\n");
	if (adc_set_adc_mux(adc_dev, (0b00 << 6) | (0b0110 << 2)) != EOK)
		return -1;

	int i, j;
	for (i = 0; i < NUM_CHANNELS; ++i) {
		if (adc_set_ch_gain(adc_dev, i, 1, 0b01) != EOK)
			return -1;
		if (adc_set_ch_calib(adc_dev, i, 0.5 * GAIN_BASE, 0) != EOK)
			return -1;
	}

	if (adc_enable(adc_dev, 1) != EOK)
		return -1;

	sleep(1);

	if (adc_enable(adc_dev, 0) != EOK)
		return -1;

	double meas1[NUM_CHANNELS], meas2[NUM_CHANNELS];
	uint32_t gain_err, offset_err;

	for (i = 0; i < NUM_CHANNELS; ++i) {
		meas1[i] = 0;

		for(j = 0; j < 100; ++j)
			meas1[i] += adc_raw2meas(adc_convert(buf[NUM_CHANNELS*j+i]), 0.5);

		meas1[i] = meas1[i] / 100;
	}

	if (adc_set_adc_mux(adc_dev, (0b00 << 6) | (0b0010 << 2)) != EOK)
		return -1;

	if (adc_enable(adc_dev, 1) != EOK)
		return -1;

	sleep(1);

	if (adc_enable(adc_dev, 0) != EOK)
		return -1;

	printf("Calibrating offset and gain correction\n");

	for (i = 0; i < NUM_CHANNELS; ++i) {
		meas2[i] = 0;

		for(j = 0; j < 100; ++j)
			meas2[i] += adc_raw2meas(adc_convert(buf[NUM_CHANNELS*j+i]), 0.5);

		meas2[i] = meas2[i] / 100;

		adc_calib_gain_offset(VREF_2_5V, meas1[i], VREF_280mV, meas2[i],
			GAIN, &gain_err, &offset_err);

		printf("CH: %d, V1: %f, V2: %f, M1: %f, M2 %f, GAIN_REG: 0x%x, OFF_REG: 0x%x\n",
			i, VREF_2_5V, VREF_280mV, meas1[i], meas2[i], gain_err, offset_err);

		if (adc_set_ch_calib(adc_dev, i, GAIN_BASE, 0) != EOK)
			return -1;

		if (adc_set_ch_gain(adc_dev, i, GAIN, 0b00) != EOK)
			return -1;
	}

	if (adc_set_adc_mux(adc_dev, (0b00 << 6) | (0b0110 << 2)) != EOK)
		return -1;

	if (adc_enable(adc_dev, 1) != EOK)
		return -1;

	sleep(1);

	if (adc_enable(adc_dev, 0) != EOK)
		return -1;

	uint32_t data = 0;

	printf("# X Y Z\n");
	for (i = 0; i < 3; ++i) {
		for (j = 0; j < buf_size / NUM_CHANNELS; ++j) {
			data = buf[NUM_CHANNELS * j + i];
			printf("%d, %d, %f, %d\n",
				j,
				adc_convert(data),
				adc_raw2meas(adc_convert(data), VSCALE * GAIN),
				(data >> 28) & 0x7);
		}
	}

	for (i = 4; i < 7; ++i) {
		for (j = 0; j < buf_size / NUM_CHANNELS; ++j) {
			data = buf[NUM_CHANNELS * j + i];
			printf("%d, %d, %f, %d\n",
				j,
				adc_convert(data),
				adc_raw2meas(adc_convert(data), CSCALE * GAIN),
				(data >> 28) & 0x7);
		}
	}

	return 0;
}
