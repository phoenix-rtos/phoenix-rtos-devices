/*
 * Phoenix-RTOS
 *
 * i.MX RT1064 AD7779 driver API
 *
 * Copyright 2018, 2020 Phoenix Systems
 * Author: Krystian Wasik, Marcin Baran
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef ADC_API_H
#define ADC_API_H

#include <stdint.h>

#include <arch.h>
#include <sys/types.h>

#define ADC_DEVICE_DIR                  "/dev"
#define ADC_DEVICE_FILE_NAME            "ad7779"
#define ADC_DRIVER                      "/dev/ad7779"

typedef enum {
	adc_dev_ctl__enable,
	adc_dev_ctl__disable,
	adc_dev_ctl__reset,
	adc_dev_ctl__status,
	adc_dev_ctl__set_adc_mux,
	adc_dev_ctl__set_config,
	adc_dev_ctl__get_config,
	adc_dev_ctl__get_buffers,
	adc_dev_ctl__set_channel_gain,
	adc_dev_ctl__get_channel_gain,
	adc_dev_ctl__set_channel_calib,
	adc_dev_ctl__get_channel_calib,
	adc_dev_ctl__set_channel_config,
	adc_dev_ctl__get_channel_config
} adc_dev_ctl_type_t;

typedef struct {
	adc_dev_ctl_type_t type;

	union {
		/* buffers */
		struct {
			addr_t paddr;
			size_t num;
			size_t size;
		} buffers;

		/* config */
		struct {
			uint32_t sampling_rate;
			uint8_t channels;
			uint8_t enabled_ch;
			uint8_t bits;
		} config;

		/* status */
		struct {
			uint8_t ch_status[8];
			uint8_t dsp_status[4];
			uint8_t general_err[2];
			uint8_t status[3];
		} status;

		/* adc mux */
		uint8_t mux;

		/* if hardware reset */
		uint8_t reset_hard;

		/* channel calib */
		struct {
			uint32_t offset;
			uint32_t gain;
			uint8_t channel;
		} calib;

		/* channel gain */
		struct {
			uint32_t channel;
			uint8_t val;
		} gain;

		/* channel config */
		struct {
			uint32_t channel;
			unsigned gain: 6;
			unsigned meter_rx_mode: 1;
			unsigned ref_monitor_mode: 1;
		} ch_config;
	};
} adc_dev_ctl_t;

#endif /* ADC_API_H */
