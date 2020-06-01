/*
 * Phoenix-RTOS
 *
 * i.MX RT1064 AD7779 driver.
 *
 * Copyright 2018, 2019, 2020 Phoenix Systems
 * Author: Krystian Wasik, Aleksander Kaminski, Marcin Baran
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef AD7779_H
#define AD7779_H

#include <stdint.h>

#define AD7779_OK                       (0)
#define AD7779_ARG_ERROR                (-1)
#define AD7779_CTRL_IO_ERROR            (-2)
#define AD7779_CTRL_HEADER_ERROR        (-3)
#define AD7779_GPIO_INIT_ERROR          (-4)
#define AD7779_GPIO_IO_ERROR            (-5)
#define AD7779_VERIFY_FAILED            (-6)

#define AD7779_NUM_OF_CHANNELS          (8)
#define AD7779_NUM_OF_BITS              (24)

#define AD7779_MAX_SAMPLE_RATE_LP       (8000)
#define AD7779_MAX_SAMPLE_RATE_HR       (16000)

typedef enum {
	ad7779_ref__ext = 0,
	ad7779_ref__int,
	ad7779_ref__supply,
	ad7779_ref__ext_inverted
} ad7779_ref_t;

typedef enum {
	ad7779_meter__280mV = 0b0010,
	ad7779_meter__ext,
	ad7779_meter__ext_inverted,
	ad7779_meter__ext_negative,
	ad7779_meter__int,
	ad7779_meter__int_inverted,
	ad7779_meter__int_positive,
	ad7779_meter__ext_positive
} ad7779_meter_t;

typedef enum {
	ad7779_chmode__normal = 0b00,
	ad7779_chmode__meter_rx,
	ad7779_chmode__ref_monitor
} ad7779_chmode_t;

typedef enum {
	ad7779_mode__low_power,
	ad7779_mode__high_resolution,
} ad7779_mode_t;

int ad7779_init(void);

int ad7779_set_adc_mux(ad7779_ref_t ref, ad7779_meter_t meter);

int ad7779_get_mode(ad7779_mode_t *mode);
int ad7779_set_mode(ad7779_mode_t mode);

int ad7779_get_sampling_rate(uint32_t *fs);
int ad7779_set_sampling_rate(uint32_t fs);

int ad7779_get_channel_mode(uint8_t channel, ad7779_chmode_t *mode);
int ad7779_set_channel_mode(uint8_t channel, ad7779_chmode_t mode);

int ad7779_get_channel_gain(uint8_t channel, uint8_t *gain);
int ad7779_set_channel_gain(uint8_t channel, uint8_t gain);

int ad7779_get_channel_offset(uint8_t channel, uint32_t *offset);
int ad7779_set_channel_offset(uint8_t channel, uint32_t offset);

int ad7779_get_channel_gain_correction(uint8_t channel, uint32_t *gain);
int ad7779_set_channel_gain_correction(uint8_t channel, uint32_t gain);

/* For debugging purposes */
int ad7779_print_status(void);

#endif /* AD7779_H */
