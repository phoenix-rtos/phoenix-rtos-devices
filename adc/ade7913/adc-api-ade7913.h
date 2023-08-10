/*
 * Phoenix-RTOS
 *
 * ADE7913 driver API
 *
 * Copyright 2021, 2023 Phoenix Systems
 * Author: Marcin Baran, Gerard Swiderski
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

#define ADC_DEVICE_FILE_NAME "/dev/ade7913"


typedef struct {
	enum {
		ade7913_dev_ctl__enable = 0,
		ade7913_dev_ctl__disable,
		ade7913_dev_ctl__reset,
		ade7913_dev_ctl__status,
		ade7913_dev_ctl__set_config,
		ade7913_dev_ctl__get_config,
		ade7913_dev_ctl__get_buffers,
	} type;

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
			uint8_t devices;
			uint8_t bits;
		} config;
	};
} ade7913_dev_ctl_t;


#endif /* ADC_API_H */
