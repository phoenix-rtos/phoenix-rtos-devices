/*
 * Phoenix-RTOS
 *
 * USB WLAN driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Julian Uziembło
 *
 * %LICENSE%
 */

#ifndef _USBWLAN_H_
#define _USBWLAN_H_

#include <stdint.h>


typedef struct {
	/* clang-format off */
	enum { usbwlan_dl, usbwlan_ctrlIn, usbwlan_ctrlOut, usbwlan_regRead, usbwlan_regWrite, usbwlan_abort } type;
	/* clang-format on */

	union {
		struct {
			uint8_t cmd;
			uint16_t wIndex;
		} dl;

		struct {
			uint8_t cmd;
			uint32_t regaddr;
		} reg;
	};
} usbwlan_i_t;


#endif /* _USBWLAN_H_ */
