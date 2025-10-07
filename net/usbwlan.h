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
#include <unistd.h>


typedef struct {
	/* clang-format off */
	enum { usbwlan_dl, usbwlan_ctrl_in, usbwlan_ctrl_out, usbwlan_reg_read, usbwlan_reg_write } type;
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
