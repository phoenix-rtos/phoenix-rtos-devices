/*
 * Phoenix-RTOS
 *
 * ACPI internal message API between acpisrv and libacpi
 *
 * Copyright 2024 Phoenix Systems
 * Author: Adam Greloch
 *
 * %LICENSE%
 */


#ifndef _ACPI_MSG_H_
#define _ACPI_MSG_H_

#include <stdint.h>

typedef struct {
	uint32_t addr;
	uint8_t irq;
} acpi_irq_t;


typedef struct {
	enum { acpi_msg_irq } type;

	union {
		acpi_irq_t irq;
	};
} acpi_msg_t;

#endif
