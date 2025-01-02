#ifndef _ACPI_H_
#define _ACPI_H_

#include <stdint.h>

typedef struct {
	enum { acpi_pci } type;
	union {
		struct {
			uint8_t bus;
			uint8_t dev;
			uint8_t func;
		} pci;
	};
} acpi_device_t;


int acpi_getIrq(acpi_device_t *device, uint8_t *irq);


#endif
