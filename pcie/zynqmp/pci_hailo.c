#include <stdio.h>
#include <sys/mman.h>
#include "pci_core.h"

void hailo_dumpBAR(pci_dev_t *hailo, int bar_number)
{
	for (int i = 0; i < hailo->size[bar_number] / 4; i++) {
		uint32_t data = *((volatile uint32_t *)hailo->bar[bar_number] + i);
		printf("%lx : %0x\n", (uint64_t)va2pa((uint32_t *)hailo->bar[bar_number] + i), data);
	}
}
