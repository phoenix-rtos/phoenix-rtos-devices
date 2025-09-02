#include "../../core/pci_utils.h"
#include "phoenix_comp.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>


struct firmware {
	u8 *data;    /* Buffer containing raw binary firmware */
	size_t size; /* Size of data */
};

int request_firmware_direct(const struct firmware **firmware, const char *file_name, struct device *dev);
void release_firmware(const struct firmware *firmware);
