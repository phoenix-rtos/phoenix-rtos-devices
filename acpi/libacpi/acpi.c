#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/msg.h>

#include "acpi-msg.h"
#include "libacpi.h"


static void acpi_hostLookup(oid_t *oid)
{
	int ret;

	for (;;) {
		ret = lookup("devfs/acpi", NULL, oid);
		if (ret >= 0) {
			break;
		}

		ret = lookup("/dev/acpi", NULL, oid);
		if (ret >= 0) {
			break;
		}

		usleep(1000000);
	}
}


int acpi_getIrq(acpi_device_t *device, uint8_t *irq)
{
	int ret;
	oid_t oid;
	msg_t msg = { 0 };
	acpi_msg_t *amsg = (acpi_msg_t *)&msg.i.raw;
	acpi_msg_t *oamsg = (acpi_msg_t *)&msg.o.raw;

	msg.type = mtDevCtl;
	amsg->type = acpi_msg_irq;

	switch (device->type) {
		case acpi_pci:
			amsg->irq.addr = (device->pci.dev << 16) | 0xFFFF;
			break;
		default:
			return -ENOSYS;
	}

	acpi_hostLookup(&oid);

	ret = msgSend(oid.port, &msg);
	if (ret < 0) {
		return ret;
	}

	if (msg.o.err < 0) {
		return msg.o.err;
	}

	*irq = oamsg->irq.irq;

	return msg.o.err;
}
