/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Generic ata controller driver
 *
 * Copyright 2012 Phoenix Systems
 * Author: Marcin Stragowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

 #define NO_TRACE 1

 #include <stdint.h>
 #include <pc-pci.h>
 #include "pc-ata.h"

 /* Function searches ata devices */
int ata_generic_init(ata_opt_t *opt)
{
	ata_opt_t *aopt = (opt ? opt : &ata_defaults);
	pci_device_t *pdev = 0;
	unsigned int i = 0;
	int devs_found = 0;

	static const file_ops_t ata_ops = {
		.read = ata_read,
		.write = ata_write,
		.open = ata_open
	};

/*	if (aopt->force) { 
		ata_init_one(0, aopt);
	}
	else*/ {
		/* iterate through pci to find ata-bus devices */
		for (i = 0; ata_pci_tbl[i].cl != 0; i++) {
			do {
				dev_pciAlloc(&ata_pci_tbl[i], &pdev);

				if (pdev && !ata_init_one(pdev, aopt))
					devs_found++;
			} while (pdev);
		}

		if (!devs_found)
			return -ENOENT;
	}

	if (dev_register(MAKEDEV(MAJOR_DRIVE, 0), &ata_ops) < 0) {
		main_printf(ATTR_ERROR, "dev/ata: Can't register ata device!\n");
		return -ENOENT;
	}

	return devs_found;
}