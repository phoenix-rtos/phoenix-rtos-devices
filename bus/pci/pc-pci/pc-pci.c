/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * PCI enumerator
 *
 * Copyright 2012-2015 Phoenix Systems
 * Copyright 2006 Pawel Pisarczyk
 * Author: Pawel Pisarczyk
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>

#include <hal/ia32/cpu.h>
#include <sys/threads.h>
#include <sys/interrupt.h>

#include <unistd.h>
#include <sys/msg.h>

#include "pc-pci.h"


struct {
	handle_t mutex;
	pci_device_t *devices;
} pci_common;


/* Function reads word from PCI configuration space */
static u32 _pci_get(u8 bus, u8 dev, u8 func, u8 reg)
{
	u32 v;

	hal_outl((void *)0xcf8, 0x80000000 | ((u32)bus << 16 ) | ((u32)dev << 11) | ((u32)func << 8) | (reg << 2));
	v = hal_inl((void *)0xcfc);
	return v;
}


/* Function writes word to PCI configuration space */
static u32 _pci_set(u8 bus, u8 dev, u8 func, u8 reg, u32 v)
{
	hal_outl((void *)0xcf8, 0x80000000 | ((u32)bus << 16 ) | ((u32)dev << 11) | ((u32)func << 8) | (reg << 2));
	hal_outl((void *)0xcfc, v);
	return v;
}

/* Function inserts PCI device descriptor */
static int _pci_insert(pci_device_t *prev, pci_device_t *dev)
{
	prev->next->prev = dev;
	dev->next = prev->next;
	prev->next = dev;
	dev->prev = prev;
	return EOK;
}


/* Function adds PCI device descriptor to list */
static int _pci_add(pci_device_t **list, pci_device_t *dev)
{
	if (!(*list)) {
		*list = dev;
		dev->next = dev;
		dev->prev = dev;
	}
	else
		_pci_insert((*list)->prev, dev);

	return EOK;
}

#if 0
/* Function removes PCI descriptor from list */
static int _pci_remove(pci_device_t **list, pci_device_t *dev)
{
	dev->prev->next = dev->next;
	dev->next->prev = dev->prev;
	
	if (dev->next == dev)
		(*list) = NULL;
	else if (dev == (*list))
		(*list) = dev->next;
	
	return EOK;
}
#endif

/*
* function available for other processess
* allocating device on the pci bus
*/
int dev_pciAlloc(const pci_id_t *id, pci_device_t **adev)
{
	pci_device_t *dev;
  
	mutexLock(pci_common.mutex);
	if (!pci_common.devices) {
		mutexUnlock(pci_common.mutex);
		return -ENOMEM;
	}

	*adev = NULL;
	dev = pci_common.devices;
  
	do {
		if (!dev->usage) {

			if ((id->vendor == PCI_ANY) || ((id->vendor != PCI_ANY) && (id->vendor == dev->vendor)))
				if ((id->device == PCI_ANY) || ((id->device != PCI_ANY) && (id->device == dev->device)))
					if ((id->cl == PCI_ANY) || ((id->cl != PCI_ANY) && (id->cl == dev->cl))) {
						dev->usage++;
						*adev = dev;
						break;
					}
		}
		dev = dev->next;    
	} while (dev != pci_common.devices);
  
	mutexUnlock(pci_common.mutex);
	return EOK;
}

/*
* function available for other processess
* setting device as a master on the bus
*/
void dev_setBusmaster(pci_device_t *dev, u8 enable) 
{
	u32 dv;
	dv = _pci_get(dev->b, dev->d, dev->f, 1);
	if (enable)
		dv = dv | 1 << 2;
	else
		dv = dv & ~(1 << 2);

	_pci_set(dev->b, dev->d, dev->f, 1, dv);
	
	dev->command = dv & 0xffff;
}

/*
* enumaration on the pci bus and creating the the pci process
*/
void _pci_init(void)
{
	unsigned int b, d, f, i;
	u32 dv;
	pci_device_t *dev;

	/*main_printf(ATTR_DEV, "dev: [pci  ] Enumerating PCI ");*/
	printf("dev: [pci  ] Enumerating PCI ");

	mutexCreate(&pci_common.mutex);	
	pci_common.devices = NULL;

	for (b = 0; b < 256; b++) {
		for (d = 0; d < 32; d++) {
			for (f = 0; f < 8; f++) {
				dv = _pci_get(b, d, f, 0);

				if (dv == 0xffffffff)
					continue;

				/*if ((dev = vm_kmalloc(sizeof(pci_device_t))) == NULL)*/
				if ((dev = malloc(sizeof(pci_device_t))) == NULL)
					break;

				dev->usage = 0;
				dev->b = b;
				dev->d = d;
				dev->f = f;
				
				dev->device = dv >> 16;
				dev->vendor = dv & 0xffff;
	
				dv = _pci_get(b, d, f, 1);
				dev->status = dv >> 16;
				dev->command = dv & 0xffff;


				dev->cl = (_pci_get(b, d, f, 2) >> 16);
				dev->progif = (_pci_get(b, d, f, 2) >> 8) & 0xff;
				dev->revision = _pci_get(b, d, f, 2) & 0xff;
				dev->type = _pci_get(b, d, f, 3) >> 16 & 0xff;
				dev->irq = _pci_get(b, d, f, 15) & 0xff;
      
				/* Get resources */
				for (i = 0; i < 6; i++) {
					dev->resources[i].base = _pci_get(b, d, f, 4 + i);

					/* Get resource limit */
					_pci_set(b, d, f, 4 + i, 0xffffffff);
					dev->resources[i].limit = _pci_get(b, d, f, 4 + i);
					dev->resources[i].limit = (1 << hal_cpuGetFirstBit(dev->resources[i].limit & ((dev->resources[i].limit & 1) ? ~0x03 : ~0xf)));

					_pci_set(b, d, f, 4 + i, dev->resources[i].base);
				}

				/* Add device to list */
				_pci_add(&pci_common.devices, dev);
				
				/*main_printf(ATTR_DEV, ".");*/
				//printf(".");
				printf("\n:%2u:%2u:%2u-->%6u,%6u-->%3u,%3u",
					dev->b,dev->d,dev->f,dev->device & 0xFFFF,dev->vendor & 0xFFFF,
					(dev->cl >> 8) & 0xFF,dev->cl & 0xFF);

				if (((dev->type & 0x80) == 0) && f == 0)
					break; /* not a multifunction device */
			}
		}
	}
	
	/*main_printf(ATTR_DEV, "\n");*/
	//printf("\n");	

	return;
}

int main() {
	u32 port;

	printf("\npci bus: Initializing %s\n","");
	_pci_init();
	printf("\npci bus: Initialized %s\n","");

	portCreate(&port);
	if (portRegister(port, "/dev/pci") < 0) {
		printf("Can't register port %d\n", port);	
		return -1;
	} else {
		printf("Registered port %d\n", port);
	}

	for(;;) {
		
		usleep(100);
	}
	return 0;
}
