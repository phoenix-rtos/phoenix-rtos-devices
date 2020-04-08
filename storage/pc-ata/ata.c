/*
 * Phoenix-RTOS
 *
 * Generic ATA controller driver
 *
 * Copyright 2012-2015, 2019, 2020 Phoenix Systems
 * Author: Marcin Stragowski, Kamil Amanowicz, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <endian.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/list.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <arch/ia32/io.h>

#include "ata.h"


ata_common_t ata_common;


static ata_bus_t *buses;


static uint32_t ata_readreg(void *base, uint8_t reg, uint8_t size)
{
	uintptr_t addr = (uintptr_t)base;

	if (addr & 0x1) {
		/* Read from IO-port */
		addr &= ~0x3;
		addr += reg;

		switch (size) {
		case 1:
			return inb((void *)addr);
		case 2:
			return le16toh(inw((void *)addr));
		case 4:
			return le32toh(inl((void *)addr));
		}
	}
	else {
		/* Read from memory */
		addr &= ~0xF;
		addr += reg;

		switch (size) {
		case 1:
			return *((uint8_t *)addr);
		case 2:
			return le16toh(*((uint16_t *)addr));
		case 4:
			return le32toh(*((uint32_t *)addr));
		}
	}

	return -1;
}


static void ata_writereg(void *base, uint8_t reg, uint32_t val, uint8_t size)
{
	uintptr_t addr = (uintptr_t)base;

	if (addr & 0x1) {
		/* Write to IO-port */
		addr &= ~0x3;
		addr += reg;

		switch (size) {
		case 1:
			outb((void *)addr, (uint8_t)val);
			break;
		case 2:
			outw((void *)addr, htole16((uint16_t)val));
			break;
		case 4:
			outl((void *)addr, htole32(val));
			break;
		}
	}
	else {
		/* Write to memory */
		addr &= ~0xF;
		addr += reg;

		switch (size) {
		case 1:
			*((uint8_t *)addr) = (uint8_t)val;
			break;
		case 2:
			*((uint16_t *)addr) = htole16((uint16_t)val);
			break;
		case 4:
			*((uint32_t *)addr) = htole32(val);
			break;
		}
	};
}


static void ata_delay(ata_bus_t *bus)
{
	void *ctrl = bus->ctrl;

	/* Single status/altstatus register read takes 100ns */
	ata_readreg(ctrl, REG_ALTSTATUS, 1);
	ata_readreg(ctrl, REG_ALTSTATUS, 1);
	ata_readreg(ctrl, REG_ALTSTATUS, 1);
	ata_readreg(ctrl, REG_ALTSTATUS, 1);
}


static void ata_resetbus(ata_bus_t *bus)
{
	void *ctrl = bus->ctrl;

	/* Software reset all drives on the bus */
	ata_writereg(ctrl, REG_CTRL, CTRL_SOFTRST | CTRL_NIEN, 1);
	usleep(10);

	/* Reset the bus to normal operation */
	ata_writereg(ctrl, REG_CTRL, CTRL_NIEN, 1);
	usleep(10);
}


static void ata_select(ata_dev_t *dev, uint8_t devnum, uint64_t lba, uint16_t nsectors, uint8_t mode)
{
	ata_bus_t *bus = dev->bus;
	void *base = bus->base;
	uint16_t c = 0, h = 0, s = 0;

	/* Select addressing mode */
	switch (mode) {
	case AMODE_CHS:
		/* Convert LBA to CHS */
		c = (uint16_t)(lba / ((uint64_t)dev->heads * dev->sectors));
		h = (uint16_t)(lba / dev->sectors % dev->heads) & DEVSEL_HEAD;
		s = (uint16_t)(lba % dev->sectors) + 1;
		break;

	case AMODE_LBA28:
		h = (uint16_t)(lba >> 24) & DEVSEL_HEAD;
	case AMODE_LBA48:
		c = (uint16_t)(lba >> 8);
		h |= DEVSEL_LBA;
		s = (uint16_t)(lba >> 0);
		break;
	}

	/* Select the device */
	ata_writereg(base, REG_DEVSEL, h | devnum * DEVSEL_DEVNUM | DEVSEL_SET0 | DEVSEL_SET1, 1);
	/* Wait for the device to push its status onto the bus */
	ata_delay(bus);

	/* Write features */
	ata_writereg(base, REG_FEATURES, 0, 1);

	/* Write other registers */
	if (mode == AMODE_LBA48) {
		ata_writereg(base, REG_NSECTORS,  (nsectors >>  8) & 0xFF, 1);
		ata_writereg(base, REG_SECTOR,    (lba      >> 24) & 0xFF, 1);
		ata_writereg(base, REG_LCYLINDER, (lba      >> 32) & 0xFF, 1);
		ata_writereg(base, REG_HCYLINDER, (lba      >> 40) & 0xFF, 1);
	}
	ata_writereg(base, REG_NSECTORS,  nsectors & 0xFF, 1);
	ata_writereg(base, REG_SECTOR,    (s >> 0) & 0xFF, 1);
	ata_writereg(base, REG_LCYLINDER, (c >> 0) & 0xFF, 1);
	ata_writereg(base, REG_HCYLINDER, (c >> 8) & 0xFF, 1);
}


static int ata_wait(ata_bus_t *bus, uint8_t clear, uint8_t set)
{
	uint8_t status;

	do {
		status = (uint8_t)ata_readreg(bus->base, REG_STATUS, 1);

		if (status & STATUS_ERR)
			return -1;

		if (status & STATUS_DF)
			return -2;

	} while ((status & clear) || (status & set) != set);

	return 0;
}


static ssize_t ata_pio(ata_dev_t *dev, uint16_t nsectors, uint8_t *buff, uint8_t dir)
{
	ata_bus_t *bus = dev->bus;
	void *base = bus->base;
	uint16_t data;
	int i, j;
	ssize_t err, ret = 0;

	for (i = 0; i < nsectors; i++) {
		/* Wait until BSY clears and DRQ sets */
		if ((err = ata_wait(bus, STATUS_BSY, STATUS_DRQ)))
			return err;

		switch (dir) {
		case DIR_READ:
			/* Read one sector */
			for (j = 0; j < dev->secsize; j += 2) {
				data = (uint16_t)ata_readreg(base, REG_DATA, 2);
				(buff + ret)[j + 0] = (data >> 0) & 0xFF;
				(buff + ret)[j + 1] = (data >> 8) & 0xFF;
			}
			break;

		case DIR_WRITE:
			/* Write one sector */
			for (j = 0; j < dev->secsize; j += 2) {
				data  = (buff + ret)[j + 0] << 0;
				data |= (buff + ret)[j + 1] << 8;
				ata_writereg(base, REG_DATA, data, 2);
			}
			break;
		}

		ret += dev->secsize;
	}

	/* Wait until DRQ clears and RDY sets */
	if ((err = ata_wait(bus, STATUS_DRQ, STATUS_RDY)))
		return err;

	return ret;
}


static ssize_t _ata_access(ata_dev_t *dev, uint64_t lba, uint16_t nsectors, uint8_t cmd, uint8_t *buff)
{
	ata_bus_t *bus = dev->bus;
	void *base = bus->base;
	ssize_t err, ret = -1;

	/* Wait until BSY clears */
	if ((err = ata_wait(bus, STATUS_BSY, 0)))
		return err;

	/* Select the device and prepare for the transfer */
	ata_select(dev, (dev == bus->devs[SLAVE]), lba, nsectors, dev->mode);

	/* Send the command */
	ata_writereg(base, REG_CMD, cmd, 1);

	/* Do the transfer */
	switch (cmd) {
	case CMD_READ_PIO:
	case CMD_READ_PIO_EXT:
		ret = ata_pio(dev, nsectors, buff, DIR_READ);
		break;

	case CMD_WRITE_PIO:
	case CMD_WRITE_PIO_EXT:
		if ((ret = ata_pio(dev, nsectors, buff, DIR_WRITE)) < 0)
			break;

		/* Flush the hardware cache */
		cmd = (dev->mode == AMODE_LBA48) ? CMD_CACHE_FLUSH_EXT : CMD_CACHE_FLUSH;
		ata_writereg(base, REG_CMD, cmd, 1);
		break;
	}

	return ret;
}


ssize_t ata_read(ata_dev_t *dev, offs_t offs, char *buff, size_t len)
{
	uint8_t cmd;
	ssize_t ret;

	switch (dev->mode) {
	case AMODE_CHS:
	case AMODE_LBA28:
		cmd = CMD_READ_PIO;
		break;

	case AMODE_LBA48:
		cmd = CMD_READ_PIO_EXT;
		break;

	default:
		return -1;
	}

	mutexLock(dev->bus->lock);
	ret = _ata_access(dev, (uint64_t)(offs / dev->secsize), (uint16_t)(len / dev->secsize), cmd, (uint8_t *)buff);
	mutexUnlock(dev->bus->lock);

	return ret;
}


ssize_t ata_write(ata_dev_t *dev, offs_t offs, const char *buff, size_t len)
{
	uint8_t cmd;
	ssize_t ret;

	switch (dev->mode) {
	case AMODE_CHS:
	case AMODE_LBA28:
		cmd = CMD_WRITE_PIO;
		break;

	case AMODE_LBA48:
		cmd = CMD_WRITE_PIO_EXT;
		break;

	default:
		return -1;
	}

	mutexLock(dev->bus->lock);
	ret = _ata_access(dev, (uint64_t)(offs / dev->secsize), (uint16_t)(len / dev->secsize), cmd, (uint8_t *)buff);
	mutexUnlock(dev->bus->lock);

	return ret;
}


static int ata_initdev(ata_bus_t *bus, ata_dev_t *dev, uint8_t devnum)
{
	void *base = bus->base;
	uint8_t hcyl, lcyl, info[INFO_SIZE];
	uint16_t data;
	int err, i;

	dev->bus = bus;

	/* Select the device */
	ata_select(dev, devnum, 0, 0, -1);

	/* Floating bus check */
	if (ata_readreg(base, REG_STATUS, 1) == 0xFF)
		return -ENXIO;

	/* Send identify command */
	ata_writereg(base, REG_CMD, CMD_IDENTIFY, 1);

	/* Status = 0 => device doesn't exist */
	if (!ata_readreg(base, REG_STATUS, 1))
		return -ENXIO;

	lcyl = (uint8_t)ata_readreg(base, REG_LCYLINDER, 1);
	hcyl = (uint8_t)ata_readreg(base, REG_HCYLINDER, 1);

	/* Detect device type */
	if (!lcyl && !hcyl) {
		dev->type = ATA;
	}
	else if (lcyl == 0x3C && hcyl == 0xC3) {
		dev->type = SATA;
	}
	else if (lcyl == 0x14 && hcyl == 0xEB) {
		dev->type = ATAPI;
		/* Send identify packet command */
		ata_writereg(base, REG_CMD, CMD_IDENTIFY_PACKET, 1);
	}
	else if (lcyl == 0x69 && hcyl == 0x96) {
		dev->type = SATAPI;
		/* Send identify packet command */
		ata_writereg(base, REG_CMD, CMD_IDENTIFY_PACKET, 1);
	}
	else {
		/* Unknown device */
		return -ENXIO;
	}

	/* Wait until BSY clears and DRQ sets */
	if ((err = ata_wait(bus, STATUS_BSY, STATUS_DRQ)))
		return err;

	/* Read device info */
	for (i = 0; i < INFO_SIZE; i += 2) {
		data = (uint16_t)ata_readreg(base, REG_DATA, 2);
		info[i + 0] = (data >> 8) & 0xFF;
		info[i + 1] = (data >> 0) & 0xFF;
	}

	/* Wait until DRQ clears and RDY sets */
	if ((err = ata_wait(bus, STATUS_DRQ, STATUS_RDY)))
		return err;

	if (dev->type == ATA) {
		dev->mode = ((info[98] >> 1) & 1) + ((info[166] >> 2) & 1);
		dev->cylinders = (info[3]   << 0) | (info[2]   << 8);
		dev->heads     = (info[7]   << 0) | (info[6]   << 8);
		dev->sectors   = (info[13]  << 0) | (info[12]  << 8);
		dev->secsize   = (info[235] << 0) | (info[234] << 8) | (info[237] << 16) | (info[236] << 24);

		if (!dev->secsize)
			dev->secsize = SECTOR_SIZE;

		switch (dev->mode) {
		case AMODE_CHS:
			dev->size = (uint64_t)dev->cylinders * dev->heads * dev->sectors;
			break;

		case AMODE_LBA28:
			dev->size =
				(info[121] <<  0) |
				(info[120] <<  8) |
				(info[123] << 16) |
				(info[122] << 24);
			break;

		case AMODE_LBA48:
			dev->size =
				(((uint64_t)info[201]) <<  0) |
				(((uint64_t)info[200]) <<  8) |
				(((uint64_t)info[203]) << 16) |
				(((uint64_t)info[202]) << 24) |
				(((uint64_t)info[205]) << 32) |
				(((uint64_t)info[204]) << 40) |
				(((uint64_t)info[207]) << 48) |
				(((uint64_t)info[206]) << 56);
			break;
		}
		dev->size *= dev->secsize;
	}

	return EOK;
}


static int ata_initbus(void *base, void *ctrl, ata_bus_t *bus)
{
	bus->base = base;
	bus->ctrl = ctrl;

	/* Floating bus check */
	if (ata_readreg(base, REG_STATUS, 1) == 0xFF)
		return -ENXIO;

	if ((bus->devs[MASTER] = malloc(sizeof(ata_dev_t))) == NULL)
		return -ENOMEM;
	
	if ((bus->devs[SLAVE] = malloc(sizeof(ata_dev_t))) == NULL) {
		free(bus->devs[MASTER]);
		return -ENOMEM;
	}

	/* Software reset the bus */
	ata_resetbus(bus);

	if (ata_initdev(bus, bus->devs[MASTER], MASTER)) {
		free(bus->devs[MASTER]);
		bus->devs[MASTER] = NULL;
	}

	if (ata_initdev(bus, bus->devs[SLAVE], SLAVE)) {
		free(bus->devs[SLAVE]);
		bus->devs[SLAVE] = NULL;
	}

	if (bus->devs[MASTER] == NULL && bus->devs[SLAVE] == NULL)
		return -ENXIO;

	if (mutexCreate(&bus->lock) < 0) {
		free(bus->devs[MASTER]);
		free(bus->devs[SLAVE]);
		return -ENOMEM;
	}

	LIST_ADD(&buses, bus);

	if (bus->devs[MASTER] != NULL) {
		LIST_ADD(&ata_common.devices, bus->devs[MASTER]);
		ata_common.count++;
	}

	if (bus->devs[SLAVE] != NULL) {
		LIST_ADD(&ata_common.devices, bus->devs[SLAVE]);
		ata_common.count++;
	}

	return EOK;
}


int ata_init(void)
{
	ata_bus_t *bus1, *bus2;

	ata_common.count = 0;
	ata_common.devices = NULL;
	buses = NULL;

	if ((bus1 = malloc(sizeof(ata_bus_t))) == NULL)
		return -ENOMEM;

	if ((bus2 = malloc(sizeof(ata_bus_t))) == NULL) {
		free(bus1);
		return -ENOMEM;
	}

	if (ata_initbus((void *)(ATA1_BASE | 0x1), (void *)(ATA1_CTRL | 0x1), bus1))
		free(bus1);

	if (ata_initbus((void *)(ATA2_BASE | 0x1), (void *)(ATA2_CTRL | 0x1), bus2))
		free(bus2);

	return EOK;
}
