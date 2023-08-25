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
#include <unistd.h>
#include <string.h>

#include <sys/io.h>
#include <sys/list.h>
#include <sys/threads.h>

#include "ata.h"

#define ATA_SECTORSZ_MAX 4096 /* Max supported sector size */


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
		addr &= ~0xf;
		addr += reg;

		switch (size) {
		case 1:
			return *(volatile uint8_t *)addr;
		case 2:
			return le16toh(*(volatile uint16_t *)addr);
		case 4:
			return le32toh(*(volatile uint32_t *)addr);
		}
	}

	return -EINVAL;
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
		addr &= ~0xf;
		addr += reg;

		switch (size) {
		case 1:
			*(volatile uint8_t *)addr = (uint8_t)val;
			break;
		case 2:
			*(volatile uint16_t *)addr = htole16((uint16_t)val);
			break;
		case 4:
			*(volatile uint32_t *)addr = htole32(val);
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


static int ata_wait(ata_bus_t *bus, uint8_t clear, uint8_t set, uint8_t err)
{
	uint8_t status;

	do {
		if ((status = (uint8_t)ata_readreg(bus->base, REG_STATUS, 1)) & err)
			return -1;
	} while ((status & clear) || (status & set) != set);

	return EOK;
}


static int ata_select(ata_dev_t *dev, uint64_t lba, uint16_t sectors, uint8_t mode)
{
	static ata_dev_t *ldev = NULL;
	ata_bus_t *bus = dev->bus;
	void *base = bus->base;
	uint16_t c = 0, h = 0, s = 0;
	int err;

	/* Select addressing mode */
	switch (mode) {
	case CHS:
		/* Convert LBA to CHS */
		c = (uint16_t)(lba / ((uint64_t)dev->heads * dev->sectors));
		h = (uint16_t)(lba / dev->sectors % dev->heads) & DEVSEL_HEAD;
		s = (uint16_t)(lba % dev->sectors) + 1;
		break;

	case LBA28:
		h = (uint16_t)(lba >> 24) & DEVSEL_HEAD;
	case LBA48:
		c = (uint16_t)(lba >> 8);
		h |= DEVSEL_LBA;
		s = (uint16_t)(lba >> 0);
		break;
	}

	if (dev != ldev) {
		/* Select the device */
		ata_writereg(base, REG_DEVSEL, (h & 0xff) | (dev == bus->devs[SLAVE]) * DEVSEL_DEVNUM | DEVSEL_SET0 | DEVSEL_SET1, 1);
		/* Wait for the device to push its status onto the bus */
		ata_delay(bus);

		/* Write features - set PIO mode */
		ata_writereg(base, REG_FEATURES, 0x03, 1);
		ata_writereg(base, REG_NSECTORS, dev->pio, 1);

		/* Send set features command */
		ata_writereg(base, REG_CMD, CMD_SET_FEATUERS, 1);
		/* Wait until BSY clears (ignore error status bit) */
		if ((err = ata_wait(bus, STATUS_BSY, 0, STATUS_DF)) < 0)
			return err;

		/* Don't update ldev on device initialization */
		if (mode != -1)
			ldev = dev;
	}

	/* Write other registers */
	if (mode == LBA48) {
		ata_writereg(base, REG_NSECTORS,  (sectors  >>  8) & 0xff, 1);
		ata_writereg(base, REG_SECTOR,    (lba      >> 24) & 0xff, 1);
		ata_writereg(base, REG_LCYLINDER, (lba      >> 32) & 0xff, 1);
		ata_writereg(base, REG_HCYLINDER, (lba      >> 40) & 0xff, 1);
	}
	ata_writereg(base, REG_NSECTORS,  sectors  & 0xff, 1);
	ata_writereg(base, REG_SECTOR,    (s >> 0) & 0xff, 1);
	ata_writereg(base, REG_LCYLINDER, (c >> 0) & 0xff, 1);
	ata_writereg(base, REG_HCYLINDER, (c >> 8) & 0xff, 1);

	return EOK;
}


static ssize_t ata_pio(ata_dev_t *dev, uint16_t sectors, uint8_t *buff, uint8_t dir)
{
	ata_bus_t *bus = dev->bus;
	void *base = bus->base;
	ssize_t ret = 0;
	uint16_t data, i;
	uint32_t j;
	int err;

	for (i = 0; i < sectors; i++) {
		/* Wait until BSY clears and DRQ sets */
		if ((err = ata_wait(bus, STATUS_BSY, STATUS_DRQ, STATUS_ERR | STATUS_DF)) < 0)
			return err;

		switch (dir) {
		case READ:
			/* Read one sector */
			for (j = 0; j < dev->sectorsz; j += 2) {
				data = (uint16_t)ata_readreg(base, REG_DATA, 2);
				(buff + ret)[j + 0] = (data >> 0) & 0xff;
				(buff + ret)[j + 1] = (data >> 8) & 0xff;
			}
			break;

		case WRITE:
			/* Write one sector */
			for (j = 0; j < dev->sectorsz; j += 2) {
				data  = (buff + ret)[j + 0] << 0;
				data |= (buff + ret)[j + 1] << 8;
				ata_writereg(base, REG_DATA, data, 2);
			}
			break;
		}

		ret += dev->sectorsz;
	}

	/* Wait until DRQ clears and RDY sets */
	if ((err = ata_wait(bus, STATUS_DRQ, STATUS_RDY, STATUS_ERR | STATUS_DF)) < 0)
		return err;

	return ret;
}


static ssize_t _ata_access(ata_dev_t *dev, uint64_t lba, uint16_t sectors, uint8_t cmd, uint8_t *buff)
{
	ata_bus_t *bus = dev->bus;
	void *base = bus->base;
	ssize_t ret;

	/* Wait until BSY clears (ignore error status bit) */
	if ((ret = ata_wait(bus, STATUS_BSY, 0, STATUS_DF)) < 0)
		return ret;

	/* Select the device and prepare for the transfer */
	if ((ret = ata_select(dev, lba, sectors, dev->mode)) < 0)
		return ret;

	/* Send the command */
	ata_writereg(base, REG_CMD, cmd, 1);

	/* Do the transfer */
	switch (cmd) {
	case CMD_READ_PIO:
	case CMD_READ_PIO_EXT:
		ret = ata_pio(dev, sectors, buff, READ);
		break;

	case CMD_WRITE_PIO:
	case CMD_WRITE_PIO_EXT:
		if ((ret = ata_pio(dev, sectors, buff, WRITE)) < 0)
			break;

		/* Flush the hardware cache */
		cmd = (dev->mode == LBA48) ? CMD_CACHE_FLUSH_EXT : CMD_CACHE_FLUSH;
		ata_writereg(base, REG_CMD, cmd, 1);
		break;
	}

	return ret;
}


ssize_t _ata_byteAccess(ata_dev_t *dev, offs_t offs, char *buff, size_t len, uint8_t cmd)
{
	ssize_t ret;
	ssize_t cnt = 0;
	uint8_t readCmd;
	int rmw;
	uint8_t sectorBuff[ATA_SECTORSZ_MAX];

	if (cmd == CMD_WRITE_PIO) {
		rmw = 1;
		readCmd = CMD_READ_PIO;
	}
	else if (cmd == CMD_WRITE_PIO_EXT) {
		rmw = 1;
		readCmd = CMD_READ_PIO_EXT;
	}
	else {
		rmw = 0;
	}

	/* Handle unaligned beginning */
	offs_t offsMisalign = offs % (offs_t)dev->sectorsz;
	if (offsMisalign != 0) {
		size_t chunk = dev->sectorsz - offsMisalign;
		if (chunk > len) {
			chunk = len;
		}

		if (rmw != 0) {
			ret = _ata_access(dev, offs / (offs_t)dev->sectorsz, 1, readCmd, sectorBuff);
			if (ret < 0) {
				return ret;
			}
			memcpy(sectorBuff + offsMisalign, buff, chunk);
		}

		ret = _ata_access(dev, offs / (offs_t)dev->sectorsz, 1, cmd, sectorBuff);
		if (ret < 0) {
			return ret;
		}

		if (rmw == 0) {
			memcpy(buff, sectorBuff + offsMisalign, chunk);
		}

		offs += dev->sectorsz - offsMisalign;
		len -= chunk;
		buff += chunk;
		cnt += chunk;
	}

	/* Handle aligned part */
	size_t lenAligned = len - (len % (size_t)dev->sectorsz);
	if (lenAligned > 0) {
		ret = _ata_access(dev, offs / (offs_t)dev->sectorsz, lenAligned / dev->sectorsz, cmd, (uint8_t *)buff);
		if (ret < 0) {
			return ret;
		}

		len -= lenAligned;
		offs += lenAligned;
		buff += lenAligned;
		cnt += lenAligned;
	}

	/* Handle unaligned end */
	if (len > 0) {
		if (rmw != 0) {
			ret = _ata_access(dev, offs / (offs_t)dev->sectorsz, 1, readCmd, sectorBuff);
			if (ret < 0) {
				return ret;
			}
			memcpy(sectorBuff, buff, len);
		}

		ret = _ata_access(dev, offs / (offs_t)dev->sectorsz, 1, cmd, sectorBuff);
		if (ret < 0) {
			return ret;
		}

		if (rmw == 0) {
			memcpy(buff, sectorBuff, len);
		}

		cnt += len;
	}

	return cnt;
}


ssize_t ata_read(ata_dev_t *dev, offs_t offs, char *buff, size_t len)
{
	uint8_t cmd;
	ssize_t ret;

	if (!len)
		return 0;

	switch (dev->mode) {
	case CHS:
	case LBA28:
		cmd = CMD_READ_PIO;
		break;

	case LBA48:
		cmd = CMD_READ_PIO_EXT;
		break;

	default:
		return -1;
	}

	mutexLock(dev->bus->lock);
	ret = _ata_byteAccess(dev, offs, buff, len, cmd);
	mutexUnlock(dev->bus->lock);

	return ret;
}


ssize_t ata_write(ata_dev_t *dev, offs_t offs, const char *buff, size_t len)
{
	uint8_t cmd;
	ssize_t ret;

	if (!len)
		return 0;

	switch (dev->mode) {
	case CHS:
	case LBA28:
		cmd = CMD_WRITE_PIO;
		break;

	case LBA48:
		cmd = CMD_WRITE_PIO_EXT;
		break;

	default:
		return -1;
	}

	mutexLock(dev->bus->lock);
	ret = _ata_byteAccess(dev, offs, (char *)buff, len, cmd);
	mutexUnlock(dev->bus->lock);

	return ret;
}


static int ata_initdev(ata_bus_t *bus, ata_dev_t *dev)
{
	void *base = bus->base;
	uint8_t info[512];
	uint16_t data;
	int err, i;

	dev->pio = PIO_DEFAULT;
	dev->bus = bus;

	/* Select the device */
	if ((err = ata_select(dev, 0, 0, -1)) < 0)
		return err;

	/* Floating bus check */
	if (ata_readreg(base, REG_STATUS, 1) == 0xff)
		return -ENODEV;

	/* Send identify command */
	ata_writereg(base, REG_CMD, CMD_IDENTIFY, 1);

	/* Status = 0 => device doesn't exist */
	if (!ata_readreg(base, REG_STATUS, 1))
		return -ENODEV;

	/* Wait until BSY clears and DRQ sets */
	if ((err = ata_wait(bus, STATUS_BSY, STATUS_DRQ, STATUS_ERR | STATUS_DF)) < 0)
		return err;

	/* Check if it's ATA device */
	if ((ata_readreg(base, REG_HCYLINDER, 1) << 8) | ata_readreg(base, REG_LCYLINDER, 1))
		return -ENODEV;

	/* Read device identification info */
	for (i = 0; i < sizeof(info) / sizeof(info[0]); i += 2) {
		data = (uint16_t)ata_readreg(base, REG_DATA, 2);
		info[i + 0] = (data >> 8) & 0xff;
		info[i + 1] = (data >> 0) & 0xff;
	}

	/* Wait until DRQ clears and RDY sets */
	if ((err = ata_wait(bus, STATUS_DRQ, STATUS_RDY, STATUS_ERR | STATUS_DF)) < 0)
		return err;

	/* Check advanced PIO mode support */
	if ((info[107] << 0) & 2) {
		switch ((info[129] << 0) | (info[128] << 8)) {
		case 0x01:
			dev->pio = PIO_3;
			break;

		case 0x03:
			dev->pio = PIO_4;
			break;
		}
	}

	dev->mode = ((info[98] >> 1) & 1) + ((info[166] >> 2) & 1);
	dev->cylinders = (info[3]   << 0) | (info[2]   << 8);
	dev->heads     = (info[7]   << 0) | (info[6]   << 8);
	dev->sectors   = (info[13]  << 0) | (info[12]  << 8);
	dev->sectorsz  = (info[235] << 0) | (info[234] << 8) | (info[237] << 16) | (info[236] << 24);

	/* sectorsz = 0 => use default ATA sector size */
	if (dev->sectorsz == 0) {
		dev->sectorsz = 512;
	}

	if (dev->sectorsz > ATA_SECTORSZ_MAX) {
		return -E2BIG;
	}

	switch (dev->mode) {
	case CHS:
		dev->size = (uint64_t)dev->cylinders * dev->heads * dev->sectors;
		break;

	case LBA28:
		dev->size =
			((uint64_t)info[121] <<  0) |
			((uint64_t)info[120] <<  8) |
			((uint64_t)info[123] << 16) |
			((uint64_t)info[122] << 24);
		break;

	case LBA48:
		dev->size =
			((uint64_t)info[201] <<  0) |
			((uint64_t)info[200] <<  8) |
			((uint64_t)info[203] << 16) |
			((uint64_t)info[202] << 24) |
			((uint64_t)info[205] << 32) |
			((uint64_t)info[204] << 40) |
			((uint64_t)info[207] << 48) |
			((uint64_t)info[206] << 56);
		break;
	}
	dev->size *= dev->sectorsz;

	return EOK;
}


static int ata_initbus(void *base, void *ctrl, ata_bus_t *bus)
{
	int err;

	bus->base = base;
	bus->ctrl = ctrl;

	/* Floating bus check */
	if (ata_readreg(base, REG_STATUS, 1) == 0xff)
		return -ENODEV;

	if ((bus->devs[MASTER] = (ata_dev_t *)malloc(sizeof(ata_dev_t))) == NULL)
		return -ENOMEM;

	if ((bus->devs[SLAVE] = (ata_dev_t *)malloc(sizeof(ata_dev_t))) == NULL) {
		free(bus->devs[MASTER]);
		return -ENOMEM;
	}

	/* Software reset the bus */
	ata_resetbus(bus);

	if (ata_initdev(bus, bus->devs[MASTER]) < 0) {
		free(bus->devs[MASTER]);
		bus->devs[MASTER] = NULL;
	}

	if (ata_initdev(bus, bus->devs[SLAVE]) < 0) {
		free(bus->devs[SLAVE]);
		bus->devs[SLAVE] = NULL;
	}

	if (bus->devs[MASTER] == NULL && bus->devs[SLAVE] == NULL)
		return -ENODEV;

	if ((err = mutexCreate(&bus->lock)) < 0) {
		free(bus->devs[MASTER]);
		free(bus->devs[SLAVE]);
		return err;
	}

	LIST_ADD(&buses, bus);

	if (bus->devs[MASTER] != NULL) {
		LIST_ADD(&ata_common.devs, bus->devs[MASTER]);
		ata_common.ndevs++;
	}

	if (bus->devs[SLAVE] != NULL) {
		LIST_ADD(&ata_common.devs, bus->devs[SLAVE]);
		ata_common.ndevs++;
	}

	return EOK;
}


int ata_init(void)
{
	ata_bus_t *bus1, *bus2;

	ata_common.ndevs = 0;
	ata_common.devs = NULL;
	buses = NULL;

	if ((bus1 = (ata_bus_t *)malloc(sizeof(ata_bus_t))) == NULL)
		return -ENOMEM;

	if ((bus2 = (ata_bus_t *)malloc(sizeof(ata_bus_t))) == NULL) {
		free(bus1);
		return -ENOMEM;
	}

	if (ata_initbus((void *)(ATA1_BASE | 0x1), (void *)(ATA1_CTRL | 0x1), bus1) < 0)
		free(bus1);

	if (ata_initbus((void *)(ATA2_BASE | 0x1), (void *)(ATA2_CTRL | 0x1), bus2) < 0)
		free(bus2);

	return EOK;
}
