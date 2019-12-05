/*
 * Phoenix-RTOS
 *
 * Generic ata controller driver
 *
 * Copyright 2012, 2018, 2019 Phoenix Systems
 * Author: Marcin Stragowski, Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <arch.h>
#include <errno.h>

#include <sys/threads.h>
#include <sys/msg.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#include <arch/ia32/io.h>

#include "atadrv.h"
#include "atasrv_priv.h"

static pci_id_t ata_pci_tbl[] = {
	{ PCI_ANY, PCI_ANY, PCI_ANY, PCI_ANY, 0x0101},
	{ 0x1106, 0x3249, PCI_ANY, PCI_ANY, 0x0104},
	{0,}
};

ata_opt_t ata_defaults = {
	.force = 0,
	.use_int = 0,
	.use_dma = 0,
	.use_multitransfer = 0
};

struct ata_bus buses[8] = {};
int buses_cnt = 0;
pci_device_t pci_dev[8] = {};
uint32_t port;

static int ata_interrupt(unsigned int irq, void *dev_instance);

#define ata_ch_read(ac, reg) inb((void *)0 + (ac)->reg_addr[(reg)])
#define ata_ch_write(ac, reg, data) outb((void *)0 + (ac)->reg_addr[(reg)], (data))
#define ata_ch_read_buffer(ac, reg, buff, quads) insl((void *)0 + (ac)->reg_addr[(reg)], (buff), (quads))

static inline void insl(void *addr, void *buffer, uint32_t quads)\
{
	int i;
	for (i = 0; i < quads; i += 1)
		*(uint32_t*)(buffer + (i * 4)) = inl(addr);
}

void ata_chInitRegs(struct ata_channel *ac)
{
	/* primary registers */
	ac->reg_addr[0x00] = ac->base + 0;
	ac->reg_addr[0x01] = ac->base + 1;
	ac->reg_addr[0x02] = ac->base + 2;
	ac->reg_addr[0x03] = ac->base + 3;
	ac->reg_addr[0x04] = ac->base + 4;
	ac->reg_addr[0x05] = ac->base + 5;
	ac->reg_addr[0x06] = ac->base + 6;
	ac->reg_addr[0x07] = ac->base + 7;
	/* additional registers - accessible
	 * only after proper ATA_REG_CONTROL*/
	ac->reg_addr[0x08] = ac->base + 2;
	ac->reg_addr[0x09] = ac->base + 3;
	ac->reg_addr[0x0A] = ac->base + 4;
	ac->reg_addr[0x0B] = ac->base + 5;
	/* control register */
	ac->reg_addr[0x0C] = ac->ctrl + 2;
	ac->reg_addr[0x0D] = ac->ctrl + 3;
	/* bus master stuff */
	ac->reg_addr[0x0E] = ac->bmide + 0;
	ac->reg_addr[0x0F] = ac->bmide + 1;
	ac->reg_addr[0x10] = ac->bmide + 2;
	ac->reg_addr[0x11] = ac->bmide + 3;
	ac->reg_addr[0x12] = ac->bmide + 4;
	ac->reg_addr[0x13] = ac->bmide + 5;
	ac->reg_addr[0x14] = ac->bmide + 6;
	ac->reg_addr[0x15] = ac->bmide + 7;
}

#define ATA_WASTE400MS(ac) do { ata_ch_read(ac, ATA_REG_ALTSTATUS); \
	ata_ch_read(ac, ATA_REG_ALTSTATUS); \
	ata_ch_read(ac, ATA_REG_ALTSTATUS); \
	ata_ch_read(ac, ATA_REG_ALTSTATUS); } while(0)

#define ATA_WASTE100MS(ac) do { ata_ch_read(ac, ATA_REG_ALTSTATUS); } while(0)


/*
 * advanced_check = 1 reads ATA_REG_STATUS
 * advanced_check = 2 reads ATA_REG_ALTSTATUS
 */
int ata_polling(struct ata_channel *ac, uint8_t advanced_check)
{
	ATA_WASTE400MS(ac);

	while (ata_ch_read(ac, ATA_REG_STATUS) & ATA_SR_BSY);

	if (advanced_check) {
		uint8_t state = ata_ch_read(ac, (advanced_check == 2 ? ATA_REG_ALTSTATUS : ATA_REG_STATUS));

		if (state & ATA_SR_ERR)
			return -1; // Error.

		if (state & ATA_SR_DF)
			return -2; // Device Fault.

		if ((state & ATA_SR_DRQ) == 0)
			return -3; // DRQ should be set
	}

	return 0;
}

int ata_wait(struct ata_channel *ac, uint8_t irq)
{
	int err = 0;

	if (irq) {
		mutexLock(ac->irq_spin);
		if (ac->irq_invoked == 0) {
			if ((err = condWait(ac->waitq, ac->irq_spin, 0)) < 0) {
				mutexUnlock(ac->irq_spin);
				return err;
			}
		}
		ac->irq_invoked = 0;
		mutexUnlock(ac->irq_spin);
	}
	return ata_polling(ac, 0);
}


/* TODO: divide to dma_setup, command_setup, pio_access, dma_access */
int ata_access(uint8_t direction, struct ata_dev *ad, uint32_t lba, uint8_t numsects, void *buffer)
{
	struct ata_channel *ac = ad->ac;

	uint8_t lba_mode = 0; /* 0: CHS, 1:LBA28, 2: LBA48 */
	uint8_t cmd;
	uint8_t astatus = 0;
	uint8_t slavebit = ad->drive;
	uint8_t lba_io[6] = { 0 };
	uint8_t head, sect;
	uint16_t bus = ac->base;
	uint16_t words = 256;
	uint16_t cyl, i, b;

	int err = 0;

	ata_ch_write(ac, ATA_REG_CONTROL, ac->no_int << 1);

	ata_ch_write(ac, ATA_REG_BMSTATUS, ATA_BMR_STAT_ERR);

	if (ad->info.capabilities_1 & 0x200)  { // Drive supports LBA?
		/* LBA28 */
		lba_mode  = 1;
		lba_io[0] = (lba & 0x000000FF) >> 0;
		lba_io[1] = (lba & 0x0000FF00) >> 8;
		lba_io[2] = (lba & 0x00FF0000) >> 16;
		head      = (lba & 0x0F000000) >> 24;
	} else {
		/* CHS: */
		lba_mode  = 0;
		sect      = (lba % 63) + 1;
		cyl       = (lba + 1  - sect) / (16 * 63);
		lba_io[0] = sect;
		lba_io[1] = (cyl >> 0) & 0xFF;
		lba_io[2] = (cyl >> 8) & 0xFF;
		head      = (lba + 1  - sect) % (16 * 63) / (63); // Head number is written to HDDEVSEL lower 4-bits.
	}

	while (ata_ch_read(ac, ATA_REG_STATUS) & (ATA_SR_BSY | ATA_SR_DRQ));

	//  Select Drive from the controller;
	if (lba_mode == 0) {
		ata_ch_write(ac, ATA_REG_HDDEVSEL, 0xA0 | (slavebit << 4) | head); // Drive & CHS.
	} else {
		ata_ch_write(ac, ATA_REG_HDDEVSEL, 0xE0 | (slavebit << 4) | head); // Drive & LBA
	}

	while (ata_ch_read(ac, ATA_REG_STATUS) & (ATA_SR_BSY | ATA_SR_DRQ));

	ata_ch_write(ac, ATA_REG_SECCOUNT0, numsects);
	ata_ch_write(ac, ATA_REG_LBA0, lba_io[0]);
	ata_ch_write(ac, ATA_REG_LBA1, lba_io[1]);
	ata_ch_write(ac, ATA_REG_LBA2, lba_io[2]);

	cmd = direction ? ATA_CMD_WRITE_PIO : ATA_CMD_READ_PIO;

	ata_ch_write(ac, ATA_REG_COMMAND, cmd);

	ATA_WASTE400MS(ac);
	if (direction == 0) {
		// PIO Read.
		i = 0;
		while (i < numsects) {
			err = ata_wait(ac, !ac->no_int);
			if (ac->no_int)
				ac->status = ata_ch_read(ac, ATA_REG_STATUS);

			if (err) break;

			if ((ac->status & (ATA_SR_BSY | ATA_SR_DRQ)) == ATA_SR_DRQ) {
				for (b = 0; b < words; b++)
					*(uint16_t*)(buffer + (b * 2)) = inw((void *)0 + bus);

				buffer += (words * 2);
				i++;
				ATA_WASTE400MS(ac);
			}

			if (ac->status & (ATA_SR_BSY | ATA_SR_DF | ATA_SR_ERR))
				break;

			if (!(ac->status & ATA_SR_DRQ))
				break;
		}
	} else {
		while ((astatus = ac->status = ata_ch_read(ac, ATA_REG_ALTSTATUS)) & ATA_SR_BSY); // Wait for BSY to be zero.
		// PIO Write.
		i = 0;

		while (i < numsects) {

			if ((ac->status & (ATA_SR_BSY | ATA_SR_DRQ)) == ATA_SR_DRQ ) {
				for (b = 0; b < words; b++)
					outw((void *)0 + bus, *((uint16_t*)(buffer + (b*2))));

				buffer += (words * 2);
				i++;
				ATA_WASTE400MS(ac);
			}

			if (ac->status & (ATA_SR_BSY | ATA_SR_DF | ATA_SR_ERR))
				break;

			if (!(ac->status & ATA_SR_DRQ))
				break;

			if ((err = ata_wait(ac, !ac->no_int)) != 0)
				ata_polling(ac, 1); // Polling.

			if (ac->no_int || err)
				astatus = ac->status = ata_ch_read(ac, ATA_REG_STATUS);
			else
				astatus = ac->status;

		}

		ata_ch_write(ac, ATA_REG_COMMAND, ATA_CMD_CACHE_FLUSH);

		if (!ac->no_int) {
			if ((err = ata_wait(ac, 1)) != 0)
				ata_polling(ac, 1); // Polling.
		}
	}

	return i;
}

// TODO: fixme 64 bits
static int ata_io(struct ata_dev *ad, offs_t offs, char *buff, unsigned int len, int direction)
{
	uint32_t begin_lba = 0;
	uint32_t sectors = 0;
	uint32_t ret = 0;

	if (ad == NULL)
		return -EINVAL;

	if (!ad->reserved)
		return -ENOENT;

	if (((uint32_t)offs % ad->sector_size) || (len % ad->sector_size)) {
		printf("panic on the disco sector %s\n", "");
		return -EINVAL;
	}

	begin_lba = (uint32_t)offs / ad->sector_size; // starting sector
	sectors = len / ad->sector_size;

	for (; sectors >= ATA_MAX_PIO_DRQ - 1; sectors -= ATA_MAX_PIO_DRQ - 1) {
		ata_access(direction, ad, begin_lba, (uint8_t)ATA_MAX_PIO_DRQ - 1, buff);

		begin_lba += ATA_MAX_PIO_DRQ - 1;
		buff += (ATA_MAX_PIO_DRQ - 1) * ad->sector_size;
		ret += (ATA_MAX_PIO_DRQ - 1) * ad->sector_size;
	}

	if (sectors) {
		ata_access(direction, ad, begin_lba, (uint8_t)sectors, buff);
		ret += sectors * ad->sector_size;
	}

	return ret;
}


int atadrv_read(ata_dev_t *dev, offs_t offs, char *buff, unsigned int len)
{
	int ret;

	mutexLock(dev->ac->access_lock);
	ret = ata_io(dev, offs, buff, len, ATA_READ);
	mutexUnlock(dev->ac->access_lock);
	return ret;
}


int atadrv_write(ata_dev_t *dev, offs_t offs, const char *buff, unsigned int len)
{
	int ret;

	mutexLock(dev->ac->access_lock);
	ret = ata_io(dev, offs, buff, len, ATA_WRITE);
	mutexUnlock(dev->ac->access_lock);
	return ret;
}


static int ata_interrupt(unsigned int irq, void *dev_instance)
{
	struct ata_channel *ac = (struct ata_channel*)dev_instance;
	int res = 0; //IHRES_IGNORE;

	ac->bmstatus_irq = ata_ch_read(ac, ATA_REG_BMSTATUS); // Read Status Register.
	ac->status = ata_ch_read(ac, ATA_REG_STATUS); // Read Status Register.
	if (ac->bmstatus_irq & ATA_BMR_STAT_INTR)
		ata_ch_write(ac,ATA_REG_BMSTATUS,ATA_BMR_STAT_INTR);

	ac->irq_invoked = 1;

	return ac->waitq;
}


int ata_init_bus(struct ata_bus *ab)
{
	uint8_t i, j;
	uint32_t B0 = ab->dev->resources[0].base;
	uint32_t B1 = ab->dev->resources[1].base;
	uint32_t B2 = ab->dev->resources[2].base;
	uint32_t B3 = ab->dev->resources[3].base;
	uint32_t B4 = ab->dev->resources[4].base;

	ab->ac[ATA_PRIMARY].no_int = ab->config.use_int ? 0 : 1;
	ab->ac[ATA_SECONDARY].no_int = ab->config.use_int ? 0 : 1;
	ab->ac[ATA_PRIMARY].ab = ab;
	ab->ac[ATA_SECONDARY].ab = ab;

	/* special case - some controllers report 255 instead of 0 if they do not have irq assigned */
	if (ab->dev->irq == 255)
		ab->dev->irq = 0;

	ab->ac[ATA_PRIMARY].irq_reg = (ab->dev->irq) + ATA_DEF_INTR_PRIMARY * (!ab->dev->irq);
	ab->ac[ATA_SECONDARY].irq_reg = (ab->dev->irq) + ATA_DEF_INTR_SECONDARY * (!ab->dev->irq);

	ab->ac[ATA_PRIMARY].base  = (B0 & 0xFFFFFFFC) + 0x1F0 * (!B0);
	ab->ac[ATA_PRIMARY].ctrl  = (B1 & 0xFFFFFFFC) + 0x3F4 * (!B1);

	ab->ac[ATA_SECONDARY].base  = (B2 & 0xFFFFFFFC) + 0x170 * (!B2);
	ab->ac[ATA_SECONDARY].ctrl  = (B3 & 0xFFFFFFFC) + 0x376 * (!B3);

	ab->ac[ATA_PRIMARY  ].bmide = (B4 & 0xFFFFFFFC) + ATA_REG_BMPRIMARY;
	ab->ac[ATA_SECONDARY].bmide = (B4 & 0xFFFFFFFC) + ATA_REG_BMSECONDARY;

	ata_chInitRegs(&(ab->ac[ATA_PRIMARY]));
	ata_chInitRegs(&(ab->ac[ATA_SECONDARY]));

	if (ab->ac[ATA_PRIMARY].no_int != 0)
		ata_ch_write(&(ab->ac[ATA_PRIMARY]), ATA_REG_CONTROL, 2);

	if (ab->ac[ATA_SECONDARY].no_int != 0)
		ata_ch_write(&(ab->ac[ATA_SECONDARY]), ATA_REG_CONTROL, 2);

	mutexCreate(&(ab->ac[ATA_PRIMARY].irq_spin));
	mutexCreate(&(ab->ac[ATA_SECONDARY].irq_spin));

	mutexCreate(&(ab->ac[ATA_PRIMARY].access_lock));
	mutexCreate(&(ab->ac[ATA_SECONDARY].access_lock));


	/* Detect ATA-ATAPI Devices: */
	for (i = 0; i < 2; i++) {

		condCreate(&(ab->ac[i].waitq));

		//ab->ac[i].bmstatus = ata_ch_read(&(ab->ac[i]), ATA_REG_BMSTATUS) & 0x60;

		for (j = 0; j < 2; j++) {
			uint8_t err = 0, status = 0;

			ab->ac[i].devices[j].ac = &ab->ac[i];
			ab->ac[i].devices[j].reserved = 0;

			ata_ch_write(&(ab->ac[i]), ATA_REG_HDDEVSEL, 0xA0 | (j << 4));

			ata_ch_write(&(ab->ac[i]), ATA_REG_COMMAND, ATA_CMD_IDENTIFY);

			if ((status = ata_ch_read(&(ab->ac[i]), ATA_REG_STATUS)) == 0)
				continue;

			while ((status & ATA_SR_BSY) && !(status & ATA_SR_DRQ)) {

				if ((status & ATA_SR_ERR) || (status & ATA_SR_DF)) {
					err = 1;
					break;
				}
				status = ata_ch_read(&(ab->ac[i]), ATA_REG_STATUS);
			}

			/* it's probably atapi device - we do not support it */
			if (err != 0)
				continue;

			ata_ch_read_buffer(&(ab->ac[i]), ATA_REG_DATA, &(ab->ac[i].devices[j].info), 128);

			ab->ac[i].devices[j].reserved = 1;
			ab->ac[i].devices[j].type = 0; /* IDE_ATA */
			ab->ac[i].devices[j].channel = i;
			ab->ac[i].devices[j].drive = j;
			ab->ac[i].devices[j].sector_size  = ab->ac[i].devices[j].info.log_sector_size;

			if (!ab->ac[i].devices[j].sector_size)
				ab->ac[i].devices[j].sector_size = ATA_DEF_SECTOR_SIZE;

			ab->ac[i].devices[j].size = ab->ac[i].devices[j].info.lba28_totalsectors;

			atasrv_registerDevice(&ab->ac[i].devices[j]);
			//printf("[%d:%d] %.5f GiB\n", i, j, (double)ab->ac[i].devices[j].size * ab->ac[i].devices[j].sector_size / 1000 / 1000 / 1000);
		}
	}

	/* TODO: rework interrupt handling (pass only bus not channel etc) */
	if (interrupt(ab->ac[0].irq_reg, ata_interrupt, (void *)&(ab->ac[0]), ab->ac[0].waitq, &ab->ac[0].inth) < 0)
		return -EINVAL;

	if (ab->ac[0].irq_reg != ab->ac[1].irq_reg)
		if (interrupt(ab->ac[1].irq_reg, ata_interrupt, (void *)&(ab->ac[1]), ab->ac[1].waitq, &ab->ac[1].inth) < 0)
			return -EINVAL;

	return 0;
}

int ata_init_one(pci_device_t *pdev, ata_opt_t *opt)
{
	/* we support now only 8 ata buses */
	if (buses_cnt > 7)
		return -1;

	buses[buses_cnt].dev = pdev;
	memcpy(&(buses[buses_cnt].config), opt, sizeof(ata_opt_t));

	ata_init_bus(&(buses[buses_cnt]));

	buses_cnt++;

	return 0;
}

/* Function searches ata devices */
int ata_generic_init(ata_opt_t *opt)
{
	ata_opt_t *aopt = (opt ? opt : &ata_defaults);
	unsigned int i = 0;
	int devs_found = 0;
	platformctl_t pctl;

	pctl.action = pctl_get;
	pctl.type = pctl_pci;

	buses_cnt = 0;
	/* iterate through pci to find ata-bus devices */
	for (i = 0; ata_pci_tbl[i].cl != 0; i++) {
		pctl.pci.id = ata_pci_tbl[i];
		if (platformctl(&pctl) != EOK)
			break;
		pci_dev[devs_found] = pctl.pci.dev;

		if (!ata_init_one(&pci_dev[devs_found], aopt)) {
			devs_found++;
		}
	}

	return devs_found;
}


int ata_init(void)
{
	return ata_generic_init(NULL) <= 0;
}