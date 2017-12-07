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

 #include <stdlib.h>
 #include <stdio.h>
 #include <unistd.h>

 #include <string.h>
 #include <arch/ia32/io.h>
 #include <sys/threads.h>
 #include <sys/msg.h>
 #include <sys/interrupt.h>

 #include <pc-pci.h>
 #include "pc-ata.h"

 //static int ata_open(vnode_t *vnode, file_t* file);
 static int ata_read(struct ata_dev *ad, offs_t offs, char *buff, unsigned int len);
 static int ata_write(struct ata_dev *ad, offs_t offs, char *buff, unsigned int len);

 static pci_id_t ata_pci_tbl[] = {
 	{ PCI_ANY, PCI_ANY, PCI_ANY, PCI_ANY, 0x0101},
 	{ 0x1106, 0x3249, PCI_ANY, PCI_ANY, 0x0104},
 	{0,}
 };

 ata_opt_t ata_defaults = {
 	.force = 0,
 	.use_int = 1,
 	.use_dma = 0,
 	.use_multitransfer = 0
 };

 struct ata_bus buses[8] = {};
 int buses_cnt = 0;
 pci_device_t pci_dev[8] = {};
 u32 port;

static int ata_interrupt(unsigned int irq, void *dev_instance);

 /* hw access definitions */
 #define ata_inb(xaddr) inb((void*)0 + xaddr)
 #define ata_outb(xaddr, data) outb((void*)0 + xaddr, data)
 #define ata_inw(xaddr) inw((void*)0 + xaddr)
 #define ata_outw(xaddr, data) outw((void*)0 + xaddr, data)
 #define ata_inl(xaddr) inl((void*)0 + xaddr)
 #define ata_outl(xaddr, data) outl((void*)0 + xaddr, data)
 #define ata_insl(base, buff, quads) insl((void*)0 + base, buff, quads)

 #define ata_ch_read(ac, reg) ata_inb((ac)->reg_addr[(reg)])
 #define ata_ch_write(ac, reg, data) ata_outb((ac)->reg_addr[(reg)], (data))
 #define ata_ch_read_buffer(ac, reg, buff, quads) ata_insl((ac)->reg_addr[(reg)], (buff), (quads))
 #define ata_ch_hob_enable(ac) ata_ch_write((ac), ATA_REG_CONTROL, ATA_CTRL_HOB | (ac)->no_int)
 #define ata_ch_hob_disable(ac) ata_ch_write((ac), ATA_REG_CONTROL, (ac)->no_int)

 static void insl(void *addr, void *buffer, u32 quads)\
 {
 	int i;
 	for (i = 0; i < quads; i += 1) {
 		*(u32*)(buffer + (i * 4)) = inl(addr);
 	}
 }


 void print_status(u8 st)
 {
 	if (st & ATA_SR_BSY)
 		printf("BSY, ");
 	if (st & ATA_SR_DRDY)
 		printf("DRDY, ");
 	if (st & ATA_SR_DF)
 		printf("DF, ");
 	if (st & ATA_SR_DSC)
 		printf("DSC, ");
 	if (st & ATA_SR_DRQ)
 		printf("DRQ, ");
 	if (st & ATA_SR_CORR)
 		printf("CORR, ");
 	if (st & ATA_SR_IDX)
 		printf("IDX, ");
 	if (st & ATA_SR_ERR)
 		printf("ERR");
 	printf("\n---- %s\n","");
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
 int ata_polling(struct ata_channel *ac, u8 advanced_check)
 {
 	ATA_WASTE400MS(ac);

 	while (ata_ch_read(ac, ATA_REG_STATUS) & ATA_SR_BSY);

 	if (advanced_check) {
 		u8 state = ata_ch_read(ac, (advanced_check == 2 ? ATA_REG_ALTSTATUS : ATA_REG_STATUS));

 		if (state & ATA_SR_ERR)
 			return -1; // Error.

 		if (state & ATA_SR_DF)
 			return -2; // Device Fault.

 		if ((state & ATA_SR_DRQ) == 0)
 			return -3; // DRQ should be set
 	}

 	return 0;
 }

 int ata_wait(struct ata_channel *ac, u8 irq)
 {
 	int err = 0;

 	if (irq) {
        mutexLock(ac->irq_spin);
        if (ac->irq_invoked == 0) {
            if ((err = condWait(ac->waitq, ac->irq_spin, /* 5sec */ 5000)) < 0) {
                printf("cond timeout %s\n", "");
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

 int ata_access(u8 direction, struct ata_dev *ad, u32 lba, u8 numsects, void *buffer)
 {
 	struct ata_channel *ac = ad->ac;

 	u8 lba_mode = 0; /* 0: CHS, 1:LBA28, 2: LBA48 */
 	u8 ret = 0;
 	u8 cmd;
 	u8 astatus = 0;
 	u8 slavebit = ad->drive;
 	u8 lba_io[6];
 	u8 head, sect;
 	u16 bus = ac->base;
 	u16 words = 256;
 	u16 cyl, i, b;

 	int err = 0;

 	ata_ch_write(ac, ATA_REG_CONTROL, ac->no_int << 1);

    ata_ch_write(ac, ATA_REG_BMSTATUS, ATA_BMR_STAT_ERR);

    if (ad->info.capabilities_1 & 0x200)  { // Drive supports LBA?
        /* LBA28 */
        lba_mode  = 1;
        lba_io[0] = (lba & 0x000000FF) >> 0;
        lba_io[1] = (lba & 0x0000FF00) >> 8;
        lba_io[2] = (lba & 0x00FF0000) >> 16;
        lba_io[3] = 0;
        lba_io[4] = 0;
        lba_io[5] = 0;
        head      = (lba & 0x0F000000) >> 24;
    } else {
 		/* CHS: */
 		lba_mode  = 0;
 		sect      = (lba % 63) + 1;
 		cyl       = (lba + 1  - sect) / (16 * 63);
 		lba_io[0] = sect;
 		lba_io[1] = (cyl >> 0) & 0xFF;
 		lba_io[2] = (cyl >> 8) & 0xFF;
 		lba_io[3] = 0;
 		lba_io[4] = 0;
 		lba_io[5] = 0;
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

 	if (lba_mode == 0 && direction == 0) cmd = ATA_CMD_READ_PIO;
 	if (lba_mode == 1 && direction == 0) cmd = ATA_CMD_READ_PIO;
 	if (lba_mode == 0 && direction == 1) cmd = ATA_CMD_WRITE_PIO;
 	if (lba_mode == 1 && direction == 1) cmd = ATA_CMD_WRITE_PIO;

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
 				for (b = 0; b < words; b++) {
 					*(u16*)(buffer + (b * 2)) = ata_inw(bus);
 				}
                buffer += (words * 2);
                i++;
                ATA_WASTE400MS(ac);
 		     }

 			if (ac->status & (ATA_SR_BSY | ATA_SR_DF | ATA_SR_ERR)) {
 				//TRACE("Error");
 				break;
 			}

 			if (!(ac->status & ATA_SR_DRQ)) {
 				//TRACE("Error");
 				break;
 			}

 			if (numsects - i < 1) {
 				ATA_WASTE100MS(ac);
 				ac->status = ata_ch_read(ac, ATA_REG_STATUS);
 				if (ac->status & ( ATA_SR_BSY | ATA_SR_DF | ATA_SR_DRQ | ATA_SR_ERR )) {
 					printf("Error %s\n", "");
                    print_status(ac->status);
 					break;
 				}

 				break;
 			}
 		}

 		ret = i;
 	} else {
 		while ((astatus = ac->status = ata_ch_read(ac, ATA_REG_ALTSTATUS)) & ATA_SR_BSY); // Wait for BSY to be zero.
 		// PIO Write.
 		i = 0;

 		while (i < numsects) {

 			if ((ac->status & (ATA_SR_BSY | ATA_SR_DRQ)) == ATA_SR_DRQ ) {
 				for (b = 0; b < words; b++) {
 					ata_outw(bus, *((u16*)(buffer + (b*2))));
 				}
 				buffer += (words * 2);
 				i++;
 				ATA_WASTE400MS(ac);
 			}

 			if (ac->status & (ATA_SR_BSY | ATA_SR_DF | ATA_SR_ERR)) {
                print_status(ac->status);
 				printf("Error %s\n", "");
 				break;
 			}

 			if (!(ac->status & ATA_SR_DRQ)) {
 				printf("DRQ Error %s\n", "");
 				break;
 			}

 			if ((err = ata_wait(ac, !ac->no_int)) != 0)
 				ata_polling(ac, 1); // Polling.


 			if (ac->no_int || err)
 				astatus = ac->status = ata_ch_read(ac, ATA_REG_STATUS);
 			else
                astatus = ac->status;

 			if (numsects - i < 1) {
 				if (astatus & ( ATA_SR_BSY | ATA_SR_DF | ATA_SR_DRQ | ATA_SR_ERR )) {
 					printf("Error %s\n", "");
 					print_status(ac->status);
 					break;
 				}
 				break;
 			}
 		}

 		ret = i;

 		ata_ch_write(ac, ATA_REG_COMMAND, ((char []) {   ATA_CMD_CACHE_FLUSH,
 				ATA_CMD_CACHE_FLUSH,
 				ATA_CMD_CACHE_FLUSH_EXT}[lba_mode]));

 		if (!ac->no_int) {
 			if ((err = ata_wait(ac, 1)) != 0) {
 				ata_polling(ac, 1); // Polling.
 			}
 		}
 	}

 	return ret;
 }

 // TODO: fixme 64 bits
 static int ata_read(struct ata_dev *ad, offs_t offs, char *buff, unsigned int len)
 {
 	u32 begin_lba = 0;
 	u32 sectors = 0;
 	u32 ret = 0;

 	if (ad == NULL) {
 		return -EINVAL;
 	}

 	if (!ad->reserved)
 		return -ENOENT;

 	if (((u32)offs % ad->sector_size) || (len % ad->sector_size)) {
 		printf("panic on the disco sector ");
        return -EINVAL;
 	}

 	begin_lba = (u32)offs / ad->sector_size; // starting sector
 	sectors = len / ad->sector_size;

 	for (; sectors >= ATA_MAX_PIO_DRQ; sectors -= ATA_MAX_PIO_DRQ) {
 		// sectors actually read
 		ata_access(ATA_READ, ad, begin_lba, (u8)ATA_MAX_PIO_DRQ, buff); // FIXME: BUG: TODO: ATA_MAX_PIO_DRQ is getting squashed to 8bits here!!!

 		begin_lba += ATA_MAX_PIO_DRQ;
 		buff += ATA_MAX_PIO_DRQ * ad->sector_size;
 		ret += ATA_MAX_PIO_DRQ * ad->sector_size;
 	}

 	if (sectors) {
 		ata_access(ATA_READ, ad, begin_lba, (u8)sectors, buff);
 		ret += sectors * ad->sector_size;
 	}

 	return ret;
 }

 static int ata_write(struct ata_dev *ad, offs_t offs, char *buff, unsigned int len)
 {
 	u32 begin_lba = 0;
 	u32 sectors = 0;
 	u32 ret = 0;

 	if (ad == NULL)
 		return -EINVAL;

 	if (!ad->reserved)
 		return -ENOENT;

 	if (((u32)offs % ad->sector_size) || (len % ad->sector_size)) {
 		printf("panic on the disco sector ");
        return -EINVAL;
 	}

 	begin_lba = (u32)offs / ad->sector_size; // starting sector
 	sectors = len / ad->sector_size;

 	for (; sectors >= ATA_MAX_PIO_DRQ; sectors -= ATA_MAX_PIO_DRQ) {
 		ata_access(ATA_WRITE, ad, begin_lba, (u8)ATA_MAX_PIO_DRQ, buff); // FIXME: BUG: TODO: ATA_MAX_PIO_DRQ is getting squashed to 8bits here!!!
 		begin_lba += ATA_MAX_PIO_DRQ;
 		buff += ATA_MAX_PIO_DRQ * ad->sector_size;
 		ret += ATA_MAX_PIO_DRQ * ad->sector_size;
 	}

 	if (sectors) {
 		ata_access(ATA_WRITE, ad, begin_lba, (u8)sectors, buff);
 		ret += sectors * ad->sector_size;
 	}

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

 	return res;
}


 int ata_init_bus(struct ata_bus *ab)
 {
 	u8 i, j;
 	u32 B0 = ab->dev->resources[0].base;
 	u32 B1 = ab->dev->resources[1].base;
 	u32 B2 = ab->dev->resources[2].base;
 	u32 B3 = ab->dev->resources[3].base;
 	u32 B4 = ab->dev->resources[4].base;

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
 	ab->ac[ATA_SECONDARY].ctrl  = (B3 & 0xFFFFFFFC) + 0x374 * (!B3);

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

 	/* Detect ATA-ATAPI Devices: */
 	for (i = 0; i < 2; i++) {

        condCreate(&(ab->ac[i].waitq));

 		ab->ac[i].bmstatus = ata_ch_read(&(ab->ac[i]), ATA_REG_BMSTATUS) & 0x60;

 		for (j = 0; j < 2; j++) {
 			u8 err = 0, type = IDE_ATA, status = 0;

 			ab->ac[i].devices[j].ac = &ab->ac[i];
 			ab->ac[i].devices[j].reserved = 0;

 			ata_ch_write(&(ab->ac[i]), ATA_REG_HDDEVSEL, 0xA0 | (j << 4));
 			usleep(1000000);

 			ata_ch_write(&(ab->ac[i]), ATA_REG_COMMAND, ATA_CMD_IDENTIFY);
 			usleep(1000000);

 			if (ata_ch_read(&(ab->ac[i]), ATA_REG_STATUS) == 0)
                continue;

 			while (1) {
 				status = ata_ch_read(&(ab->ac[i]), ATA_REG_STATUS);

 				if ((status & ATA_SR_ERR) || (status & ATA_SR_DF)) {
 					err = 1;
 					break;
 				}

 				if (!(status & ATA_SR_BSY) && (status & ATA_SR_DRQ))
 					break;
 			}

 			/* it's probably atapi device - we do not support it */
 			if (err != 0)
                continue;

 			ata_ch_read_buffer(&(ab->ac[i]), ATA_REG_DATA, &(ab->ac[i].devices[j].info), 128);

 			ab->ac[i].devices[j].reserved = 1;
 			ab->ac[i].devices[j].type = type;
 			ab->ac[i].devices[j].channel = i;
 			ab->ac[i].devices[j].drive = j;
 			ab->ac[i].devices[j].sector_size  = ab->ac[i].devices[j].info.log_sector_size;

       if (!ab->ac[i].devices[j].sector_size)
 				ab->ac[i].devices[j].sector_size = ATA_DEF_SECTOR_SIZE;

       if (ab->ac[i].devices[j].info.commands2_sup & ATA_INFO_COMMANDS_2_LBA48)
 				ab->ac[i].devices[j].size = ab->ac[i].devices[j].info.lba48_totalsectors;
 			else
 				ab->ac[i].devices[j].size = ab->ac[i].devices[j].info.lba28_totalsectors;

 			printf("dev: [ata  ] Found ");
 			printf("[%d:%d] %.5f GiB\n", i, j, (double)ab->ac[i].devices[j].size * ab->ac[i].devices[j].sector_size / 1000 / 1000 / 1000);
        }
 	}

 	/* TODO: rework interrupt handling (pass only bus not channel etc) */
	if (interrupt(ab->ac[0].irq_reg, ata_interrupt, (void *)&(ab->ac[0]), ab->ac[0].waitq) < 0)
 		return -EINVAL;

 	if (ab->ac[0].irq_reg != ab->ac[1].irq_reg)
 		if (interrupt(ab->ac[1].irq_reg, ata_interrupt, (void *)&(ab->ac[1]), ab->ac[1].waitq) < 0)
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
    pci_id_t pci_id;
	unsigned int i = 0;
	int devs_found = 0;
    oid_t pci;
    int bytes = 0;
    int b = 0;


    while (lookup("/dev/pci", &pci) < 0) {
        printf("ata: no pci port found %s\n", "");
        b++;
        usleep(1000000);
        if (b == 10) {
            printf("ata: fail to find pci %s\n", "");
            return 0;
        }
    }

    buses_cnt = 0;
	/* iterate through pci to find ata-bus devices */
	for (i = 0; ata_pci_tbl[i].cl != 0; i++) {
		do {
            pci_id = ata_pci_tbl[i];
            bytes = send(pci.port, READ, &pci_id, sizeof(pci_id_t), NORMAL, &pci_dev[devs_found], sizeof(pci_device_t));

            if(!bytes)
                break;

            printf("ata :%2u:%2u:%2u-->%6u,%6u-->%3u,%3u \n",
            pci_dev[devs_found].b, pci_dev[devs_found].d,
            pci_dev[devs_found].f, pci_dev[devs_found].device & 0xFFFF,
            pci_dev[devs_found].vendor & 0xFFFF,
            (pci_dev[devs_found].cl >> 8) & 0xFF,pci_dev[devs_found].cl & 0xFF);

            if (!ata_init_one(&pci_dev[devs_found], aopt)) {
                devs_found++;
            }
        } while (1);
    }
	if (!devs_found) {
        printf("ata: no devices found %s\n", "");
        return -ENOENT;
  }

	return devs_found;
}

static void ata_thread(void *arg)
{

  unsigned int tmp;
  void *buff = malloc(2048);
  msghdr_t hdr;
  ata_msg_t *msg;
  struct ata_dev ad;



  msg = buff;
  for (;;) {
		tmp = recv(port, buff, 2048, &hdr);
        ad = buses[msg->bus].ac[msg->channel].devices[msg->device];

		switch (hdr.op) {
		case READ:
				tmp = ata_read(&ad, msg->offset, msg->data, msg->len);
				if (hdr.type == NORMAL)
					respond(port, EOK, msg->data, tmp);
			break;
		case WRITE:
				tmp = ata_write(&ad, msg->offset, msg->data, msg->len);
				if (hdr.type == NORMAL)
					respond(port, EOK, &tmp, sizeof(tmp));
			break;
    case DEVCTL:
    case OPEN:
    case CLOSE:
    default:
      break;
    }
  }
  free(buff);
}

int main(void)
{

    int stacksz = 2048;
    char *stack;
    ata_msg_t *msg;

    printf("\nata: Initializing %s\n","");
    ata_generic_init(NULL);
    printf("\nata: Initialized %s\n","");

    portCreate(&port);
    if (portRegister(port, "/dev/ata") < 0) {
        printf("ata: Can't register port %d\n", port);
        return -1;
    } else {
      printf("ata: Registered port %d\n", port);
    }

    if ((stack = malloc(stacksz)) == NULL || beginthread(ata_thread, 4, stack, stacksz, NULL) != EOK) {
      printf("ata: not enough memory to start ata thread! %s\n", "");
        free(stack);
    }

    msg = malloc(sizeof(ata_msg_t) + 1024);
    for(;;) {
    usleep(3000000);

    msg->channel = 0;
    msg->bus = 0;
    msg->device = 0;
    msg->offset += 1024;
    msg->len = 1024;
    memset(msg->data, 116, 1024);
    //printf("bytes %d %d\n", bytes, msg->data[0]);
    send(port, WRITE, msg, sizeof(ata_msg_t) + 1024, NORMAL, NULL, 0);
    //printf("bytes %d %d\n", bytes, msg->data[0]);

    //memset(msg->data, 33, 512);
    //printf("msg->data[0] = %d\n", msg->data[0]);
    send(port, READ, msg, sizeof(ata_msg_t) + 1024, NORMAL, &msg->data, 1024);
    //printf("recv msg->data[0] = %d\n", msg->data[0]);

  }
  free(msg);
  free(stack);
  return 0;
}
