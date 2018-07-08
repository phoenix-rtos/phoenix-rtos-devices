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

//#include <hal/if.h>
//#include <fs/if.h>
//#include <vm/if.h>
//#include <proc/if.h>
//#include <main/if.h>

//#include <dev/if.h>
//#include <dev/pci/pci.h>
//#include <dev/storage/ata/ata.h>

static int ata_open(vnode_t *vnode, file_t* file);
static int ata_read(file_t* file, offs_t offs, char *buff, unsigned int len);
static int ata_write(file_t* file, offs_t offs, char *buff, unsigned int len);

static pci_id_t ata_pci_tbl[] = {
	{ PCI_ANY, PCI_ANY, PCI_ANY, PCI_ANY, 0x0101},
	{ 0x1106, 0x3249, PCI_ANY, PCI_ANY, 0x0104},
	{0,}
};


ata_opt_t ata_defaults = {
	.force = 0,
	.use_int = 1,
	.use_dma = 1,
	.use_multitransfer = 0
};


struct ata_bus buses[8] = {};
int buses_cnt = 0;

static int ata_interrupt(unsigned int irq, cpu_context_t *ctx, void *dev_instance);

// stupid sleep;
static void sleep(int ms) 
{
	ktime_t when_unlock = timesys_getTime() + (ms * 1000);
	while (timesys_getTime() < when_unlock) {
		__asm__ ("NOP;");
	}
}


/* hw access definitions */
#define ata_inb(xaddr) hal_inb((void*)0 + xaddr)
#define ata_outb(xaddr, data) hal_outb((void*)0 + xaddr, data)
#define ata_inw(xaddr) hal_inw((void*)0 + xaddr)
#define ata_outw(xaddr, data) hal_outw((void*)0 + xaddr, data)
#define ata_inl(xaddr) hal_inl((void*)0 + xaddr)
#define ata_outl(xaddr, data) hal_outl((void*)0 + xaddr, data)
#define ata_insl(base, buff, quads) insl((void*)0 + base, buff, quads)

#define ata_ch_read(ac, reg) ata_inb((ac)->reg_addr[(reg)])
#define ata_ch_write(ac, reg, data) ata_outb((ac)->reg_addr[(reg)], (data))
#define ata_ch_read_buffer(ac, reg, buff, quads) ata_insl((ac)->reg_addr[(reg)], (buff), (quads))
#define ata_ch_hob_enable(ac) ata_ch_write((ac), ATA_REG_CONTROL, ATA_CTRL_HOB | (ac)->no_int)
#define ata_ch_hob_disable(ac) ata_ch_write((ac), ATA_REG_CONTROL, (ac)->no_int)


static void insl(void *addr, void *buffer, u32 quads)\
{
#if 1
	int i;
	for (i = 0; i < quads; i += 1) {
		*(u32*)(buffer + (i * 4)) = hal_inl(addr);
	}
#else
	__asm__ volatile
	(" \
		 pushw %%es; \
		 pushw %%ax; \
		 movw %%ds, %%ax; \
		 movw %%ax, %%es; \
		 cld; \
		 rep; \
		 insl; \
		 popw %%ax; \
		 popw %%es;"
		 :
		 :"d"(addr), "D"(buffer), "c"(quads)
		 :"eax","cc");
#endif
}


void print_status(u8 st)
{
	if (st & ATA_SR_BSY)
		main_printf(ATTR_DEBUG, "BSY, ");
	if (st & ATA_SR_DRDY)
		main_printf(ATTR_DEBUG, "DRDY, ");
	if (st & ATA_SR_DF)
		main_printf(ATTR_DEBUG, "DF, ");
	if (st & ATA_SR_DSC)
		main_printf(ATTR_DEBUG, "DSC, ");
	if (st & ATA_SR_DRQ)
		main_printf(ATTR_DEBUG, "DRQ, ");
	if (st & ATA_SR_CORR)
		main_printf(ATTR_DEBUG, "CORR, ");
	if (st & ATA_SR_IDX)
		main_printf(ATTR_DEBUG, "IDX, ");
	if (st & ATA_SR_ERR)
		main_printf(ATTR_DEBUG, "ERR");
	main_printf(ATTR_DEBUG, "\n");
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


/*int ata_print_error(struct ata_dev *ad, u8 err) 
{
	if (err == 0)
		return err;

	main_printf(ATTR_ERROR, "dev/ata:");

	if (err == -2) {
		main_printf(ATTR_ERROR, "- Device Fault\n"); err = 19;
	}
	else if (err == -1) {
		unsigned char st = ata_ch_read(ad->ac, ATA_REG_ERROR);
		if (st & ATA_ER_AMNF)   { main_printf(ATTR_ERROR, "- No Address Mark Found\n");    err = 7;  }
		if (st & ATA_ER_TK0NF)  { main_printf(ATTR_ERROR, "- No Media or Media Error\n");  err = 3;  }
		if (st & ATA_ER_ABRT)   { main_printf(ATTR_ERROR, "- Command Aborted\n");          err = 20; }
		if (st & ATA_ER_MCR)    { main_printf(ATTR_ERROR, "- No Media or Media Error\n");  err = 3;  }
		if (st & ATA_ER_IDNF)   { main_printf(ATTR_ERROR, "- ID mark not Found\n");        err = 21; }
		if (st & ATA_ER_MC)     { main_printf(ATTR_ERROR, "- No Media or Media Error\n");  err = 3;  }
		if (st & ATA_ER_UNC)    { main_printf(ATTR_ERROR, "- Uncorrectable Data Error\n"); err = 22; }
		if (st & ATA_ER_BBK)    { main_printf(ATTR_ERROR, "- Bad Sectors\n");              err = 13; }
	} else  if (err == -3) {
		main_printf(ATTR_ERROR, "- Reads Nothing \n"); err = 23;
	} else  if (err == -4) {
		main_printf(ATTR_ERROR, "- Write Protected\n"); err = 8;
	}

	return err;
}*/


int ata_wait(struct ata_channel *ac, u8 irq) 
{
	int err = 0;

	if (irq) {
		proc_spinlockSet(&(ac->irq_spin));
		if (ac->irq_invoked == 0) {
			if ((err = proc_threadCondWait(&(ac->waitq), &(ac->irq_spin), /* 5sec */ 5000000)) < 0) {
				proc_spinlockClear(&(ac->irq_spin), sopGetCycles);
				return err;
			}
		}
		ac->irq_invoked = 0;
		proc_spinlockClear(&(ac->irq_spin), sopGetCycles);
	}
  return ata_polling(ac, 0);
}

#define PRD_ENT_LIMIT 65536

int ata_fill_prd(u8 *prd_virt, u32 prd_size, void *target_buff, u32 target_size)
{
	addr_t resaddr = 0;
	int ret = vm_kmapResolve(target_buff, &resaddr);
	(void)ret;

	if (!resaddr)
		return -1;
	
	u32 tsz = target_size;
	volatile u32 *prdptr = (void*)prd_virt;

	while (tsz) {
		u32 sz = tsz >= PRD_ENT_LIMIT ? PRD_ENT_LIMIT : tsz;
		tsz -= sz;

		(*prdptr) = resaddr;
		prdptr++;
		(*prdptr) = sz & 0x0000FFFF;

		resaddr += sz;
		
		if (tsz < 1) {
			(*prdptr) |= 0x80000000;
		}

		prdptr++;
	}

	TRACE("PRD \n %x %x %x %x\n %x %x %x %x ",
		prd_virt[3],
		prd_virt[2],
		prd_virt[1],
		prd_virt[0],
		prd_virt[7],
		prd_virt[6],
		prd_virt[5],
		prd_virt[4]);

	/* flush the buffer asms */
	{ int k; __asm__ ("xorl %%eax, %%eax; xchgl %%eax, %0"::"m" (k)); }
	asm volatile("lock; addl $0,0(%%esp)": : :"memory");

	return 0;
}


/* TODO: divide to dma_setup, command_setup, pio_access, dma_access */

int ata_access(u8 direction, struct ata_dev *ad, u32 lba, u8 numsects, void *buffer)
{
	struct ata_channel *ac = ad->ac;

	u8 lba_mode = 0; /* 0: CHS, 1:LBA28, 2: LBA48 */
	u8 ret = 0;
	u8 dma = ad->dma;
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

	if (dma && (ac->no_int == 0)) {
		/* disable/stop the dma channel, clear interrupt and error bits */
		ata_ch_write(ac, ATA_REG_BMCOMMAND, ATA_BMR_CMD_STOP);
		ata_ch_write(ac, ATA_REG_BMSTATUS, ac->bmstatus | ATA_BMR_STAT_INTR | ATA_BMR_STAT_ERR);

		/* set up prd */
		ata_fill_prd((u8*)ac->prd_virt, ac->prd_size, buffer, numsects * ad->sector_size);
		/* write prd to ata register */
		hal_outl((void*)(u32)ac->bmide + ATA_REG_BMPRD - 0x0E , ac->prd_phys & 0xFFFFFFFE);
		
		// set if dma read or write request will be used
		ata_ch_write(ac, ATA_REG_BMCOMMAND, (direction == 0 ? ATA_BMR_CMD_RDENABLE : ATA_BMR_CMD_WRENABLE));
	} else
		ata_ch_write(ac, ATA_REG_BMSTATUS, ATA_BMR_STAT_ERR);

	if (lba >= 0x10000000) {
		/* LBA48: */
		lba_mode  = 2;
		lba_io[0] = (lba & 0x000000FF) >> 0;
		lba_io[1] = (lba & 0x0000FF00) >> 8;
		lba_io[2] = (lba & 0x00FF0000) >> 16;
		lba_io[3] = (lba & 0xFF000000) >> 24;
		lba_io[4] = 0;
		lba_io[5] = 0;
		head      = 0; // Lower 4-bits of HDDEVSEL are not used here.
	}
	else if (ad->info.capabilities_1 & 0x200)  { // Drive supports LBA?
	
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

	// Write Parameters;
	if (lba_mode == 2) {
		ata_ch_hob_enable(ac);
		ata_ch_write(ac, ATA_REG_SECCOUNT1, 0);
		ata_ch_write(ac, ATA_REG_LBA3, lba_io[3]);
		ata_ch_write(ac, ATA_REG_LBA4, lba_io[4]);
		ata_ch_write(ac, ATA_REG_LBA5, lba_io[5]);
		ata_ch_hob_disable(ac);
	}

	ata_ch_write(ac, ATA_REG_SECCOUNT0, numsects);
	ata_ch_write(ac, ATA_REG_LBA0, lba_io[0]);
	ata_ch_write(ac, ATA_REG_LBA1, lba_io[1]);
	ata_ch_write(ac, ATA_REG_LBA2, lba_io[2]);

	if (lba_mode == 0 && dma == 0 && direction == 0) cmd = ATA_CMD_READ_PIO;
	if (lba_mode == 1 && dma == 0 && direction == 0) cmd = ATA_CMD_READ_PIO;
	if (lba_mode == 2 && dma == 0 && direction == 0) cmd = ATA_CMD_READ_PIO_EXT;
	if (lba_mode == 0 && dma == 1 && direction == 0) cmd = ATA_CMD_READ_DMA;
	if (lba_mode == 1 && dma == 1 && direction == 0) cmd = ATA_CMD_READ_DMA;
	if (lba_mode == 2 && dma == 1 && direction == 0) cmd = ATA_CMD_READ_DMA_EXT;
	if (lba_mode == 0 && dma == 0 && direction == 1) cmd = ATA_CMD_WRITE_PIO;
	if (lba_mode == 1 && dma == 0 && direction == 1) cmd = ATA_CMD_WRITE_PIO;
	if (lba_mode == 2 && dma == 0 && direction == 1) cmd = ATA_CMD_WRITE_PIO_EXT;
	if (lba_mode == 0 && dma == 1 && direction == 1) cmd = ATA_CMD_WRITE_DMA;
	if (lba_mode == 1 && dma == 1 && direction == 1) cmd = ATA_CMD_WRITE_DMA;
	if (lba_mode == 2 && dma == 1 && direction == 1) cmd = ATA_CMD_WRITE_DMA_EXT;

	ata_ch_write(ac, ATA_REG_COMMAND, cmd);
	TRACE("Command sent %x", cmd);

	if (dma) {
		u32 bmcmd = ata_ch_read(ac, ATA_REG_BMCOMMAND);
		u32 bmsta = ata_ch_read(ac, ATA_REG_BMSTATUS);

		ata_ch_write(ac, ATA_REG_BMCOMMAND, (direction == 0 ? ATA_BMR_CMD_RDENABLE : ATA_BMR_CMD_WRENABLE) | ATA_BMR_CMD_START);

		bmcmd = ata_ch_read(ac, ATA_REG_BMCOMMAND);
		bmsta = ata_ch_read(ac, ATA_REG_BMSTATUS);

		err = ata_wait(ac, 1);

		if (err) {
			TRACE("DMA timeout");
			return -1;
		}

		/* disable/stop the dma channel */

		TRACE("c irq BMSTA %x", ac->bmstatus_irq);
		bmsta = ac->bmstatus_irq;           // read BM status from interrupt
		bmsta &= ~ATA_BMR_STAT_ACT;         // ignore Active bit

		ata_ch_write(ac, ATA_REG_BMCOMMAND, ATA_BMR_CMD_STOP);

		bmcmd = ata_ch_read(ac, ATA_REG_BMCOMMAND);
		bmsta |= ata_ch_read(ac, ATA_REG_BMSTATUS);
		TRACE("d BMCMD %x BMSTA %x", bmcmd , bmsta);

		if (bmsta & ATA_BMR_STAT_ERR) {
			TRACE("Error");
			return -1;
		} else if (bmsta & ATA_BMR_STAT_ACT) {
			TRACE("Error: command should complete by now");
			return -1;
		}

		if (ac->status & (ATA_SR_BSY | ATA_SR_DF | ATA_SR_DRQ | ATA_SR_ERR)) {
			print_status(ac->status);
			return -1;
		}

		ret = numsects; 
	} else {
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
					TRACE("Error");
					break;
				}

				if (!(ac->status & ATA_SR_DRQ)) {
					TRACE("Error");
					break;
				}

				if (numsects - i < 1) {
					ATA_WASTE100MS(ac);
					ac->status = ata_ch_read(ac, ATA_REG_STATUS);
					if (ac->status & ( ATA_SR_BSY | ATA_SR_DF | ATA_SR_DRQ | ATA_SR_ERR )) {
						TRACE("Error");
						break;
					}

					break;
				}

				TRACE("ro %d %d %d",i, numsects, b);
			}

			ret = i;
		} else {
			while ((astatus = ac->status = ata_ch_read(ac, ATA_REG_ALTSTATUS)) & ATA_SR_BSY); // Wait for BSY to be zero.

			// PIO Write.
			i = 0;

			while (i < numsects) {

				if ((astatus & (ATA_SR_BSY | ATA_SR_DRQ)) == ATA_SR_DRQ ) {
					TRACE("setting");
					for (b = 0; b < words; b++) {
						ata_outw(bus, *((u16*)(buffer + (b*2))));
					}

					buffer += (words * 2);
					i++;
					ATA_WASTE400MS(ac);
				}

				TRACE("io %d %d %d",i, numsects, b);

				if (astatus & (ATA_SR_BSY | ATA_SR_DF | ATA_SR_ERR)) {
					TRACE("Error");
					break;
				}

				if (!(astatus & ATA_SR_DRQ)) {
					TRACE("DRQ Error");
					break;
				}

				if ((err = ata_wait(ac, !ac->no_int)) != 0) {
					ata_polling(ac, 1); // Polling.
				}

				if (ac->no_int || err)
					astatus = ac->status = ata_ch_read(ac, ATA_REG_STATUS);
				else
					astatus = ac->status;

				if (numsects - i < 1) {
					if (astatus & ( ATA_SR_BSY | ATA_SR_DF | ATA_SR_DRQ | ATA_SR_ERR )) {
						TRACE("Error");

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
	}

	if (ata_ch_read(ac, ATA_REG_BMSTATUS) & ATA_BMR_STAT_ERR) {
		TRACE("Error");
		//error
	}
	return ret;
}


// TODO: fixme 64 bits
static int ata_read(file_t* file, offs_t offs, char *buff, unsigned int len)
{
	vnode_t *vnode = file->vnode;
	u32 begin_lba = 0;
	u32 sectors = 0;
	u32 ret = 0;
	struct ata_dev *ad = vnode->file_priv;

	if (ad == NULL) {
		return -EINVAL;
	}

	if (!ad->reserved)
		return -ENOENT;

	if (((u32)offs % ad->sector_size) || (len % ad->sector_size)) {
		TRACE("%d%d  %d %d", offs, ad->sector_size, len);
		panic("panic on the disco sector ");
	}


	//if (((lba + numsects) > ad->size) && (ad->type == IDE_ATA)) {
	//	ret = -EINVAL;  /* invalid seek */

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

static int ata_write(file_t* file, offs_t offs, char *buff, unsigned int len)
{
	vnode_t *vnode = file->vnode;
	u32 begin_lba = 0;
	u32 sectors = 0;
	u32 ret = 0;
	struct ata_dev *ad = vnode->file_priv;

	if (ad == NULL)
		return -EINVAL;

	if (!ad->reserved)
		return -ENOENT;

	if (((u32)offs % ad->sector_size) || (len % ad->sector_size)) {
		panic("panic on the disco sector ");
	}

	//if (((lba + numsects) > ad->size) && (ad->type == IDE_ATA)) {
	//	ret = -EINVAL;  /* invalid seek */

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


static int ata_open(vnode_t *vnode, file_t* file)
{
	struct ata_dev *ad;

	if (MINOR(vnode->dev) >= 2 * 2 * buses_cnt)
		return -EINVAL;

	// select device
	int bus  =  MINOR(vnode->dev) / (2 * 2);
	int chan = (MINOR(vnode->dev) % (2 * 2)) / 2;
	int dev  = (MINOR(vnode->dev) % (2 * 2)) % 2;

	if (!((ad = &buses[bus].ac[chan].devices[dev])->reserved))
		return -ENOENT;

	vnode->size = ad->size * ad->sector_size;
	vnode->file_priv = ad;
	return 0;
}


static int ata_interrupt(unsigned int irq, cpu_context_t *ctx, void *dev_instance)
{
	struct ata_channel *ac = (struct ata_channel*)dev_instance;
	int res = IHRES_IGNORE;
	TRACE("ATA irq %d %p", irq, ac);

	proc_spinlockSet(&ac->irq_spin);
	ac->bmstatus_irq = ata_ch_read(ac, ATA_REG_BMSTATUS); // Read Status Register.
	ac->status = ata_ch_read(ac, ATA_REG_STATUS); // Read Status Register.
	if (ac->bmstatus_irq & ATA_BMR_STAT_INTR) {
		TRACE("Intr raised");
		ata_ch_write(ac,ATA_REG_BMSTATUS,ATA_BMR_STAT_INTR);
	}

	ac->irq_invoked = 1;
	proc_threadCondSignal(&(ac->waitq));
	proc_spinlockClear(&ac->irq_spin, sopGetCycles);

	TRACE("ATA leave irq %d", irq);
	return res;
}


void ata_print(u8 attr, u8 *s, unsigned int len)
{
  unsigned int l;

	for (l = 0; l < len; l += 2) {	  
	  if ((s[l] == 32) && (s[l + 1] == 32))
	    break;
    main_printf(attr, "%c%c", s[l + 1], s[l]);
  }
  return;
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

	proc_spinlockCreate(&(ab->ac[ATA_PRIMARY].irq_spin), "ata.primary");
	proc_spinlockCreate(&(ab->ac[ATA_SECONDARY].irq_spin), "ata.secondary");

	/* Detect ATA-ATAPI Devices: */
	for (i = 0; i < 2; i++) {

	  proc_thqCreate(&(ab->ac[i].waitq));

		/* init prd regions - one per channel */
		
		page_t *page = 0;
		ab->ac[i].prd_size = 128 * 1024;

		page = vm_pageAlloc(ab->ac[i].prd_size / SIZE_PAGE, vm_pageAlloc);

		if (page == NULL)
			return -EINVAL;

		if (vm_kmap(page, PGHD_WRITE | PGHD_PRESENT, (void**) &(ab->ac[i].prd_virt) ) == EOK)
			vm_kmapResolve((void*)ab->ac[i].prd_virt, &(ab->ac[i].prd_phys));

		hal_memset((void *)ab->ac[i].prd_virt, 0, ab->ac[i].prd_size);

		if (ab->ac[i].prd_phys & 0x0000FFFF) {
			addr_t old = ab->ac[i].prd_phys;
			ab->ac[i].prd_phys = ((ab->ac[i].prd_phys & 0xFFFF0000) + 0x10000);
			ab->ac[i].prd_virt = ab->ac[i].prd_virt + (ab->ac[i].prd_phys - old);
		}

		ab->ac[i].bmstatus = ata_ch_read(&(ab->ac[i]), ATA_REG_BMSTATUS) & 0x60;

		for (j = 0; j < 2; j++) {

			u8 err = 0, type = IDE_ATA, status = 0;

			ab->ac[i].devices[j].ac = &ab->ac[i];
			ab->ac[i].devices[j].reserved = 0;

			ata_ch_write(&(ab->ac[i]), ATA_REG_HDDEVSEL, 0xA0 | (j << 4));
			sleep(1);

			ata_ch_write(&(ab->ac[i]), ATA_REG_COMMAND, ATA_CMD_IDENTIFY);
			sleep(1);

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

      if (ab->ac[i].bmstatus & (ATA_BMR_STAT_DEV0_DMA << j) && ab->config.use_dma) {
				ab->ac[i].devices[j].dma = 1;
				ab->ac[i].no_int = 0;
      }
	
			main_printf(ATTR_DEV, "dev: [ata  ] Found ");
			ata_print(ATTR_DEV, ab->ac[i].devices[j].info.model, sizeof(ab->ac[i].devices[j].info.model));
			main_printf(ATTR_DEV, "[%d:%d] %.5f GiB\n", i, j, (double)ab->ac[i].devices[j].size * ab->ac[i].devices[j].sector_size / 1000 / 1000 / 1000);
		}
	}
	
	/* TODO: rework interrupt handling (pass only bus not channel etc) */
	if (hal_interruptsSetHandler(ab->ac[0].irq_reg, ata_interrupt, (void *)&(ab->ac[0])) < 0)
		return -EINVAL;

	if (ab->ac[0].irq_reg != ab->ac[1].irq_reg)
		if (hal_interruptsSetHandler(ab->ac[1].irq_reg, ata_interrupt, (void *)&(ab->ac[1])) < 0)
			return -EINVAL;

	return 0;
}


int ata_init_one(pci_device_t *pdev, ata_opt_t *opt)
{
	/* we support now only 8 ata buses */
	if (buses_cnt > 7)
		return -1;

	if (pdev == NULL && opt->force) {
		pdev = vm_kmalloc(sizeof(pci_device_t));
		main_memset(pdev, 0, sizeof(pci_device_t));
	}

	buses[buses_cnt].dev = pdev;
	hal_memcpy(&(buses[buses_cnt].config), opt, sizeof(ata_opt_t));

	ata_init_bus(&(buses[buses_cnt]));

	buses_cnt++;

	return 0;
}


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
