/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND flash driver.
 *
 * Copyright 2018 Phoenix Systems
 * Author: Jan Sikorski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>

#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>
#include <phoenix/arch/imx6ull.h>

#include "imx6ull-flashdrv.h"


enum {
	apbh_ctrl0 = 0, apbh_ctrl0_set, apbh_ctrl0_clr, apbh_ctrl0_tog,
	apbh_ctrl1, apbh_ctrl1_set, apbh_ctrl1_clr, apbh_ctrl1_tog,
	apbh_ctrl2, apbh_ctrl2_set, apbh_ctrl2_clr, apbh_ctrl2_tog,
	apbh_channel_ctrl, apbh_channel_ctrl_set, apbh_channel_ctrl_clr, apbh_channel_ctrl_tog, apbh_devsel,

	apbh_ch0_curcmdar = 64, apbh_ch0_nxtcmdar = 68, apbh_ch0_cmd = 72, apbh_ch0_bar = 76,
	apbh_ch0_sema = 80, apbh_ch0_debug1 = 84, apbh_ch0_debug2 = 88,

	apbh_version = 512,
	apbh_next_channel = 92,
};


enum {
	dma_noxfer = 0,	dma_write = 1, dma_read = 2, dma_sense = 3,

	dma_chain = 1 << 2,
	dma_irqcomp = 1 << 3,
	dma_nandlock = 1 << 4,
	dma_w4ready = 1 << 5,
	dma_decrsema = 1 << 6,
	dma_w4endcmd = 1 << 7,
	dma_hot = 1 << 8,
};


typedef struct {
	uint32_t next;
	uint16_t flags;
	uint16_t bufsz;
	uint32_t buffer;
	uint32_t pio[];
} dma_t;


enum { bch_ctrl = 0, bch_ctrl_set, bch_ctrl_clr, bch_ctrl_tog, bch_status0,
	bch_status0_set, bch_status0_clr, bch_status0_tog, bch_mode, bch_mode_set,
	bch_mode_clr, bch_mode_tog, bch_encodeptr, bch_encodeptr_set,
	bch_encodeptr_clr, bch_encodeptr_tog, bch_dataptr, bch_dataptr_set,
	bch_dataptr_clr, bch_dataptr_tog, bch_metaptr, bch_metaptr_set,
	bch_metaptr_clr, bch_metaptr_tog, bch_layoutselect = 28,
	bch_layoutselect_set, bch_layoutselect_clr, bch_layoutselect_tog,
	bch_flash0layout0, bch_flash0layout0_set, bch_flash0layout0_clr,
	bch_flash0layout0_tog, bch_flash0layout1, bch_flash0layout1_set,
	bch_flash0layout1_clr, bch_flash0layout1_tog, bch_flash1layout0,
	bch_flash1layout0_set, bch_flash1layout0_clr, bch_flash1layout0_tog,
	bch_flash1layout1, bch_flash1layout1_set, bch_flash1layout1_clr,
	bch_flash1layout1_tog, bch_flash2layout0, bch_flash2layout0_set,
	bch_flash2layout0_clr, bch_flash2layout0_tog, bch_flash2layout1,
	bch_flash2layout1_set, bch_flash2layout1_clr, bch_flash2layout1_tog,
	bch_flash3layout0, bch_flash3layout0_set, bch_flash3layout0_clr,
	bch_flash3layout0_tog, bch_flash3layout1, bch_flash3layout1_set,
	bch_flash3layout1_clr, bch_flash3layout1_tog, bch_debug0, bch_debug0_set,
	bch_debug0_clr, bch_debug0_tog, bch_dbgkesread, bch_dbgkesread_set,
	bch_dbgkesread_clr, bch_dbgkesread_tog, bch_dbgcsferead, bch_dbgcsferead_set,
	bch_dbgcsferead_clr, bch_dbgcsferead_tog, bch_dbgsyndgenread,
	bch_dbgsyndgenread_set, bch_dbgsyndgenread_clr, bch_dbgsyndgenread_tog,
	bch_dbgahbmread, bch_dbgahbmread_set, bch_dbgahbmread_clr,
	bch_dbgahbmread_tog, bch_blockname, bch_blockname_set, bch_blockname_clr,
	bch_blockname_tog, bch_version, bch_version_set, bch_version_clr,
	bch_version_tog, bch_debug1, bch_debug1_set, bch_debug1_clr, bch_debug1_tog };


enum {
	gpmi_ctrl0 = 0, gpmi_ctrl0_set, gpmi_ctrl0_clr, gpmi_ctrl0_tog, gpmi_compare,
	gpmi_eccctrl = gpmi_compare + 4, gpmi_eccctrl_set, gpmi_eccctrl_clr, gpmi_eccctrl_tog,
	gpmi_ecccount, gpmi_payload = gpmi_ecccount + 4,

	gpmi_auxiliary = gpmi_payload + 4, gpmi_ctrl1 = gpmi_auxiliary + 4, gpmi_ctrl1_set,
	gpmi_ctrl1_clr, gpmi_ctrl1_tog, gpmi_timing0, gpmi_timing1 = gpmi_timing0 + 4,
	gpmi_timing2 = gpmi_timing1 + 4, gpmi_data = gpmi_timing2 + 4, gpmi_stat = gpmi_data + 4,
	gpmi_debug = gpmi_stat + 4, gpmi_version = gpmi_debug + 4, gpmi_debug2 = gpmi_version + 4,
	gpmi_debug3 = gpmi_debug2 + 4, gpmi_read_ddr_dll_ctrl = gpmi_debug3 + 4,
	gpmi_write_ddr_dll_ctrl = gpmi_read_ddr_dll_ctrl + 4,
	gpmi_read_ddr_dll_sts = gpmi_write_ddr_dll_ctrl + 4,
	gpmi_write_ddr_dll_sts = gpmi_read_ddr_dll_sts + 4,
};


enum {
	gpmi_address_increment = 1 << 16,
	gpmi_data_bytes = 0, gpmi_command_bytes = 1 << 17, gpmi_address_bytes = 2 << 17,
	gpmi_chip = 1 << 20,
	gpmi_8bit = 1 << 23,
	gpmi_write = 0, gpmi_read = 1 << 24, gpmi_read_compare = 2 << 24, gpmi_wait_for_ready = 3 << 24,
	gpmi_lock_cs = 1 << 27,
};


typedef struct {
	dma_t dma;
	uint32_t ctrl0;
} gpmi_dma1_t;


typedef struct {
	dma_t dma;
	uint32_t ctrl0;
	uint32_t compare;
	uint32_t eccctrl;
} gpmi_dma3_t;


typedef struct {
	dma_t dma;
	uint32_t ctrl0;
	uint32_t compare;
	uint32_t eccctrl;
	uint32_t ecccount;
	uint32_t payload;
	uint32_t auxiliary;
} gpmi_dma6_t;


typedef struct {
	char cmd1;
	char addrsz;
	signed char data;
	char cmd2;
} flashdrv_command_t;


static const flashdrv_command_t commands[flash_num_commands] = {
	{ 0xff, 0,  0, 0x00 }, /* reset */
	{ 0x90, 1,  0, 0x00 }, /* read_id */
	{ 0xec, 1,  0, 0x00 }, /* read_parameter_page */
	{ 0xed, 1,  0, 0x00 }, /* read_unique_id */
	{ 0xee, 1,  0, 0x00 }, /* get_features */
	{ 0xef, 1,  4, 0x00 }, /* set_features */
	{ 0x70, 0,  0, 0x00 }, /* read_status */
	{ 0x78, 3,  0, 0x00 }, /* read_status_enhanced */
	{ 0x05, 2,  0, 0xe0 }, /* random_data_read */
	{ 0x06, 5,  0, 0xe0 }, /* random_data_read_two_plane */
	{ 0x85, 2, -2, 0x00 }, /* random_data_input */
	{ 0x85, 5, -2, 0x00 }, /* program_for_internal_data_move_column */
	{ 0x00, 0,  0, 0x00 }, /* read_mode */
	{ 0x00, 5,  0, 0x30 }, /* read_page */
	{ 0x31, 0,  0, 0x00 }, /* read_page_cache_sequential */
	{ 0x00, 5,  0, 0x31 }, /* read_page_cache_random */
	{ 0x3f, 0,  0, 0x00 }, /* read_page_cache_last */
	{ 0x80, 5, -1, 0x10 }, /* program_page */
	{ 0x80, 5, -1, 0x15 }, /* program_page_cache */
	{ 0x60, 3,  0, 0xd0 }, /* erase_block */
	{ 0x00, 5,  0, 0x35 }, /* read_for_internal_data_move */
	{ 0x85, 5, -2, 0x10 }, /* program_for_internal_data_move */
	{ 0x23, 3,  0, 0x00 }, /* block_unlock_low */
	{ 0x24, 3,  0, 0x00 }, /* block_unlock_high */
	{ 0x2a, 0,  0, 0x00 }, /* block_lock */
	{ 0x2c, 0,  0, 0x00 }, /* block_lock_tight */
	{ 0x7a, 3,  0, 0x00 }, /* block_lock_read_status */
	{ 0x80, 5,  0, 0x10 }, /* otp_data_lock_by_block */
	{ 0x80, 5, -1, 0x10 }, /* otp_data_program */
	{ 0x00, 5,  0, 0x30 }, /* otp_data_read */
};


typedef struct _flashdrv_dma_t {
	dma_t *last;
	dma_t *first;
	char buffer[];
} flashdrv_dma_t;


typedef struct {
	uint8_t manufacturerid;
	uint8_t deviceid;
	uint8_t bytes[3];
} __attribute__((packed)) flash_id_t;


struct {
	volatile uint32_t *gpmi;
	volatile uint32_t *bch;
	volatile uint32_t *dma;
	volatile uint32_t *mux;

	handle_t mutex, wait_mutex, bch_cond, dma_cond;
	handle_t intbch, intdma, intgpmi;
	unsigned rawmetasz; /* user metadata + ECC16 metadata bytes */

	int result, bch_status, bch_done;

	uint8_t *uncached_buf;
	flashdrv_info_t info;
} flashdrv_common;


static inline int dma_pio(int pio)
{
	return (pio & 0xf) << 12;
}


static inline int dma_size(dma_t *dma)
{
	return sizeof(dma_t) + ((dma->flags >> 12) & 0xf) * sizeof(uint32_t);
}


static int dma_terminate(dma_t *dma, int err)
{
	memset(dma, 0, sizeof(*dma));

	dma->flags = dma_irqcomp | dma_decrsema | dma_noxfer;
	dma->buffer = (uint32_t)err;

	return sizeof(*dma);
}


static int dma_check(dma_t *dma, dma_t *fail)
{
	memset(dma, 0, sizeof(*dma));

	dma->flags = dma_hot | dma_sense;
	dma->buffer = (uint32_t)va2pa(fail);

	return sizeof(*dma);
}


static void dma_sequence(dma_t *prev, dma_t *next)
{
	if (prev != NULL) {
		prev->flags |= dma_chain;
		prev->next = (uint32_t)va2pa(next);
	}
}


static void dma_run(dma_t *dma, int channel)
{
	*(flashdrv_common.dma + apbh_ch0_nxtcmdar + channel * apbh_next_channel) = (uint32_t)va2pa(dma);
	*(flashdrv_common.dma + apbh_ch0_sema + channel * apbh_next_channel) = 1;
}


static int dma_irqHandler(unsigned int n, void *data)
{
	int comp, err;

	/* Check interrupt flags */
	comp = *(flashdrv_common.dma + apbh_ctrl1) & (1 << 0);
	err = *(flashdrv_common.dma + apbh_ctrl2) & ((1 << 16) | (1 << 0));

	/* Transform error status: 0: no error, 1: DMA termination, 2: AHB bus error */
	/* (DMA termination with completion flag is not treated as error) */
	err = (err >> 16) + (err & 1);
	err -= err & comp;

	/* Clear interrupt flags */
	*(flashdrv_common.dma + apbh_ctrl1_clr) = 1;
	*(flashdrv_common.dma + apbh_ctrl2_clr) = 1;

	/* If no error was detected return value set by DMA terminator descriptor (dma->buffer field prepared in dma_terminate()) */
	flashdrv_common.result = (err == 0) ? *(flashdrv_common.dma + apbh_ch0_bar) : -err;

	return 1;
}


static int bch_irqHandler(unsigned int n, void *data)
{
	/* Clear interrupt flags */
	flashdrv_common.bch_status = *(flashdrv_common.bch + bch_status0);
	flashdrv_common.bch_done = 1;
	*(flashdrv_common.bch + bch_ctrl_clr) = 1;
	return 1;
}


static int gpmi_irqHandler(unsigned int n, void *data)
{
	return 1;
}



static int nand_cmdaddr(gpmi_dma3_t *cmd, int chip, void *buffer, uint16_t addrsz)
{
	memset(cmd, 0, sizeof(*cmd));

	cmd->dma.flags = dma_hot | dma_w4endcmd | dma_nandlock | dma_read | dma_pio(3);
	cmd->dma.bufsz = (addrsz & 0x7) + 1;
	cmd->dma.buffer = (uint32_t)va2pa(buffer);

	cmd->ctrl0 = chip * gpmi_chip | gpmi_write | gpmi_command_bytes | gpmi_lock_cs | gpmi_8bit | cmd->dma.bufsz;

	if (addrsz)
		cmd->ctrl0 |= gpmi_address_increment;

	return sizeof(*cmd);
}


static int nand_read(gpmi_dma3_t *cmd, int chip, void *buffer, uint16_t bufsz)
{
	memset(cmd, 0, sizeof(*cmd));

	cmd->dma.flags = dma_hot | dma_nandlock | dma_w4endcmd | dma_write | dma_pio(3);
	cmd->dma.bufsz = bufsz;
	cmd->dma.buffer = (uint32_t)va2pa(buffer);

	cmd->ctrl0 = chip * gpmi_chip | gpmi_read | gpmi_data_bytes | gpmi_8bit | cmd->dma.bufsz;

	return sizeof(*cmd);
}


static int nand_readcompare(gpmi_dma3_t *cmd, int chip, uint16_t mask, uint16_t value)
{
	memset(cmd, 0, sizeof(*cmd));

	cmd->dma.flags = dma_hot | dma_nandlock | dma_w4endcmd | dma_noxfer | dma_pio(3);

	cmd->ctrl0 = chip * gpmi_chip | gpmi_read_compare | gpmi_data_bytes | gpmi_8bit | 1;
	cmd->compare = mask << 16 | value;

	return sizeof(*cmd);
}


static int nand_ecread(gpmi_dma6_t *cmd, int chip, void *payload, void *auxiliary, uint16_t bufsz)
{
	int eccmode = (payload == NULL) ? 0x100 : 0x1ff;
	memset(cmd, 0, sizeof(*cmd));

	cmd->dma.flags = dma_hot | dma_nandlock | dma_w4endcmd | dma_noxfer | dma_pio(6);
	cmd->dma.bufsz = 0;
	cmd->dma.buffer = 0;

	cmd->ctrl0 = chip * gpmi_chip | gpmi_read | gpmi_data_bytes | gpmi_8bit | bufsz;
	cmd->compare = 0;
	cmd->eccctrl = 1 << 12 | eccmode;
	cmd->ecccount = bufsz;
	if (payload != NULL)
		cmd->payload = (uint32_t)va2pa(payload);
	cmd->auxiliary = (uint32_t)va2pa(auxiliary);

	return sizeof(*cmd);
}


static int nand_disablebch(gpmi_dma3_t *cmd, int chip)
{
	memset(cmd, 0, sizeof(*cmd));

	cmd->dma.flags = dma_hot | dma_w4endcmd | dma_nandlock | dma_noxfer | dma_pio(3);
	cmd->ctrl0 = chip * gpmi_chip | gpmi_wait_for_ready | gpmi_lock_cs | gpmi_data_bytes | gpmi_8bit;

	return sizeof(*cmd);
}


static int nand_write(gpmi_dma3_t *cmd, int chip, void *buffer, uint16_t bufsz)
{
	memset(cmd, 0, sizeof(*cmd));

	cmd->dma.flags = dma_hot | dma_nandlock | dma_w4endcmd | dma_read | dma_pio(3);
	cmd->dma.bufsz = bufsz;
	cmd->dma.buffer = (uint32_t)va2pa(buffer);

	cmd->ctrl0 = chip * gpmi_chip | gpmi_write | gpmi_lock_cs | gpmi_data_bytes | gpmi_8bit | cmd->dma.bufsz;

	return sizeof(*cmd);
}


static int nand_ecwrite(gpmi_dma6_t *cmd, int chip, void *payload, void *auxiliary, uint16_t bufsz)
{
	int eccmode = (payload == NULL) ? 0x100 : 0x1ff;

	memset(cmd, 0, sizeof(*cmd));

	cmd->dma.flags = dma_hot | dma_nandlock | dma_w4endcmd | dma_noxfer | dma_pio(6);
	cmd->dma.bufsz = 0;
	cmd->dma.buffer = 0;

	cmd->ctrl0 = chip * gpmi_chip | gpmi_write | gpmi_lock_cs | gpmi_data_bytes | gpmi_8bit;
	cmd->compare = 0;
	cmd->eccctrl = 1 << 13 | 1 << 12 | eccmode;
	cmd->ecccount = bufsz;
	if (payload != NULL)
		cmd->payload = (uint32_t)va2pa(payload);
	cmd->auxiliary = (uint32_t)va2pa(auxiliary);

	return sizeof(*cmd);
}


static int nand_w4ready(gpmi_dma1_t *cmd)
{
	memset(cmd, 0, sizeof(*cmd));

	cmd->dma.flags = dma_hot | dma_w4endcmd | dma_w4ready | dma_noxfer | dma_pio(1);
	cmd->ctrl0 = gpmi_wait_for_ready | gpmi_8bit;

	return sizeof(*cmd);
}


static int flashdrv_addrPrep(uint32_t *paddr, int *bank)
{
	*bank = *paddr >> flashdrv_common.info.pbits;
	if (*bank >= flashdrv_common.info.chips) {
		return -1;
	}

	*paddr &= ~(0xffffffffu << flashdrv_common.info.pbits);

	return 0;
}


static void flashdrv_setDevClock(int dev, int state)
{
	platformctl_t p = { 0 };
	p.action = pctl_set;
	p.type = pctl_devclock;
	p.devclock.dev = dev;
	p.devclock.state = state;

	platformctl(&p);
}


flashdrv_dma_t *flashdrv_dmanew(void)
{
	flashdrv_dma_t *dma = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
	if (dma != MAP_FAILED) {
		dma->last = NULL;
		dma->first = NULL;
	}

	return dma;
}


void flashdrv_dmadestroy(flashdrv_dma_t *dma)
{
	if (dma != MAP_FAILED && dma != NULL) {
		munmap(dma, _PAGE_SIZE);
	}
}


int flashdrv_wait4ready(flashdrv_dma_t *dma, int err)
{
	void *next = dma->last, *prev = dma->last;
	int sz;
	dma_t *terminator;

	if (next != NULL)
		next += dma_size(dma->last);
	else
		next = dma->buffer;

	terminator = next;

	if (err != EOK) {
		sz = dma_terminate(terminator, err);
		next += sz;
	}

	sz = nand_w4ready(next);
	dma_sequence(prev, next);
	dma->last = next;
	next += sz;

	if (dma->first == NULL)
		dma->first = dma->last;

	sz = dma_check(next, terminator);
	dma_sequence(dma->last, next);
	dma->last = next;

	return EOK;
}


int flashdrv_disablebch(flashdrv_dma_t *dma, int chip)
{
	void *next = dma->last;

	if (next != NULL)
		next += dma_size(dma->last);
	else
		next = dma->buffer;

	nand_disablebch(next, chip);
	dma_sequence(dma->last, next);
	dma->last = next;

	if (dma->first == NULL)
		dma->first = dma->last;

	return EOK;
}


int flashdrv_finish(flashdrv_dma_t *dma)
{
	void *next = dma->last;

	if (next != NULL)
		next += dma_size(dma->last);
	else
		next = dma->buffer;

	dma_terminate(next, EOK);
	dma_sequence(dma->last, next);
	dma->last = next;

	if (dma->first == NULL)
		dma->first = dma->last;

	return EOK;
}

int flashdrv_issue(flashdrv_dma_t *dma, int c, int chip, void *addr, unsigned datasz, void *data, void *aux)
{
	void *next = dma->last;
	int sz;
	char *cmdaddr;

	if (next != NULL)
		next += dma_size(dma->last);
	else
		next = dma->buffer;

	if (commands[c].data > 0 && datasz != commands[c].data)
		return -EINVAL;

	if (commands[c].data == -1 && !datasz)
		return -EINVAL;

	if (!commands[c].data && datasz)
		return -EINVAL;

	cmdaddr = next;
	cmdaddr[0] = commands[c].cmd1;
	memcpy(cmdaddr + 1, addr, commands[c].addrsz);
	cmdaddr[7] = commands[c].cmd2;
	next += 8;

	sz = nand_cmdaddr(next, chip, cmdaddr, commands[c].addrsz);
	dma_sequence(dma->last, next);
	dma->last = next;
	next += sz;

	if (dma->first == NULL)
		dma->first = dma->last;

	if (datasz) {
		if (aux == NULL)
			/* No error correction */
			sz = nand_write(next, chip, data, datasz);
		else
			sz = nand_ecwrite(next, chip, data, aux, datasz);

		dma_sequence(dma->last, next);
		dma->last = next;
		next += sz;
	}

	if (commands[c].cmd2) {
		sz = nand_cmdaddr(next, chip, cmdaddr + 7, 0);
		dma_sequence(dma->last, next);
		dma->last = next;
	}

	return EOK;
}


int flashdrv_readback(flashdrv_dma_t *dma, int chip, int bufsz, void *buf, void *aux)
{
	void *next = dma->last;

	if (next != NULL)
		next += dma_size(dma->last);
	else
		next = dma->buffer;

	if (aux == NULL)
		/* No error correction */
		nand_read(next, chip, buf, bufsz);
	else
		nand_ecread(next, chip, buf, aux, bufsz);

	dma_sequence(dma->last, next);
	dma->last = next;

	if (dma->first == NULL)
		dma->first = dma->last;

	return EOK;
}


int flashdrv_readcompare(flashdrv_dma_t *dma, int chip, uint16_t mask, uint16_t value, int err)
{
	void *next = dma->last, *terminator;
	int sz;

	if (next != NULL)
		next += dma_size(dma->last);
	else
		next = dma->buffer;

	terminator = next;
	sz = dma_terminate(terminator, err);
	next += sz;

	sz = nand_readcompare(next, chip, mask, value);
	dma_sequence(dma->last, next);
	dma->last = next;
	next += sz;

	dma_check(next, terminator);
	dma_sequence(dma->last, next);
	dma->last = next;

	if (dma->first == NULL)
		dma->first = dma->last;

	return EOK;
}


int flashdrv_reset(flashdrv_dma_t *dma, int chip)
{
	int channel = 0, err;
	dma->first = NULL;
	dma->last = NULL;

	flashdrv_issue(dma, flash_reset, chip, NULL, 0, NULL, NULL);
	flashdrv_finish(dma);

	mutexLock(flashdrv_common.mutex);
	flashdrv_common.result = 1;
	dma_run((dma_t *)dma->first, channel);

	mutexLock(flashdrv_common.wait_mutex);
	while (flashdrv_common.result > 0)
		condWait(flashdrv_common.dma_cond, flashdrv_common.wait_mutex, 0);
	mutexUnlock(flashdrv_common.wait_mutex);

	err = flashdrv_common.result;
	mutexUnlock(flashdrv_common.mutex);

	return err;
}


static int flashdrv_readid(flashdrv_dma_t *dma, int chip, flash_id_t *flash_id)
{
	int channel = 0, err;
	char addr[1] = { 0 };

	dma->first = NULL;
	dma->last = NULL;

	flashdrv_wait4ready(dma, EOK);
	flashdrv_issue(dma, flash_read_id, chip, addr, 0, NULL, NULL);
	flashdrv_wait4ready(dma, EOK);
	flashdrv_readback(dma, chip, 5, flash_id, NULL);
	flashdrv_disablebch(dma, chip);
	flashdrv_wait4ready(dma, EOK);
	flashdrv_finish(dma);

	mutexLock(flashdrv_common.mutex);
	flashdrv_common.result = 1;
	dma_run((dma_t *)dma->first, channel);

	mutexLock(flashdrv_common.wait_mutex);
	while (flashdrv_common.result > 0)
		condWait(flashdrv_common.dma_cond, flashdrv_common.wait_mutex, 0);
	mutexUnlock(flashdrv_common.wait_mutex);

	err = flashdrv_common.result;
	mutexUnlock(flashdrv_common.mutex);

	return err;
}


int flashdrv_write(flashdrv_dma_t *dma, uint32_t paddr, void *data, char *aux)
{
	int chip = 0, channel = 0, sz;
	char addr[5] = { 0 };
	int skipMeta = 0, err;

	if (flashdrv_addrPrep(&paddr, &chip) < 0) {
		return -1;
	}

	memcpy(addr + 2, &paddr, 3);

	if (data == NULL) {
		sz = flashdrv_common.rawmetasz;
	}
	else {
		sz = flashdrv_common.info.writesz + flashdrv_common.info.metasz;
		if (aux == NULL) {
			aux = (char *)flashdrv_common.uncached_buf;
			skipMeta = 1;
		}
	}

	dma->first = NULL;
	dma->last = NULL;

	flashdrv_wait4ready(dma, EOK);
	flashdrv_issue(dma, flash_program_page, chip, addr, sz, data, aux);
	flashdrv_wait4ready(dma, EOK);
	flashdrv_issue(dma, flash_read_status, chip, NULL, 0, NULL, NULL);
	flashdrv_readcompare(dma, chip, 0x3, 0, -1);
	flashdrv_finish(dma);

	mutexLock(flashdrv_common.mutex);

	if (data == NULL) {
		/* Trick BCH controller into thinking that the whole page consists of just the metadata block */
		*(flashdrv_common.bch + bch_flash0layout0) &= ~(0xff << 24);

		*(flashdrv_common.bch + bch_flash0layout1) &= ~(0xffff << 16);
		*(flashdrv_common.bch + bch_flash0layout1) |= flashdrv_common.rawmetasz << 16;
	}
	else if (skipMeta) {
		/* Perform partial page programming (don't change metadata and its ECC) */
		memset(aux, 0xff, flashdrv_common.rawmetasz);

		/* Treat metadata and its ECC as raw byte area without ECC */
		*(flashdrv_common.bch + bch_flash0layout0) &= ~(0x1fff << 11);
		*(flashdrv_common.bch + bch_flash0layout0) |= flashdrv_common.rawmetasz << 16;
	}

	flashdrv_common.result = 1;
	dma_run((dma_t *)dma->first, channel);

	mutexLock(flashdrv_common.wait_mutex);
	while (flashdrv_common.result > 0)
		condWait(flashdrv_common.dma_cond, flashdrv_common.wait_mutex, 0);
	mutexUnlock(flashdrv_common.wait_mutex);

	err = flashdrv_common.result;

	if (data == NULL) {
		*(flashdrv_common.bch + bch_flash0layout0) |= 8 << 24;

		*(flashdrv_common.bch + bch_flash0layout1) &= ~(0xffff << 16);
		*(flashdrv_common.bch + bch_flash0layout1) |= (flashdrv_common.info.writesz + flashdrv_common.info.metasz) << 16;
	}
	else if (skipMeta) {
		*(flashdrv_common.bch + bch_flash0layout0) &= ~(0x1fff << 11);
		*(flashdrv_common.bch + bch_flash0layout0) |= 16 << 16 | 8 << 11;
	}

	mutexUnlock(flashdrv_common.mutex);

	return err;
}


int flashdrv_read(flashdrv_dma_t *dma, uint32_t paddr, void *data, flashdrv_meta_t *aux)
{
	int chip = 0, channel = 0, sz = 0, result;
	char addr[5] = { 0 };

	if (flashdrv_addrPrep(&paddr, &chip) < 0) {
		return -1;
	}

	memcpy(addr + 2, &paddr, 3);

	if (data != NULL)
		sz = flashdrv_common.info.writesz + flashdrv_common.info.metasz;
	else
		sz = flashdrv_common.rawmetasz;

	dma->first = NULL;
	dma->last = NULL;

	flashdrv_wait4ready(dma, EOK);
	flashdrv_issue(dma, flash_read_page, chip, addr, 0, NULL, NULL);
	flashdrv_wait4ready(dma, EOK);
	flashdrv_readback(dma, chip, sz, data, aux);
	flashdrv_disablebch(dma, chip);
	flashdrv_finish(dma);

	mutexLock(flashdrv_common.mutex);
	flashdrv_common.result = 1;
	flashdrv_common.bch_done = 0;
	dma_run((dma_t *)dma->first, channel);

	mutexLock(flashdrv_common.wait_mutex);
	while (!flashdrv_common.bch_done)
		condWait(flashdrv_common.bch_cond, flashdrv_common.wait_mutex, 0);

	while (flashdrv_common.result > 0)
		condWait(flashdrv_common.dma_cond, flashdrv_common.wait_mutex, 0);
	mutexUnlock(flashdrv_common.wait_mutex);

	result = flashdrv_common.bch_status;
	mutexUnlock(flashdrv_common.mutex);

	return result;
}


int flashdrv_erase(flashdrv_dma_t *dma, uint32_t paddr)
{
	int chip = 0, channel = 0, result;
	dma->first = NULL;
	dma->last = NULL;

	if (flashdrv_addrPrep(&paddr, &chip) < 0) {
		return -1;
	}

	flashdrv_wait4ready(dma, EOK);
	flashdrv_issue(dma, flash_erase_block, chip, &paddr, 0, NULL, NULL);
	flashdrv_wait4ready(dma, EOK);
	flashdrv_issue(dma, flash_read_status, chip, NULL, 0, NULL, NULL);
	flashdrv_readcompare(dma, chip, 0x1, 0, -1);
	flashdrv_finish(dma);

	mutexLock(flashdrv_common.mutex);
	flashdrv_common.result = 1;
	dma_run((dma_t *)dma->first, channel);

	mutexLock(flashdrv_common.wait_mutex);
	while (flashdrv_common.result > 0)
		condWait(flashdrv_common.dma_cond, flashdrv_common.wait_mutex, 0);
	mutexUnlock(flashdrv_common.wait_mutex);

	result = flashdrv_common.result;
	mutexUnlock(flashdrv_common.mutex);

	return result;
}


int flashdrv_writeraw(flashdrv_dma_t *dma, uint32_t paddr, void *data, int sz)
{
	int chip = 0, channel = 0, err;
	char addr[5] = { 0 };

	if (flashdrv_addrPrep(&paddr, &chip) < 0) {
		return -1;
	}

	memcpy(addr + 2, &paddr, 3);

	dma->first = NULL;
	dma->last = NULL;

	flashdrv_wait4ready(dma, EOK);
	flashdrv_issue(dma, flash_program_page, chip, addr, sz, data, NULL);
	flashdrv_wait4ready(dma, EOK);
	flashdrv_issue(dma, flash_read_status, chip, NULL, 0, NULL, NULL);
	flashdrv_readcompare(dma, 0, 0x3, 0, -1);
	flashdrv_finish(dma);

	mutexLock(flashdrv_common.mutex);
	flashdrv_common.result = 1;
	dma_run((dma_t *)dma->first, channel);

	mutexLock(flashdrv_common.wait_mutex);
	while (flashdrv_common.result > 0)
		condWait(flashdrv_common.dma_cond, flashdrv_common.wait_mutex, 0);
	mutexUnlock(flashdrv_common.wait_mutex);

	err = flashdrv_common.result;
	mutexUnlock(flashdrv_common.mutex);

	return err;
}


int flashdrv_readraw(flashdrv_dma_t *dma, uint32_t paddr, void *data, int sz)
{
	int chip = 0, channel = 0, err;
	char addr[5] = { 0 };

	if (flashdrv_addrPrep(&paddr, &chip) < 0) {
		return -1;
	}

	memcpy(addr + 2, &paddr, 3);

	dma->first = NULL;
	dma->last = NULL;

	flashdrv_wait4ready(dma, EOK);
	flashdrv_issue(dma, flash_read_page, chip, addr, 0, NULL, NULL);
	flashdrv_wait4ready(dma, EOK);
	flashdrv_readback(dma, chip, sz, data, NULL);
	flashdrv_disablebch(dma, chip);
	flashdrv_wait4ready(dma, EOK);
	flashdrv_finish(dma);

	mutexLock(flashdrv_common.mutex);
	flashdrv_common.result = 1;
	dma_run((dma_t *)dma->first, channel);

	mutexLock(flashdrv_common.wait_mutex);
	while (flashdrv_common.result > 0)
		condWait(flashdrv_common.dma_cond, flashdrv_common.wait_mutex, 0);
	mutexUnlock(flashdrv_common.wait_mutex);

	err = flashdrv_common.result;
	mutexUnlock(flashdrv_common.mutex);

	return err;
}


/* valid addresses are only the beginning of erase block */
int flashdrv_isbad(flashdrv_dma_t *dma, uint32_t paddr)
{
	int chip = 0, channel = 0, isbad = 0;
	uint8_t *data = flashdrv_common.uncached_buf;
	char addr[5] = { 0 };

	if (flashdrv_addrPrep(&paddr, &chip) < 0) {
		return -1;
	}

	memcpy(addr + 2, &paddr, 3);

	assert((paddr % 64) == 0);

	dma->first = NULL;
	dma->last = NULL;

	/* read RAW first 16 bytes (metadata) to skip ECC checks */
	flashdrv_wait4ready(dma, EOK);
	flashdrv_issue(dma, flash_read_page, chip, addr, 0, NULL, NULL);
	flashdrv_wait4ready(dma, EOK);
	flashdrv_readback(dma, chip, 16, data, NULL);
	flashdrv_disablebch(dma, chip);
	flashdrv_wait4ready(dma, EOK);
	flashdrv_finish(dma);

	mutexLock(flashdrv_common.mutex);
	flashdrv_common.result = 1;
	dma_run((dma_t *)dma->first, channel);

	mutexLock(flashdrv_common.wait_mutex);
	while (flashdrv_common.result > 0)
		condWait(flashdrv_common.dma_cond, flashdrv_common.wait_mutex, 0);
	mutexUnlock(flashdrv_common.wait_mutex);

	isbad = flashdrv_common.result;
	if (isbad < 0)
		isbad = 1; /* read error, assume badblock */
	else
		isbad = data[0] == 0x00; /* badblock marker present */
	mutexUnlock(flashdrv_common.mutex);

	return isbad;
}


int flashdrv_markbad(flashdrv_dma_t *dma, uint32_t paddr)
{
	int chip = 0, channel = 0, err;
	uint8_t *data = flashdrv_common.uncached_buf;
	const unsigned int metasz = 16;
	char addr[5] = { 0 };

	if (flashdrv_addrPrep(&paddr, &chip) < 0) {
		return -1;
	}

	memcpy(addr + 2, &paddr, 3);

	assert((paddr % 64) == 0);

	dma->first = NULL;
	dma->last = NULL;

	/* NOTE: writing without ECC */
	flashdrv_wait4ready(dma, EOK);
	flashdrv_issue(dma, flash_program_page, chip, addr, metasz, data, NULL);
	flashdrv_wait4ready(dma, EOK);
	flashdrv_issue(dma, flash_read_status, chip, NULL, 0, NULL, NULL);
	flashdrv_readcompare(dma, 0, 0x3, 0, -1);
	flashdrv_finish(dma);

	mutexLock(flashdrv_common.mutex);

	memset(data, 0xff, flashdrv_common.info.writesz + flashdrv_common.info.metasz);
	memset(data, 0x0, metasz);

	flashdrv_common.result = 1;
	dma_run((dma_t *)dma->first, channel);

	mutexLock(flashdrv_common.wait_mutex);
	while (flashdrv_common.result > 0)
		condWait(flashdrv_common.dma_cond, flashdrv_common.wait_mutex, 0);
	mutexUnlock(flashdrv_common.wait_mutex);

	err = flashdrv_common.result;
	mutexUnlock(flashdrv_common.mutex);

	return err;
}


static void setup_flash_info(void)
{
	flash_id_t *flash_id = (flash_id_t *)flashdrv_common.uncached_buf;
	flashdrv_dma_t *dma = flashdrv_dmanew();

	memset(flash_id, 0, sizeof(*flash_id));

	/* FIXME no way to handle this error, avoid crash for now */
	if (dma != MAP_FAILED) {
		flashdrv_reset(dma, 0);
		flashdrv_readid(dma, 0, flash_id);
	}

	if (flash_id->manufacturerid == 0x98 && flash_id->deviceid == 0xd3) {
		flashdrv_common.info.name = "Kioxia TH58NV 16Gbit NAND";
		flashdrv_common.info.size = 4096ull * 64 * 4096; /* per chip */
		flashdrv_common.info.writesz = 4096u;
		flashdrv_common.info.metasz = 256u;
		flashdrv_common.info.erasesz = 4096u * 64;
		flashdrv_common.info.oobsz = 16;
		flashdrv_common.info.oobavail = 16;
		flashdrv_common.info.pbits = 18;

		/* Detect if second internal chip is present */
		flashdrv_reset(dma, 1);
		flashdrv_readid(dma, 1, flash_id);
		if (flash_id->manufacturerid == 0x98 && flash_id->deviceid == 0xd3) {
			flashdrv_common.info.chips = 2;
			flashdrv_common.info.size *= 2;
		}
		else {
			flashdrv_common.info.name = "Kioxia TH58NV 16Gbit NAND (8Gbit accessible)";
			flashdrv_common.info.chips = 1;
		}
	}
	else if (flash_id->manufacturerid == 0x2c && flash_id->deviceid == 0xd3) {
		flashdrv_common.info.name = "Micron MT29F8G 8Gbit NAND";
		flashdrv_common.info.size = 4096ull * 64 * 4096;
		flashdrv_common.info.writesz = 4096u;
		flashdrv_common.info.metasz = 224u;
		flashdrv_common.info.erasesz = 4096u * 64;
		flashdrv_common.info.oobsz = 16;
		flashdrv_common.info.oobavail = 16;
		flashdrv_common.info.chips = 1;
		flashdrv_common.info.pbits = 18;
	}
	else {
		/* use sane defaults */
		flashdrv_common.info.name = "Unknown 8Gbit NAND";
		flashdrv_common.info.size = 4096ull * 64 * 4096;
		flashdrv_common.info.writesz = 4096u;
		flashdrv_common.info.metasz = 224u;
		flashdrv_common.info.erasesz = 4096u * 64;
		flashdrv_common.info.oobsz = 16;
		flashdrv_common.info.oobavail = 16;
		flashdrv_common.info.chips = 1;
		flashdrv_common.info.pbits = 18;
	}

	flashdrv_dmadestroy(dma);
}


void flashdrv_init(void)
{
	flashdrv_common.dma = mmap(NULL, 2 * _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0x1804000);
	flashdrv_common.gpmi = mmap(NULL, 2 * _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0x1806000);
	flashdrv_common.bch = mmap(NULL, 4 * _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0x1808000);
	flashdrv_common.mux = mmap(NULL, 4 * _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0x20e0000);
	flashdrv_common.uncached_buf = mmap(NULL, 2 * _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_CONTIGUOUS | MAP_ANONYMOUS, -1, 0);

	/* 16 bytes of user metadada + ECC16 * bits per parity level (13) / 8 = 26 bytes for ECC  */
	flashdrv_common.rawmetasz = 16 + 26;

	flashdrv_common.dma_cond = flashdrv_common.bch_cond = flashdrv_common.mutex = 0;

	condCreate(&flashdrv_common.bch_cond);
	condCreate(&flashdrv_common.dma_cond);
	mutexCreate(&flashdrv_common.mutex);
	mutexCreate(&flashdrv_common.wait_mutex);

	flashdrv_setDevClock(pctl_clk_apbhdma, 3);
	flashdrv_setDevClock(pctl_clk_rawnand_u_gpmi_input_apb, 3);
	flashdrv_setDevClock(pctl_clk_rawnand_u_gpmi_bch_input_gpmi_io, 3);
	flashdrv_setDevClock(pctl_clk_rawnand_u_gpmi_bch_input_bch, 3);
	flashdrv_setDevClock(pctl_clk_rawnand_u_bch_input_apb, 3);

	flashdrv_setDevClock(pctl_clk_iomuxc, 3);

	*(flashdrv_common.dma + apbh_ctrl0) &= ~(1 << 31 | 1 << 30);
	*(flashdrv_common.gpmi + gpmi_ctrl0) &= ~(1 << 31 | 1 << 30);

	*(flashdrv_common.bch + bch_ctrl_clr) = (1 << 31);
	*(flashdrv_common.bch + bch_ctrl_clr) = (1 << 30);

	*(flashdrv_common.bch + bch_ctrl_set) = (1 << 31);
	while (!(*(flashdrv_common.bch + bch_ctrl) & (1 << 30)));

	*(flashdrv_common.bch + bch_ctrl_clr) = (1 << 31);
	*(flashdrv_common.bch + bch_ctrl_clr) = (1 << 30);

	/* GPMI clock = 198 MHz (~5ns period), address setup = 3 (~15ns), data hold = 2 (~10ns), data setup = 3 (~15ns) */
	*(flashdrv_common.gpmi + gpmi_timing0) = (3 << 16) | (2 << 8) | (3 << 0);
	/* Set wait for ready timeout */
	*(flashdrv_common.gpmi + gpmi_timing1) = 0xffff << 16;

	/* enable irq on channel 0 */
	*(flashdrv_common.dma + apbh_ctrl1) |= 1 << 16;

	/* flush irq flag */
	if (*(flashdrv_common.dma + apbh_ctrl1_clr) & 1)
		*(flashdrv_common.dma + apbh_ctrl1_clr) = 1;

	for (int i = 0; i < 17; ++i) {
		/* set all NAND pins to NAND function */
		*(flashdrv_common.mux + i + 94) = 0;
	}

	/* Set DECOUPLE_CS, WRN no delay, GANGED_RDYBUSY, BCH, RDN_DELAY, WP, #R/B busy-low */
	*(flashdrv_common.gpmi + gpmi_ctrl1) = (1 << 24) | (3 << 22) | (1 << 19) | (1 << 18) | (14 << 12) | (1 << 3) | (1 << 2);
	/* Enable DLL */
	*(flashdrv_common.gpmi + gpmi_ctrl1_set) = (1 << 17);

	interrupt(32 + 13, dma_irqHandler, NULL, flashdrv_common.dma_cond, &flashdrv_common.intdma);
	interrupt(32 + 16, gpmi_irqHandler, NULL, 0, &flashdrv_common.intgpmi);

	/* read NAND configuration before BCH setup */
	setup_flash_info();

	/* set BCH up */
	*(flashdrv_common.bch + bch_ctrl_set) = 1 << 8;
	*(flashdrv_common.bch + bch_layoutselect) = 0;

	/* 8 blocks/page, 16 bytes metadata, ECC16, GF13, 0 word data0 */
	*(flashdrv_common.bch + bch_flash0layout0) = 8 << 24 | 16 << 16 | 8 << 11 | 0 << 10 | 0;

	/* flash layout takes only 4096 + 224 bytes, but it's universal across used NAND chips */
	/* 4096 + 256 page size, ECC14, GF13, 128 word dataN (512 bytes) */
	*(flashdrv_common.bch + bch_flash0layout1) = (flashdrv_common.info.writesz + flashdrv_common.info.metasz) << 16 | 7 << 11 | 0 << 10 | 128;

	interrupt(32 + 15, bch_irqHandler, NULL, flashdrv_common.bch_cond, &flashdrv_common.intbch);
}


const flashdrv_info_t *flashdrv_info(void)
{
	return &flashdrv_common.info;
}
