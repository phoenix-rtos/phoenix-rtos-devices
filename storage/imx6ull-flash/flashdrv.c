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
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/interrupt.h>
#include <sys/platform.h>

#include "flashdrv.h"
#include "arch/imx6ull.h"


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
	u32 next;
	u16 flags;
	u16 bufsz;
	u32 buffer;
	u32 pio[];
} dma_t;


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
	u32 ctrl0;
} gpmi_dma1_t;


typedef struct {
	dma_t dma;
	u32 ctrl0;
	u32 compare;
	u32 eccctrl;
} gpmi_dma3_t;


typedef struct {
	dma_t dma;
	u32 ctrl0;
	u32 compare;
	u32 eccctrl;
	u32 ecccount;
	u32 payload;
	u32 auxiliary;
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
	int sz;
	dma_t *last;
	dma_t *first;
	char *free;
	char buffer[];
} flashdrv_dma_t;


struct {
	volatile u32 *gpmi;
	volatile u32 *bch;
	volatile u32 *dma;
	volatile u32 *mux;

	handle_t mutex, cond;
	int result;
} flashdrv_common;


static inline int dma_pio(int pio)
{
	return (pio & 0xf) << 12;
}


static inline int dma_size(dma_t *dma)
{
	return sizeof(dma_t) + ((dma->flags >> 12) & 0xf) * sizeof(u32);
}


static int dma_terminate(dma_t *dma, int err)
{
	memset(dma, 0, sizeof(*dma));

	dma->flags = dma_irqcomp | dma_decrsema | dma_noxfer;
	dma->buffer = (u32)err;

	return sizeof(*dma);
}


static int dma_check(dma_t *dma, dma_t *fail)
{
	memset(dma, 0, sizeof(*dma));

	dma->flags = dma_hot | dma_sense;
	dma->buffer = (u32)va2pa(fail);

	return sizeof(*dma);
}


static void dma_sequence(dma_t *prev, dma_t *next)
{
	if (prev != NULL) {
		prev->flags |= dma_chain;
		prev->next = (u32)va2pa(next);
	}
}


#if 0
static void dma_vsequence(dma_t *prev, ...)
{
	va_list ap;
	dma_t *next;

	va_start(ap, prev);
	while ((next = va_arg(ap, dma_t *)) != NULL) {
		dma_sequence(prev, next);
		prev = next;
	}
	va_end(ap);

	prev->next = 0;
}
#endif


static void dma_run(dma_t *dma, int channel)
{
	*(flashdrv_common.dma + apbh_ch0_nxtcmdar + channel * apbh_next_channel) = (u32)va2pa(dma);
	*(flashdrv_common.dma + apbh_ch0_sema + channel * apbh_next_channel) = 1;
}


static int dma_irqHandler(unsigned int n, void *data)
{
	/* Clear interrupt flags */
	*(flashdrv_common.dma + apbh_ctrl1) &= ~0xffff;
	/* TODO: report errors, etc? */
	flashdrv_common.result = *(flashdrv_common.dma + apbh_ch0_bar);
	return 1;
}


static int nand_cmdaddr(gpmi_dma3_t *cmd, int chip, void *buffer, u16 addrsz)
{
	memset(cmd, 0, sizeof(*cmd));

	cmd->dma.flags = dma_hot | dma_w4endcmd | dma_nandlock | dma_read | dma_pio(3);
	cmd->dma.bufsz = (addrsz & 0x7) + 1;
	cmd->dma.buffer = (u32)va2pa(buffer);

	cmd->ctrl0 = chip * gpmi_chip | gpmi_write | gpmi_command_bytes | gpmi_lock_cs | gpmi_8bit | cmd->dma.bufsz;

	if (addrsz)
		cmd->ctrl0 |= gpmi_address_increment;

	return sizeof(*cmd);
}


static int nand_read(gpmi_dma1_t *cmd, int chip, void *buffer, u16 bufsz)
{
	memset(cmd, 0, sizeof(*cmd));

	cmd->dma.flags = dma_hot | dma_nandlock | dma_w4endcmd | dma_write | dma_pio(1);
	cmd->dma.bufsz = bufsz;
	cmd->dma.buffer = (u32)va2pa(buffer);

	cmd->ctrl0 = chip * gpmi_chip | gpmi_read | gpmi_lock_cs | gpmi_data_bytes | gpmi_8bit | cmd->dma.bufsz;

	return sizeof(*cmd);
}


static int nand_write(gpmi_dma1_t *cmd, int chip, void *buffer, u16 bufsz)
{
	memset(cmd, 0, sizeof(*cmd));

	cmd->dma.flags = dma_hot | dma_nandlock | dma_w4endcmd | dma_read | dma_pio(1);
	cmd->dma.bufsz = bufsz;
	cmd->dma.buffer = (u32)va2pa(buffer);

	cmd->ctrl0 = chip * gpmi_chip | gpmi_write | gpmi_lock_cs | gpmi_data_bytes | gpmi_8bit | cmd->dma.bufsz;

	return sizeof(*cmd);
}


static int nand_w4ready(gpmi_dma1_t *cmd, int chip)
{
	memset(cmd, 0, sizeof(*cmd));

	cmd->dma.flags = dma_hot | dma_w4endcmd | dma_w4ready | dma_noxfer | dma_pio(1);
	cmd->ctrl0 = chip * gpmi_chip | gpmi_wait_for_ready | gpmi_8bit | gpmi_lock_cs;

	return sizeof(*cmd);
}


#if 0
static int flashdrv_getDevClock(int dev)
{
	platformctl_t p = { 0 };
	p.action = pctl_get;
	p.type = pctl_devclock;
	p.devclock.dev = dev;

	platformctl(&p);
	return p.devclock.state;
}
#endif


static void flashdrv_setDevClock(int dev, int state)
{
	platformctl_t p = { 0 };
	p.action = pctl_set;
	p.type = pctl_devclock;
	p.devclock.dev = dev;
	p.devclock.state = state;

	platformctl(&p);
}


static int flashdrv_overflow(flashdrv_dma_t *dma)
{
	return dma->last != NULL && ((void *)dma->last - (void *)dma) > (SIZE_PAGE - 2 * sizeof(gpmi_dma6_t));
}


flashdrv_dma_t *flashdrv_dmanew(void)
{
	flashdrv_dma_t *dma = mmap(NULL, SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);
	dma->last = NULL;
	dma->first = NULL;

	return dma;
}


void flashdrv_dmadestroy(flashdrv_dma_t *dma)
{
	munmap(dma, SIZE_PAGE);
}


int flashdrv_wait4ready(flashdrv_dma_t *dma, int chip, int err)
{
	void *next = dma->last, *prev = dma->last;
	int sz;
	dma_t *terminator;

	if (next != NULL)
		next += dma_size(dma->last);
	else
		next = dma->buffer;

	if (flashdrv_overflow(dma))
		return -ENOMEM;

	terminator = next;

	if (err != EOK) {
		sz = dma_terminate(terminator, err);
		next += sz;
	}

	sz = nand_w4ready(next, chip);
	dma_sequence(prev, next);
	dma->last = next;
	next += sz;

	if (dma->first == NULL)
		dma->first = dma->last;

	if (flashdrv_overflow(dma))
		return -ENOMEM;

	sz = dma_check(next, terminator);
	dma_sequence(dma->last, next);
	dma->last = next;

	return EOK;
}


int flashdrv_finish(flashdrv_dma_t *dma)
{
	void *next = dma->last;

	if (next != NULL)
		next += dma_size(dma->last);
	else
		next = dma->buffer;

	if (flashdrv_overflow(dma))
		return -ENOMEM;

	dma_terminate(next, EOK);
	dma_sequence(dma->last, next);
	dma->last = next;

	if (dma->first == NULL)
		dma->first = dma->last;

	return EOK;
}


int flashdrv_issue(flashdrv_dma_t *dma, int c, int chip, void *addr, unsigned datasz, void *data)
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

	if (flashdrv_overflow(dma))
		return -ENOMEM;

	cmdaddr = next;
	cmdaddr[0] = commands[c].cmd1;
	memcpy(cmdaddr + 2, addr, commands[c].addrsz);
	cmdaddr[8] = commands[c].cmd2;
	next += 12;

	sz = nand_cmdaddr(next, chip, cmdaddr, commands[c].addrsz);
	dma_sequence(dma->last, next);
	dma->last = next;
	next += sz;

	if (dma->first == NULL)
		dma->first = dma->last;

	if (datasz) {
		if (flashdrv_overflow(dma))
			return -ENOMEM;

		sz = nand_write(next, chip, data, datasz);
		dma_sequence(dma->last, next);
		dma->last = next;
		next += sz;
	}

	if (commands[c].cmd2) {
		if (flashdrv_overflow(dma))
			return -ENOMEM;

		sz = nand_cmdaddr(next, chip, cmdaddr + 8, 0);
		dma_sequence(dma->last, next);
		dma->last = next;
	}

	return EOK;
}


int flashdrv_readback(flashdrv_dma_t *dma, int chip, int bufsz, void *buf)
{
	void *next = dma->last;

	if (next != NULL)
		next += dma_size(dma->last);
	else
		next = dma->buffer;

	if (flashdrv_overflow(dma))
		return -ENOMEM;

	nand_read(next, chip, buf, bufsz);
	dma_sequence(dma->last, next);
	dma->last = next;

	if (dma->first == NULL)
		dma->first = dma->last;

	return EOK;
}


void flashdrv_rundma(flashdrv_dma_t *dma)
{
	int channel = 0;

	mutexLock(flashdrv_common.mutex);
	dma_run((dma_t *)dma->first, channel);
	condWait(flashdrv_common.cond, flashdrv_common.mutex, 0);
	mutexUnlock(flashdrv_common.mutex);
}


void flashdrv_init(void)
{
	flashdrv_common.dma  = mmap(NULL, 2 * SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_DEVICE, OID_PHYSMEM, 0x1804000);
	flashdrv_common.gpmi = mmap(NULL, 2 * SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_DEVICE, OID_PHYSMEM, 0x1806000);
	flashdrv_common.bch  = mmap(NULL, 4 * SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_DEVICE, OID_PHYSMEM, 0x1808000);
	flashdrv_common.mux  = mmap(NULL, 4 * SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_DEVICE, OID_PHYSMEM, 0x20e0000);

	flashdrv_common.cond = flashdrv_common.mutex = 0;

	condCreate(&flashdrv_common.cond);
	mutexCreate(&flashdrv_common.mutex);

	flashdrv_setDevClock(apbhdma, 3);
	flashdrv_setDevClock(rawnand_u_gpmi_input_apb, 3);
	flashdrv_setDevClock(rawnand_u_gpmi_bch_input_gpmi_io, 3);
	flashdrv_setDevClock(rawnand_u_gpmi_bch_input_bch, 3);
	flashdrv_setDevClock(rawnand_u_bch_input_apb, 3);

	flashdrv_setDevClock(iomuxc, 3);
	flashdrv_setDevClock(iomux_ipt_clk_io, 3);
	flashdrv_setDevClock(iomuxc_gpr, 3);
	flashdrv_setDevClock(iomuxc_snvs, 3);
	flashdrv_setDevClock(iomux_snvs_gpr, 3);

	*(flashdrv_common.dma) &= ~(1 << 31 | 1 << 30);
	*(flashdrv_common.bch) &= ~(1 << 31 | 1 << 30);
	*(flashdrv_common.gpmi + gpmi_ctrl0) &= ~(1 << 31 | 1 << 30);

	/* Set wait for ready timeout */
	*(flashdrv_common.gpmi + gpmi_timing1) = 0xffff << 16;

	/* enable irq on channel 0 */
	*(flashdrv_common.dma + apbh_ctrl1) |= 1 << 16;

	for (int i = 0; i < 17; ++i) {
		/* set all NAND pins to NAND function */
		*(flashdrv_common.mux + i + 94) = 0;
	}

	/* set #R/B busy-low, WP */
	*(flashdrv_common.gpmi + gpmi_ctrl1) |= (1 << 2) | (1 << 3);

	interrupt(32 + 13, dma_irqHandler, NULL, flashdrv_common.cond);
}


int main(int argc, char **argv)
{
	flashdrv_dma_t *dma;
	int err;

	struct {
		char address[8];
		int input;
		int output;
		int status;
	} *buffer;

	buffer = mmap(NULL, SIZE_PAGE, PROT_READ | PROT_WRITE, MAP_UNCACHED, OID_NULL, 0);
	memset(buffer->address, 0, 5);

	buffer->input = 0xdeadbeef;

	flashdrv_init();

	dma = flashdrv_dmanew();

	do {
		if ((err = flashdrv_issue(dma, flash_reset, 0, NULL, 0, NULL)) < 0)
			break;
		if ((err = flashdrv_wait4ready(dma, 0, EOK)) < 0)
			break;
		if ((err = flashdrv_issue(dma, flash_program_page, 0, &buffer->address, 4, &buffer->input)) < 0)
			break;
		if ((err = flashdrv_wait4ready(dma, 0, EOK)) < 0)
			break;
		if ((err = flashdrv_issue(dma, flash_read_status, 0, NULL, 0, NULL)) < 0)
			break;
		if ((err = flashdrv_readback(dma, 0, 1, &buffer->status)) < 0)
			break;
		if ((err = flashdrv_wait4ready(dma, 0, EOK)) < 0)
			break;
		if ((err = flashdrv_issue(dma, flash_read_page, 0, &buffer->address, 0, NULL)) < 0)
			break;
		if ((err = flashdrv_wait4ready(dma, 0, EOK)) < 0)
			break;
		if ((err = flashdrv_readback(dma, 0, 4, &buffer->output)) < 0)
			break;
		if ((err = flashdrv_finish(dma)) < 0)
			break;
	} while (0);

	if (!err) {
		printf("run!\n");
		//flashdrv_rundma(dma);
	}

	flashdrv_dmadestroy(dma);

	// printf("id: %x %x %x %x %x\n", buffer->data[0], buffer->data[1], buffer->data[2], buffer->data[3], buffer->data[4]);

	printf("wrote: %x    read: %x    write status: %x  result: %x\n", buffer->input, buffer->output, buffer->status, flashdrv_common.result);

	for (;;)
		usleep(10000000);
}

