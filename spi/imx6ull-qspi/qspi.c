/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL QSPI lib
 *
 * Copyright 2023 Phoenix Systems
 * Author: Hubert Badocha
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stddef.h>
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <stdbool.h>
#include <string.h>
#include <phoenix/arch/imx6ull.h>
#include <board_config.h>

#include "qspi.h"

/* Default values for not configured boards */
#ifndef QSPI_FLASH_A1_SIZE
#define QSPI_FLASH_A1_SIZE (256 * 1024 * 1024)
#endif
#ifndef QSPI_FLASH_A2_SIZE
#define QSPI_FLASH_A2_SIZE (0)
#endif
#ifndef QSPI_FLASH_B1_SIZE
#define QSPI_FLASH_B1_SIZE (0)
#endif
#ifndef QSPI_FLASH_B2_SIZE
#define QSPI_FLASH_B2_SIZE (0)
#endif

#define QSPI_MMAP_BASE 0x60000000

#define QSPI_BASE ((void *)0x21e0000)

#define QSPI_LCKCR        (0x304 / sizeof(uint32_t))
#define QSPI_LCKCR_LOCK   0x01
#define QSPI_LCKCR_UNLOCK 0x02

#define QSPI_LUTKEY (0x300 / sizeof(uint32_t))
#define QSPI_LUTKEY_KEY 0x5af05af0

#define QSPI_LUTn(n)           ((0x310 + 4 * (n)) / sizeof(uint32_t))
#define QSPI_SEQ_START_REGN(n) (4 * (n))

#define QSPI_IPCR                         (0x08 / sizeof(uint32_t))
#define QSPI_IPCR_SET_SEQID(reg, seqid)   ((reg) & ~(0x0f << 24)) | ((seqid & 0x0f) << 24)
#define QSPI_IPCR_SET_IDATSZ(reg, idatsz) ((reg) & ~(0xffff)) | (idatsz)

#define QSPI_FLSHCR (0x0C / sizeof(uint32_t))

#define QSPI_MCR          (0)
#define QSPI_MCR_CLR_TXF  (0x01 << 11)
#define QSPI_MCR_CLR_RXF  (0x01 << 10)
#define QSPI_MCR_CLR_MDIS (0x01 << 14)

#define QSPI_BFGENCR (0x20 / sizeof(uint32_t))

#define QSPI_SR        (0x15c / sizeof(uint32_t))
#define QSPI_SR_BUSY   (0x01)
#define QSPI_SR_RXWE   (0x01 << 16)
#define QSPI_SR_TXEDA  (0x01 << 24)
#define QSPI_SR_TXFULL (0x01 << 27)

#define QSPI_RBDRn(n) ((0x200 + 4 * (n)) / sizeof(uint32_t))

#define QSPI_RBSR         (0x10c / sizeof(uint32_t))
#define QSPI_RBSR_TO_READ (((*QSPI_RBSR_ADRR) >> 8) & 0x3f)

#define QSPI_SFAR (0x100 / sizeof(uint32_t))

#define QSPI_RBCT       (0x110 / sizeof(uint32_t))
#define QSPI_RBCT_RXBRD (0x01 << 8)

#define QSPI_FR      (0x160 / sizeof(uint32_t))
#define QSPI_FR_RBDF (0x01 << 16)

#define QSPI_TBDR (0x154 / sizeof(uint32_t))

#define QSPI_SFA1AD (0x180 / sizeof(uint32_t))
#define QSPI_SFA2AD (0x184 / sizeof(uint32_t))
#define QSPI_SFB1AD (0x188 / sizeof(uint32_t))
#define QSPI_SFB2AD (0x18c / sizeof(uint32_t))

#define WATERMARK 16

#define NUM_LUT_SEQ 16

typedef struct {
	int mux;
	char val;
	char sion;
} qspi_pctl_mux_t;

typedef struct {
	int pad;
	char speed;
	char dse;
	char sre;
} qspi_pctl_pad_t;

qspi_pctl_mux_t qspi_pctl_mux[2][8] = {
	{ { pctl_mux_nand_d7, 2, 0 }, { pctl_mux_nand_ale, 2, 0 }, { pctl_mux_nand_wp, 2, 0 }, { pctl_mux_nand_rdy, 2, 0 }, { pctl_mux_nand_ce0, 2, 0 }, { pctl_mux_nand_ce1, 2, 0 }, { pctl_mux_nand_cle, 2, 0 }, { pctl_mux_nand_dqs, 2, 0 } },
	{ { pctl_mux_nand_re, 2, 0 }, { pctl_mux_nand_we, 2, 0 }, { pctl_mux_nand_d0, 2, 0 }, { pctl_mux_nand_d1, 2, 0 }, { pctl_mux_nand_d2, 2, 0 }, { pctl_mux_nand_d3, 2, 0 }, { pctl_mux_nand_d4, 2, 0 }, { pctl_mux_nand_d5, 2, 0 } },
};

qspi_pctl_pad_t qspi_pctl_pad[2][8] = {
	{ { pctl_pad_nand_d7, 1, 4, 0 }, { pctl_pad_nand_ale, 1, 4, 0 }, { pctl_pad_nand_wp, 1, 4, 0 }, { pctl_pad_nand_rdy, 1, 4, 0 }, { pctl_pad_nand_ce0, 1, 4, 0 }, { pctl_pad_nand_ce1, 1, 4, 0 }, { pctl_pad_nand_cle, 1, 4, 0 }, { pctl_pad_nand_dqs, 1, 4, 0 } },
	{ { pctl_pad_nand_re, 1, 4, 0 }, { pctl_pad_nand_we, 1, 4, 0 }, { pctl_pad_nand_d0, 1, 4, 0 }, { pctl_pad_nand_d1, 1, 4, 0 }, { pctl_pad_nand_d2, 1, 4, 0 }, { pctl_pad_nand_d3, 1, 4, 0 }, { pctl_pad_nand_d4, 1, 4, 0 }, { pctl_pad_nand_d5, 1, 4, 0 } },
};


struct {
	volatile uint32_t *base;
	uint32_t flash_base_addr[2];
	bool init;
} qspi_common = { .base = NULL, .init = false, .flash_base_addr = { QSPI_MMAP_BASE, QSPI_MMAP_BASE + QSPI_FLASH_A1_SIZE + QSPI_FLASH_A2_SIZE } };

// TODO ustawić ile ohmów w rejestrze na flashu.
// TODO find clock frequency - ~50Mhz

static void set_mux(qspi_dev_t dev_no)
{
	platformctl_t ctl;
	int i;

	ctl.action = pctl_set;
	ctl.type = pctl_iomux;

	for (i = 0; i < 8; i++) {
		ctl.iomux.mux = qspi_pctl_mux[dev_no][i].mux;
		ctl.iomux.mode = qspi_pctl_mux[dev_no][i].val;
		ctl.iomux.sion = qspi_pctl_mux[dev_no][i].sion;
		platformctl(&ctl);
	}

	ctl.action = pctl_set;
	ctl.type = pctl_iopad;

	for (i = 0; i < 8; i++) {
		ctl.iopad.pad = qspi_pctl_pad[dev_no][i].pad;
		ctl.iopad.speed = qspi_pctl_pad[dev_no][i].speed;
		ctl.iopad.dse = qspi_pctl_pad[dev_no][i].dse;
		ctl.iopad.sre = qspi_pctl_pad[dev_no][i].sre;
		platformctl(&ctl);
	}
}

static void set_clk()
{
	platformctl_t ctl;

	ctl.action = pctl_set;
	ctl.type = pctl_devclock;
	ctl.devclock.dev = pctl_clk_qspi;
	ctl.devclock.state = 0x03;

	platformctl(&ctl);
}

int qspi_setLutSeq(const lut_seq_t *lut, unsigned int lut_seq)
{
	if (lut_seq >= NUM_LUT_SEQ) {
		return -1;
	}
	if (*(qspi_common.base + QSPI_LCKCR) & QSPI_LCKCR_LOCK) {
		*(qspi_common.base + QSPI_LUTKEY) = QSPI_LUTKEY_KEY;
		*(qspi_common.base + QSPI_LCKCR) = (*(qspi_common.base + QSPI_LCKCR) & ~(QSPI_LCKCR_UNLOCK | QSPI_LCKCR_LOCK)) | QSPI_LCKCR_UNLOCK;
	}

	for (int i = 0; i < 4; i++) {
		*(qspi_common.base + QSPI_LUTn(QSPI_SEQ_START_REGN(lut_seq) + i)) = (((uint32_t)lut->instrs[2 * i + 1]) << 16) | lut->instrs[2 * i];
	}

	*(qspi_common.base + QSPI_LUTKEY) = QSPI_LUTKEY_KEY;
	*(qspi_common.base + QSPI_LCKCR) = (*(qspi_common.base + QSPI_LCKCR) & ~(QSPI_LCKCR_UNLOCK | QSPI_LCKCR_LOCK)) | QSPI_LCKCR_LOCK;
}

static void set_IPCR(int seq_num, size_t idatsz)
{
	*(qspi_common.base + QSPI_IPCR) = QSPI_IPCR_SET_IDATSZ(QSPI_IPCR_SET_SEQID(*(qspi_common.base + QSPI_IPCR), seq_num), idatsz & 0xffff);
}

void qspi_setTCSH(uint8_t cycles)
{
	*(qspi_common.base + QSPI_FLSHCR) = (*(qspi_common.base + QSPI_FLSHCR) & ~(0x0f << 8)) | (cycles << 8);
}

void qspi_setTCSS(uint8_t cycles)
{
	*(qspi_common.base + QSPI_FLSHCR) = (*(qspi_common.base + QSPI_FLSHCR) & ~(0x0f)) | cycles;
}

int _qspi_readBusy(qspi_dev_t dev, unsigned int lut_seq, uint32_t addr, void *buf, size_t size)
{
	uint8_t to_read, byte, i;
	uint16_t len = 0;
	uint32_t reg;
	int err;


	if (dev != qspi_flash_a && dev != qspi_flash_b) {
		return -ENODEV;
	}
	if (lut_seq >= NUM_LUT_SEQ || size > MAX_READ_LEN) {
		return -1;
	}

	*(qspi_common.base + QSPI_RBCT) = (*(qspi_common.base + QSPI_RBCT) & ~(0x1f) | WATERMARK);
	*(qspi_common.base + QSPI_SFAR) = qspi_common.flash_base_addr[dev] + addr;
	*(qspi_common.base + QSPI_MCR) |= QSPI_MCR_CLR_RXF;

	set_IPCR(lut_seq, size);

	while (len < size) {
		do {
			reg = *(qspi_common.base + QSPI_SR);
		} while ((reg & QSPI_SR_RXWE) == 0 && reg & QSPI_SR_BUSY);

		for (i = 0; i < WATERMARK + 1 && len < size; i++) {
			reg = *(qspi_common.base + QSPI_RBDRn(i));
			for (byte = 0; byte < 4 && len + byte < size; byte++) {
				((uint8_t *)buf)[len + byte] = reg & 0xff;
				reg >>= 8;
			}
			len += byte;
		}
		*(qspi_common.base + QSPI_FR) |= QSPI_FR_RBDF; /* Buffer pop. */
	}

	while (*(qspi_common.base + QSPI_SR) & QSPI_SR_BUSY)
		;
	*(qspi_common.base + QSPI_MCR) |= QSPI_MCR_CLR_RXF;

	return EOK;
}

static size_t write_tx(const void *data, size_t size)
{
	size_t byte, sent = 0;
	uint32_t reg;

	while ((*(qspi_common.base + QSPI_SR) & QSPI_SR_TXFULL) == 0 && sent < size) {
		reg = 0;
		for (byte = 0; byte < 4 && sent + byte < size; byte++) {
			reg |= ((uint8_t *)data)[sent + byte] << (8 * byte);
		}
		*(qspi_common.base + QSPI_TBDR) = reg;
		sent += byte;
	}

	return sent;
}

int _qspi_write(qspi_dev_t dev, unsigned int lut_seq, uint32_t addr, const void *data, size_t size)
{
	size_t sent = 0;
	int err;
	if (dev != qspi_flash_a && dev != qspi_flash_b) {
		return -ENODEV;
	}
	if (lut_seq >= NUM_LUT_SEQ || size > MAX_READ_LEN) {
		return -1;
	}

	if (*(qspi_common.base + QSPI_SR) & QSPI_SR_TXEDA) /* Check if TX buffer is not empty. */
		*(qspi_common.base + QSPI_MCR) |= QSPI_MCR_CLR_TXF;

	*(qspi_common.base + QSPI_SFAR) = qspi_common.flash_base_addr[dev] + addr;

	sent += write_tx(data, size);

	set_IPCR(lut_seq, size);

	while (sent < size) {
		sent += write_tx(data + sent, size - sent);
	}

	while (*(qspi_common.base + QSPI_SR) & QSPI_SR_BUSY)
		;

	return EOK;
}

int _qspi_init(qspi_dev_t dev)
{
	int err;
	if (dev != qspi_flash_a && dev != qspi_flash_b) {
		printf("qspi: invalid device %d.\n", dev);
		return -ENODEV;
	}

	if (!qspi_common.init) {
		if ((qspi_common.base = mmap(NULL, 0x4000, PROT_READ | PROT_WRITE, MAP_DEVICE, OID_PHYSMEM, QSPI_BASE)) == MAP_FAILED) {
			printf("qspi: could not map qspi paddr %p.\n", QSPI_BASE);
			return -ENOMEM;
		}
		set_clk();
	}

	set_mux(dev);

	if (!qspi_common.init) {
		/* Enable module */
		*(qspi_common.base + QSPI_MCR) &= ~QSPI_MCR_CLR_MDIS;
		/* Set write to RXBR buffer */
		*(qspi_common.base + QSPI_RBCT) |= QSPI_RBCT_RXBRD;
		/* Set flash sizes */
		*(qspi_common.base + QSPI_SFA1AD) = (*(qspi_common.base + QSPI_SFA1AD) & (0x3ff)) | ((QSPI_MMAP_BASE + QSPI_FLASH_A1_SIZE) & ~(0x3ff));
		*(qspi_common.base + QSPI_SFA2AD) = (*(qspi_common.base + QSPI_SFA2AD) & (0x3ff)) | ((QSPI_MMAP_BASE + QSPI_FLASH_A2_SIZE) & ~(0x3ff));
		*(qspi_common.base + QSPI_SFB1AD) = (*(qspi_common.base + QSPI_SFB1AD) & (0x3ff)) | ((QSPI_MMAP_BASE + QSPI_FLASH_B1_SIZE) & ~(0x3ff));
		*(qspi_common.base + QSPI_SFB2AD) = (*(qspi_common.base + QSPI_SFB2AD) & (0x3ff)) | ((QSPI_MMAP_BASE + QSPI_FLASH_B2_SIZE) & ~(0x3ff));
	}
	qspi_common.init = true;

	return EOK;
}
