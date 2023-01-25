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
#include "qspi.h"
#include <board_config.h>

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

#define QSPI_BASE ((uint32_t *)0x21E0000)

#define QSPI_LCKCR_ADDR   (qspi_common.base + 0x304 / sizeof(uint32_t))
#define QSPI_LCKCR_LOCK   0x01
#define QSPI_LCKCR_UNLOCK 0x02

#define QSPI_LUT_KEY_ADDR (qspi_common.base + 0x300 / sizeof(uint32_t))
#define QSPI_LUT_KEY      0x5AF05AF0

#define QSPI_LUTn_ADDR(n)      (qspi_common.base + (0x310 + 4 * (n)) / sizeof(uint32_t))
#define QSPI_SEQ_START_REGN(n) (4 * (n))

#define QSPI_IPCR_ADDR                    (qspi_common.base + 0x08 / sizeof(uint32_t))
#define QSPI_IPCR_SET_SEQID(reg, seqid)   ((reg) & ~(0x0F << 24)) | ((seqid & 0x0F) << 24)
#define QSPI_IPCR_SET_IDATSZ(reg, idatsz) ((reg) & ~(0xFFFF)) | (idatsz)

#define QSPI_FLSHCR_ADDR (qspi_common.base + 0x0C / sizeof(uint32_t))

#define QSPI_MCR_ADDR     (qspi_common.base)
#define QSPI_MCR_CLR_TXF  (0x01 << 11)
#define QSPI_MCR_CLR_RXF  (0x01 << 10)
#define QSPI_MCR_CLR_MDIS (0x01 << 14)

#define QSPI_BFGENCR_ADDR (qspi_common.base + 0x20 / sizeof(uint32_t))

#define QSPI_SR_ADDR   (qspi_common.base + 0x15C / sizeof(uint32_t))
#define QSPI_SR_BUSY   (0x01)
#define QSPI_SR_RXWE   (0x01 << 16)
#define QSPI_SR_TXEDA  (0x01 << 24)
#define QSPI_SR_TXFULL (0x01 << 27)

#define QSPI_RBDRn_ADRR(n) (qspi_common.base + (0x200 + 4 * (n)) / sizeof(uint32_t))

#define QSPI_RBSR_ADRR    (qspi_common.base + 0x10C / sizeof(uint32_t))
#define QSPI_RBSR_TO_READ (((*QSPI_RBSR_ADRR) >> 8) & 0x3F)

#define QSPI_SFAR_ADDR (qspi_common.base + 0x100 / sizeof(uint32_t))

#define QSPI_RBCT_ADDR  (qspi_common.base + 0x110 / sizeof(uint32_t))
#define QSPI_RBCT_RXBRD (0x01 << 8)

#define QSPI_FR_ADDR (qspi_common.base + 0x160 / sizeof(uint32_t))
#define QSPI_FR_RBDF (0x01 << 16)

#define QSPI_TBDR_ADDR (qspi_common.base + 0x154 / sizeof(uint32_t))

#define QSPI_SFA1AD (qspi_common.base + 0x180 / sizeof(uint32_t))
#define QSPI_SFA2AD (qspi_common.base + 0x184 / sizeof(uint32_t))
#define QSPI_SFB1AD (qspi_common.base + 0x188 / sizeof(uint32_t))
#define QSPI_SFB2AD (qspi_common.base + 0x18C / sizeof(uint32_t))

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
	if (*QSPI_LCKCR_ADDR & QSPI_LCKCR_LOCK) {
		*QSPI_LUT_KEY_ADDR = QSPI_LUT_KEY;
		*QSPI_LCKCR_ADDR = (*QSPI_LCKCR_ADDR & ~(QSPI_LCKCR_UNLOCK | QSPI_LCKCR_LOCK)) | QSPI_LCKCR_UNLOCK;
	}

	for (int i = 0; i < 4; i++) {
		*QSPI_LUTn_ADDR(QSPI_SEQ_START_REGN(lut_seq) + i) = (((uint32_t)lut->instrs[2 * i + 1]) << 16) | lut->instrs[2 * i];
	}

	*QSPI_LUT_KEY_ADDR = QSPI_LUT_KEY;
	*QSPI_LCKCR_ADDR = (*QSPI_LCKCR_ADDR & ~(QSPI_LCKCR_UNLOCK | QSPI_LCKCR_LOCK)) | QSPI_LCKCR_LOCK;
}

static void set_IPCR(int seq_num, size_t idatsz)
{
	*QSPI_IPCR_ADDR = QSPI_IPCR_SET_IDATSZ(QSPI_IPCR_SET_SEQID(*QSPI_IPCR_ADDR, seq_num), idatsz & 0xFFFF);
}

void qspi_setTCSH(uint8_t cycles)
{
	*QSPI_FLSHCR_ADDR = (*QSPI_FLSHCR_ADDR & ~(0x0F << 8)) | (cycles << 8);
}

void qspi_setTCSS(uint8_t cycles)
{
	*QSPI_FLSHCR_ADDR = (*QSPI_FLSHCR_ADDR & ~(0x0F)) | cycles;
}

int _qspi_read(qspi_dev_t dev, unsigned int lut_seq, uint32_t addr, void *buf, size_t size)
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

	*QSPI_RBCT_ADDR = (*QSPI_RBCT_ADDR & ~(0x1F) | WATERMARK);

	printf("RBCT: %#x\n", *QSPI_RBCT_ADDR);

	*QSPI_SFAR_ADDR = qspi_common.flash_base_addr[dev] + addr;

	*QSPI_MCR_ADDR |= QSPI_MCR_CLR_RXF;

	printf("RXWE: %#x\n", *QSPI_SR_ADDR & QSPI_SR_RXWE);
	reg = *QSPI_RBDRn_ADRR(0);
	printf("PRE DR: %#x\n", reg);
	set_IPCR(lut_seq, size);

	printf("RXWE: %#x\n", *QSPI_SR_ADDR & QSPI_SR_RXWE);
	while (len < size) {
		printf("RXWE: %#x\n", *QSPI_SR_ADDR & QSPI_SR_RXWE);
		while (((*QSPI_SR_ADDR & QSPI_SR_RXWE) ^ QSPI_SR_BUSY) == 0) /* Wait for data ready */
			;

		printf("RXWE: %#x\n", *QSPI_SR_ADDR & QSPI_SR_RXWE);

		for (i = 0; i < WATERMARK + 1 && len < size; i++) {
			reg = *QSPI_RBDRn_ADRR(i);
			printf("DR: %#x\n", reg);
			for (byte = 0; byte < 4 && len + byte < size; byte++) {
				((uint8_t *)buf)[len + byte] = reg & 0xFF;
				reg >>= 8;
			}
			len += byte;
		}
		printf("POP\n");
		*QSPI_FR_ADDR |= QSPI_FR_RBDF; /* Buffer pop. */
	}

	while (*QSPI_SR_ADDR & QSPI_SR_BUSY)
		;
	*QSPI_MCR_ADDR |= QSPI_MCR_CLR_RXF;

	return EOK;
}

static size_t write_tx(const void *data, size_t size)
{
	size_t byte, sent = 0;
	uint32_t reg;

	while ((*QSPI_SR_ADDR & QSPI_SR_TXFULL) == 0 && sent < size) {
		reg = 0;
		for (byte = 0; byte < 4 && sent + byte < size; byte++) {
			reg |= ((uint8_t *)data)[sent + byte] << (8 * byte);
		}
		*QSPI_TBDR_ADDR = reg;
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

	if (*QSPI_SR_ADDR & QSPI_SR_TXEDA) /* Check if TX buffer is not empty. */
		*QSPI_MCR_ADDR |= QSPI_MCR_CLR_TXF;

	*QSPI_SFAR_ADDR = qspi_common.flash_base_addr[dev] + addr;

	sent += write_tx(data, size);
	printf("SENT: %d, BUSY: %#x \n", sent, *QSPI_SR_ADDR & QSPI_SR_BUSY);
	printf("SIZE: %d\n", size);

	set_IPCR(lut_seq, size);
	printf("SENT: %d, BUSY: %#x \n", sent, *QSPI_SR_ADDR & QSPI_SR_BUSY);

	while (sent < size) {
		sent += write_tx(data + sent, size - sent);
		printf("SENT: %d, BUSY: %#x \n", sent, *QSPI_SR_ADDR & QSPI_SR_BUSY);
	}

	printf("WAITING\n");
	while (*QSPI_SR_ADDR & QSPI_SR_BUSY)
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
		*QSPI_MCR_ADDR &= ~QSPI_MCR_CLR_MDIS;
		/* Set write to RXBR buffer */
		*QSPI_RBCT_ADDR |= QSPI_RBCT_RXBRD;
		/* Set flash sizes */
		*QSPI_SFA1AD = (*QSPI_SFA1AD & (0x3ff)) | ((QSPI_MMAP_BASE + QSPI_FLASH_A1_SIZE) & ~(0x3ff));
		*QSPI_SFA2AD = (*QSPI_SFA2AD & (0x3ff)) | ((QSPI_MMAP_BASE + QSPI_FLASH_A2_SIZE) & ~(0x3ff));
		*QSPI_SFB1AD = (*QSPI_SFB1AD & (0x3ff)) | ((QSPI_MMAP_BASE + QSPI_FLASH_B1_SIZE) & ~(0x3ff));
		*QSPI_SFB2AD = (*QSPI_SFB2AD & (0x3ff)) | ((QSPI_MMAP_BASE + QSPI_FLASH_B2_SIZE) & ~(0x3ff));
	}
	qspi_common.init = true;

	return EOK;
}
